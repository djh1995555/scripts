#!/usr/bin/env python3
# encoding: utf-8

import os
import re
import sys
import json
import argparse
import numpy as np
import rosbag
import yaml
from collections import OrderedDict
import pandas as pd

try:
    from mfbagpy.rosapi_style import mfbag
except ModuleNotFoundError:
    class MFBAG:
        def MFBag(self, input):
            return []
    mfbag = MFBAG()

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from utils.report_plotter import ReportPlotter

def clean_json_string(s):
    return s.replace('\r\n', '').replace('\n', '').replace('\r', '').strip()



from json_fields_map import _SECTION_ORDER, _DISPLAY_NAMES, _FIELD_TO_SECTION


def to_scalar_array(values):
    try:
        arr = np.array(values, dtype=float)
        if arr.ndim == 1 and len(arr) > 0:
            return arr
    except (ValueError, TypeError):
        pass
    return None


# MPC 预测序列(vec)字段取这 4 个分量各画一条曲线
VEC_INDICES = [0, 5, 10, 25]

# trace 默认色循环（与 plotly 内置 default 一致，显式给出以便 legend 色块对齐）
PLOTLY_COLORS = [
    '#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd',
    '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf',
]


def parse_field_value(value):
    """把 extra.json 单字段的 value 归一化:
    标量 -> float (空/解析失败 -> nan);
    向量 (逗号分隔多元素串 / 多元素 list) -> list[float], 保留完整序列。
    末尾空段(形如 "a,b,c,")会被丢弃。"""
    while isinstance(value, (list, tuple)):
        value = value[0] if len(value) > 0 else None
    if value is None:
        return np.nan
    if isinstance(value, bool):
        return float(value)
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        s = value.strip()
        if not s:
            return np.nan
        parts = [p.strip() for p in s.split(',')]
        while parts and parts[-1] == '':
            parts.pop()
        if len(parts) <= 1:
            try:
                return float(parts[0]) if parts else np.nan
            except ValueError:
                return np.nan
        vec = []
        for p in parts:
            try:
                vec.append(float(p))
            except ValueError:
                vec.append(np.nan)
        return vec
    return np.nan


class ReportGenerator:
    def __init__(self, bag_path, config_path=None):
        self._bag_path = os.path.abspath(bag_path)
        self._config_path = os.path.abspath(config_path) if config_path else None

        bag_dir = os.path.dirname(self._bag_path)
        bag_stem = os.path.splitext(os.path.basename(self._bag_path))[0]
        self._output_html = os.path.join(bag_dir, f"{bag_stem}_report.html")

        self._plotter = ReportPlotter('ReportGenerator')
        self._figure_height = 400       # px per subplot row
        self._exclude_fields = {'timestamp'}

        self._timestamps = []
        self._raw_fields = {}
        self._full_db = OrderedDict()
        self._target_data = OrderedDict()
        self._config = {}
        self._section_order = []
        self._display_names = {}
        self._field_section = {}

    # ------------------------------------------------------------------
    def run(self):
        print(f"[1/5] Reading bag: {self._bag_path}")
        self._read_bag()

        print("[2/5] Loading section map (inline)")
        self._section_order = _SECTION_ORDER
        self._display_names = _DISPLAY_NAMES
        self._field_section = _FIELD_TO_SECTION

        print("[3/5] Organising data into sections")
        self._organise_full_db()

        if self._config_path:
            print(f"[4/5] Extracting target signals from config: {self._config_path}")
            self._load_config()
            self._extract_target_signals()
        else:
            print("[4/5] No config provided – skipping target signal extraction")

        print(f"[5/5] Writing report: {self._output_html}")
        self._generate_report()
        print("Done.")

    # ------------------------------------------------------------------
    @staticmethod
    def _walk_leaves(obj, cmd_type, prefix, out):
        """递归采集子消息数值/bool 叶子，key='{cmd_type}::{dotted_path}'。
        嵌套子消息递归（如 steering_type.value）；标量/bool→float；数值 list→list[float]。"""
        slots = getattr(obj, '__slots__', None)
        types = getattr(obj, '_slot_types', None)
        if not slots or not types:
            return out
        for name, _typ in zip(slots, types):
            try:
                val = getattr(obj, name, None)
            except Exception:
                val = None
            if val is None:
                continue
            path = f'{prefix}.{name}' if prefix else name
            if hasattr(val, '__slots__'):                      # 嵌套子消息
                ReportGenerator._walk_leaves(val, cmd_type, path, out)
            elif isinstance(val, bool):
                out[f'{cmd_type}::{path}'] = float(val)
            elif isinstance(val, (int, float)):
                out[f'{cmd_type}::{path}'] = float(val)
            elif isinstance(val, (list, tuple)):
                nums = [float(x) for x in val if isinstance(x, (int, float, bool))]
                if nums:
                    out[f'{cmd_type}::{path}'] = nums
        return out

    def _collect_native_fields(self, msg):
        """采集 ControlCommand 原生子消息（非 extra）的数值叶子。
        跳过 extra（由 extra.json 单独处理）；available==0 的命令跳过。
        返回 {key: value}，key 形如 'BrakeCommand::brake_command'、
        'SteeringCommand::steering_type.value'。"""
        out = {}
        slots = getattr(msg, '__slots__', None)
        types = getattr(msg, '_slot_types', None)
        if not slots or not types:
            return out
        for slot_name, slot_type in zip(slots, types):
            if slot_name == 'extra' or 'endpoint_msgs/' not in slot_type:
                continue
            cmd_obj = getattr(msg, slot_name, None)
            if cmd_obj is None:
                continue
            avail = getattr(cmd_obj, 'available', None)
            if isinstance(avail, (int, float, bool)) and int(avail) == 0:
                continue
            cmd_type = slot_type.split('/')[-1]                  # e.g. BrakeCommand
            # 找 *_data 子消息（如 brake_command_data）；找不到则用 cmd_obj 本身
            data_obj = None
            c_slots = getattr(cmd_obj, '__slots__', None) or []
            c_types = getattr(cmd_obj, '_slot_types', None) or []
            for dn, dt in zip(c_slots, c_types):
                if dn.endswith('_data') and 'endpoint_msgs/' in dt:
                    data_obj = getattr(cmd_obj, dn, None)
                    break
            if data_obj is None:
                data_obj = cmd_obj
            self._walk_leaves(data_obj, cmd_type, '', out)
        return out

    # ------------------------------------------------------------------
    def _read_bag(self):
        is_mfbag = self._bag_path.endswith('.mfbag')
        bag = mfbag.MFBag(self._bag_path) if is_mfbag else rosbag.Bag(self._bag_path)

        if not is_mfbag:
            try:
                if bag.get_compression_info().compression == 'lz4':
                    print("  Decompressing (lz4)…")
                    os.system(f"rosbag decompress {self._bag_path} -f")
                    bag = rosbag.Bag(self._bag_path)
            except Exception:
                pass

        timestamps = []
        raw_fields = {}

        for _topic, msg, _t in bag.read_messages(topics=["/msd/endpoint/control_command"]):
            if is_mfbag:
                time_now = msg.obj.header.stamp / 1e9
                msg = msg.obj
            else:
                time_now = msg.header.stamp.secs + msg.header.stamp.nsecs / 1e9

            try:
                json_str = clean_json_string(msg.extra.json)
                json_data = json.loads(json_str)
            except Exception as e:
                print(f"  Warning: JSON parse error at t={time_now:.3f}: {e}")
                continue

            timestamps.append(time_now)
            n = len(timestamps)

            for field, value in json_data.items():
                # 标量->float(空/失败为nan); 向量->list[float]保留完整序列
                fv = parse_field_value(value)
                if field not in raw_fields:
                    raw_fields[field] = [np.nan] * (n - 1)
                raw_fields[field].append(fv)

            # 原生 ControlCommand 子消息字段（非 extra.json）
            # key 形如 'BrakeCommand::brake_command'；value 已是 float / list[float]
            for field, value in self._collect_native_fields(msg).items():
                if field not in raw_fields:
                    raw_fields[field] = [np.nan] * (n - 1)
                raw_fields[field].append(value)

            for field in raw_fields:
                while len(raw_fields[field]) < n:
                    raw_fields[field].append(np.nan)

        try:
            bag.close()
        except Exception:
            pass

        if not timestamps:
            raise ValueError("No messages found on /msd/endpoint/control_command")

        t0 = timestamps[0]
        self._timestamps = [t - t0 for t in timestamps]
        self._raw_fields = raw_fields
        print(f"  {len(timestamps)} frames  |  {len(raw_fields)} fields")

    # ------------------------------------------------------------------
    def _organise_full_db(self):
        self._full_db = OrderedDict()
        for sid in self._section_order:
            self._full_db[sid] = {'timestamp': self._timestamps}
        self._full_db['Other'] = {'timestamp': self._timestamps}

        for field, values in self._raw_fields.items():
            # 原生字段 key 含 '::'（如 'BrakeCommand::brake_command'）→ section=Command 名
            if '::' in field:
                sid, leaf = field.split('::', 1)
            else:
                sid = self._field_section.get(field, 'Other')
                leaf = field
            if sid not in self._full_db:
                self._full_db[sid] = {'timestamp': self._timestamps}
            self._full_db[sid][leaf] = values

        for sid in list(self._full_db.keys()):
            if len(self._full_db[sid]) <= 1:
                del self._full_db[sid]

        print(f"  {len(self._full_db)} non-empty sections")

    # ------------------------------------------------------------------
    def _load_config(self):
        with open(self._config_path, 'r', encoding='utf-8') as f:
            self._config = yaml.safe_load(f)

    def _find_field_data(self, group, field):
        if group in self._full_db and field in self._full_db[group]:
            return self._full_db[group][field], self._full_db[group]['timestamp']
        for sid, data in self._full_db.items():
            if field in data and field != 'timestamp':
                return data[field], data['timestamp']
        return None, None

    def _extract_target_signals(self):
        target_panel = self._config.get('target_panel', {})
        for panel_name, signals in target_panel.items():
            panel = OrderedDict()
            for display_name, signal_ref in signals.items():
                ref = str(signal_ref)
                # '::' 分隔 = 原生 Command 子消息（如 'BrakeCommand::brake_command'）
                # ':'  分隔 = extra.json 分组（旧格式 'group:field'）
                # 无分隔 = extra.json 字段名
                if '::' in ref:
                    group, field = ref.split('::', 1)
                elif ':' in ref:
                    group, field = ref.split(':', 1)
                else:
                    group, field = '', ref
                values, ts = self._find_field_data(group, field)
                if values is not None:
                    panel[display_name] = (values, ts)
                else:
                    print(f"  Warning: '{signal_ref}' not found in bag data")
            self._target_data[panel_name] = panel

    # ------------------------------------------------------------------
    def _make_traces(self, entries):
        """entries: [(legend_name, values, timestamps)] -> [go.Scatter]
        标量字段画一条曲线; 向量字段(每帧为 list)按 VEC_INDICES 取分量画多条
        每条 trace 显式着色（PLOTLY_COLORS 循环），供外部 HTML legend 色块对齐"""
        import plotly.graph_objects as go
        traces = []
        color_i = 0
        for name, values, timestamps in entries:
            t_arr = np.array(timestamps, dtype=float)
            is_vec = any(isinstance(v, (list, tuple)) for v in values)
            if is_vec:
                for idx in VEC_INDICES:
                    col = [(v[idx] if isinstance(v, (list, tuple)) and idx < len(v) else np.nan)
                           for v in values]
                    arr = np.array(col, dtype=float)
                    arr_clean = (pd.DataFrame({'v': arr})['v']
                                 .interpolate(method='linear', limit_direction='both')
                                 .to_numpy())
                    if np.isnan(arr_clean).all():
                        color_i += 1
                        continue
                    color = PLOTLY_COLORS[color_i % len(PLOTLY_COLORS)]
                    traces.append(go.Scatter(x=t_arr, y=arr_clean,
                                             name=f"{name}[{idx}]", mode='lines',
                                             line=dict(color=color)))
                    color_i += 1
            else:
                arr = to_scalar_array(values)
                if arr is None:
                    color_i += 1
                    continue
                arr_clean = (pd.DataFrame({'v': arr})['v']
                             .interpolate(method='linear', limit_direction='both')
                             .to_numpy())
                if np.isnan(arr_clean).all():
                    color_i += 1
                    continue
                color = PLOTLY_COLORS[color_i % len(PLOTLY_COLORS)]
                traces.append(go.Scatter(x=t_arr, y=arr_clean, name=name, mode='lines',
                                         line=dict(color=color)))
                color_i += 1
        return traces

    def _build_subplot_fig(self, panel_traces_list):
        """panel_traces_list: [(title, [go.Scatter, ...])]
        Returns (styled plotly Figure, legend_by_panel) 或 (None, []).
        legend_by_panel = [[(name, color), ...], ...]，每个子图一组（按 fig.data
        顺序累计 global index），供外部 HTML legend 按子图分组。
        不用 plotly 内置 legend（SVG 无法滚动），改由外部 HTML legend 承载
        （每子图一个 max-height + overflow-y:auto 容器），避免项数多时溢出覆盖相邻子图。"""
        from plotly.subplots import make_subplots

        panel_traces_list = [(t, trs) for t, trs in panel_traces_list if trs]
        if not panel_traces_list:
            return None, []

        n_rows = len(panel_traces_list)
        total_height = self._figure_height * n_rows
        v_spacing = min(0.015, 0.9 / n_rows) if n_rows > 1 else 0.0

        sf = make_subplots(
            rows=n_rows, cols=1,
            shared_xaxes=True,
            vertical_spacing=v_spacing,
            subplot_titles=[t for t, _ in panel_traces_list],
        )

        legend_by_panel = []
        for row_idx, (_, traces) in enumerate(panel_traces_list, start=1):
            panel_items = []
            for trace in traces:
                sf.add_trace(trace, row=row_idx, col=1)
                color = trace.line.color if trace.line and trace.line.color else PLOTLY_COLORS[0]
                panel_items.append((trace.name, color))
            legend_by_panel.append(panel_items)

        sf.update_layout(
            height=total_height,
            autosize=True,
            template='plotly_dark',
            paper_bgcolor='#111',
            plot_bgcolor='#111',
            showlegend=False,
            margin=dict(r=20, t=30, l=60),
        )
        sf.update_xaxes(showticklabels=True)
        for ann in sf.layout.annotations:
            ann.font.color = 'white'
            ann.font.size = 20
        return sf, legend_by_panel

    # ------------------------------------------------------------------
    def _build_legend_html(self, legend_by_panel, plot_id, panel_height):
        """legend_by_panel: [[(name,color),...], ...] 每个子图一组。
        生成垂直堆叠的多个 legend-scroll，每组 max-height=panel_height 可独立滚动；
        组高 flex 等分对齐各子图，不再把所有子图 legend 混在一个容器。
        data-trace 为该 trace 在 fig.data 中的全局 index（供 Plotly.restyle）。"""
        parts = [f'<div class="legend-stack" data-plotid="{plot_id}">']
        gidx = 0
        for panel in legend_by_panel:
            items = []
            for name, color in panel:
                c = color or PLOTLY_COLORS[0]
                safe_name = str(name).replace('"', '&quot;')
                items.append(
                    f'<div class="legend-item" data-plot="{plot_id}" data-trace="{gidx}" title="{safe_name}">'
                    f'<span class="legend-swatch" style="background:{c}"></span>'
                    f'<span class="legend-text">{safe_name}</span></div>'
                )
                gidx += 1
            parts.append(
                f'<div class="legend-scroll" style="max-height:{panel_height}px">'
                + ''.join(items) + '</div>'
            )
        parts.append('</div>')
        return ''.join(parts)

    # ------------------------------------------------------------------
    def _generate_report(self):
        from plotly.offline import get_plotlyjs
        import plotly.io as pio
        import uuid

        # ── 特别关注：eagerly rendered subplot figure ─────────────────────
        target_panel_traces = []
        for panel_name, signals in self._target_data.items():
            entries = [(name, v, ts) for name, (v, ts) in signals.items()]
            traces = self._make_traces(entries)
            if traces:
                target_panel_traces.append((panel_name, traces))

        target_fig, target_legend = self._build_subplot_fig(target_panel_traces)
        target_html = ""
        target_div_id = None
        if target_fig is not None:
            h = self._figure_height * len(target_panel_traces)
            target_div_id = 'target-plot-fig'
            target_json_id = f'fig-json-{target_div_id}'
            legend_html = self._build_legend_html(target_legend, target_div_id, self._figure_height)
            # 懒渲染结构（plot div + json script + legend）；由 sync_js 在 flex
            # 布局完成后立即 newPlot，避免 flex 初始化时 plot-cell clientWidth=0
            # 导致 plotly 画空（子图不见）。
            target_html = (
                '<h2 style="text-align:center;color:white">特别关注信号</h2>\n'
                f'<div class="lazy-panel fig-row" data-plotid="{target_div_id}" data-jsonid="{target_json_id}" '
                f'style="min-height:{h}px">'
                f'<div class="plot-cell" id="{target_div_id}" style="height:{h}px"></div>'
                f'{legend_html}'
                f'</div>\n'
                f'<script type="application/json" id="{target_json_id}">{target_fig.to_json()}</script>\n'
            )

        # ── 全量数据库：figure JSON 存入 <script> tag，懒渲染 ──────────────
        db_blocks = []   # list of html strings
        db_json_ids = [] # [(plot_div_id, json_script_id)]

        for sid, section_data in reversed(list(self._full_db.items())):
            timestamps = section_data['timestamp']
            display = self._display_names.get(sid, sid)
            entries = [
                (field, values, timestamps)
                for field, values in section_data.items()
                if field not in self._exclude_fields
            ]
            traces = self._make_traces(entries)
            if not traces:
                continue

            fig, legend = self._build_subplot_fig([(display, traces)])
            if fig is None:
                continue

            plot_div_id = str(uuid.uuid4())
            json_script_id = f"fig-json-{plot_div_id}"
            fig_json = fig.to_json()
            h = self._figure_height
            legend_html = self._build_legend_html(legend, plot_div_id, self._figure_height)

            block = (
                f'<div class="lazy-panel fig-row" data-plotid="{plot_div_id}" data-jsonid="{json_script_id}" '
                f'style="min-height:{h}px">'
                f'<div class="plot-cell" id="{plot_div_id}" style="height:{h}px"></div>'
                f'{legend_html}'
                f'</div>\n'
                f'<script type="application/json" id="{json_script_id}">{fig_json}</script>\n'
            )
            db_blocks.append(block)
            db_json_ids.append((plot_div_id, json_script_id))

        if not target_html and not db_blocks:
            print("Warning: no plottable data found")
            return

        # ── JS：懒渲染 + x 轴联动 ──────────────────────────────────────────
        # x-axis state is stored globally; newly rendered panels inherit it.
        # Sync is debounced (50 ms) and only touches already-rendered divs.
        eager_id_js = json.dumps(target_div_id)
        sync_js = """<script>
(function() {
  var currentRange = null;   // null = autorange, else {x0, x1}
  var syncing = false;
  var syncTimer = null;

  function applyRange(el) {
    if (!el || !el._fullLayout) return;
    if (currentRange === null) {
      Plotly.relayout(el, {'xaxis.autorange': true});
    } else {
      Plotly.relayout(el, {'xaxis.range[0]': currentRange.x0,
                            'xaxis.range[1]': currentRange.x1});
    }
  }

  function scheduleSync(sourceId) {
    if (syncTimer) clearTimeout(syncTimer);
    syncTimer = setTimeout(function() {
      syncing = true;
      document.querySelectorAll('.plotly-graph-div[data-synced="1"]').forEach(function(el) {
        if (el.id === sourceId) return;
        applyRange(el);
      });
      syncing = false;
    }, 50);
  }

  function attachSync(el) {
    el.setAttribute('data-synced', '1');
    el.on('plotly_relayout', function(ed) {
      if (syncing) return;
      var x0 = ed['xaxis.range[0]'], x1 = ed['xaxis.range[1]'];
      if (ed['xaxis.autorange']) {
        currentRange = null;
      } else if (x0 !== undefined && x1 !== undefined) {
        currentRange = {x0: x0, x1: x1};
      } else {
        return;
      }
      scheduleSync(el.id);
    });
    // apply current global range immediately
    if (currentRange !== null) applyRange(el);
  }

  // attach sync to eagerly rendered target figure
  var eagerId = """ + eager_id_js + """;
  if (eagerId) {
    var eagerEl = document.getElementById(eagerId);
    var eagerJson = document.getElementById('fig-json-' + eagerId);
    if (eagerEl && eagerJson && !eagerEl._fullData) {
      try {
        var ed = JSON.parse(eagerJson.textContent);
        Plotly.newPlot(eagerEl, ed.data, ed.layout, {responsive: true});
        attachSync(eagerEl);
        var w = eagerEl.closest('.lazy-panel');
        if (w) { w.dataset.rendered = '1'; }
      } catch (e) {}
    }
  }

  // lazy-render DB panels with IntersectionObserver
  var observer = new IntersectionObserver(function(entries) {
    entries.forEach(function(entry) {
      if (!entry.isIntersecting) return;
      var wrapper = entry.target;
      if (wrapper.dataset.rendered) return;
      wrapper.dataset.rendered = '1';
      observer.unobserve(wrapper);

      var plotId  = wrapper.dataset.plotid;
      var jsonId  = wrapper.dataset.jsonid;
      var plotDiv = document.getElementById(plotId);
      var jsonEl  = document.getElementById(jsonId);
      if (!plotDiv || !jsonEl) return;

      var figData = JSON.parse(jsonEl.textContent);
      Plotly.newPlot(plotDiv, figData.data, figData.layout, {responsive: true}).then(function() {
        attachSync(plotDiv);
      });
    });
  }, {rootMargin: '600px 0px'});  // pre-render 600 px before viewport

  document.querySelectorAll('.lazy-panel').forEach(function(el) {
    observer.observe(el);
  });

  // ── legend 点击：切换 trace 可见性（含懒渲染预触发）─────────────────
  function ensurePlot(plotId) {
    var el = document.getElementById(plotId);
    if (!el) return null;
    if (el._fullData) return el;                  // 已渲染（target eager 或 observer 已触发）
    var jsonEl = document.getElementById('fig-json-' + plotId);
    if (!jsonEl) return null;                     // target 无独立 json script
    try {
      var figData = JSON.parse(jsonEl.textContent);
      Plotly.newPlot(el, figData.data, figData.layout, {responsive: true});
      attachSync(el);
      var wrapper = el.closest('.lazy-panel');
      if (wrapper) { wrapper.dataset.rendered = '1'; observer.unobserve(wrapper); }
    } catch (e) { return null; }
    return el;
  }
  // legend 交互：单击=切换该项；双击=只显示该项（再双击恢复全部）
  function syncLegendOff(plotId) {
    var el = document.getElementById(plotId);
    if (!el || !el._fullData) return;
    document.querySelectorAll('.legend-item[data-plot="' + plotId + '"]').forEach(function(it) {
      var i = parseInt(it.dataset.trace, 10);
      var t = el._fullData[i];
      var off = t && (t.visible === 'legendonly' || t.visible === false);
      it.classList.toggle('legend-off', off);
    });
  }
  function singleClick(item) {
    var plotId = item.dataset.plot;
    var idx = parseInt(item.dataset.trace, 10);
    var el = ensurePlot(plotId);
    if (!el || !el._fullData) return;
    var t = el._fullData[idx];
    if (!t) return;
    var isOn = !(t.visible === 'legendonly' || t.visible === false);
    Plotly.restyle(el, {visible: isOn ? 'legendonly' : true}, [idx]).then(function() {
      syncLegendOff(plotId);
    });
  }
  function doubleClick(item) {
    var plotId = item.dataset.plot;
    var idx = parseInt(item.dataset.trace, 10);
    var el = ensurePlot(plotId);
    if (!el || !el._fullData) return;
    var n = el._fullData.length;
    var isOnly = true;                                   // 当前是否已是"仅 idx 显示"
    for (var j = 0; j < n; j++) {
      var v = el._fullData[j].visible;
      var hidden = (v === 'legendonly' || v === false);
      if (j === idx ? hidden : !hidden) { isOnly = false; break; }
    }
    var visArr = [];
    for (var j = 0; j < n; j++) visArr.push(isOnly ? true : (j === idx ? true : 'legendonly'));
    Plotly.restyle(el, {visible: visArr}).then(function() { syncLegendOff(plotId); });
  }
  var clickTimer = null;
  document.body.addEventListener('click', function(e) {
    var item = e.target.closest('.legend-item');
    if (!item) return;
    if (clickTimer) { clearTimeout(clickTimer); clickTimer = null; doubleClick(item); }
    else { var it = item; clickTimer = setTimeout(function() { clickTimer = null; singleClick(it); }, 220); }
  });
})();
</script>"""

        db_block = ""
        if db_blocks:
            db_block = '<h2 style="text-align:center;color:white;margin-top:30px">全量数据库</h2>\n'
            db_block += '\n'.join(db_blocks)

        html = f"""<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>Control Debug Report</title>
  <script type="text/javascript">{get_plotlyjs()}</script>
  <style>
    .fig-row {{ display:flex; gap:8px; align-items:flex-start; margin:8px 0; }}
    .plot-cell {{ flex:1 1 auto; min-width:0; }}
    .legend-stack {{
      flex:0 0 200px; display:flex; flex-direction:column; gap:2px;
      min-width:0; align-self:stretch;
    }}
    .legend-scroll {{
      flex:1 1 0; min-height:0; overflow-y:auto; overflow-x:hidden;
      background:#1a1a1a; border:1px solid #333; border-radius:4px;
      padding:4px 6px; font-size:12px; line-height:1.5;
    }}
    .legend-item {{ display:flex; align-items:center; gap:6px; cursor:pointer;
      padding:2px 4px; border-radius:3px; white-space:nowrap; }}
    .legend-item:hover {{ background:#2a2a2a; }}
    .legend-item.legend-off {{ opacity:0.4; }}
    .legend-item.legend-off .legend-text {{ text-decoration:line-through; }}
    .legend-swatch {{ display:inline-block; width:18px; height:3px; flex:0 0 18px; }}
    .legend-text {{ overflow:hidden; text-overflow:ellipsis; }}
  </style>
</head>
<body style="background-color:#111;color:white;font-family:sans-serif;padding:10px;margin:0;box-sizing:border-box">
  <h1 style="text-align:center">Control Debug Report</h1>
  <p style="text-align:center;color:#aaa">{self._bag_path}</p>
  {target_html}
  {db_block}
  {sync_js}
</body>
</html>"""

        with open(self._output_html, 'w', encoding='utf-8') as f:
            f.write(html)

        print(f"  target: {len(target_panel_traces)} subplots, db: {len(db_blocks)} panels (lazy)")


# ----------------------------------------------------------------------
def fetch_and_report(md5_dir_pairs, output_dir, config_path=None):
    """
    Mode 2: for each (md5, dir_name) pair, download via `mdi fetch <md5>`,
    place under output_dir/<dir_name>/, then generate a report.
    dir_name=None means use the bag filename stem.
    """
    import subprocess
    import glob

    os.makedirs(output_dir, exist_ok=True)

    for md5, dir_name in md5_dir_pairs:
        md5 = md5.strip()
        if not md5:
            continue

        print(f"\n{'='*60}")
        print(f"[MD5] {md5}" + (f"  ->  {dir_name}" if dir_name else ""))

        # ── 下载 bag ──────────────────────────────────────────────────
        fetch_cmd = f"mdi fetch {md5}"
        print(f"  Fetching: {fetch_cmd}")
        result = subprocess.run(
            fetch_cmd, shell=True, cwd=output_dir,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
        )
        print(result.stdout.strip())

        if result.returncode != 0:
            print(f"  ERROR: fetch failed for {md5}, skipping")
            continue

        # ── 找到下载的 bag 文件（最新 mtime）─────────────────────────
        bag_files = glob.glob(os.path.join(output_dir, '*.bag')) + \
                    glob.glob(os.path.join(output_dir, '*.mfbag'))
        if not bag_files:
            print(f"  ERROR: no bag file found after fetch for {md5}, skipping")
            continue

        bag_path = max(bag_files, key=os.path.getmtime)
        bag_stem = os.path.splitext(os.path.basename(bag_path))[0]

        # ── 移动到目标子目录（优先用指定名，否则用 bag 文件名）────────
        target_dir_name = dir_name if dir_name else bag_stem
        bag_dir = os.path.join(output_dir, target_dir_name)
        os.makedirs(bag_dir, exist_ok=True)
        dest_bag = os.path.join(bag_dir, os.path.basename(bag_path))
        os.rename(bag_path, dest_bag)
        print(f"  Moved bag to: {dest_bag}")

        # ── 生成报告 ─────────────────────────────────────────────────
        try:
            ReportGenerator(dest_bag, config_path).run()
        except Exception as e:
            print(f"  ERROR: report generation failed: {e}")


def main():
    parser = argparse.ArgumentParser(
        description='Generate control debug report from a bag file or via MD5 fetch',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Mode 1: single bag
  python report_generator.py /path/to/foo.bag --config config/target_signal_lat.yaml

  # Mode 2: fetch by MD5, optional dir names (same order as --md5)
  python report_generator.py --md5 abc123 def456 --dir-name scene1 scene2 --output-dir /data/reports

  # Mode 2: MD5 file, each line is  md5  or  md5:dir_name
  python report_generator.py --md5-file md5_list.txt --output-dir /data/reports
""")

    parser.add_argument('bag_path', nargs='*', default=None,
                        help='One or more .bag/.mfbag file paths (Mode 1)')
    parser.add_argument('--config', type=str, default=None,
                        help='Path to YAML config with target_panel signals')

    # Mode 2 arguments
    parser.add_argument('--md5', nargs='+', metavar='MD5',
                        help='One or more MD5 values to fetch (Mode 2)')
    parser.add_argument('--dir-name', nargs='+', metavar='NAME',
                        help='Directory names corresponding to each --md5 (same order); '
                             'omit to use bag filename')
    parser.add_argument('--md5-file', type=str, metavar='FILE',
                        help='Text file with one entry per line: '
                             '"<md5>" or "<md5>:<dir_name>" (Mode 2)')
    parser.add_argument('--output-dir', type=str, default='.',
                        help='Root directory for downloaded bags and reports (Mode 2)')

    args = parser.parse_args()

    # ── Mode 2 ────────────────────────────────────────────────────────
    md5_dir_pairs = []   # list of (md5, dir_name_or_None)

    if args.md5:
        dir_names = args.dir_name or []
        if dir_names and len(dir_names) != len(args.md5):
            parser.error(f'--dir-name count ({len(dir_names)}) must match --md5 count ({len(args.md5)})')
        for i, md5 in enumerate(args.md5):
            md5_dir_pairs.append((md5, dir_names[i] if i < len(dir_names) else None))

    if args.md5_file:
        with open(args.md5_file, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                if ':' in line:
                    md5, dir_name = line.split(':', 1)
                    md5_dir_pairs.append((md5.strip(), dir_name.strip()))
                else:
                    md5_dir_pairs.append((line, None))

    if md5_dir_pairs:
        fetch_and_report(md5_dir_pairs, os.path.abspath(args.output_dir), args.config)
        return

    # ── Mode 1 ────────────────────────────────────────────────────────
    if not args.bag_path:
        parser.error('Provide one or more bag paths (Mode 1) or --md5/--md5-file (Mode 2)')

    for bag_path in args.bag_path:
        print(f"\n{'='*60}")
        try:
            ReportGenerator(bag_path, args.config).run()
        except Exception as e:
            print(f"  ERROR: {e}")


if __name__ == '__main__':
    main()
