#!/usr/bin/env python3
# encoding: utf-8
"""Koopman 三方分析 · 单文件工具链

Z(工况) × M(模型评分) × C(控制误差) 的相关性与传导分析。
逻辑层 + matplotlib 静态报告 + frontend-design 风格前端页面，全部在此一个文件。

用法（在 koopman_analysis/ 下）:
  python3 koopman_analysis.py                              # 长图 -> output/koopman_report.png
  python3 koopman_analysis.py --out x.pdf                  # 多页 PDF
  python3 koopman_analysis.py --frontend [dir]             # 静态前端(index.html + 分页PNG)
  python3 koopman_analysis.py --preflight <bag/目录>       # 只做字段准入检查(抽样)

analysis_list.txt 每行一个条目（# 注释；路径相对 list 所在目录及其上级自动解析）:
  场景目录 bags/score/score_1 / 父目录 bags/score / bag 文件 / @其他.txt

背景: bokeh 2.4 大文档有间歇性 JS "unreachable code" 空白问题, 故全静态渲染。
"""
import argparse
import glob
import json
import os
import sys
import warnings

import numpy as np
import pandas as pd

import matplotlib
matplotlib.use("Agg")
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

warnings.filterwarnings("ignore")

# ---------------- 中文字体（方块修复：注册系统 Noto，优先 Sans SC） ----------------
try:
    for f in ["/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
              "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"]:
        if os.path.exists(f):
            fm.fontManager.addfont(f)
    plt.rcParams["font.family"] = "sans-serif"
    plt.rcParams["font.sans-serif"] = ["Noto Sans SC", "Noto Sans CJK SC",
                                       "Noto Sans CJK JP", "WenQuanYi Zen Hei",
                                       "SimHei", "Microsoft YaHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
except Exception:
    pass

plt.rcParams.update({
    "font.size": 10, "axes.titlesize": 12, "axes.labelsize": 10,
    "xtick.labelsize": 9, "ytick.labelsize": 9, "legend.fontsize": 8,
    "figure.dpi": 110, "axes.grid": True, "grid.alpha": 0.25,
})

# ---------------- 配置与字段清单 ----------------
DEFAULT_CONF = {
    "topic": "/msd/endpoint/control_command",
    "gap_sec": 1.0,             # 时间断点切 segment 阈值
    "min_segment_sec": 2.0,
    "active_field": "koopman_vel_u",   # 本帧 Koopman 在算的标志（停车段 health 冻结假值）
    "c_window_sec": 0.5,
    "maneuver_window_sec": 1.0,
    "maneuver_r_bins": [0.02, 0.08],
    "speed_bins": 3,
    "nbins": 8,
    "event_quantile": 0.90,
    "event_min_gap_sec": 2.5,
    "event_window_sec": 2.5,
    "xcorr_max_lag_sec": 3.0,
    "n_boot": 200,
    "sample_plot": 2000,
}

Z_FIELDS = {
    "vel": "koopman_vel_u", "vel_v": "vel_by", "yawrate": "mpc_init_dyawrate",
    "steer": "koopman_wheel_angle_meas",
    "w_fl": "w_fl_mps_slip", "w_fr": "w_fr_mps_slip",
    "w_rl": "w_rl_mps_slip", "w_rr": "w_rr_mps_slip",
    "y0": "mpc_init_dy", "phi0": "mpc_init_dphi",
}
M_HEALTH = [
    "koopman_health_score_total", "koopman_health_score_valid",
    "koopman_health_plan_accuracy_score", "koopman_health_plan_stability_score",
    "koopman_health_ood_score", "koopman_health_executed_error_p99_score",
    "koopman_health_executed_jump_p99_score",
    "koopman_health_plan_accuracy_y_score", "koopman_health_plan_accuracy_phi_score",
    "koopman_health_plan_accuracy_vy_score", "koopman_health_plan_accuracy_r_score",
    "koopman_health_ood_level", "koopman_health_full_ood",
    "koopman_health_executed_error_p99_hit", "koopman_health_executed_jump_p99_hit",
    "koopman_gain_raw", "koopman_gain_lp", "koopman_gain",
    "koopman_step_mpc_ms", "koopman_Z0_norm", "koopman_Z0_x0_err",
]
M_GAIN = "koopman_gain_lp"
C_FIELDS = {"lat_err": "lat_err", "phi_err": "phi_err", "overshoot": "lat_overshoot"}

SOURCES = ["plan", "executed"]
STATES = ["y", "phi", "vy", "r"]
STEPS = [1, 10, 25]


def acc_key(src, st, step, q):
    return f"koopman_acc_{src}_{st}_{step}step_{q}"


M_ACC = {(s, t, k, q): acc_key(s, t, k, q)
         for s in SOURCES for t in STATES for k in STEPS for q in ("error", "rmse")}
ALL_FIELDS = (list(Z_FIELDS.values()) + M_HEALTH + list(C_FIELDS.values())
              + list(M_ACC.values()))

# G4 双权重仲裁用（冻结参考表拷自 koopman_accuracy_monitor.cpp, kSteps 行序）
REF_P95_ERR = np.array([
    [0.0033747882, 0.0009718543, 0.0110845529, 0.0060129662],
    [0.0123161383, 0.0032405276, 0.0122240517, 0.0084285939],
    [0.0294653046, 0.0070892066, 0.0143522122, 0.0117081791],
    [0.0660418004, 0.0128958143, 0.0170288965, 0.0163783292],
    [0.1327665389, 0.0211895593, 0.0199478514, 0.0222809367],
    [0.2445160232, 0.0315247715, 0.0229181888, 0.0288989268],
])
STEP_ROW = {1: 0, 5: 1, 10: 2, 15: 3, 20: 4, 25: 5}
STATE_W = np.array([10.0 / 42.0, 10.0 / 42.0, 2.0 / 42.0, 20.0 / 42.0])
HW_CODE = np.array([0.4, 0.3, 0.3])   # 当前代码权重
HW_PAGE = np.array([0.2, 0.3, 0.5])   # 设计页 v2 权重

COLORS = ["#246bfe", "#059669", "#dc2626", "#7c3aed", "#ea580c"]


def score_kernel(q, ref):
    if not np.isfinite(q) or ref <= 0:
        return 0.0
    ratio = q / ref
    if ratio <= 0.5:
        return 1.0
    d = ratio - 0.5
    return 1.0 / (1.0 + d * d)


def synth_accuracy(df, hw):
    """合成 plan_accuracy 分(/50, 近似: q 用 window rmse 代 P95), 仅相对比较"""
    total = pd.Series(0.0, index=df.index)
    for si, s in enumerate(STATES):
        for hi, k in enumerate(STEPS):
            q = df[acc_key("plan", s, k, "rmse")]
            ref = REF_P95_ERR[STEP_ROW[k], si]
            total = total + 50.0 * STATE_W[si] * hw[hi] * q.map(lambda v: score_kernel(v, ref))
    return total


# ---------------- 数据加载 ----------------
def clean_json_string(s):
    return s.replace("\r\n", "").replace("\n", "").replace("\r", "").strip()


def load_bag(bag_path, conf, max_frames=None):
    """bag -> DataFrame(全部所需字段 + t)。max_frames: 只读前 N 帧(准入快速检查)"""
    rec = {"t": []}
    for f in ALL_FIELDS:
        rec[f] = []
    n_msg, n_fail = 0, 0
    with __import__("rosbag").Bag(bag_path) as bag:
        for topic, msg, t in bag.read_messages(topics=[conf["topic"]]):
            n_msg += 1
            if max_frames and n_msg > max_frames:
                n_msg -= 1
                break
            try:
                m = getattr(msg, "obj", msg)
                stamp = getattr(m.header, "stamp", None)
                if hasattr(stamp, "secs"):
                    ts = stamp.secs + stamp.nsecs / 1e9
                else:
                    ts = stamp / 1e9 if stamp else t.to_sec()
                js = json.loads(clean_json_string(m.extra.json))
            except Exception:
                n_fail += 1
                continue
            rec["t"].append(ts)
            for f in ALL_FIELDS:
                v = js.get(f, np.nan)
                try:
                    v = float(v)
                except (TypeError, ValueError):
                    v = np.nan
                rec[f].append(v)
    return pd.DataFrame(rec), {"n_msg": n_msg, "n_fail": n_fail, "n_frame": len(rec["t"])}


def preflight(df, meta):
    """字段覆盖率(有效帧内)。返回 cov 字典"""
    active = df[DEFAULT_CONF["active_field"]].notna().values
    n_active = int(active.sum())
    cov = {f: (df[f].notna().values & active).sum() / n_active if n_active else 0.0
           for f in ALL_FIELDS}
    cov.update({"_n_active": n_active, "_n_frame": meta["n_frame"], "_json_fail": meta["n_fail"]})
    return cov


# ---------------- 清洗与特征 ----------------
def clean_and_segment(df, conf):
    """有效帧筛选 + segment 切分 + 派生量"""
    active = df[conf["active_field"]].notna() & np.isfinite(df[conf["active_field"]])
    df = df[active].copy()
    if "koopman_history_warmup" in df:
        df = df[(df["koopman_history_warmup"].fillna(0) > 0) | df["koopman_history_warmup"].isna()]
    df = df.sort_values("t").reset_index(drop=True)
    if df.empty:
        df["segment"] = []
        return df
    df["segment"] = "S" + (df["t"].diff().fillna(np.inf) > conf["gap_sec"]).cumsum().astype(str)
    seg_len = df.groupby("segment")["t"].agg(lambda s: s.iloc[-1] - s.iloc[0])
    df = df[df["segment"].isin(seg_len[seg_len >= conf["min_segment_sec"]].index)].reset_index(drop=True)
    if df.empty:
        return df
    fs = 1.0 / float(np.median(np.diff(df["t"].values)))
    wc = max(int(conf["c_window_sec"] * fs), 2)
    wm = max(int(conf["maneuver_window_sec"] * fs), 2)
    # Z 派生
    df["abs_r"] = df["mpc_init_dyawrate"].abs()
    df["abs_steer"] = df["koopman_wheel_angle_meas"].abs()
    df["a_y"] = df["koopman_vel_u"] * df["mpc_init_dyawrate"]
    # 机动类型（segment 内滑窗）
    df["r_mean"] = df.groupby("segment")["abs_r"].transform(
        lambda s: s.rolling(wm, min_periods=1).mean())
    b1, b2 = conf["maneuver_r_bins"]
    df["maneuver"] = np.select([df["r_mean"] < b1, df["r_mean"] < b2],
                               ["straight", "gentle"], default="sharp")
    # C 派生（segment 内滑窗, 跨 gap 自动切断）
    for cname in ("lat_err", "phi_err"):
        g = df.groupby("segment")[cname]
        df[f"{cname}_rms"] = g.transform(lambda s: s.rolling(wc, min_periods=wc // 2)
                                         .apply(lambda x: np.sqrt(np.mean(x ** 2)), raw=True))
        df[f"{cname}_p2p"] = g.transform(lambda s: s.rolling(wc, min_periods=wc // 2)
                                         .apply(lambda x: x.max() - x.min(), raw=True))
        df[f"{cname}_hfe"] = g.transform(lambda s: s.diff().rolling(wc, min_periods=wc // 2)
                                         .apply(lambda x: np.sqrt(np.mean(x ** 2)), raw=True))
    try:
        df["speed_bin"] = pd.qcut(df["koopman_vel_u"], conf["speed_bins"], duplicates="drop").astype(str)
    except Exception:
        df["speed_bin"] = "all"
    return df


# ---------------- 统计工具 ----------------
def spearman(x, y):
    x, y = np.asarray(x, float), np.asarray(y, float)
    m = np.isfinite(x) & np.isfinite(y)
    if m.sum() < 10:
        return np.nan
    rx, ry = pd.Series(x[m]).rank(), pd.Series(y[m]).rank()
    return float(np.corrcoef(rx, ry)[0, 1])


def boot_ci_spearman(x, y, n_boot=200):
    """时间连续块 bootstrap 的 Spearman CI（~2s 块, 保留时间自相关, 不依赖 segment 数）"""
    x = np.asarray(x, float); y = np.asarray(y, float)
    m = np.isfinite(x) & np.isfinite(y)
    x, y = x[m], y[m]
    n = len(x)
    if n < 60:
        return (np.nan, np.nan)
    block = max(100, n // 20)
    nblocks = int(np.ceil(n / block))
    blocks = [np.arange(i * block, min((i + 1) * block, n)) for i in range(nblocks)]
    rng = np.random.default_rng(0)
    rs = []
    for _ in range(n_boot):
        idx = np.concatenate([blocks[b] for b in rng.choice(nblocks, size=nblocks, replace=True)])
        if len(idx) >= 30:
            r = spearman(x[idx], y[idx])
            if np.isfinite(r):
                rs.append(r)
    if len(rs) < 50:
        return (np.nan, np.nan)
    return (float(np.percentile(rs, 2.5)), float(np.percentile(rs, 97.5)))


def quantile_binned_stat(df, xcol, ycol, nbins):
    """按 x 分位分箱, 返回 (bin_center, p50, p95, n)"""
    d = df[[xcol, ycol]].dropna()
    if len(d) < nbins * 5:
        return None
    try:
        d["bin"] = pd.qcut(d[xcol], nbins, duplicates="drop")
    except Exception:
        return None
    g = d.groupby("bin")
    return (g[xcol].median().values, g[ycol].median().values,
            g[ycol].quantile(0.95).values, g.size().values)


def heatmap_2d(df, xcol, ycol, zcol, nx=12, ny=12):
    """(x,y) 分位网格统计 z 中位数。返回 (x_center, y_center, grid_z, grid_n)"""
    d = df[[xcol, ycol, zcol]].dropna()
    if len(d) < nx * ny:
        return None
    try:
        xq = pd.qcut(d[xcol], nx, labels=False, duplicates="drop")
        yq = pd.qcut(d[ycol], ny, labels=False, duplicates="drop")
    except Exception:
        return None
    d = d.assign(xb=xq.values, yb=yq.values)
    g = d.groupby(["yb", "xb"])[zcol]
    z, n = g.median(), g.size()
    nx_eff = int(pd.Series(xq.unique()).max()) + 1
    ny_eff = int(pd.Series(yq.unique()).max()) + 1
    xc = d.groupby("xb")[xcol].median().reindex(range(nx_eff)).values
    yc = d.groupby("yb")[ycol].median().reindex(range(ny_eff)).values
    grid = np.full((ny_eff, nx_eff), np.nan)
    gcnt = np.zeros((ny_eff, nx_eff))
    for (iy, ix), v in z.items():
        grid[iy, ix] = v
        gcnt[iy, ix] = n.loc[(iy, ix)]
    return xc, yc, grid, gcnt


def xcorr_lag_curve(df, mcol, ccol, max_lag_sec, fs_hint=50.0):
    """时滞互相关: M(t) 与 C(t+lag)。返回 (lags_sec, corr)"""
    d = df[["t", mcol, ccol]].dropna()
    if len(d) < 100:
        return None
    t = d["t"].values
    m = d[mcol].values - np.nanmean(d[mcol].values)
    c = d[ccol].values - np.nanmean(d[ccol].values)
    denom = np.nanstd(m) * np.nanstd(c)
    if denom == 0:
        return None
    max_lag = int(max_lag_sec * fs_hint)
    lags, corrs = [], []
    for lag in range(-max_lag, max_lag + 1, 2):
        if lag >= 0:
            a, b = (m[:len(m) - lag] if lag else m), c[lag:]
        else:
            a, b = m[-lag:], c[:len(c) + lag]
        if len(a) < 50:
            continue
        corrs.append(float(np.mean(np.nan_to_num(a) * np.nan_to_num(b)) / denom))
        lags.append(lag / fs_hint)
    return np.array(lags), np.array(corrs)


def nonoverlap_events(df, col, q_thr, min_gap_sec):
    """列超分位的非重叠事件段(相邻间隔<min_gap 合并), 返回 [(t_start,t_end,peak_idx)]"""
    d = df[["t", col]].dropna()
    if len(d) < 50:
        return []
    thr = d[col].quantile(q_thr)
    hot = d[d[col] >= thr]
    if hot.empty:
        return []
    events = []
    cur_start, cur_end, cur_peak = hot["t"].iloc[0], hot["t"].iloc[-1], hot.index[0]
    for i, (idx, row) in enumerate(hot.iterrows()):
        if i == 0:
            continue
        if row["t"] - cur_end > min_gap_sec:
            events.append((cur_start, cur_end, cur_peak))
            cur_start, cur_peak = row["t"], idx
        cur_end = row["t"]
        if d.loc[idx, col] >= d.loc[cur_peak, col]:
            cur_peak = idx
    events.append((cur_start, cur_end, cur_peak))
    return events


def window_stat_after(df, t_start, col, win_sec):
    d = df[(df["t"] >= t_start) & (df["t"] <= t_start + win_sec)][col].dropna()
    return float(np.sqrt(np.mean(d ** 2))) if len(d) > 5 else np.nan


def downsample(x, y, n):
    if len(x) <= n:
        return x, y
    step = max(len(x) // n, 1)
    return x[::step], y[::step]


# ---------------- 准入检查（--preflight） ----------------
def preflight_report(bag_path, conf, max_frames=3000):
    """详细字段准入报告(终端输出)"""
    print("=" * 78)
    print(f"BAG: {bag_path}")
    df, meta = load_bag(bag_path, conf, max_frames=max_frames)
    cov = preflight(df, meta)
    n = meta["n_frame"]
    print(f"  帧数={n} json失败={meta['n_fail']} 有效帧={cov['_n_active']}"
          f"  ({'抽样' if max_frames and n >= max_frames else '全量'})")
    for name, fields in [("Z 工况", list(Z_FIELDS.values())), ("M 评分", M_HEALTH),
                         ("C 控制误差", list(C_FIELDS.values()))]:
        print(f"\n  [{name}]")
        missing = []
        for f in fields:
            c = cov.get(f, 0.0)
            if c <= 0:
                missing.append(f); continue
            col = df[f].dropna()
            vals = ", ".join(f"{v:.4g}" for v in col.head(3))
            print(f"    {'OK ' if c > 0.95 else 'PART'} {f:52s} 覆盖率 {c:6.1%}  样本[{vals}]")
        if missing:
            print(f"    MISS 缺失: {', '.join(missing)}")
    want = set(M_ACC.values())
    have = {f for f in want if cov.get(f, 0.0) > 0}
    print(f"\n  [koopman_acc_* 族] 存在 {len(have)}/{len(want)}"
          f"  覆盖率>95%: {sum(1 for f in want if cov.get(f, 0) > 0.95)}/{len(want)}")


# ---------------- matplotlib 报告层 ----------------
def _legend_out(ax, fontsize=8):
    """图例统一放图外右侧, 避免与数据/文字重叠"""
    ax.legend(fontsize=fontsize, loc="upper left", bbox_to_anchor=(1.02, 1.0),
              frameon=True, borderaxespad=0.0)


def _q(series, q):
    return float(series.quantile(q)) if len(series) else np.nan


def _binned_curve(ax, df, xcol, ycol, color, label):
    res = quantile_binned_stat(df, xcol, ycol, 8)
    if res is None:
        return
    cx, p50, p95, n = res
    ax.plot(cx, p50, color=color, marker="o", ms=4, lw=1.6, label=f"{label} P50")
    ax.plot(cx, p95, color=color, ls="--", lw=1.0, alpha=0.7)


def _table_wrap(s, per_cell_chars):
    """按估算宽度把长文本换行(不截断)。返回含 \n 的字符串。"""
    s = str(s)
    if len(s) <= per_cell_chars:
        return s
    out, cur = [], []
    n = 0
    for ch in s:
        cur.append(ch)
        n += 2 if ord(ch) > 127 else 1   # 中文字符宽 2
        if n >= per_cell_chars:
            out.append("".join(cur)); cur, n = [], 0
    if cur:
        out.append("".join(cur))
    return "\n".join(out)


def _table_text_width(s):
    """文本显示宽度(中文字符算 2)"""
    return sum(2 if ord(ch) > 127 else 1 for ch in str(s))


def _table_page(fig, df, title):
    """独占整行的表格页。列宽/行高按内容自适应: 长文本列宽、短列窄, 行高按换行行数。"""
    ax = fig.add_subplot(111)
    ax.axis("off")
    n, m = df.shape
    fs = min(10, max(7, int(10 - max(0, n - 4) * 0.5)))
    # 列宽分配: 按每列最大显示宽度的平方根比例(长列宽, 短列窄), 总宽归一
    cell_raw = df.copy()
    for c in range(m):
        if cell_raw.dtypes.iloc[c] == "object":
            continue                       # 字符串列(如 bag 名)原样保留, 不做 round
        cell_raw.iloc[:, c] = pd.to_numeric(cell_raw.iloc[:, c], errors="coerce").round(3)
    cell_raw = cell_raw.astype(object).where(pd.notnull(df), "")
    col_max = []
    for c in range(m):
        wmax = _table_text_width(df.columns[c])
        for r in range(n):
            wmax = max(wmax, _table_text_width(cell_raw.iat[r, c]))
        col_max.append(wmax)
    total_w = sum(col_max)
    col_frac = [max(0.03, w / total_w) for w in col_max]
    col_frac = [f / sum(col_frac) for f in col_frac]
    # 每列可容纳字符数(按列宽与总宽 13.4in 折算)
    per_col_chars = [max(8, int(13.4 * 8 * frac)) for frac in col_frac]
    cell_text = [[_table_wrap(v, per_col_chars[c]) for c, v in enumerate(row)]
                 for row in cell_raw.values]
    col_labels = [_table_wrap(df.columns[c], per_col_chars[c]) for c in range(m)]
    cell = ax.table(cellText=cell_text, colLabels=col_labels,
                    loc="center", cellLoc="center")
    cell.auto_set_font_size(False)
    cell.set_fontsize(fs)
    cell.scale(1.0, 1.5)
    for (r, c), tcell in cell.get_celld().items():
        tcell.set_width(col_frac[c])
        tcell.set_facecolor("#f7faff" if r > 0 else "#e8f0fc")
        tcell.set_edgecolor("#c8d6e8")
        tcell.get_text().set_color("#10213d")
        if r == 0:
            tcell.get_text().set_weight("bold")
    ax.set_title(title, fontsize=13, pad=14)


def _time_series(ax, bags_dfs, col, ylab, title):
    for (name, df, cov), color in zip(bags_dfs, COLORS):
        if len(df):
            ax.plot(df["t"] - df["t"].iloc[0], df[col], color=color,
                    label=os.path.basename(name)[:12], lw=1.2)
    ax.set_title(title); ax.set_xlabel("t/s"); ax.set_ylabel(ylab)
    _legend_out(ax)


def _table_size(df):
    """表格页 fig 高度按内容自适应: 行数×平均行高(含换行余量)"""
    n, m = df.shape
    fs = min(10, max(7, int(10 - max(0, n - 4) * 0.5)))
    # 估算最大行高: 列宽按最大文本宽均分, 每行换行数=最大列文本宽/该列宽
    cell = df.copy()
    for c in range(m):
        if cell.dtypes.iloc[c] == "object":
            continue
        cell.iloc[:, c] = pd.to_numeric(cell.iloc[:, c], errors="coerce").round(3)
    cell = cell.astype(object).where(pd.notnull(df), "")
    col_max = [_table_text_width(df.columns[c]) for c in range(m)]
    for r in range(n):
        for c in range(m):
            col_max[c] = max(col_max[c], _table_text_width(cell.iat[r, c]))
    total_w = sum(col_max)
    row_h = 0.0
    for r in range(n):
        max_lines = 1
        for c in range(m):
            col_frac = max(0.03, col_max[c] / total_w)
            per_chars = max(8, int(13.4 * 8 * col_frac))
            lines = max(1, -(-_table_text_width(cell.iat[r, c]) // per_chars))
            max_lines = max(max_lines, lines)
        row_h += 0.30 * max_lines
    return (13.4, max(2.2, row_h + 1.6))


# ---- 页函数 ----
def ov_table(fig, bags_dfs):
    rows = [{"bag": os.path.basename(n), "frames": c["_n_frame"], "valid": c["_n_active"],
             "health_p05": _q(d["koopman_health_score_total"], 0.05) if len(d) else np.nan,
             "laterr_rms": float(np.sqrt((d["lat_err"] ** 2).mean())) if len(d) else np.nan,
             "p99hit": int(d["koopman_health_executed_error_p99_hit"].sum()) if len(d) else 0}
            for n, d, c in bags_dfs]
    _table_page(fig, pd.DataFrame(rows), "Overview: 数据包汇总")


def ov_timeseries(fig, bags_dfs):
    axs = fig.subplots(1, 3)
    _time_series(axs[0], bags_dfs, "koopman_health_score_total", "总分", "health_score_total")
    _time_series(axs[1], bags_dfs, "lat_err", "lat_err (m)", "lat_err")
    _time_series(axs[2], bags_dfs, "koopman_gain_lp", "gain_lp", "gain_lp")


def _g1_state_page(state):
    def fn(fig, bags_dfs):
        axs = fig.subplots(1, 3)
        zvars = [("koopman_vel_u", "vel_u"), ("abs_r", "|r|"), ("abs_steer", "|steer|")]
        ycol = acc_key("plan", state, 10, "rmse")
        for ax, (zcol, zlab) in zip(axs, zvars):
            for (name, df, cov), color in zip(bags_dfs, COLORS):
                if df.empty:
                    continue
                _binned_curve(ax, df, zcol, ycol, color, os.path.basename(name)[:10])
            ax.set_title(f"plan_{state}_10step rmse vs {zlab}")
            ax.set_xlabel(zlab); ax.set_ylabel("rmse"); _legend_out(ax)
        fig.suptitle(f"G1 模型薄弱工况: 状态 {state} · 实线 P50 / 虚线 P95", y=0.97, fontsize=13)
    return fn


def g1c_heatmap(fig, bags_dfs):
    alldf = pd.concat([df for _, df, _ in bags_dfs if len(df)], ignore_index=True)
    if len(alldf) < 200:
        return
    axs = fig.subplots(1, 2)
    for ax, (zcol, ttl, cmap) in zip(
            axs, [(acc_key("plan", "r", 10, "rmse"), "G1c plan_r_10step rmse 中位数", "RdYlBu_r"),
                  ("koopman_health_ood_level", "G1c 同网格 ood_level", "viridis")]):
        res = heatmap_2d(alldf, "koopman_vel_u", "abs_r", zcol)
        if res is None:
            continue
        xc, yc, grid, gcnt = res
        im = ax.imshow(grid, aspect="auto", origin="lower", cmap=cmap)
        ax.set_xticks(range(len(xc))); ax.set_xticklabels([f"{v:.1f}" for v in xc], fontsize=8)
        ax.set_yticks(range(len(yc))); ax.set_yticklabels([f"{v:.3f}" for v in yc], fontsize=8)
        ax.set_xlabel("vx 分位格"); ax.set_ylabel("|r| 分位格")
        ax.set_title(ttl, fontsize=12)
        fig.colorbar(im, ax=ax, fraction=0.05)


def _g2_metric_page(cname):
    def fn(fig, bags_dfs):
        axs = fig.subplots(1, 2)
        zvars = [("koopman_vel_u", "vel_u"), ("abs_r", "|r|")]
        for ax, (zcol, zlab) in zip(axs, zvars):
            for (name, df, cov), color in zip(bags_dfs, COLORS):
                if df.empty:
                    continue
                _binned_curve(ax, df, zcol, cname, color, os.path.basename(name)[:10])
            ax.set_title(f"{cname} vs {zlab}")
            ax.set_xlabel(zlab); ax.set_ylabel(cname); _legend_out(ax)
        fig.suptitle(f"G2 场景难度基线: {cname} · 实线 P50 / 虚线 P95", y=0.97, fontsize=13)
    return fn


def g3_transfer(fig, bags_dfs):
    mcol = acc_key("plan", "r", 10, "rmse")
    ccol = "lat_err_rms"
    alldf = pd.concat([df for _, df, _ in bags_dfs if len(df)], ignore_index=True)
    axs = fig.subplots(2, 2)

    ax = axs[0, 0]
    for (name, df, cov), color in zip(bags_dfs, COLORS):
        if df.empty:
            continue
        x, y = downsample(df[mcol].values, df[ccol].values, 1500)
        ax.scatter(x, y, s=6, alpha=0.45, color=color, label=os.path.basename(name)[:10])
    ax.set_title("G3d M-C 散点: plan_r_10step rmse × lat_err rms")
    ax.set_xlabel("plan_r_10step rmse"); ax.set_ylabel("lat_err rms (m)"); _legend_out(ax)

    ax = axs[0, 1]
    res = xcorr_lag_curve(alldf, mcol, ccol, 3.0)
    if res:
        ax.plot(res[0], res[1], color=COLORS[0], lw=1.6)
        ax.axvline(0, ls=":", color="grey")
    ax.set_title("G3b 时滞互相关 (正 lag = M 领先)")
    ax.set_xlabel("lag (s)"); ax.set_ylabel("corr")

    ax = axs[1, 0]
    ld = []
    for name, df, cov in bags_dfs:
        if df.empty:
            continue
        for (man, spd), d in df.groupby(["maneuver", "speed_bin"]):
            if len(d) < 60:
                continue
            r = spearman(d[mcol], d[ccol])
            if np.isfinite(r):
                ld.append({"maneuver": man, "rho": r})
    if ld:
        ldf = pd.DataFrame(ld)
        for man, color in [("straight", COLORS[0]), ("gentle", COLORS[1]), ("sharp", COLORS[2])]:
            d = ldf[ldf["maneuver"] == man]
            if len(d):
                ax.scatter([man] * len(d), d["rho"], s=40, color=color, alpha=0.75,
                           label=f"{man}(n={len(d)})")
        ax.axhline(0, color="grey", ls=":")
        ax.set_title("G3a 分层 Spearman (按机动类型)")
        ax.set_ylabel("rho"); _legend_out(ax)

    ax = axs[1, 1]
    ev_rows = []
    for name, df, cov in bags_dfs:
        if df.empty:
            continue
        for q, tag in [(0.90, "high"), (0.10, "low")]:
            for (t0, t1, pk) in nonoverlap_events(df, mcol, q, 2.5):
                c = window_stat_after(df, t1, "lat_err_rms", 2.5)
                ev_rows.append({"grp": tag, "c": c})
    if ev_rows:
        ev = pd.DataFrame(ev_rows).dropna()
        for g, color in [("high", COLORS[2]), ("low", COLORS[1])]:
            d = ev[ev["grp"] == g]["c"]
            if len(d):
                ax.scatter([g] * len(d), d, s=30, color=color, alpha=0.75, label=f"{g}(n={len(d)})")
        ax.set_title("G3c 非重叠事件对照: M 高/低段后 2.5s lat_err rms")
        ax.set_ylabel("lat_err rms after 2.5s"); _legend_out(ax)
    fig.suptitle("G3 传导关联 (M 为事后已兑现误差, 关联非因果)", y=0.97, fontsize=13)


def g4_score(fig, bags_dfs):
    alldf = pd.concat([df for _, df, _ in bags_dfs if len(df)], ignore_index=True)
    axs = fig.subplots(2, 2)

    ax = axs[0, 0]
    for (name, df, cov), color in zip(bags_dfs, COLORS):
        if df.empty:
            continue
        mat = np.array([[spearman(df[acc_key("plan", s, k, "rmse")], df["lat_err_rms"])
                         for k in [1, 10, 25]] for s in ["y", "phi", "vy", "r"]])
        ax.plot([1, 10, 25], np.nanmean(mat, axis=0), color=color, lw=2,
                label=f"{os.path.basename(name)[:10]} mean")
    ax.axhline(0, color="grey", ls=":")
    ax.set_title("G4a horizon 相关: plan rmse × lat_err rms")
    ax.set_xlabel("horizon step"); ax.set_ylabel("rho"); _legend_out(ax)

    ax = axs[0, 1]
    comp = [("acc_y", "koopman_health_plan_accuracy_y_score"),
            ("acc_phi", "koopman_health_plan_accuracy_phi_score"),
            ("acc_vy", "koopman_health_plan_accuracy_vy_score"),
            ("acc_r", "koopman_health_plan_accuracy_r_score"),
            ("plan_stab", "koopman_health_plan_stability_score"), ("gain_lp", M_GAIN)]
    xs, ys = [], []
    rng = np.random.default_rng(1)
    for (name, df, cov) in bags_dfs:
        if df.empty:
            continue
        for i, (lab, col) in enumerate(comp):
            r = spearman(df[col], df["lat_err_rms"])
            if np.isfinite(r):
                xs.append(i + rng.uniform(-0.18, 0.18)); ys.append(r)
    ax.scatter(xs, ys, s=40, color=COLORS[0], alpha=0.75)
    ax.set_xticks(range(len(comp))); ax.set_xticklabels([c[0] for c in comp], fontsize=8)
    ax.axhline(0, color="grey", ls=":")
    ax.set_title("G4b 评分分量 × lat_err rms 相关 (负=分高误差小)")

    ax = axs[1, 0]
    if len(alldf) > 200:
        dd = pd.DataFrame({"a_c": synth_accuracy(alldf, HW_CODE),
                           "a_p": synth_accuracy(alldf, HW_PAGE),
                           "c": alldf["lat_err_rms"]}).dropna()
        ax.scatter(dd["a_c"], dd["c"], s=4, alpha=0.3, color=COLORS[0],
                   label=f"代码版 rho={spearman(dd['a_c'], dd['c']):+.2f}")
        ax.scatter(dd["a_p"], dd["c"], s=4, alpha=0.3, color=COLORS[2],
                   label=f"页面版 rho={spearman(dd['a_p'], dd['c']):+.2f}")
        ax.set_title("G4d horizon 权重仲裁 (近似, 仅相对比较)")
        ax.set_xlabel("合成 accuracy (/50)"); ax.set_ylabel("lat_err rms"); _legend_out(ax)

    ax = axs[1, 1]
    res = xcorr_lag_curve(alldf, M_GAIN, "lat_err_rms", 3.0)
    if res:
        ax.plot(res[0], res[1], color=COLORS[3], lw=1.6)
        ax.axvline(0, ls=":", color="grey")
        ax.set_title("G4c gain_lp × lat_err rms 时滞互相关")
        ax.set_xlabel("lag (s)"); ax.set_ylabel("corr")
    fig.suptitle("G4 评分体系校验", y=0.97, fontsize=13)


def g5_cov(fig, bags_dfs):
    rows = [{"bag": os.path.basename(n), "frames": c["_n_frame"], "valid": c["_n_active"],
             "json_fail": c["_json_fail"], "vel_u_cov": c["koopman_vel_u"],
             "health_cov": c["koopman_health_score_total"], "lat_err_cov": c["lat_err"]}
            for n, _, c in bags_dfs]
    _table_page(fig, pd.DataFrame(rows), "G5 数据质量: 字段覆盖率")


def g5_segments(fig, bags_dfs):
    rows = []
    for name, df, cov in bags_dfs:
        if df.empty:
            continue
        t0 = df["t"].iloc[0]
        for seg, d in df.groupby("segment"):
            rows.append({"bag": os.path.basename(name), "seg": seg,
                         "t0": round(float(d["t"].iloc[0] - t0), 1),
                         "dur": round(float(d["t"].iloc[-1] - d["t"].iloc[0]), 1),
                         "n": len(d), "vel": round(float(d["koopman_vel_u"].mean()), 1),
                         "health": round(float(d["koopman_health_score_total"].mean()), 0),
                         "laterr_rms": round(float(np.sqrt((d["lat_err"] ** 2).mean())), 4)})
    _table_page(fig, pd.DataFrame(rows), "G5 segment 明细")




def _finalize_fig(fig):
    """页收尾: 有 suptitle 则顶部留 6% 给标题, 避免与子图 title 重叠"""
    if fig._suptitle is not None:
        fig.tight_layout(rect=[0, 0, 1, 0.94])
    else:
        fig.tight_layout()

def _pages_spec(bags_dfs):
    """页面定义。统一 figsize: 单行图表页 (13.4, 3.9), 双行 2x2 页 (13.4, 8.4),
    表格页单独(独占整行, 宽度 13.4)。"""
    W_1ROW, H_1ROW = 13.4, 3.9      # 单行图表页（G1/G2/Overview-时序/G1c）
    W_2ROW, H_2ROW = 13.4, 8.4      # 双行 2x2 页（G3/G4）
    W_TBL = 13.4                    # 表格页宽度
    pages = [
        ("Overview-表格", _table_size(
            pd.DataFrame([{"bag": os.path.basename(n)} for n, _, _ in bags_dfs])), ov_table),
        ("Overview-时序", (W_1ROW, H_1ROW), ov_timeseries),
    ]
    for st in ["y", "phi", "vy", "r"]:
        pages.append((f"G1-{st}", (W_1ROW, H_1ROW), _g1_state_page(st)))
    pages.append(("G1c-Heatmap", (W_1ROW, H_1ROW + 1.0), g1c_heatmap))
    for cname in ["lat_err_rms", "lat_err_p2p", "lat_err_hfe", "phi_err_rms"]:
        pages.append((f"G2-{cname}", (W_1ROW, H_1ROW), _g2_metric_page(cname)))
    pages.append(("G3-Transfer", (W_2ROW, H_2ROW), g3_transfer))
    pages.append(("G4-Score", (W_2ROW, H_2ROW), g4_score))
    cov_df = pd.DataFrame([{"bag": os.path.basename(n)} for n, _, _ in bags_dfs])
    pages.append(("G5-覆盖率", _table_size(cov_df), g5_cov))
    n_seg = sum(df["segment"].nunique() if len(df) else 0 for _, df, _ in bags_dfs)
    pages.append(("G5-segment", (W_TBL, max(2.4, n_seg * 0.55 + 1.4)), g5_segments))
    return pages


def build_report_pages(bags_dfs, out_dir):
    """输出分页 PNG, 返回 [(title, path)]"""
    results = []
    for i, (title, size, fn) in enumerate(_pages_spec(bags_dfs)):
        fig = plt.figure(figsize=size)
        fn(fig, bags_dfs)
        _finalize_fig(fig)
        p = os.path.join(out_dir, f"p{i+1:02d}_{title}.png")
        fig.savefig(p, dpi=110); plt.close(fig)
        results.append((title, p))
    return results


def build_report(bags_dfs, out_path):
    """长图 PNG 或 PDF"""
    pages = _pages_spec(bags_dfs)
    if out_path.endswith(".pdf"):
        with PdfPages(out_path) as pdf:
            for title, size, fn in pages:
                fig = plt.figure(figsize=size)
                fn(fig, bags_dfs); _finalize_fig(fig); pdf.savefig(fig); plt.close(fig)
        print(f"[OK] pdf -> {out_path}")
        return
    from PIL import Image
    pngs = []
    for i, (title, size, fn) in enumerate(pages):
        fig = plt.figure(figsize=size)
        fn(fig, bags_dfs)
        _finalize_fig(fig)
        p = f"{out_path[:-4]}_tmp{i}.png"
        fig.savefig(p, dpi=110); plt.close(fig); pngs.append(p)
    GAP = 100
    imgs = [Image.open(p) for p in pngs]
    w = max(im.width for im in imgs)
    canvas = Image.new("RGB", (w, sum(im.height for im in imgs) + GAP * (len(imgs) - 1)), "white")
    y = 0
    for im in imgs:
        canvas.paste(im, (0, y)); y += im.height + GAP
    canvas.save(out_path)
    for p in pngs:
        os.remove(p)
    print(f"[OK] report -> {out_path} ({len(pages)} 页拼接)")


# ---------------- frontend-design 前端页面 ----------------
_ASCII_NAME = {"Overview-表格": "overview-table", "Overview-时序": "overview-ts",
               "G1-y": "g1-y", "G1-phi": "g1-phi", "G1-vy": "g1-vy", "G1-r": "g1-r",
               "G1c-Heatmap": "g1c-heat", "G2-lat_err_rms": "g2-laterr-rms",
               "G2-lat_err_p2p": "g2-laterr-p2p", "G2-lat_err_hfe": "g2-laterr-hfe",
               "G2-phi_err_rms": "g2-phierr-rms", "G3-Transfer": "g3-transfer",
               "G4-Score": "g4-score", "G5-覆盖率": "g5-cov", "G5-segment": "g5-seg"}

_SECTIONS = [
    ("overview", "总览", "每包的有效帧、健康分、误差与 P99 命中。",
     ["Overview-表格"], "1col"),
    ("overview-ts", "关键时序", "health / lat_err / gain 三条时序。",
     ["Overview-时序"], "1col"),
    ("g1", "模型薄弱工况", "四个物理状态的 10 步预测 rmse 随速度 / 横摆率 / 转角的分位分布（实线 P50，虚线 P95）。",
     ["G1-y", "G1-phi", "G1-vy", "G1-r"], "2col"),
    ("g1c", "覆盖度与 OOD", "同一 (vx, |r|) 分位网格上的误差中位数与训练覆盖度对照——覆盖差的地方误差是否大。",
     ["G1c-Heatmap"], "1col"),
    ("g2", "场景难度基线", "控制误差各维度随工况的分布，与 G1 对照区分「模型差」与「场景难」。",
     ["G2-lat_err_rms", "G2-lat_err_p2p", "G2-lat_err_hfe", "G2-phi_err_rms"], "2col"),
    ("g3", "传导关联", "模型误差到控制误差的关联：散点、时滞互相关、分层 Spearman、非重叠事件对照。",
     ["G3-Transfer"], "1col"),
    ("g4", "评分体系校验", "horizon 相关趋势、分量外部效度、gain 先导性、双权重仲裁。",
     ["G4-Score"], "1col"),
    ("g5", "数据质量", "字段覆盖率准入表。",
     ["G5-覆盖率"], "1col"),
    ("g5-seg", "控制段明细", "有效控制段清单。",
     ["G5-segment"], "1col"),
]

CH_CLASS = {"overview": "ch-z", "overview-ts": "ch-m", "g1": "ch-z",
            "g1c": "ch-m", "g2": "ch-c", "g3": "ch-z",
            "g4": "ch-m", "g5": "ch-c", "g5-seg": "ch-z"}

_CSS = """/* ===== 浅色工程面板 · 三通道主题 ===== */
*{box-sizing:border-box}
html{scroll-behavior:smooth}
body{margin:0;background:#eef3fa;color:#10213d;
  font-family:"Noto Sans SC","PingFang SC","Microsoft YaHei",system-ui,sans-serif;
  -webkit-font-smoothing:antialiased}
.wrap{max-width:1500px;margin:0 auto;padding:0 26px}
/* 通道色 token（Z 蓝 / M 绿 / C 琥珀） */
.ch-z{--ch:#246bfe}.ch-m{--ch:#059669}.ch-c{--ch:#ea580c}
/* ===== Hero ===== */
.hero{background:
  radial-gradient(1200px 520px at 78% -10%, rgba(36,107,254,.14), transparent 60%),
  radial-gradient(900px 420px at -10% 110%, rgba(5,150,105,.10), transparent 60%),
  linear-gradient(150deg,#0b1c33,#10294e 62%,#0d2242);
  color:#e8f0fb;padding:64px 0 52px;position:relative;overflow:hidden}
.eyebrow{display:inline-flex;gap:8px;align-items:center;padding:6px 12px;border:1px solid rgba(255,255,255,.18);
  border-radius:999px;font-size:12px;letter-spacing:.08em;color:#bfd3f4}
.eyebrow i{width:7px;height:7px;border-radius:50%;background:#ea580c}
h1{font-size:clamp(30px,4.4vw,50px);line-height:1.1;letter-spacing:-.03em;margin:18px 0 10px;font-weight:800}
.lead{max-width:640px;color:#9fb4d6;font-size:15px;line-height:1.7}
.meta{display:flex;flex-wrap:wrap;gap:10px;margin-top:18px;font-size:12px;color:#7e95bb}
.meta span{padding:5px 10px;border:1px solid rgba(255,255,255,.14);border-radius:8px}
.stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:14px;margin-top:38px;position:relative}
.stat{background:rgba(255,255,255,.05);border:1px solid rgba(255,255,255,.12);border-radius:14px;padding:16px 18px}
.stat b{display:block;font-size:28px;letter-spacing:-.02em;font-family:"SF Mono",Consolas,monospace;font-weight:700}
.stat b.hl{color:#6ea8ff}.stat b.gn{color:#4ade80}.stat b.amber{color:#ff9f68}
.stat span{display:block;margin-top:4px;color:#8ba2c4;font-size:12px}
.note{position:absolute;right:0;top:-6px;color:#6483b3;font-size:11px;font-family:"SF Mono",monospace}
.tracks{display:flex;height:5px}
.tracks i{flex:1}
.tracks i:nth-child(1){background:#246bfe}
.tracks i:nth-child(2){background:#059669}
.tracks i:nth-child(3){background:#ea580c}
nav{position:sticky;top:0;z-index:20;background:rgba(238,243,250,.92);backdrop-filter:blur(8px);
  border-bottom:1px solid #dbe5f1}
nav .inner{display:flex;gap:4px;overflow-x:auto;padding:10px 0}
nav a{white-space:nowrap;padding:7px 14px;border-radius:9px;color:#29415f;font-size:13px;text-decoration:none;
  font-weight:600;transition:.15s;font-family:"SF Mono",Consolas,monospace}
nav a:hover{background:#e2ebf7;color:#246bfe}
.section{padding:46px 0 10px;scroll-margin-top:64px}
.sec-head{display:flex;align-items:baseline;gap:14px;margin-bottom:6px}
.sec-head .dot{width:9px;height:9px;border-radius:2px;background:var(--ch,#246bfe)}
.sec-head h2{font-size:21px;font-weight:800;letter-spacing:-.01em;margin:0}
.sec-head .tag{font-family:"SF Mono",Consolas,monospace;font-size:12px;color:#5b79a0;font-weight:700}
.sec-desc{color:#5b6d85;font-size:13px;margin:0 0 18px}
.card{background:#fff;border:1px solid #dbe5f1;border-radius:16px;box-shadow:0 10px 30px rgba(31,56,93,.08);
  padding:14px;margin-bottom:18px}
.card img{width:100%;height:auto;display:block;border-radius:10px}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:18px}
@media(max-width:900px){.grid2{grid-template-columns:1fr}}
footer{color:#5b6d85;font-size:12px;text-align:center;padding:34px 0 44px}
@media(prefers-reduced-motion:reduce){html{scroll-behavior:auto}}
a:focus-visible{outline:2px solid #246bfe;outline-offset:2px}"""


# ============================ ECharts 交互版 ============================
# 交互图: ECharts(自包含 echarts.min.js, 无外网依赖, gzip~335KB)。数据序列化进 HTML。
def _jnum(v):
    if hasattr(v, "item"):
        v = v.item()
    return v if isinstance(v, (int, float, str, bool)) or v is None else float(v)


def _ec_base(ttl, xlab, ylab):
    return {"title": {"text": ttl, "left": "center", "textStyle": {"fontSize": 14}},
            "tooltip": {"trigger": "axis"},
            "legend": {"top": 30},
            "grid": {"left": 60, "right": 30, "top": 70, "bottom": 60}}


def _ec_line(x, series, xlab, ylab, ttl, height=360):
    opt = _ec_base(ttl, xlab, ylab)
    opt["xAxis"] = {"type": "category", "data": [_jnum(v) for v in x], "name": xlab}
    opt["yAxis"] = {"type": "value", "name": ylab}
    opt["series"] = [{"name": n, "type": "line",
                      "data": [_jnum(v) if v == v else None for v in ys],
                      "showSymbol": False, "connectNulls": True,
                      "emphasis": {"focus": "series"}} for n, ys in series]
    return opt


def _ec_scatter(x, y, name, ttl, xlab, ylab, color=None, height=340):
    # 注意: 散点不加 dataZoom——echarts5 的 dataZoom 在部分 scatter 数据上内部报
    # "Cannot read properties of undefined (reading 'get')"。散点聚焦悬浮读数即可。
    opt = _ec_base(ttl, xlab, ylab)
    opt.pop("dataZoom", None)
    opt["tooltip"] = {"trigger": "item"}
    pts = []
    for a, b in zip(x, y):
        a, b = _jnum(a), _jnum(b)
        # 过滤 NaN 与 y=0(停车段 lat_err_rms=0, echarts dataZoom 在 x 重复+y=0 数据集上内部报错)
        if a == a and b == b and b != 0.0:
            pts.append([a, b])
    pt = {"name": name, "type": "scatter", "data": pts, "symbolSize": 6}
    if color:
        pt["color"] = color          # 用顶层 color 而非 itemStyle.color——echarts5 的
                                      # itemStyle.color 在部分 scatter 上内部报 get undefined
    opt["series"] = [pt]
    return opt


def _ec_heatmap(xb, yb, z, ttl, xlab, ylab, height=480):
    data = []
    for i, yy in enumerate(yb):
        for j, xx in enumerate(xb):
            v = z[i][j]
            if v is not None and v == v:
                data.append([_jnum(j), _jnum(i), _jnum(v)])
    zmin, zmax = float(np.nanmin(z)), float(np.nanmax(z))
    if zmax - zmin < 1e-12:          # 常数网格(如全程 OOD=IN): echarts visualMap 需 min<max, 否则内部报错
        zmin, zmax = zmin - 1.0, zmax + 1.0
    return {"title": {"text": ttl, "left": "center", "textStyle": {"fontSize": 14}},
            "tooltip": {"position": "top"},
            "grid": {"left": 70, "right": 90, "top": 55, "bottom": 60},
            "xAxis": {"type": "category", "data": [_jnum(v) for v in xb], "name": xlab},
            "yAxis": {"type": "category", "data": [_jnum(v) for v in yb], "name": ylab},
            "visualMap": {"min": zmin, "max": zmax,
                          "orient": "vertical", "right": 5, "top": 60,
                          "inRange": {"color": ["#0d2242", "#246bfe", "#ea580c"]}},
            "series": [{"type": "heatmap", "data": data}]}


def _interactive_charts(bags_dfs):
    """收集交互图: [(div_id, option, height)]"""
    charts = []
    _cid = [0]
    def uid(prefix):
        _cid[0] += 1
        return f"ch{_cid[0]:02d}_{prefix}"
    alldf = pd.concat([df for _, df, _ in bags_dfs if len(df)], ignore_index=True) \
        if any(len(df) for _, df, _ in bags_dfs) else pd.DataFrame()
    # Overview 时序
    for col, name in [("koopman_health_score_total", "health_total"),
                      ("lat_err", "lat_err"), ("koopman_gain_lp", "gain_lp")]:
        for (bg, df, cov) in bags_dfs:
            if df.empty:
                continue
            t = df["t"] - df["t"].iloc[0]
            charts.append((uid(f"ov_{col[:8]}"),
                           _ec_line(t.values, [(os.path.basename(bg)[:10], df[col].values)],
                                    "t/s", col, f"Overview {name}"), 320))
    # G1 分箱
    zvars = [("koopman_vel_u", "vel_u"), ("abs_r", "|r|"), ("abs_steer", "|steer|")]
    for st in STATES:
        ycol = acc_key("plan", st, 10, "rmse")
        for zcol, zlab in zvars:
            series, xc = [], None
            for (bg, df, cov) in bags_dfs:
                res = quantile_binned_stat(df, zcol, ycol, 8)
                if res:
                    cx, p50, p95, n = res
                    xc = cx
                    series.append((f"{os.path.basename(bg)[:8]} P50", p50))
                    series.append((f"{os.path.basename(bg)[:8]} P95", p95))
            if series and xc is not None:
                charts.append((uid(f"g1_{st}_{zcol[:5]}"),
                               _ec_line(xc, series, zlab, "rmse",
                                        f"G1 plan_{st}_10step vs {zlab}"), 340))
    # G1c 热力图
    if len(alldf) > 200:
        for zcol, ttl in [(acc_key("plan", "r", 10, "rmse"), "G1c plan_r_10step rmse"),
                          ("koopman_health_ood_level", "G1c ood_level")]:
            res = heatmap_2d(alldf, "koopman_vel_u", "abs_r", zcol)
            if res:
                xc, yc, grid, gcnt = res
                charts.append((uid("g1c"),
                               _ec_heatmap([f"{v:.1f}" for v in xc],
                                           [f"{v:.3f}" for v in yc], grid.tolist(),
                                           ttl, "vx", "|r|"), 480))
    # G2
    for cname in ["lat_err_rms", "lat_err_p2p", "lat_err_hfe", "phi_err_rms"]:
        for zcol, zlab in [("koopman_vel_u", "vel_u"), ("abs_r", "|r|")]:
            series, xc = [], None
            for (bg, df, cov) in bags_dfs:
                res = quantile_binned_stat(df, zcol, cname, 8)
                if res:
                    cx, p50, p95, n = res
                    xc = cx
                    series.append((f"{os.path.basename(bg)[:8]} P50", p50))
                    series.append((f"{os.path.basename(bg)[:8]} P95", p95))
            if series and xc is not None:
                charts.append((uid(f"g2_{cname[:8]}"),
                               _ec_line(xc, series, zlab, cname,
                                        f"G2 {cname} vs {zlab}"), 320))
    # G3 散点 + 互相关
    mcol = acc_key("plan", "r", 10, "rmse")
    for (bg, df, cov), c in zip(bags_dfs, COLORS):
        if df.empty:
            continue
        x, y = downsample(df[mcol].values, df["lat_err_rms"].values, 800)
        charts.append((uid("g3"),
                       _ec_scatter(x, y, os.path.basename(bg)[:10],
                                   f"G3 M-C 散点 {os.path.basename(bg)[:10]}",
                                   "plan_r_10step rmse", "lat_err rms", color=c), 340))
    res = xcorr_lag_curve(alldf, mcol, "lat_err_rms", 3.0)
    if res:
        charts.append((uid("g3x"),
                       _ec_line(res[0], [("corr", res[1])], "lag (s)", "corr",
                                "G3 时滞互相关"), 320))
    # G4
    for (bg, df, cov) in bags_dfs:
        if df.empty:
            continue
        mat = [[spearman(df[acc_key("plan", s, k, "rmse")], df["lat_err_rms"])
                for k in [1, 10, 25]] for s in STATES]
        charts.append((uid("g4"),
                       _ec_line([1, 10, 25],
                                [(s, mat[i]) for i, s in enumerate(STATES)],
                                "horizon", "rho",
                                f"G4 horizon 相关 {os.path.basename(bg)[:10]}"), 320))
    res = xcorr_lag_curve(alldf, M_GAIN, "lat_err_rms", 3.0)
    if res:
        charts.append((uid("g4g"),
                       _ec_line(res[0], [("corr", res[1])], "lag (s)", "corr",
                                "G4 gain 先导性"), 320))
    return charts


def build_frontend_interactive(bags_dfs, out_dir):
    """ECharts 交互版: index.html + echarts.min.js(自包含, 无外网依赖)。"""
    os.makedirs(out_dir, exist_ok=True)
    src_ech = None
    for cand in ["/tmp/node_modules/echarts/dist/echarts.min.js",
                 os.path.join(os.path.dirname(os.path.abspath(__file__)), "echarts.min.js")]:
        if os.path.exists(cand):
            src_ech = cand
            break
    if src_ech is None:
        print("[warn] echarts.min.js 未找到(交互版需先 npm install echarts 或放 echarts.min.js 到本目录)")
        return None
    import shutil
    shutil.copy(src_ech, os.path.join(out_dir, "echarts.min.js"))
    charts = _interactive_charts(bags_dfs)
    divs = []
    inits = []
    for div_id, opt, height in charts:
        divs.append(f'<div id="{div_id}" style="width:100%;height:{height}px"></div>')
        inits.append(f'try{{echarts.init(document.getElementById("{div_id}")).setOption('
                     f'{json.dumps(opt, ensure_ascii=False, separators=(",", ":"))});'
                     f'}}catch(e){{console.error("chart {div_id}", e);}}')
    n_frames = int(sum(c["_n_active"] for _, _, c in bags_dfs))
    nav = "\n  ".join('<a href="#%s">%s</a>' % (a, t) for a, t, *_ in _SECTIONS)
    sec_heads = []
    for a, t, d, pl, l in _SECTIONS:
        sec_heads.append(
            '<section class="section %s" id="%s"><div class="sec-head">'
            '<span class="dot"></span><h2>%s</h2><span class="tag">%s</span></div>'
            '<p class="sec-desc">%s</p></section>' % (CH_CLASS.get(a, "ch-z"), a, t, a.upper(), d))
    html = (f"<!doctype html><html lang='zh-CN'><head><meta charset='utf-8'>"
            f"<meta name='viewport' content='width=device-width, initial-scale=1'>"
            f"<title>Koopman 横向模型 · 三方分析（交互版）</title>"
            f"<script src='echarts.min.js'></script><style>{_CSS}</style></head><body>"
            f"<header class='hero'><div class='wrap'>"
            f"<span class='eyebrow'><i></i>KOOPMAN LATERAL · INTERACTIVE</span>"
            f"<h1>Koopman 横向模型三方分析</h1>"
            f"<p class='lead'>工况 Z × 模型评分 M × 控制误差 C。交互图支持缩放、图例开关、悬浮读数。"
            f"输入来自回放 bag 的 <code>/msd/endpoint/control_command</code> 每帧 JSON。</p>"
            f"<div class='meta'><span>{len(bags_dfs)} 数据包 · {n_frames:,} 有效帧</span>"
            f"<span>ECharts 交互</span></div></div></header>"
            f"<div class='tracks'><i></i><i></i><i></i></div>"
            f"<nav><div class='wrap inner'>{nav}</div></nav>"
            f"<main class='wrap'>{''.join(sec_heads)}"
            f"<div class='card'>{''.join(divs)}</div></main>"
            f"<script>{''.join(inits)}</script>"
            f"<div class='tracks' style='margin-top:34px'><i></i><i></i><i></i></div>"
            f"<footer>Koopman 三方分析 · ECharts 交互版</footer></body></html>")
    idx = os.path.join(out_dir, "index.html")
    with open(idx, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"[OK] interactive -> {idx} ({len(charts)} 图, echarts 自包含)")
    return idx


def build_frontend(bags_dfs, out_dir):
    """生成前端页面目录: index.html + 分页 PNG（hero 真实数据 + 章节导航 + 卡片）"""
    os.makedirs(out_dir, exist_ok=True)
    pages = build_report_pages(bags_dfs, out_dir)
    title2file = {}
    for title, path in pages:
        newp = os.path.join(out_dir, f"p{_ASCII_NAME.get(title, title)}.png")
        if path != newp:
            if os.path.exists(newp):
                os.remove(newp)
            os.rename(path, newp)
        title2file[title] = os.path.basename(newp)

    alldf = pd.concat([df for _, df, _ in bags_dfs if len(df)], ignore_index=True) \
        if any(len(df) for _, df, _ in bags_dfs) else pd.DataFrame()
    health = alldf["koopman_health_score_total"] if len(alldf) else pd.Series(dtype=float)
    laterr = float(np.sqrt((alldf["lat_err"] ** 2).mean())) if len(alldf) else np.nan
    pr10 = float(alldf[acc_key("plan", "r", 10, "rmse")].median()) if len(alldf) else np.nan
    fmt = lambda v, nd=1: "—" if not np.isfinite(v) else f"{v:.{nd}f}"

    hero = "\n      ".join([
        f'<div class="stat hl"><b>{fmt(health.mean())}</b><span>健康分均值 /100</span></div>',
        f'<div class="stat"><b>{fmt(health.quantile(0.05))}</b><span>健康分 P5（低分尾部）</span></div>',
        (f'<div class="stat amber"><b>{laterr * 100:.1f} cm</b><span>lat_err RMS</span></div>'
         if np.isfinite(laterr) else '<div class="stat amber"><b>—</b><span>lat_err RMS</span></div>'),
        (f'<div class="stat gn"><b>{pr10 * 100:.2f}%</b><span>plan r·10step 误差中位</span></div>'
         if np.isfinite(pr10) else '<div class="stat gn"><b>—</b><span>plan r·10step 误差</span></div>'),
    ])
    nav = "\n  ".join('<a href="#%s">%s</a>' % (a, t) for a, t, *_ in _SECTIONS)
    # 章节通道色: 按 G 分组轮换 Z/M/C 三色（编码"三源"主题）
    sec_parts = []
    for a, t, d, pl, l in _SECTIONS:
        cards = "".join(
            '<div class="card"><img src="%s" alt="%s" loading="lazy"></div>' % (title2file[p], p)
            for p in pl if p in title2file)
        grid_cls = "grid2 " if l == "2col" else ""
        sec_parts.append(
            '<section class="section %s" id="%s"><div class="sec-head">'
            '<span class="dot"></span><h2>%s</h2>'
            '<span class="tag">%s</span></div>'
            '<p class="sec-desc">%s</p>'
            '<div class="%s">%s</div></section>'
            % (CH_CLASS.get(a, "ch-z"), a, t, a.upper(), d, grid_cls, cards))
    secs = "\n\n  ".join(sec_parts)

    n_frames = int(sum(c["_n_active"] for _, _, c in bags_dfs))
    n_segs = int(sum(df["segment"].nunique() if len(df) else 0 for _, df, _ in bags_dfs))
    html = (f"<!doctype html><html lang='zh-CN'><head><meta charset='utf-8'>"
            f"<meta name='viewport' content='width=device-width, initial-scale=1'>"
            f"<title>Koopman 横向模型 · 三方分析报告</title><style>{_CSS}</style></head><body>"
            f"<header class='hero'><div class='wrap'>"
            f"<span class='eyebrow'><i></i>KOOPMAN LATERAL · ONLINE HEALTH</span>"
            f"<h1>Koopman <em>横向模型</em>三方分析</h1>"
            f"<p class='lead'>工况 <code>Z</code> × 模型评分 <code>M</code> × 控制误差 "
            f"<code>C</code>。输入来自回放 bag 的 "
            f"<code>/msd/endpoint/control_command</code> 每帧 JSON，全部在 "
            f"<code>koopman_analysis</code> 工具链内清洗、对齐、归因。</p>"
            f"<div class='meta'><span>{len(bags_dfs)} 数据包 · {n_frames:,} 有效帧</span>"
            f"<span>{n_segs} 个连续控制段</span><span>matplotlib 静态渲染</span></div>"
            f"<div class='stats'>{hero}</div></div></header>"
            f"<div class='tracks'><i></i><i></i><i></i></div>"
            f"<nav><div class='wrap inner'>{nav}</div></nav>"
            f"<main class='wrap'>{secs}</main>"
            f"<div class='tracks' style='margin-top:34px'><i></i><i></i><i></i></div>"
            f"<footer>Koopman 三方分析 · 生成于 debug/koopman_analysis · 示例包数据</footer>"
            f"</body></html>")
    idx = os.path.join(out_dir, "index.html")
    with open(idx, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"[OK] frontend -> {idx} ({len(pages)} 页 PNG + index.html)")
    return idx


# ---------------- CLI ----------------
def resolve_entry(item, base_dirs):
    if os.path.exists(item):
        return item
    for base in base_dirs:
        cand = os.path.join(base, item) if not os.path.isabs(item) else item
        if os.path.exists(cand):
            return cand
    return item


def expand_bag_list(spec, base_dirs=None):
    base_dirs = base_dirs or []
    bags = []
    for item in spec:
        if item.startswith("@"):
            path = resolve_entry(item[1:], base_dirs)
            with open(path) as f:
                sub = [l.strip() for l in f if l.strip() and not l.startswith("#")]
            bags += expand_bag_list(sub, [os.path.dirname(os.path.abspath(path))] + base_dirs)
        else:
            item = resolve_entry(item, base_dirs)
            if os.path.isdir(item):
                bags += sorted(glob.glob(os.path.join(item, "**", "*.bag"), recursive=True))
            else:
                bags.append(item)
    return [b for b in bags if not b.endswith(".orig.bag")]


def load_all(bags, conf):
    bags_dfs = []
    for b in bags:
        print(f"[load] {b}")
        df, meta = load_bag(b, conf)
        cov = preflight(df, meta)
        df = clean_and_segment(df, conf)
        bags_dfs.append((b, df, cov))
        print(f"  frames={meta['n_frame']} valid={cov['_n_active']} "
              f"segs={df['segment'].nunique() if len(df) else 0}")
    return bags_dfs


def main():
    ap = argparse.ArgumentParser(
        description="Koopman 三方分析 · 单文件工具链（长图/PDF/前端/准入）")
    ap.add_argument("--list", default="analysis_list.txt")
    ap.add_argument("--out", default="output/koopman_report.png")
    ap.add_argument("--frontend", nargs="?", const="output/frontend", default=None,
                    metavar="DIR", help="生成静态 PNG 前端（index.html + 分页PNG）")
    ap.add_argument("--frontend-interactive", nargs="?", const="output/frontend_ia", default=None,
                    metavar="DIR", help="生成 ECharts 交互版前端（index.html + echarts.min.js 自包含；实验性）")
    ap.add_argument("--preflight", default=None, metavar="PATHS",
                    help="只做字段准入检查(抽样3000帧; 逗号分隔 bag/目录)")
    ap.add_argument("--preflight-max-frames", type=int, default=3000)
    args = ap.parse_args()

    conf = dict(DEFAULT_CONF)
    if args.preflight:
        spec = [s.strip() for s in args.preflight.split(",") if s.strip()]
        for b in expand_bag_list(spec):
            preflight_report(b, conf, max_frames=args.preflight_max_frames or None)
        return

    if not os.path.exists(args.list):
        sys.exit(f"list 不存在: {args.list}")
    list_dir = os.path.dirname(os.path.abspath(args.list))
    bags = expand_bag_list(
        [l.strip() for l in open(args.list) if l.strip() and not l.strip().startswith("#")],
        [list_dir, os.path.dirname(list_dir)])
    if not bags:
        sys.exit("no bags found")
    bags_dfs = load_all(bags, conf)

    if args.frontend:
        build_frontend(bags_dfs, args.frontend)
    elif args.frontend_interactive:
        build_frontend_interactive(bags_dfs, args.frontend_interactive)
    else:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)
        build_report(bags_dfs, args.out)


if __name__ == "__main__":
    main()
