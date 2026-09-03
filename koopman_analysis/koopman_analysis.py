#!/usr/bin/env python3
# encoding: utf-8
"""Koopman 三方分析 · 单文件工具链

Z(工况) × M(模型评分) × C(控制误差) 的相关性与传导分析。
唯一入口生成 Plotly 单文件离线交互 HTML。

用法（在 koopman_analysis/ 下）:
  python3 koopman_analysis.py                              # output/index.html

analysis_list.txt 每行一个条目（# 注释；路径相对 list 所在目录及其上级自动解析）:
  场景目录 bags/score/score_1 / 父目录 bags/score / bag 文件 / @其他.txt

报告使用 Plotly 单文件离线 HTML；所有图可悬停读数，G1c 三维图支持拖拽旋转。
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
    # 该字段不是 health score，但必须读入：未填满 10 帧历史时的预测/评分不能参与统计。
    "koopman_history_warmup",
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

# 统一 accuracy 分析使用的冻结参考表（拷自 koopman_accuracy_monitor.cpp, kSteps 行序）
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

COLORS = ["#246bfe", "#059669", "#dc2626", "#7c3aed", "#ea580c"]
REPORT_DPI = 300


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


def preflight(df, meta, conf=DEFAULT_CONF):
    """字段覆盖率(有效帧内)。返回 cov 字典"""
    active = df[conf["active_field"]].notna().values
    n_active = int(active.sum())
    cov = {f: (df[f].notna().values & active).sum() / n_active if n_active else 0.0
           for f in ALL_FIELDS}
    cov.update({"_n_active": n_active, "_n_frame": meta["n_frame"], "_json_fail": meta["n_fail"]})
    return cov


# ---------------- 清洗与特征 ----------------
def _estimate_sample_rate(t):
    """返回一个连续 segment 的稳健帧率；异常/极短段使用 50 Hz 保底。"""
    dt = np.diff(np.asarray(t, dtype=float))
    dt = dt[np.isfinite(dt) & (dt > 1e-4) & (dt <= DEFAULT_CONF["gap_sec"])]
    return 1.0 / float(np.median(dt)) if len(dt) else 50.0


def _variable_window_rolling(df, col, windows, kind, min_fraction):
    """按 segment 滚动计算，窗口随 segment 实际帧率变化且绝不跨时间断点。"""
    out = pd.Series(np.nan, index=df.index, dtype=float)
    for _, idx in df.groupby("segment", sort=False).groups.items():
        values = df.loc[idx, col]
        window = int(windows.loc[idx].iloc[0])
        min_periods = max(1, int(np.ceil(window * min_fraction)))
        rolling = values.rolling(window, min_periods=min_periods)
        if kind == "mean":
            result = rolling.mean()
        elif kind == "rms":
            result = rolling.apply(lambda x: np.sqrt(np.mean(x ** 2)), raw=True)
        elif kind == "p2p":
            result = rolling.max() - rolling.min()
        else:
            raise ValueError(f"unknown rolling kind: {kind}")
        out.loc[idx] = result
    return out


def clean_and_segment(df, conf):
    """有效帧筛选 + segment 切分 + 派生量"""
    active = df[conf["active_field"]].notna() & np.isfinite(df[conf["active_field"]])
    df = df[active].copy()
    if "koopman_history_warmup" in df:
        df = df[(df["koopman_history_warmup"].fillna(0) > 0) | df["koopman_history_warmup"].isna()]
    # 低速过滤: 只保留 v > 1.0 m/s 的帧
    df = df[df["koopman_vel_u"] > 1.0]
    df = df.sort_values("t").reset_index(drop=True)
    if df.empty:
        df["segment"] = []
        return df
    df["segment"] = "S" + (df["t"].diff().fillna(np.inf) > conf["gap_sec"]).cumsum().astype(str)
    seg_len = df.groupby("segment")["t"].agg(lambda s: s.iloc[-1] - s.iloc[0])
    df = df[df["segment"].isin(seg_len[seg_len >= conf["min_segment_sec"]].index)].reset_index(drop=True)
    if df.empty:
        return df
    # bag 可能含不同帧率的连续段；不能用跨段的全局中位帧率推导窗口。
    seg_fs = df.groupby("segment")["t"].transform(_estimate_sample_rate)
    wc = (conf["c_window_sec"] * seg_fs).round().clip(lower=2).astype(int)
    wm = (conf["maneuver_window_sec"] * seg_fs).round().clip(lower=2).astype(int)
    # Z 派生
    df["abs_r"] = df["mpc_init_dyawrate"].abs()
    df["abs_steer"] = df["koopman_wheel_angle_meas"].abs()
    df["a_y"] = df["koopman_vel_u"] * df["mpc_init_dyawrate"]
    # 机动类型（segment 内滑窗）
    df["r_mean"] = _variable_window_rolling(df, "abs_r", wm, "mean", min_fraction=0.0)
    b1, b2 = conf["maneuver_r_bins"]
    df["maneuver"] = np.select([df["r_mean"] < b1, df["r_mean"] < b2],
                               ["straight", "gentle"], default="sharp")
    # C 派生（segment 内滑窗, 跨 gap 自动切断）
    for cname in ("lat_err", "phi_err"):
        df[f"{cname}_rms"] = _variable_window_rolling(df, cname, wc, "rms", min_fraction=0.5)
        df[f"{cname}_p2p"] = _variable_window_rolling(df, cname, wc, "p2p", min_fraction=0.5)
        df[f"{cname}_hfe"] = _variable_window_rolling(
            df.assign(**{cname: df.groupby("segment")[cname].diff()}), cname, wc, "rms", min_fraction=0.5)
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


def heatmap_3d(df, xcol, ycol, zcol, nx=8, ny=8, nz=8):
    """(x,y,z) 三维分位网格统计。返回 (xc, yc, zc, grid3d, gcnt3d)"""
    d = df[[xcol, ycol, zcol]].dropna()
    if len(d) < nx * ny * nz:
        return None
    try:
        xq = pd.qcut(d[xcol], nx, labels=False, duplicates="drop")
        yq = pd.qcut(d[ycol], ny, labels=False, duplicates="drop")
        zq = pd.qcut(d[zcol], nz, labels=False, duplicates="drop")
    except Exception:
        return None
    d = d.assign(xb=xq.values, yb=yq.values, zb=zq.values)
    g = d.groupby(["xb", "yb", "zb"]).size()
    nx_eff = int(pd.Series(xq.unique()).max()) + 1
    ny_eff = int(pd.Series(yq.unique()).max()) + 1
    nz_eff = int(pd.Series(zq.unique()).max()) + 1
    xc = d.groupby("xb")[xcol].median().reindex(range(nx_eff)).values
    yc = d.groupby("yb")[ycol].median().reindex(range(ny_eff)).values
    zc = d.groupby("zb")[zcol].median().reindex(range(nz_eff)).values
    grid = np.zeros((nx_eff, ny_eff, nz_eff))
    for (ix, iy, iz), v in g.items():
        grid[ix, iy, iz] = v
    return xc, yc, zc, grid


def quantile_codes_and_edges(series, nbins):
    """分位分箱编码及边界。图中应展示边界区间，不能把 bin 中心误读为连续坐标。"""
    codes, edges = pd.qcut(series, nbins, labels=False, retbins=True, duplicates="drop")
    return codes.astype(int), edges


def quantile_interval_labels(edges, precision=3):
    return [f"{left:.{precision}f}–{right:.{precision}f}"
            for left, right in zip(edges[:-1], edges[1:])]


def error_heatmap_cmap():
    """误差图专用冷色系：浅蓝低误差、深蓝高误差，空白表示无样本。"""
    base = plt.get_cmap("Blues")
    cmap = matplotlib.colors.LinearSegmentedColormap.from_list(
        "koopman_error_cool", base(np.linspace(0.30, 1.0, 256)))
    cmap.set_bad("#ffffff")
    return cmap


def frequency_heatmap_cmap():
    """频次图专用暖色系：黄低频、红高频，空白表示无样本。"""
    base = plt.get_cmap("YlOrRd")
    cmap = matplotlib.colors.LinearSegmentedColormap.from_list(
        "koopman_frequency_warm", base(np.linspace(0.20, 1.0, 256)))
    cmap.set_bad("#ffffff")
    return cmap


def paired_coverage_error_grid(df, xcol, ycol, errcol, nx=8, ny=8):
    """一次分箱同时生成频次与误差网格，确保左右图的坐标和无样本掩码严格一致。"""
    d = df[[xcol, ycol, errcol]].dropna().copy()
    if len(d) < nx * ny:
        return None
    try:
        xq, xedges = quantile_codes_and_edges(d[xcol], nx)
        yq, yedges = quantile_codes_and_edges(d[ycol], ny)
    except Exception:
        return None
    d = d.assign(xb=xq.values, yb=yq.values)
    group = d.groupby(["yb", "xb"])[errcol]
    counts, errors = group.size(), group.median()
    count_grid = np.full((len(yedges) - 1, len(xedges) - 1), np.nan)
    error_grid = np.full_like(count_grid, np.nan)
    for (iy, ix), value in counts.items():
        count_grid[iy, ix] = value
        error_grid[iy, ix] = errors.loc[(iy, ix)]
    xcenters = d.groupby("xb")[xcol].median().reindex(range(len(xedges) - 1)).values
    ycenters = d.groupby("yb")[ycol].median().reindex(range(len(yedges) - 1)).values
    return xcenters, ycenters, count_grid, error_grid


def _g1c_3d_panel(ax, df, mode, cmap, err_col=None):
    """3D 面板: mode='freq' 点大小=频次; mode='err' 点颜色=误差"""
    res = heatmap_3d(df, "koopman_vel_u", "abs_r", "abs_steer", 8, 8, 8)
    if res is None:
        ax.text(0.1, 0.5, 0.5, "数据不足", transform=ax.transAxes)
        return
    xc, yc, zc, grid = res
    xs, ys, zs, sizes, colors = [], [], [], [], []
    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            for k in range(grid.shape[2]):
                v = grid[i, j, k]
                if v <= 0:
                    continue
                xs.append(xc[i]); ys.append(yc[j]); zs.append(zc[k])
                if mode == "err":
                    # 该格子的误差中位数
                    d = df[(df["koopman_vel_u"] >= 0)]  # 简化: 用全 df, 实际按 bin 过滤
                    sizes.append(min(200, 10 + v * 2))
                else:
                    sizes.append(min(200, 10 + v * 2))
                    colors.append(v)
    if mode == "freq":
        sc = ax.scatter(xs, ys, zs, s=sizes, c=colors, cmap="YlOrRd", alpha=0.7)
    else:
        # 误差模式: 颜色=误差中位数(按 bin 查)
        d = df.copy()
        d["xb"] = pd.qcut(d["koopman_vel_u"], 8, labels=False, duplicates="drop").values
        d["yb"] = pd.qcut(d["abs_r"], 8, labels=False, duplicates="drop").values
        d["zb"] = pd.qcut(d["abs_steer"], 8, labels=False, duplicates="drop").values
        med = d.groupby(["xb","yb","zb"])[err_col].median()
        col2 = []
        for i in range(grid.shape[0]):
            for j in range(grid.shape[1]):
                for k in range(grid.shape[2]):
                    v = grid[i,j,k]
                    if v > 0:
                        try: col2.append(med.loc[(i,j,k)])
                        except Exception: col2.append(np.nan)
        sc = ax.scatter(xs, ys, zs, s=sizes, c=col2, cmap="RdYlBu_r", alpha=0.7)
    ax.set_xlabel("vx"); ax.set_ylabel("|r|"); ax.set_zlabel("|steer|")
    return sc


def g1c_8panel(fig, bags_dfs):
    """G1c 4 行 × 2 列: 行1=3D(频次|误差), 行2-4=2D 投影(频次|误差)"""
    from mpl_toolkits.mplot3d import Axes3D
    alldf = _alldf(bags_dfs)
    if alldf.empty:
        return
    pairs = [("koopman_vel_u", "abs_r"), ("koopman_vel_u", "abs_steer"), ("abs_r", "abs_steer")]
    # 行1: 3D 频次 | 3D 误差
    ax31 = fig.add_subplot(4, 2, 1, projection="3d")
    sc = _g1c_3d_panel(ax31, alldf, "freq", "YlOrRd")
    ax31.set_title("覆盖度 3D (大小=频次)")
    if sc: fig.colorbar(sc, ax=ax31, shrink=0.5)
    ax32 = fig.add_subplot(4, 2, 2, projection="3d")
    sc = _g1c_3d_panel(ax32, alldf, "err", "RdYlBu_r", acc_key("plan","r",10,"rmse"))
    ax32.set_title("误差 3D (颜色=plan_r_10step rmse)")
    if sc: fig.colorbar(sc, ax=ax32, shrink=0.5)
    # 行2-4: 2D 频次 | 2D 误差
    for row, (xcol, ycol) in enumerate(pairs, 2):
        axf = fig.add_subplot(4, 2, (row-1)*2+1)
        res = heatmap_2d(alldf, xcol, ycol, "koopman_vel_u", 10, 10)
        if res:
            xc, yc, grid, gcnt = res
            im = axf.imshow(gcnt, aspect="auto", origin="lower", cmap="YlOrRd")
            axf.set_xticks(range(len(xc))); axf.set_xticklabels([f"{v:.1f}" for v in xc], fontsize=6)
            axf.set_yticks(range(len(yc))); axf.set_yticklabels([f"{v:.2f}" for v in yc], fontsize=6)
            axf.set_xlabel(_axlabel(xcol)); axf.set_ylabel(_axlabel(ycol))
            axf.set_title(f"覆盖 2D {_axlabel(xcol)}×{_axlabel(ycol)} (频次)")
            fig.colorbar(im, ax=axf, fraction=0.04)
        axe = fig.add_subplot(4, 2, (row-1)*2+2)
        res = heatmap_2d(alldf, xcol, ycol, acc_key("plan","r",10,"rmse"), 10, 10)
        if res:
            xc, yc, grid, gcnt = res
            im = axe.imshow(grid, aspect="auto", origin="lower", cmap="RdYlBu_r")
            axe.set_xticks(range(len(xc))); axe.set_xticklabels([f"{v:.1f}" for v in xc], fontsize=6)
            axe.set_yticks(range(len(yc))); axe.set_yticklabels([f"{v:.2f}" for v in yc], fontsize=6)
            axe.set_xlabel(_axlabel(xcol)); axe.set_ylabel(_axlabel(ycol))
            axe.set_title(f"误差 2D {_axlabel(xcol)}×{_axlabel(ycol)}")
            fig.colorbar(im, ax=axe, fraction=0.04)
    fig.suptitle("G1c 覆盖度(频次) 与误差 对比：行1=3D, 行2-4=2D 投影", y=0.995, fontsize=13)


def xcorr_lag_curve(df, mcol, ccol, max_lag_sec, n_points=61):
    """时滞互相关: M(t) 与 C(t+lag)。

    每个连续 segment 独立标准化并计算，再按有效配对帧数加权汇总。因此不会把两个
    bag、暂停 gap 或不同帧率段错误地拼成一条连续时序。
    """
    required = ["t", mcol, ccol]
    if not set(required).issubset(df):
        return None
    group_col = "segment_id" if "segment_id" in df else "segment"
    if group_col not in df:
        return None
    lags = np.linspace(-max_lag_sec, max_lag_sec, n_points)
    corr_sum, weight_sum = np.zeros(len(lags)), np.zeros(len(lags))
    for _, part in df.groupby(group_col, sort=False):
        d = part[required].dropna()
        if len(d) < 100:
            continue
        fs = _estimate_sample_rate(d["t"].values)
        m, c = d[mcol].to_numpy(float), d[ccol].to_numpy(float)
        for i, lag_sec in enumerate(lags):
            lag = int(round(lag_sec * fs))
            if abs(lag) >= len(m):
                continue
            if lag >= 0:
                a, b = (m[:len(m) - lag] if lag else m), c[lag:]
            else:
                a, b = m[-lag:], c[:len(c) + lag]
            if len(a) < 50 or len(b) < 50 or np.std(a) == 0 or np.std(b) == 0:
                continue
            corr = float(np.corrcoef(a, b)[0, 1])
            if np.isfinite(corr):
                corr_sum[i] += corr * len(a)
                weight_sum[i] += len(a)
    valid = weight_sum > 0
    if not valid.any():
        return None
    return lags[valid], corr_sum[valid] / weight_sum[valid]


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
    cur_start = cur_end = hot["t"].iloc[0]
    cur_peak = hot.index[0]
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


def stratified_spearman(df, xcol, ycol, min_frames=60):
    """按机动类型和速度段分层，避免把场景难度误当作模型误差传导。"""
    rows = []
    for (maneuver, speed_bin), part in df.groupby(["maneuver", "speed_bin"], observed=True):
        n = int(part[[xcol, ycol]].dropna().shape[0])
        if n >= min_frames:
            rows.append({"maneuver": maneuver, "speed_bin": str(speed_bin), "n": n,
                         "rho": spearman(part[xcol], part[ycol])})
    return pd.DataFrame(rows)


def event_response(df, mcol, ccol, quantile, min_gap_sec, window_sec):
    """统计每个高/低 M 非重叠事件后 C 的窗口 RMS，事件仅在连续段内生成。"""
    values = []
    group_col = "segment_id" if "segment_id" in df else "segment"
    for _, part in df.groupby(group_col, sort=False):
        data = part[["t", mcol, ccol]].dropna().copy()
        if len(data) < 50:
            continue
        # lower tail 通过取负复用同一个事件去重算法。
        event_col = mcol
        if quantile < 0.5:
            event_col = "_neg_m"
            data[event_col] = -data[mcol]
            q = 1.0 - quantile
        else:
            q = quantile
        for start, _, _ in nonoverlap_events(data, event_col, q, min_gap_sec):
            value = window_stat_after(data, start, ccol, window_sec)
            if np.isfinite(value):
                values.append(value)
    return np.asarray(values, dtype=float)


# ---------------- 准入检查（--preflight） ----------------
def preflight_report(bag_path, conf, max_frames=3000):
    """详细字段准入报告(终端输出)"""
    print("=" * 78)
    print(f"BAG: {bag_path}")
    df, meta = load_bag(bag_path, conf, max_frames=max_frames)
    cov = preflight(df, meta, conf)
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
    """总览（不分包）：汇总统计表 + 数据质量文字区"""
    alldf = pd.concat([d for _, d, _ in bags_dfs if len(d)], ignore_index=True)
    h = alldf["koopman_health_score_total"] if len(alldf) else pd.Series(dtype=float)
    valid_h = h[h >= 0]
    rows = [{
        "all": "汇总",
        "frames": int(sum(c["_n_frame"] for _, _, c in bags_dfs)),
        "valid": int(sum(c["_n_active"] for _, _, c in bags_dfs)),
        "health_p05": _q(valid_h, 0.05) if len(valid_h) else np.nan,
        "laterr_rms": float(np.sqrt((alldf["lat_err"] ** 2).mean())) if len(alldf) else np.nan,
        "p99_overshoot_frames": int(alldf["koopman_health_executed_error_p99_hit"].sum()) if len(alldf) else 0,
        "health_invalid_pct": float((h < 0).mean()) if len(h) else np.nan,
    }]
    df_tbl = pd.DataFrame(rows)
    _table_page(fig, df_tbl, "Overview: 数据汇总")


def _alldf(bags_dfs):
    """合并所有 bag 数据为整体"""
    return pd.concat([d for _, d, _ in bags_dfs if len(d)], ignore_index=True)
def _axlabel(col):
    """工况列 -> 完整中文轴名"""
    return {"koopman_vel_u": "纵向速度 vx (m/s)",
            "abs_r": "横摆角速度 |r| (rad/s)",
            "abs_steer": "前轮转角 |steer| (rad)"}.get(col, col)



def _g1_step_page(step):
    """G1: 某 step(1/10) 的 4 状态 × 3 工况，同一子图叠 plan(蓝实线) vs executed(绿实线)"""
    def fn(fig, bags_dfs):
        alldf = _alldf(bags_dfs)
        if alldf.empty:
            return
        zvars = [("koopman_vel_u", "vel_u"), ("abs_r", "|r|"), ("abs_steer", "|steer|")]
        axs = fig.subplots(4, 3)
        for i, state in enumerate(STATES):
            for j, (zcol, zlab) in enumerate(zvars):
                ax = axs[i, j]
                _binned_curve(ax, alldf, zcol, acc_key("plan", state, step, "rmse"),
                              COLORS[0], "plan P50")
                _binned_curve(ax, alldf, zcol, acc_key("executed", state, step, "rmse"),
                              COLORS[1], "exec P50")
                ax.set_title(f"{state} {step}step vs {zlab}")
                ax.set_xlabel(zlab); ax.set_ylabel("rmse")
                ax.legend(fontsize=6, loc="best")
        fig.suptitle(f"G1 模型薄弱工况: {step}step · 蓝=plan 绿=executed (实线P50 虚线P95)",
                     y=0.99, fontsize=13)
    return fn


def g1c_error_3d(fig, bags_dfs):
    """误差 3D 体素图: (vx,|r|,|steer|) 填色 = plan_r_10step rmse 中位数, 点大小=样本数"""
    from mpl_toolkits.mplot3d import Axes3D
    alldf = _alldf(bags_dfs)
    if alldf.empty:
        return
    ax = fig.add_subplot(111, projection="3d")
    # 3D 分箱误差中位数
    d = alldf[["koopman_vel_u","abs_r","abs_steer",acc_key("plan","r",10,"rmse")]].dropna()
    try:
        xq = pd.qcut(d["koopman_vel_u"], 8, labels=False, duplicates="drop")
        yq = pd.qcut(d["abs_r"], 8, labels=False, duplicates="drop")
        zq = pd.qcut(d["abs_steer"], 8, labels=False, duplicates="drop")
    except Exception:
        ax.text(0.1, 0.5, 0.5, "数据不足", transform=ax.transAxes); return
    d = d.assign(xb=xq.values, yb=yq.values, zb=zq.values)
    g = d.groupby(["xb","yb","zb"])[acc_key("plan","r",10,"rmse")]
    med, cnt = g.median(), g.size()
    xs, ys, zs, sizes, colors = [], [], [], [], []
    for (ix,iy,iz), v in med.items():
        c = cnt.loc[(ix,iy,iz)]
        if c > 0:
            xs.append(d.groupby("xb")["koopman_vel_u"].median().iloc[ix] if ix < d.groupby("xb")["koopman_vel_u"].median().size else np.nan)
            ys.append(d.groupby("yb")["abs_r"].median().iloc[iy])
            zs.append(d.groupby("zb")["abs_steer"].median().iloc[iz])
            sizes.append(min(200, 10+c*2)); colors.append(v)
    sc = ax.scatter(xs, ys, zs, s=sizes, c=colors, cmap="RdYlBu_r", alpha=0.7)
    ax.set_xlabel("vx"); ax.set_ylabel("|r|"); ax.set_zlabel("|steer|")
    ax.set_title("误差 3D: 填色=plan_r_10step rmse, 大小=样本数")
    fig.colorbar(sc, ax=ax, shrink=0.6)


def g1c_heatmap(fig, bags_dfs):
    """兼容旧入口：输出当前的覆盖度与误差组合视图。"""
    g1c_all(fig, bags_dfs)


def _g1c_2d_common(fig, bags_dfs, zcol, cmap, title, show_cnt=False):
    """G1c 三个 2D 投影: (vx,|r|),(vx,|steer|),(|r|,|steer|)，填色 = 样本数(频次) 或 zcol 值(误差)"""
    alldf = _alldf(bags_dfs)
    if alldf.empty:
        return
    axs = fig.subplots(1, 3)
    pairs = [("koopman_vel_u", "abs_r"), ("koopman_vel_u", "abs_steer"), ("abs_r", "abs_steer")]
    for ax, (xcol, ycol) in zip(axs, pairs):
        if show_cnt:
            # 频次: 独立分箱计数, 不依赖 zcol
            d = alldf[[xcol, ycol]].dropna()
            if len(d) < 100:
                ax.text(0.5, 0.5, "数据不足", ha="center", transform=ax.transAxes); continue
            try:
                xq = pd.qcut(d[xcol], 10, labels=False, duplicates="drop")
                yq = pd.qcut(d[ycol], 10, labels=False, duplicates="drop")
            except Exception:
                ax.text(0.5, 0.5, "数据不足", ha="center", transform=ax.transAxes); continue
            d = d.assign(xb=xq.values, yb=yq.values)
            cnt = d.groupby(["yb", "xb"]).size()
            nx_eff = int(pd.Series(xq.unique()).max()) + 1
            ny_eff = int(pd.Series(yq.unique()).max()) + 1
            xc = d.groupby("xb")[xcol].median().reindex(range(nx_eff)).values
            yc = d.groupby("yb")[ycol].median().reindex(range(ny_eff)).values
            fill = np.zeros((ny_eff, nx_eff))
            for (iy, ix), v in cnt.items():
                fill[iy, ix] = v
            cm = cmap
        else:
            res = heatmap_2d(alldf, xcol, ycol, zcol, 10, 10)
            if res is None:
                ax.text(0.5, 0.5, "数据不足", ha="center", transform=ax.transAxes); continue
            xc, yc, grid, gcnt = res
            fill = grid
            cm = cmap
        im = ax.imshow(fill, aspect="auto", origin="lower", cmap=cm)
        ax.set_xticks(range(len(xc))); ax.set_xticklabels([f"{v:.1f}" for v in xc], fontsize=7)
        ax.set_yticks(range(len(yc))); ax.set_yticklabels([f"{v:.3f}" for v in yc], fontsize=7)
        ax.set_xlabel(_axlabel(xcol)); ax.set_ylabel(_axlabel(ycol))
        ax.set_title(f"{_axlabel(xcol)} × {_axlabel(ycol)}")
        fig.colorbar(im, ax=ax, fraction=0.05)
    fig.suptitle(title, y=0.99, fontsize=13)


def g1c_heatmap_2d_proj(fig, bags_dfs):
    """覆盖度 3 个 2D 投影（与 3D 同语义: 样本数填充）"""
    _g1c_2d_common(fig, bags_dfs, "koopman_vel_u", "YlOrRd",
                   "G1c 覆盖度 2D 投影: 填色=样本数 (与 3D 同语义)", show_cnt=True)


def g1c_error_2d(fig, bags_dfs):
    """误差 3 个 2D 投影: (vx,|r|),(vx,|steer|),(|r|,|steer|) 填色=plan_r_10step rmse 中位数"""
    _g1c_2d_common(fig, bags_dfs, acc_key("plan", "r", 10, "rmse"), "RdYlBu_r",
                   "G1c 误差 2D 投影: plan_r_10step rmse 中位数")


def _g2_metric_page(cname):
    """G2: laterr/phi_err 的 rms/p2p/hfe × 3 工况(vel_u,|r|,steer) 分箱。每页 1 指标 × 3 工况"""
    def fn(fig, bags_dfs):
        alldf = _alldf(bags_dfs)
        if alldf.empty:
            return
        zvars = [("koopman_vel_u", "vel_u"), ("abs_r", "|r|"), ("abs_steer", "|steer|")]
        axs = fig.subplots(1, 3)
        for ax, (zcol, zlab) in zip(axs, zvars):
            _binned_curve(ax, alldf, zcol, cname, COLORS[0], "P50")
            ax.set_title(f"{cname} vs {zlab}")
            ax.set_xlabel(zlab); ax.set_ylabel(cname); _legend_out(ax)
        fig.suptitle(f"G2 场景难度基线: {cname} · 实线 P50 / 虚线 P95", y=0.97, fontsize=13)
    return fn


def _g1c_3d_scatter(ax, alldf, zcol, cmap, title, use_cnt=False):
    """3D 联合分位网格：坐标为 bin 索引，刻度显式展示物理边界区间。"""
    from mpl_toolkits.mplot3d import Axes3D
    axes = ["koopman_vel_u", "abs_r", "abs_steer"]
    # 覆盖度不能因误差字段缺失而丢帧；误差图才需要 zcol 有效。
    d = alldf[axes + ([] if use_cnt else [zcol])].dropna().copy()
    if len(d) < 100:
        ax.text(0.1, 0.5, 0.5, "数据不足", transform=ax.transAxes); return
    try:
        xq, xedges = quantile_codes_and_edges(d["koopman_vel_u"], 8)
        yq, yedges = quantile_codes_and_edges(d["abs_r"], 8)
        zq, zedges = quantile_codes_and_edges(d["abs_steer"], 8)
    except Exception:
        ax.text(0.1, 0.5, 0.5, "数据不足", transform=ax.transAxes); return
    d = d.assign(xb=xq.values, yb=yq.values, zb=zq.values)
    g = d.groupby(["xb","yb","zb"])
    med = None if use_cnt else g[zcol].median()
    cnt = g.size()
    xs, ys, zs, sizes, colors = [], [], [], [], []
    for (ix,iy,iz), c in cnt.items():
        if c == 0: continue
        xs.append(ix); ys.append(iy); zs.append(iz)
        sizes.append(min(200, 10+c*2))
        colors.append(c if use_cnt else med.get((ix,iy,iz), np.nan))
    ax.scatter(xs, ys, zs, s=sizes, c=colors, cmap=cmap, alpha=0.7)
    xc = d.groupby("xb")["koopman_vel_u"].median().reindex(range(len(xedges) - 1)).values
    yc = d.groupby("yb")["abs_r"].median().reindex(range(len(yedges) - 1)).values
    zc = d.groupby("zb")["abs_steer"].median().reindex(range(len(zedges) - 1)).values
    ax.set_xticks(range(len(xc))); ax.set_xticklabels([f"{v:.1f}" for v in xc], fontsize=5)
    ax.set_yticks(range(len(yc))); ax.set_yticklabels([f"{v:.3f}" for v in yc], fontsize=5)
    ax.set_zticks(range(len(zc))); ax.set_zticklabels([f"{v:.3f}" for v in zc], fontsize=5)
    ax.set_xlabel("vx 分位中心 (m/s)"); ax.set_ylabel("|r| 分位中心 (rad/s)"); ax.set_zlabel("|steer| 分位中心 (rad)")
    ax.set_title(title, fontsize=10)


def g1c_all(fig, bags_dfs):
    """G1c 单页 4行×2列: 行1=3D(频次|误差), 行2-4=2D投影(频次|误差)"""
    alldf = _alldf(bags_dfs)
    if alldf.empty:
        return
    errcol = acc_key("plan", "r", 10, "rmse")
    grid_cols = ["koopman_vel_u", "abs_r", "abs_steer"]
    coverage_df = alldf.dropna(subset=grid_cols)
    error_df = coverage_df.dropna(subset=[errcol])
    # 成对比较时，频次与误差必须来自同一批可评分帧。
    comparison_df = error_df
    # 4行2列, 行1用3D(projection), 行2-4用2D
    from mpl_toolkits.mplot3d import Axes3D
    fig.set_size_inches(16, 20)
    # 行1: 两个3D
    ax1 = fig.add_subplot(4, 2, 1, projection="3d")
    ax2 = fig.add_subplot(4, 2, 2, projection="3d")
    _g1c_3d_scatter(ax1, comparison_df, errcol, frequency_heatmap_cmap(), "3D 可评分频次（分位网格）", use_cnt=True)
    _g1c_3d_scatter(ax2, comparison_df, errcol, error_heatmap_cmap(), "3D 误差（分位网格）")
    # 行2-4: 三对2D
    pairs = [("koopman_vel_u", "abs_r"), ("koopman_vel_u", "abs_steer"), ("abs_r", "abs_steer")]
    for r, (xcol, ycol) in enumerate(pairs, start=2):
        axl = fig.add_subplot(4, 2, (r-1)*2+1)
        axr = fig.add_subplot(4, 2, (r-1)*2+2)
        # 同一个 groupby 结果分别输出频次和误差；不允许左右图各自分箱。
        paired = paired_coverage_error_grid(comparison_df, xcol, ycol, errcol, 8, 8)
        if paired is not None:
            xcenters, ycenters, count_grid, error_grid = paired
            im = axl.imshow(np.ma.masked_invalid(count_grid), aspect="auto", origin="lower",
                            cmap=frequency_heatmap_cmap())
            axl.set_xticks(range(len(xcenters))); axl.set_xticklabels([f"{v:.1f}" for v in xcenters], fontsize=5, rotation=25)
            axl.set_yticks(range(len(ycenters))); axl.set_yticklabels([f"{v:.3f}" for v in ycenters], fontsize=5)
            axl.set_xlabel(_axlabel(xcol), fontsize=8); axl.set_ylabel(_axlabel(ycol), fontsize=8)
            axl.set_title("可评分频次（与右图同一分箱；深色=低频；空白=无样本）", fontsize=9)
            fig.colorbar(im, ax=axl, fraction=0.05)
        else:
            axl.text(0.5, 0.5, "数据不足", ha="center", transform=axl.transAxes)
        if paired is not None:
            im = axr.imshow(np.ma.masked_invalid(error_grid), aspect="auto", origin="lower",
                             cmap=error_heatmap_cmap())
            axr.set_xticks(range(len(xcenters))); axr.set_xticklabels([f"{v:.1f}" for v in xcenters], fontsize=5, rotation=25)
            axr.set_yticks(range(len(ycenters))); axr.set_yticklabels([f"{v:.3f}" for v in ycenters], fontsize=5)
            axr.set_xlabel(_axlabel(xcol), fontsize=8); axr.set_ylabel(_axlabel(ycol), fontsize=8)
            axr.set_title("误差（与左图同一分箱；空白=无样本）", fontsize=9)
            fig.colorbar(im, ax=axr, fraction=0.05)
        else:
            axr.text(0.5, 0.5, "数据不足", ha="center", transform=axr.transAxes)
    fig.suptitle("G1c 可评分覆盖度与误差：频次与误差使用同一批 plan_r_10step 有效帧；刻度为分位 bin 中心；空白=无样本",
                 y=0.98, fontsize=13)


def g3_transfer(fig, bags_dfs):
    """G3: 全局与 Z 分层后的 M-C 相关性，避免只报告混杂的整体相关。"""
    alldf = _alldf(bags_dfs)
    if alldf.empty:
        return
    ax, ax_strat = fig.subplots(1, 2)
    # 四类模型预测误差: {plan,executed} × {y, r} 的 10step rmse
    cats = [("plan_y_10", acc_key("plan","y",10,"rmse")), ("plan_r_10", acc_key("plan","r",10,"rmse")),
            ("exec_y_10", acc_key("executed","y",10,"rmse")), ("exec_r_10", acc_key("executed","r",10,"rmse"))]
    xs, rhos = [], []
    for i, (lab, col) in enumerate(cats):
        r = spearman(alldf[col], alldf["lat_err_rms"])
        if np.isfinite(r):
            xs.append(i); rhos.append(r)
    ax.bar(xs, rhos, color=COLORS[0], alpha=0.8)
    ax.axhline(0, color="grey", ls=":")
    ax.set_xticks(range(len(cats))); ax.set_xticklabels([c[0] for c in cats], fontsize=9)
    ax.set_title("G3a lat_err_rms 与四类模型预测误差的相关性 (Spearman)")
    ax.set_xlabel("模型预测误差 (10step rmse)"); ax.set_ylabel("rho"); ax.grid(axis="y", alpha=0.3)

    strata = stratified_spearman(alldf, acc_key("plan", "r", 10, "rmse"), "lat_err_rms")
    if strata.empty:
        ax_strat.text(0.5, 0.5, "各分层有效帧不足 60", ha="center", va="center",
                      transform=ax_strat.transAxes)
    else:
        labels = [f"{r.maneuver}\n{r.speed_bin}" for r in strata.itertuples()]
        colors = [{"straight": COLORS[0], "gentle": COLORS[1], "sharp": COLORS[2]}.get(r.maneuver, COLORS[3])
                  for r in strata.itertuples()]
        ax_strat.bar(range(len(strata)), strata["rho"], color=colors, alpha=0.85)
        ax_strat.axhline(0, color="grey", ls=":")
        ax_strat.set_xticks(range(len(strata))); ax_strat.set_xticklabels(labels, fontsize=7)
        ax_strat.set_ylabel("Spearman rho")
        ax_strat.set_title("G3b 分层: plan_r_10 × lat_err_rms\n(机动类型 × 速度分位)")
        for i, row in enumerate(strata.itertuples()):
            ax_strat.text(i, row.rho, f"n={row.n}", ha="center",
                          va="bottom" if row.rho >= 0 else "top", fontsize=6)


def g3_temporal_corr(fig, bags_dfs):
    """G3: lat_err 与同源不同时域(1/10/25step)预测误差的相关性"""
    alldf = _alldf(bags_dfs)
    if alldf.empty:
        return
    ax = fig.add_subplot(111)
    # 同源: plan_y / plan_r / executed_y / executed_r 的 1/10/25step
    lines = []
    for src in SOURCES:
        for st in ("y", "r"):
            rr = [spearman(alldf[acc_key(src, st, k, "rmse")], alldf["lat_err_rms"])
                  for k in STEPS]
            ax.plot(STEPS, rr, marker="o", lw=1.8, label=f"{src}_{st}")
    ax.axhline(0, color="grey", ls=":")
    ax.set_title("G3b lat_err_rms 与同源不同时域预测误差的相关性 (1/10/25step)")
    ax.set_xlabel("horizon step"); ax.set_ylabel("rho"); _legend_out(ax); ax.grid(alpha=0.3)


def g3_event_response(fig, bags_dfs):
    """G3 事件对照：高/低模型误差事件之后的控制误差，事件不跨 segment 合并。"""
    alldf = _alldf(bags_dfs)
    if alldf.empty:
        return
    mcol, ccol = acc_key("plan", "r", 10, "rmse"), "lat_err"
    high = event_response(alldf, mcol, ccol, 0.90, DEFAULT_CONF["event_min_gap_sec"],
                          DEFAULT_CONF["event_window_sec"])
    low = event_response(alldf, mcol, ccol, 0.10, DEFAULT_CONF["event_min_gap_sec"],
                         DEFAULT_CONF["event_window_sec"])
    ax = fig.add_subplot(111)
    values = [v for v in (low, high) if len(v)]
    labels = [f"低 M\nn={len(low)}", f"高 M\nn={len(high)}"]
    if not values:
        ax.text(0.5, 0.5, "非重叠事件不足", ha="center", va="center", transform=ax.transAxes)
        return
    # 使用 NaN 占位保持低/高两个位置固定，便于图间比较。
    plot_values = [low if len(low) else np.array([np.nan]), high if len(high) else np.array([np.nan])]
    box = ax.boxplot(plot_values, labels=labels, patch_artist=True, showfliers=False)
    for patch, color in zip(box["boxes"], [COLORS[0], COLORS[2]]):
        patch.set_facecolor(color); patch.set_alpha(0.7)
    ax.set_ylabel(f"事件起点后 {DEFAULT_CONF['event_window_sec']:.1f}s 的 lat_err RMS")
    ax.set_title("G3c 非重叠事件对照：模型误差尾部 vs 后续控制误差")
    ax.grid(axis="y", alpha=0.3)

def g4_score(fig, bags_dfs):
    alldf = pd.concat([df for _, df, _ in bags_dfs if len(df)], ignore_index=True)
    axs = fig.subplots(2, 2)

    ax = axs[0, 0]
    if len(alldf):
        for s in STATES:
            rr = [spearman(alldf[acc_key("plan", s, k, "rmse")], alldf["lat_err_rms"])
                  for k in [1, 10, 25]]
            ax.plot([1, 10, 25], rr, lw=1.8, label=f"{s}")
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
    if len(alldf):
        for i, (lab, col) in enumerate(comp):
            r = spearman(alldf[col], alldf["lat_err_rms"])
            if np.isfinite(r):
                xs.append(i); ys.append(r)
    ax.scatter(xs, ys, s=50, color=COLORS[0], alpha=0.9)
    ax.set_xticks(range(len(comp))); ax.set_xticklabels([c[0] for c in comp], fontsize=8)
    ax.axhline(0, color="grey", ls=":")
    ax.set_title("G4b 评分分量 × lat_err rms 相关 (负=分高误差小)")

    ax = axs[1, 0]
    if len(alldf) > 200:
        dd = alldf[["koopman_health_plan_accuracy_score", "lat_err_rms"]].dropna()
        rho = spearman(dd["koopman_health_plan_accuracy_score"], dd["lat_err_rms"])
        ax.scatter(dd["koopman_health_plan_accuracy_score"], dd["lat_err_rms"], s=4, alpha=0.3, color=COLORS[0],
                   label=f"统一 accuracy rho={rho:+.2f}")
        ax.set_title("G4d plan accuracy × lat_err rms")
        ax.set_xlabel("plan accuracy score"); ax.set_ylabel("lat_err rms"); _legend_out(ax)

    ax = axs[1, 1]
    res = xcorr_lag_curve(alldf, M_GAIN, "lat_err_rms", 3.0)
    if res:
        ax.plot(res[0], res[1], color=COLORS[3], lw=1.6)
        ax.axvline(0, ls=":", color="grey")
        ax.set_title("G4c gain_lp × lat_err rms 时滞互相关")
        ax.set_xlabel("lag (s)"); ax.set_ylabel("corr")
    fig.suptitle("G4 评分体系校验", y=0.97, fontsize=13)


def _table_page_with_notes(fig, df, title, notes):
    """表格页 + 底部注释文字（数据质量等）"""
    _table_page(fig, df, title)
    y_off = 0.03
    for note in notes:
        fig.axes[0].text(0.5, y_off, note, transform=fig.transFigure, ha="center",
                         va="bottom", fontsize=10, color="#29415f")
        y_off += 0.045


def _finalize_fig(fig):
    """页收尾: 有 suptitle 则顶部留 6% 给标题, 避免与子图 title 重叠"""
    if fig._suptitle is not None:
        fig.tight_layout(rect=[0, 0, 1, 0.94])
    else:
        fig.tight_layout()

def _pages_spec(bags_dfs):
    """页面定义。不分包：所有数据合并为整体分析。
    统一 figsize: 单行图表页 (13.4, 3.9), 双行 2x2 页 (13.4, 8.4)。"""
    W_1ROW, H_1ROW = 13.4, 3.9      # 单行图表页
    W_GRID4 = 13.4, 10.0            # 4×3 矩阵页(G1)
    W_2ROW, H_2ROW = 13.4, 8.4      # 双行 2x2 页(G4)
    pages = [
        ("Overview", _table_size(
            pd.DataFrame([{"all": "汇总"}])), ov_table),
    ]
    # G1: 1/10step 共 2 页，每页叠加 plan/executed，含 4 状态 × 3 工况。
    for step in (1, 10):
        pages.append((f"G1-{step}step", W_GRID4, _g1_step_page(step)))
    # G1c: 3D 覆盖度/误差 + 三个 2D 投影
    pages.append(("G1c", (16.0, 20.0), g1c_all))
    # G2: laterr/phi_err 各 3 指标 × 3 工况 = 18 图
    for cname in ["lat_err_rms", "lat_err_p2p", "lat_err_hfe",
                  "phi_err_rms", "phi_err_p2p", "phi_err_hfe"]:
        pages.append((f"G2-{cname}", (W_1ROW, H_1ROW), _g2_metric_page(cname)))
    # G3: 相关性（四类误差 / 同源时域）
    pages.append(("G3-相关性", (13.4, 5.2), g3_transfer))
    pages.append(("G3-时域相关", (10.0, 5.0), g3_temporal_corr))
    pages.append(("G3-事件对照", (8.0, 5.0), g3_event_response))
    pages.append(("G4-Score", (W_2ROW, H_2ROW), g4_score))
    return pages


def build_report_pages(bags_dfs, out_dir, dpi=REPORT_DPI):
    """输出分页 PNG, 返回 [(title, path)]"""
    results = []
    for i, (title, size, fn) in enumerate(_pages_spec(bags_dfs)):
        fig = plt.figure(figsize=size)
        fn(fig, bags_dfs)
        _finalize_fig(fig)
        p = os.path.join(out_dir, f"p{i+1:02d}_{title}.png")
        fig.savefig(p, dpi=dpi); plt.close(fig)
        results.append((title, p))
    return results


def build_report(bags_dfs, out_path, dpi=REPORT_DPI):
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
        fig.savefig(p, dpi=dpi); plt.close(fig); pngs.append(p)
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
_ASCII_NAME = {"Overview": "overview",
               "G1-1step": "g1-1", "G1-10step": "g1-10",
               "G1c": "g1c",
               "G2-lat_err_rms": "g2-laterr-rms", "G2-lat_err_p2p": "g2-laterr-p2p",
               "G2-lat_err_hfe": "g2-laterr-hfe", "G2-phi_err_rms": "g2-phierr-rms",
               "G2-phi_err_p2p": "g2-phierr-p2p", "G2-phi_err_hfe": "g2-phierr-hfe",
               "G3-相关性": "g3-corr", "G3-时域相关": "g3-tcorr",
               "G3-事件对照": "g3-events",
               "G4-Score": "g4-score"}

_SECTIONS = [
    ("overview", "总览", "全部数据汇总的有效帧、健康分、误差与 P99 命中。",
     ["Overview"], "1col"),
    ("g1", "模型薄弱工况", "plan/executed 在 1/10step 的四状态预测误差随速度/横摆率/转角的分位分布。",
     ["G1-1step", "G1-10step"], "1col"),
    ("g1c", "覆盖度与误差", "(vx, |r|, |steer|) 的三维覆盖度、二维投影与 plan r·10step 误差分箱。",
     ["G1c"], "1col"),
    ("g2", "场景难度基线", "laterr/phi_err 的 rms/p2p/hfe 随速度/横摆率/转角的分布。",
     ["G2-lat_err_rms", "G2-lat_err_p2p", "G2-lat_err_hfe",
      "G2-phi_err_rms", "G2-phi_err_p2p", "G2-phi_err_hfe"], "1col"),
    ("g3", "传导关联", "全局与分层相关、时域趋势及非重叠事件对照；只表达关联，不推断因果。",
     ["G3-相关性", "G3-时域相关", "G3-事件对照"], "2col"),
    ("g4", "评分体系校验", "horizon 相关趋势、分量外部效度、gain 先导性、双权重仲裁。",
     ["G4-Score"], "1col"),
]

CH_CLASS = {"overview": "ch-z", "overview-ts": "ch-m", "g1": "ch-z",
            "g1c": "ch-m", "g2": "ch-c", "g3": "ch-z",
            "g4": "ch-m", "advanced": "ch-c"}

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


def _plotly_layout(fig, title, height):
    fig.update_layout(template="plotly_white", title={"text": title, "x": 0.5}, height=height,
                      margin={"l": 62, "r": 32, "t": 70, "b": 58}, hovermode="closest",
                      paper_bgcolor="#ffffff", plot_bgcolor="#ffffff")
    fig.update_xaxes(showgrid=True, gridcolor="#e5e7eb", zeroline=False)
    fig.update_yaxes(showgrid=True, gridcolor="#e5e7eb", zeroline=False)
    return fig


def _plotly_add_binned_traces(fig, df, xcol, ycol, row, col, prefix, color, showlegend,
                              separate_quantile_legend=False):
    """向 Plotly 子图加入 P50/P95 分位曲线；hover 默认展示完整 x/y 值。"""
    res = quantile_binned_stat(df, xcol, ycol, 8)
    if res is None:
        return
    center, p50, p95, count = res
    fig.add_trace({"type": "scatter", "mode": "lines+markers", "name": f"{prefix} P50",
                   "legendgroup": f"{prefix} P50" if separate_quantile_legend else prefix,
                   "showlegend": showlegend, "x": center, "y": p50,
                   "line": {"color": color, "width": 2}, "marker": {"size": 5},
                   "customdata": count,
                   "hovertemplate": f"{prefix} P50<br>x=%{{x:.5g}}<br>rmse=%{{y:.5g}}"
                                    "<br>n=%{customdata}<extra></extra>"}, row=row, col=col)
    fig.add_trace({"type": "scatter", "mode": "lines", "name": f"{prefix} P95",
                   "legendgroup": f"{prefix} P95" if separate_quantile_legend else prefix,
                   "showlegend": showlegend if separate_quantile_legend else False, "x": center, "y": p95,
                   "line": {"color": color, "width": 1, "dash": "dash"},
                   "customdata": count,
                   "hovertemplate": f"{prefix} P95<br>x=%{{x:.5g}}<br>rmse=%{{y:.5g}}"
                                    "<br>n=%{customdata}<extra></extra>"}, row=row, col=col)


def _plotly_g1_figure(alldf, step):
    from plotly.subplots import make_subplots
    zvars = [("koopman_vel_u", "vel_u"), ("abs_r", "|r|"), ("abs_steer", "|steer|")]
    fig = make_subplots(rows=4, cols=3,
                        subplot_titles=[f"{state} {step}step vs {zlabel}" for state in STATES for _, zlabel in zvars],
                        horizontal_spacing=0.08, vertical_spacing=0.10)
    for row, state in enumerate(STATES, 1):
        for col, (zcol, zlabel) in enumerate(zvars, 1):
            showlegend = row == 1 and col == 1
            _plotly_add_binned_traces(fig, alldf, zcol, acc_key("plan", state, step, "rmse"),
                                      row, col, "plan", COLORS[0], showlegend)
            _plotly_add_binned_traces(fig, alldf, zcol, acc_key("executed", state, step, "rmse"),
                                      row, col, "executed", COLORS[1], showlegend)
            fig.update_xaxes(title_text=zlabel, row=row, col=col)
            fig.update_yaxes(title_text="rmse", row=row, col=col)
    return _plotly_layout(fig,
                          f"G1 模型薄弱工况: {step}step · 蓝=plan 绿=executed (实线P50 虚线P95)",
                          1120)


def _plotly_3d_data(df, errcol):
    axes = ["koopman_vel_u", "abs_r", "abs_steer"]
    d = df[axes + [errcol]].dropna().copy()
    if len(d) < 100:
        return None
    xq, _ = quantile_codes_and_edges(d[axes[0]], 8)
    yq, _ = quantile_codes_and_edges(d[axes[1]], 8)
    zq, _ = quantile_codes_and_edges(d[axes[2]], 8)
    d = d.assign(xb=xq.values, yb=yq.values, zb=zq.values)
    group = d.groupby(["xb", "yb", "zb"])
    centers = group[axes].median()
    counts = group.size()
    errors = group[errcol].median()
    return centers, counts, errors


def _plotly_g1c_3d_figure(df, errcol, mode):
    from plotly import graph_objects as go
    data = _plotly_3d_data(df, errcol)
    fig = go.Figure()
    if data is None:
        fig.add_annotation(text="数据不足", x=0.5, y=0.5, showarrow=False)
        return _plotly_layout(fig, "G1c 3D", 600)
    centers, counts, errors = data
    count_values, error_values = counts.to_numpy(float), errors.to_numpy(float)
    marker_size = np.clip(4 + np.sqrt(count_values) * 1.2, 5, 24)
    is_count = mode == "count"
    colors = count_values if is_count else error_values
    colorbar_title = "样本数 n" if is_count else "plan_r_10 rmse"
    colorscale = "YlOrRd" if is_count else "Blues"
    custom = np.column_stack([count_values, error_values])
    fig.add_trace(go.Scatter3d(
        x=centers["koopman_vel_u"], y=centers["abs_r"], z=centers["abs_steer"],
        mode="markers", customdata=custom,
        marker={"size": marker_size, "color": colors, "colorscale": colorscale,
                "opacity": 0.82, "colorbar": {"title": colorbar_title}},
        hovertemplate="vx=%{x:.5g} m/s<br>|r|=%{y:.5g} rad/s<br>|steer|=%{z:.5g} rad"
                      "<br>n=%{customdata[0]:.0f}<br>plan_r_10 rmse=%{customdata[1]:.5g}<extra></extra>"))
    fig.update_layout(scene={"xaxis_title": "vx (m/s)", "yaxis_title": "|r| (rad/s)",
                             "zaxis_title": "|steer| (rad)"})
    label = "联合可评分频次" if is_count else "联合误差"
    return _plotly_layout(fig, f"G1c 3D {label}（拖拽旋转；悬停查看 bin 中心、n 与误差）", 620)


def _plotly_g1c_2d_figure(df, errcol, xcol, ycol):
    from plotly import graph_objects as go
    from plotly.subplots import make_subplots
    paired = paired_coverage_error_grid(df, xcol, ycol, errcol, 8, 8)
    title = f"G1c {_axlabel(xcol)} × {_axlabel(ycol)}"
    fig = make_subplots(rows=1, cols=2, subplot_titles=("可评分频次", "plan_r_10step rmse"),
                        horizontal_spacing=0.12)
    if paired is None:
        fig.add_annotation(text="数据不足", x=0.5, y=0.5, showarrow=False)
        return _plotly_layout(fig, title, 460)
    xc, yc, count_grid, error_grid = paired
    fig.add_trace(go.Heatmap(x=xc, y=yc, z=count_grid, colorscale="YlOrRd",
                             colorbar={"title": "n", "x": 0.45},
                             hovertemplate=f"{_axlabel(xcol)}=%{{x:.5g}}<br>{_axlabel(ycol)}=%{{y:.5g}}"
                                           "<br>n=%{z:.0f}<extra></extra>"), row=1, col=1)
    fig.add_trace(go.Heatmap(x=xc, y=yc, z=error_grid, customdata=count_grid, colorscale="Blues",
                             colorbar={"title": "rmse", "x": 1.02},
                             hovertemplate=f"{_axlabel(xcol)}=%{{x:.5g}}<br>{_axlabel(ycol)}=%{{y:.5g}}"
                                           "<br>rmse=%{z:.5g}<br>n=%{customdata:.0f}<extra></extra>"), row=1, col=2)
    for col in (1, 2):
        fig.update_xaxes(title_text=_axlabel(xcol), row=1, col=col)
        fig.update_yaxes(title_text=_axlabel(ycol), row=1, col=col)
    return _plotly_layout(fig, title, 500)


def _plotly_overview_figure(bags_dfs):
    """Overview 与静态版保持同一汇总表字段。"""
    from plotly import graph_objects as go
    alldf = _alldf(bags_dfs)
    h = alldf["koopman_health_score_total"] if len(alldf) else pd.Series(dtype=float)
    valid_h = h[h >= 0]
    values = [
        "汇总",
        int(sum(c["_n_frame"] for _, _, c in bags_dfs)),
        int(sum(c["_n_active"] for _, _, c in bags_dfs)),
        _q(valid_h, 0.05) if len(valid_h) else np.nan,
        float(np.sqrt((alldf["lat_err"] ** 2).mean())) if len(alldf) else np.nan,
        int(alldf["koopman_health_executed_error_p99_hit"].sum()) if len(alldf) else 0,
        float((h < 0).mean()) if len(h) else np.nan,
    ]
    headers = ["all", "frames", "valid", "health_p05", "laterr_rms",
               "p99_overshoot_frames", "health_invalid_pct"]
    shown = [v if isinstance(v, str) or isinstance(v, int) else
             ("—" if not np.isfinite(v) else f"{v:.3f}") for v in values]
    fig = go.Figure(go.Table(
        header={"values": headers, "align": "center", "fill_color": "#e8f0fc",
                "line_color": "#c8d6e8", "font": {"color": "#10213d", "size": 13}},
        cells={"values": [[v] for v in shown], "align": "center", "fill_color": "#f7faff",
               "line_color": "#c8d6e8", "font": {"color": "#10213d", "size": 13}, "height": 34}))
    return _plotly_layout(fig, "Overview: 数据汇总", 260)


def _plotly_g1c_page(df, errcol):
    """G1c 静态页的等价交互版：单卡 4×2（3D 两图 + 三组 2D 成对投影）。"""
    from plotly import graph_objects as go
    from plotly.subplots import make_subplots
    specs = [[{"type": "scene"}, {"type": "scene"}],
             [{"type": "heatmap"}, {"type": "heatmap"}],
             [{"type": "heatmap"}, {"type": "heatmap"}],
             [{"type": "heatmap"}, {"type": "heatmap"}]]
    subtitles = ["3D 可评分频次（分位网格）", "3D 误差（分位网格）"]
    pairs = [("koopman_vel_u", "abs_r"), ("koopman_vel_u", "abs_steer"),
             ("abs_r", "abs_steer")]
    compact_label = {"koopman_vel_u": "vx (m/s)", "abs_r": "|r| (rad/s)",
                     "abs_steer": "|steer| (rad)"}
    for xcol, ycol in pairs:
        # 列间距扩大后使用紧凑标题，防止长中文轴名侵入色条和相邻列。
        subtitles.extend([f"{compact_label[xcol]} × {compact_label[ycol]} · 频次",
                          f"{compact_label[xcol]} × {compact_label[ycol]} · 误差"])
    # 由相同的行权重推导色条 domain，保证每条色条与所属二维热力图严格对齐。
    row_weights, vertical_spacing, horizontal_spacing = [1.25, 1.05, 1.05, 1.05], 0.06, 0.20
    usable_height = 1.0 - vertical_spacing * (len(row_weights) - 1)
    normalized_heights = [usable_height * weight / sum(row_weights) for weight in row_weights]
    row_domains, top = {}, 1.0
    for row, height in enumerate(normalized_heights, start=1):
        row_domains[row] = (top - height, top)
        top -= height + vertical_spacing
    colorbar_y = {row: (low + high) / 2 for row, (low, high) in row_domains.items()}
    colorbar_len = {row: (high - low) * 0.88 for row, (low, high) in row_domains.items()}
    # 两列各自紧贴色条；不要把左列色条置于 0.40 的整段列间空白中央。
    left_colorbar_x = (1.0 - horizontal_spacing) / 2.0 + 0.015
    right_colorbar_x = 1.01
    fig = make_subplots(rows=4, cols=2, specs=specs, subplot_titles=subtitles,
                        horizontal_spacing=horizontal_spacing, vertical_spacing=vertical_spacing,
                        row_heights=row_weights)
    data = _plotly_3d_data(df, errcol)
    if data is None:
        fig.add_annotation(text="数据不足", x=0.5, y=0.95, xref="paper", yref="paper", showarrow=False)
    else:
        centers, counts, errors = data
        count_values, error_values = counts.to_numpy(float), errors.to_numpy(float)
        marker_size = np.clip(4 + np.sqrt(count_values) * 1.2, 5, 24)
        # 采用分位 bin 索引作为空间坐标（静态版同样如此），避免 vx 与 |r|/|steer|
        # 量纲相差数百倍时把三维点压缩到一条窄带；刻度与悬停仍展示真实 bin 中心值。
        xb = centers.index.get_level_values("xb").to_numpy(float)
        yb = centers.index.get_level_values("yb").to_numpy(float)
        zb = centers.index.get_level_values("zb").to_numpy(float)
        # 轴刻度必须从同一批原始帧按单轴分位箱计算；不能再对 3D 小格中位数取中位数，
        # 否则该刻度会与 2D 投影的 bin 中心不同。
        def bin_centers(col):
            codes, _ = quantile_codes_and_edges(df[col], 8)
            return pd.DataFrame({"bin": codes.to_numpy(), "value": df[col].to_numpy()}) \
                .groupby("bin")["value"].median().sort_index()
        xc, yc, zc = (bin_centers("koopman_vel_u"), bin_centers("abs_r"),
                      bin_centers("abs_steer"))
        custom = np.column_stack([
            xc.reindex(xb.astype(int)).to_numpy(), yc.reindex(yb.astype(int)).to_numpy(),
            zc.reindex(zb.astype(int)).to_numpy(), count_values, error_values])
        common = {"x": xb, "y": yb, "z": zb, "mode": "markers", "customdata": custom,
                  "hovertemplate": "vx 分位中心=%{customdata[0]:.5g} m/s<br>|r| 分位中心=%{customdata[1]:.5g} rad/s"
                                   "<br>|steer| 分位中心=%{customdata[2]:.5g} rad<br>n=%{customdata[3]:.0f}"
                                   "<br>plan_r_10step rmse=%{customdata[4]:.5g}<extra></extra>"}
        fig.add_trace(go.Scatter3d(**common, marker={
            "size": marker_size, "color": count_values,
            "colorscale": [[0, "#fff3bf"], [0.45, "#f59e0b"], [1, "#b91c1c"]],
            "opacity": 0.82,
            "colorbar": {"title": "样本数 n", "x": left_colorbar_x, "y": colorbar_y[1], "len": colorbar_len[1]}}), row=1, col=1)
        fig.add_trace(go.Scatter3d(**common, marker={
            "size": marker_size, "color": error_values,
            "colorscale": [[0, "#dbeafe"], [0.45, "#60a5fa"], [1, "#0b3b8c"]],
            "opacity": 0.82,
            "colorbar": {"title": "rmse", "x": right_colorbar_x, "y": colorbar_y[1], "len": colorbar_len[1]}}), row=1, col=2)
    fig.update_layout(
        scene={"aspectmode": "cube",
               "xaxis": {"title": "vx bin 中心", "tickmode": "array", "tickvals": xc.index.tolist(),
                         "ticktext": [f"{v:.3g}" for v in xc]},
               "yaxis": {"title": "|r| bin 中心", "tickmode": "array", "tickvals": yc.index.tolist(),
                         "ticktext": [f"{v:.3g}" for v in yc]},
               "zaxis": {"title": "|steer| bin 中心", "tickmode": "array", "tickvals": zc.index.tolist(),
                         "ticktext": [f"{v:.3g}" for v in zc]}},
        scene2={"aspectmode": "cube",
                "xaxis": {"title": "vx bin 中心", "tickmode": "array", "tickvals": xc.index.tolist(),
                          "ticktext": [f"{v:.3g}" for v in xc]},
                "yaxis": {"title": "|r| bin 中心", "tickmode": "array", "tickvals": yc.index.tolist(),
                          "ticktext": [f"{v:.3g}" for v in yc]},
                "zaxis": {"title": "|steer| bin 中心", "tickmode": "array", "tickvals": zc.index.tolist(),
                          "ticktext": [f"{v:.3g}" for v in zc]}})
    for row, (xcol, ycol) in enumerate(pairs, start=2):
        paired = paired_coverage_error_grid(df, xcol, ycol, errcol, 8, 8)
        if paired is None:
            fig.add_annotation(text="数据不足", x=0.5, y=0.5, showarrow=False, row=row, col=1)
            fig.add_annotation(text="数据不足", x=0.5, y=0.5, showarrow=False, row=row, col=2)
            continue
        xc, yc, count_grid, error_grid = paired
        custom = np.empty(count_grid.shape + (4,), dtype=object)
        for iy in range(len(yc)):
            for ix in range(len(xc)):
                custom[iy, ix] = [xc[ix], yc[iy], count_grid[iy, ix], error_grid[iy, ix]]
        xi, yi = list(range(len(xc))), list(range(len(yc)))
        fig.add_trace(go.Heatmap(
            x=xi, y=yi, z=count_grid, customdata=custom,
            colorscale=[[0, "#fff3bf"], [0.45, "#f59e0b"], [1, "#b91c1c"]],
            colorbar={"title": "n", "x": left_colorbar_x, "y": colorbar_y[row], "len": colorbar_len[row]},
            hovertemplate=f"{_axlabel(xcol)}=%{{customdata[0]:.5g}}<br>{_axlabel(ycol)}=%{{customdata[1]:.5g}}"
                          "<br>n=%{customdata[2]:.0f}<br>rmse=%{customdata[3]:.5g}<extra></extra>"), row=row, col=1)
        fig.add_trace(go.Heatmap(
            x=xi, y=yi, z=error_grid, customdata=custom,
            colorscale=[[0, "#dbeafe"], [0.45, "#60a5fa"], [1, "#0b3b8c"]],
            colorbar={"title": "rmse", "x": right_colorbar_x, "y": colorbar_y[row], "len": colorbar_len[row]},
            hovertemplate=f"{_axlabel(xcol)}=%{{customdata[0]:.5g}}<br>{_axlabel(ycol)}=%{{customdata[1]:.5g}}"
                          "<br>rmse=%{customdata[3]:.5g}<br>n=%{customdata[2]:.0f}<extra></extra>"), row=row, col=2)
        for col in (1, 2):
            fig.update_xaxes(tickmode="array", tickvals=xi,
                             ticktext=[f"{v:.3g}" for v in xc], title_text=compact_label[xcol], row=row, col=col)
            fig.update_yaxes(tickmode="array", tickvals=yi,
                             ticktext=[f"{v:.3g}" for v in yc], title_text=compact_label[ycol], row=row, col=col)
    fig = _plotly_layout(
        fig,
        "G1c 可评分覆盖度与误差：频次与误差使用同一批 plan_r_10step 有效帧；刻度为分位 bin 中心；空白=无样本",
        2300)
    fig.update_layout(margin={"l": 72, "r": 106, "t": 98, "b": 74})
    return fig


def _plotly_g2_figure(alldf, cname):
    from plotly.subplots import make_subplots
    zvars = [("koopman_vel_u", "vel_u"), ("abs_r", "|r|"), ("abs_steer", "steer")]
    fig = make_subplots(rows=1, cols=3, subplot_titles=[label for _, label in zvars])
    for col, (zcol, zlabel) in enumerate(zvars, 1):
        _plotly_add_binned_traces(fig, alldf, zcol, cname, 1, col, "control", COLORS[2], col == 1,
                                  separate_quantile_legend=True)
        fig.update_xaxes(title_text=zlabel, row=1, col=col)
        fig.update_yaxes(title_text=cname, row=1, col=col)
    return _plotly_layout(fig, f"G2 场景难度基线: {cname} · 实线 P50 / 虚线 P95", 440)


def _plotly_g3_transfer_figure(alldf):
    """G3 相关性：全局相关柱图 + 机动类型/速度二维分层图。"""
    from plotly import graph_objects as go
    from plotly.subplots import make_subplots
    fig = make_subplots(rows=1, cols=2, subplot_titles=(
        "G3a lat_err_rms 与四类模型预测误差的相关性 (Spearman)",
        "G3b 分层: plan_r_10 × lat_err_rms<br>(机动类型 × 速度分位)"))
    cats = [("plan_y_10", acc_key("plan", "y", 10, "rmse")),
            ("plan_r_10", acc_key("plan", "r", 10, "rmse")),
            ("exec_y_10", acc_key("executed", "y", 10, "rmse")),
            ("exec_r_10", acc_key("executed", "r", 10, "rmse"))]
    labels, rhos, counts = [], [], []
    for label, col in cats:
        rho = spearman(alldf[col], alldf["lat_err_rms"])
        if np.isfinite(rho):
            labels.append(label); rhos.append(rho)
            counts.append(int(alldf[[col, "lat_err_rms"]].dropna().shape[0]))
    fig.add_trace(go.Bar(x=labels, y=rhos, customdata=counts, marker={"color": COLORS[0]},
                         hovertemplate="metric=%{x}<br>rho=%{y:.4f}<br>n=%{customdata}<extra></extra>"), row=1, col=1)
    fig.add_hline(y=0, line_dash="dot", line_color="grey", row=1, col=1)
    fig.update_xaxes(title_text="模型预测误差 (10step rmse)", row=1, col=1)
    fig.update_yaxes(title_text="rho", row=1, col=1)

    # 速度分位必须基于全部聚合帧统一划分；若沿用每个 bag 的 speed_bin，二维格子的
    # 横轴含义会随 bag 改变。x 用等距索引，tick 文本展示实际速度分位中心。
    err_col = acc_key("plan", "r", 10, "rmse")
    d = alldf[["maneuver", "koopman_vel_u", err_col, "lat_err_rms"]].dropna().copy()
    maneuver_order = ["straight", "gentle", "sharp"]
    d = d[d["maneuver"].isin(maneuver_order)]
    try:
        speed_code, speed_edges = pd.qcut(d["koopman_vel_u"], 4, labels=False,
                                          retbins=True, duplicates="drop")
    except (ValueError, IndexError):
        speed_code, speed_edges = None, None
    if speed_code is None or len(speed_edges) < 2:
        fig.add_annotation(text="速度分位不足，无法生成二维分层图", x=0.5, y=0.5,
                           showarrow=False, row=1, col=2)
    else:
        d["speed_code"] = np.asarray(speed_code, dtype=int)
        speed_centers = (speed_edges[:-1] + speed_edges[1:]) / 2
        rho_grid = np.full((len(maneuver_order), len(speed_centers)), np.nan)
        count_grid = np.zeros_like(rho_grid, dtype=int)
        for (maneuver, code), part in d.groupby(["maneuver", "speed_code"], observed=True):
            iy = maneuver_order.index(maneuver)
            ix = int(code)
            n = len(part)
            count_grid[iy, ix] = n
            # 有数据但样本不足的格子同样留白，避免小样本相关系数造成误导。
            if n >= 60:
                rho_grid[iy, ix] = spearman(part[err_col], part["lat_err_rms"])
        customdata = np.empty(rho_grid.shape + (2,), dtype=object)
        for iy in range(len(maneuver_order)):
            for ix, center in enumerate(speed_centers):
                customdata[iy, ix] = [center, count_grid[iy, ix]]
        fig.add_trace(go.Heatmap(
            x=list(range(len(speed_centers))), y=maneuver_order, z=rho_grid,
            customdata=customdata, colorscale="RdBu", zmin=-1, zmax=1, zmid=0,
            colorbar={"title": "Spearman rho"}, hoverongaps=False,
            hovertemplate="maneuver=%{y}<br>vx 分位中心=%{customdata[0]:.5g} m/s"
                          "<br>rho=%{z:.4f}<br>n=%{customdata[1]:.0f}<extra></extra>"), row=1, col=2)
        fig.update_xaxes(tickmode="array", tickvals=list(range(len(speed_centers))),
                         ticktext=[f"{center:.3g}" for center in speed_centers],
                         title_text="vx 分位中心 (m/s)", row=1, col=2)
        fig.update_yaxes(title_text="机动类型", categoryorder="array",
                         categoryarray=maneuver_order, row=1, col=2)
    return _plotly_layout(fig, "G3 传导关联", 540)


def _plotly_g3_temporal_figure(alldf):
    """对应静态版 G3-时域相关。"""
    from plotly import graph_objects as go
    fig = go.Figure()
    for src in SOURCES:
        for state in ("y", "r"):
            values = [spearman(alldf[acc_key(src, state, step, "rmse")], alldf["lat_err_rms"])
                      for step in STEPS]
            fig.add_trace(go.Scatter(x=STEPS, y=values, mode="lines+markers", name=f"{src}_{state}",
                                     hovertemplate="step=%{x}<br>rho=%{y:.4f}<extra>%{fullData.name}</extra>"))
    fig.add_hline(y=0, line_dash="dot", line_color="grey")
    fig.update_xaxes(title_text="horizon step"); fig.update_yaxes(title_text="rho")
    return _plotly_layout(fig, "G3b lat_err_rms 与同源不同时域预测误差的相关性 (1/10/25step)", 440)


def _plotly_g3_event_figure(alldf):
    """对应静态版 G3-事件对照。"""
    from plotly import graph_objects as go
    mcol, ccol = acc_key("plan", "r", 10, "rmse"), "lat_err"
    low = event_response(alldf, mcol, ccol, 0.10, DEFAULT_CONF["event_min_gap_sec"],
                         DEFAULT_CONF["event_window_sec"])
    high = event_response(alldf, mcol, ccol, 0.90, DEFAULT_CONF["event_min_gap_sec"],
                          DEFAULT_CONF["event_window_sec"])
    fig = go.Figure()
    if not len(low) and not len(high):
        fig.add_annotation(text="非重叠事件不足", x=0.5, y=0.5, showarrow=False)
    else:
        for label, values, color in [(f"低 M<br>n={len(low)}", low, COLORS[0]),
                                     (f"高 M<br>n={len(high)}", high, COLORS[2])]:
            if len(values):
                fig.add_trace(go.Box(y=values, name=label, boxpoints=False, marker={"color": color},
                                     line={"color": color}, fillcolor=color, opacity=0.7,
                                     hovertemplate=f"{label}<br>lat_err RMS=%{{y:.5g}}<extra></extra>"))
    fig.update_yaxes(title_text=f"事件起点后 {DEFAULT_CONF['event_window_sec']:.1f}s 的 lat_err RMS")
    return _plotly_layout(fig, "G3c 非重叠事件对照：模型误差尾部 vs 后续控制误差", 440)


def _plotly_g4_figure(alldf):
    from plotly import graph_objects as go
    from plotly.subplots import make_subplots
    fig = make_subplots(rows=2, cols=2, subplot_titles=(
        "G4a horizon 相关: plan rmse × lat_err rms",
        "G4b 评分分量 × lat_err rms 相关 (负=分高误差小)",
        "G4d plan accuracy × lat_err rms",
        "G4c gain_lp × lat_err rms 时滞互相关"))
    for state in STATES:
        values = [spearman(alldf[acc_key("plan", state, step, "rmse")], alldf["lat_err_rms"])
                  for step in STEPS]
        fig.add_trace(go.Scatter(x=STEPS, y=values, mode="lines+markers", name=state,
                                 hovertemplate="step=%{x}<br>rho=%{y:.4f}<extra>%{fullData.name}</extra>"), row=1, col=1)
    fig.add_hline(y=0, line_dash="dot", line_color="grey", row=1, col=1)
    components = [("acc_y", "koopman_health_plan_accuracy_y_score"),
                  ("acc_phi", "koopman_health_plan_accuracy_phi_score"),
                  ("acc_vy", "koopman_health_plan_accuracy_vy_score"),
                  ("acc_r", "koopman_health_plan_accuracy_r_score"),
                  ("plan_stab", "koopman_health_plan_stability_score"), ("gain_lp", M_GAIN)]
    labels, values = zip(*[(label, spearman(alldf[col], alldf["lat_err_rms"])) for label, col in components])
    fig.add_trace(go.Scatter(x=list(range(len(labels))), y=values, mode="markers", name="rho",
                             marker={"size": 10, "color": COLORS[0]}, customdata=labels,
                             hovertemplate="component=%{customdata}<br>rho=%{y:.4f}<extra></extra>"), row=1, col=2)
    fig.update_xaxes(tickmode="array", tickvals=list(range(len(labels))), ticktext=labels, row=1, col=2)
    fig.add_hline(y=0, line_dash="dot", line_color="grey", row=1, col=2)
    accuracy = alldf[["koopman_health_plan_accuracy_score", "lat_err_rms"]].dropna()
    accuracy_rho = spearman(accuracy["koopman_health_plan_accuracy_score"], accuracy["lat_err_rms"])
    fig.add_trace(go.Scattergl(x=accuracy["koopman_health_plan_accuracy_score"], y=accuracy["lat_err_rms"], mode="markers",
                               name=f"统一 accuracy rho={accuracy_rho:+.2f}",
                               marker={"size": 4, "opacity": 0.3, "color": COLORS[0]},
                               hovertemplate="plan accuracy=%{x:.4f}<br>lat_err=%{y:.5g}<extra></extra>"), row=2, col=1)
    fig.update_xaxes(title_text="plan accuracy score", row=2, col=1)
    fig.update_yaxes(title_text="lat_err rms", row=2, col=1)
    lag = xcorr_lag_curve(alldf, M_GAIN, "lat_err_rms", DEFAULT_CONF["xcorr_max_lag_sec"])
    if lag is not None:
        fig.add_trace(go.Scatter(x=lag[0], y=lag[1], mode="lines+markers", name="gain corr",
                                 hovertemplate="lag=%{x:.3f}s<br>corr=%{y:.5f}<extra></extra>"), row=2, col=2)
    return _plotly_layout(fig, "G4 评分体系校验", 920)


def _future_segment_value(df, col, horizon_sec):
    """取同一连续段中 horizon_sec 后的值，不跨 bag/segment 拼接。"""
    out = pd.Series(np.nan, index=df.index, dtype=float)
    group_col = "segment_id" if "segment_id" in df else "segment"
    for _, part in df.groupby(group_col, sort=False):
        part = part[["t", col]].dropna().sort_values("t")
        if len(part) < 2:
            continue
        ts = part["t"].to_numpy(float)
        future_pos = np.searchsorted(ts, ts + horizon_sec, side="left")
        valid = future_pos < len(part)
        if valid.any():
            out.loc[part.index[valid]] = part[col].to_numpy(float)[future_pos[valid]]
    return out


def _risk_frame(alldf, horizon_sec=1.0):
    """未来控制风险分析的统一样本与信号；风险定义为未来 lat_err_rms 的 P90。"""
    err = acc_key("plan", "r", 10, "rmse")
    cols = ["t", "segment_id", "lat_err_rms", err, "koopman_health_score_total"]
    d = alldf[cols].copy()
    d["future_lat_err_rms"] = _future_segment_value(alldf, "lat_err_rms", horizon_sec)
    d = d.dropna(subset=[err, "koopman_health_score_total", "future_lat_err_rms"])
    if d.empty:
        return d
    d["risk_threshold"] = d["future_lat_err_rms"].quantile(0.90)
    d["is_future_risk"] = d["future_lat_err_rms"] >= d["risk_threshold"].iloc[0]
    d["model_signal"] = d[err].rank(pct=True)
    d["low_health_signal"] = 1.0 - d["koopman_health_score_total"].rank(pct=True)
    d["fused_signal"] = 0.65 * d["model_signal"] + 0.35 * d["low_health_signal"]
    return d


def _binary_curve(y, score, n_thresholds=81):
    """无 sklearn 依赖的 ROC / PR 点列；返回 threshold, fpr, tpr, precision, recall。"""
    d = pd.DataFrame({"y": y, "score": score}).dropna()
    if len(d) < 30 or d["y"].nunique() < 2:
        return None
    q = np.linspace(0.0, 1.0, n_thresholds)
    thresholds = np.unique(np.quantile(d["score"], q))
    rows = []
    yv = d["y"].to_numpy(bool)
    for threshold in thresholds:
        pred = d["score"].to_numpy(float) >= threshold
        tp = int(np.sum(pred & yv)); fp = int(np.sum(pred & ~yv))
        fn = int(np.sum(~pred & yv)); tn = int(np.sum(~pred & ~yv))
        rows.append({"threshold": threshold,
                     "fpr": fp / max(fp + tn, 1), "tpr": tp / max(tp + fn, 1),
                     "precision": tp / max(tp + fp, 1), "recall": tp / max(tp + fn, 1)})
    return pd.DataFrame(rows)


def _partial_spearman(df, xcol, ycol, controls):
    """先将变量转秩，再剔除工况秩的线性影响，得到去混杂后的近似 partial Spearman。"""
    d = df[[xcol, ycol] + list(controls)].dropna()
    if len(d) < 60:
        return np.nan, len(d)
    ranks = d.rank(method="average")
    design = np.column_stack([np.ones(len(ranks))] + [ranks[col].to_numpy(float) for col in controls])
    def residual(col):
        value = ranks[col].to_numpy(float)
        return value - design @ np.linalg.lstsq(design, value, rcond=None)[0]
    rx, ry = residual(xcol), residual(ycol)
    if np.std(rx) < 1e-12 or np.std(ry) < 1e-12:
        return np.nan, len(d)
    return float(np.corrcoef(rx, ry)[0, 1]), len(d)


def _coverage_priority(alldf):
    """三维分位格风险优先级：低频 × 高模型误差 × 高控制误差。"""
    axes = ["koopman_vel_u", "abs_r", "abs_steer"]
    err = acc_key("plan", "r", 10, "rmse")
    d = alldf[axes + [err, "lat_err_rms"]].dropna().copy()
    if len(d) < 8 ** 2:
        return pd.DataFrame()
    try:
        for axis, name in zip(axes, ["xb", "yb", "zb"]):
            d[name], _ = quantile_codes_and_edges(d[axis], 8)
    except Exception:
        return pd.DataFrame()
    group = d.groupby(["xb", "yb", "zb"])
    result = group[axes].median()
    result["n"] = group.size()
    result["model_rmse"] = group[err].median()
    result["control_rms"] = group["lat_err_rms"].median()
    result = result.reset_index()
    result["priority"] = ((1.0 - result["n"].rank(pct=True)) *
                          result["model_rmse"].rank(pct=True) *
                          result["control_rms"].rank(pct=True))
    return result.sort_values("priority", ascending=False)


def _event_profiles(alldf, trigger_col, fields, before_sec=1.0, after_sec=2.5):
    """以高 trigger 事件峰值为 t=0，对齐多字段时序并返回每个事件的插值窗口。"""
    grid = np.linspace(-before_sec, after_sec, 36)
    values = {field: [] for field in fields}
    group_col = "segment_id" if "segment_id" in alldf else "segment"
    for _, part in alldf.groupby(group_col, sort=False):
        cols = list(dict.fromkeys(["t", trigger_col] + fields))
        d = part[cols].dropna(subset=["t", trigger_col]).sort_values("t")
        if len(d) < 80:
            continue
        for _, _, peak_idx in nonoverlap_events(d, trigger_col, 0.90, DEFAULT_CONF["event_min_gap_sec"]):
            peak_t = d.loc[peak_idx, "t"]
            query = peak_t + grid
            for field in fields:
                valid = d[["t", field]].dropna()
                if len(valid) < 2:
                    continue
                values[field].append(np.interp(query, valid["t"], valid[field], left=np.nan, right=np.nan))
    return grid, {field: np.asarray(series, dtype=float) for field, series in values.items()}


def _plotly_risk_roc_pr_figure(alldf):
    from plotly import graph_objects as go
    from plotly.subplots import make_subplots
    d = _risk_frame(alldf, 1.0)
    fig = make_subplots(rows=1, cols=2, subplot_titles=("ROC：未来 1.0 s 大误差", "PR：未来 1.0 s 大误差"))
    signals = [("模型误差", "model_signal", COLORS[0]), ("低健康分", "low_health_signal", COLORS[2]),
               ("融合信号", "fused_signal", COLORS[1])]
    for name, col, color in signals:
        curve = _binary_curve(d.get("is_future_risk", pd.Series(dtype=bool)), d.get(col, pd.Series(dtype=float)))
        if curve is None:
            continue
        roc = curve.sort_values("fpr")
        pr = curve.sort_values("recall")
        auc_roc = float(np.trapz(roc["tpr"], roc["fpr"]))
        fig.add_trace(go.Scatter(x=roc["fpr"], y=roc["tpr"], mode="lines", name=f"{name} AUC={auc_roc:.2f}",
                                 line={"color": color}, customdata=roc["threshold"],
                                 hovertemplate="FPR=%{x:.3f}<br>TPR=%{y:.3f}<br>阈值=%{customdata:.4g}<extra>%{fullData.name}</extra>"), row=1, col=1)
        fig.add_trace(go.Scatter(x=pr["recall"], y=pr["precision"], mode="lines", name=name,
                                 line={"color": color}, customdata=pr["threshold"], showlegend=False,
                                 hovertemplate="recall=%{x:.3f}<br>precision=%{y:.3f}<br>阈值=%{customdata:.4g}<extra>%{fullData.name}</extra>"), row=1, col=2)
    fig.add_trace(go.Scatter(x=[0, 1], y=[0, 1], mode="lines", line={"color": "#94a3b8", "dash": "dot"},
                             name="随机", showlegend=False, hoverinfo="skip"), row=1, col=1)
    fig.update_xaxes(title_text="False positive rate", row=1, col=1); fig.update_yaxes(title_text="True positive rate", row=1, col=1)
    fig.update_xaxes(title_text="Recall", row=1, col=2); fig.update_yaxes(title_text="Precision", row=1, col=2)
    return _plotly_layout(fig, "扩展 1 · 未来控制风险预警能力", 470)


def _plotly_risk_threshold_figure(alldf):
    from plotly import graph_objects as go
    from plotly.subplots import make_subplots
    fig = make_subplots(rows=1, cols=2, subplot_titles=("融合信号阈值权衡（未来 1.0 s）", "不同预警时域的 ROC AUC"))
    d = _risk_frame(alldf, 1.0)
    if not d.empty:
        qvals = np.linspace(0.50, 0.95, 10)
        thresholds = np.quantile(d["fused_signal"], qvals)
        precision, recall = [], []
        y = d["is_future_risk"].to_numpy(bool)
        for threshold in thresholds:
            pred = d["fused_signal"].to_numpy(float) >= threshold
            tp = np.sum(pred & y); fp = np.sum(pred & ~y); fn = np.sum(~pred & y)
            precision.append(tp / max(tp + fp, 1)); recall.append(tp / max(tp + fn, 1))
        fig.add_trace(go.Scatter(x=qvals, y=precision, mode="lines+markers", name="precision", marker={"color": COLORS[0]},
                                 customdata=thresholds, hovertemplate="报警分位=%{x:.0%}<br>precision=%{y:.3f}<br>阈值=%{customdata:.4g}<extra></extra>"), row=1, col=1)
        fig.add_trace(go.Scatter(x=qvals, y=recall, mode="lines+markers", name="recall", marker={"color": COLORS[2]},
                                 customdata=thresholds, hovertemplate="报警分位=%{x:.0%}<br>recall=%{y:.3f}<br>阈值=%{customdata:.4g}<extra></extra>"), row=1, col=1)
    horizons = [0.5, 1.0, 2.5]
    for name, signal, color in [("模型误差", "model_signal", COLORS[0]), ("融合信号", "fused_signal", COLORS[1])]:
        aucs = []
        for horizon in horizons:
            d = _risk_frame(alldf, horizon)
            curve = _binary_curve(d.get("is_future_risk", pd.Series(dtype=bool)), d.get(signal, pd.Series(dtype=float)))
            if curve is None:
                aucs.append(np.nan); continue
            roc = curve.sort_values("fpr")
            aucs.append(float(np.trapz(roc["tpr"], roc["fpr"])))
        fig.add_trace(go.Scatter(x=horizons, y=aucs, mode="lines+markers", name=name, marker={"color": color},
                                 hovertemplate="预警时域=%{x:.1f}s<br>ROC AUC=%{y:.3f}<extra>%{fullData.name}</extra>"), row=1, col=2)
    fig.update_xaxes(title_text="报警阈值分位", tickformat=".0%", row=1, col=1); fig.update_yaxes(title_text="metric", row=1, col=1)
    fig.update_xaxes(title_text="未来时域 (s)", row=1, col=2); fig.update_yaxes(title_text="ROC AUC", row=1, col=2)
    return _plotly_layout(fig, "扩展 2 · 风险阈值与预警时域", 470)


def _plotly_deconfounded_figure(alldf):
    from plotly import graph_objects as go
    from plotly.subplots import make_subplots
    err, control = acc_key("plan", "r", 10, "rmse"), "lat_err_rms"
    controls = ["koopman_vel_u", "abs_r", "abs_steer"]
    global_rho = spearman(alldf[err], alldf[control])
    partial_rho, n = _partial_spearman(alldf, err, control, controls)
    fig = make_subplots(rows=1, cols=2, subplot_titles=("全局相关 vs 控制工况后相关", "按机动类型的去混杂相关"))
    fig.add_trace(go.Bar(x=["全局 Spearman", "控制 vx/|r|/|steer| 后"], y=[global_rho, partial_rho],
                         customdata=[len(alldf[[err, control]].dropna()), n], marker={"color": [COLORS[0], COLORS[1]]},
                         hovertemplate="%{x}<br>rho=%{y:.4f}<br>n=%{customdata}<extra></extra>"), row=1, col=1)
    fig.add_hline(y=0, line_dash="dot", line_color="grey", row=1, col=1)
    rows = []
    for maneuver, part in alldf.groupby("maneuver", observed=True):
        rho, count = _partial_spearman(part, err, control, controls)
        if np.isfinite(rho):
            rows.append((str(maneuver), rho, count))
    if rows:
        labels, values, counts = zip(*rows)
        fig.add_trace(go.Bar(x=list(labels), y=list(values), customdata=list(counts), marker={"color": COLORS[2]},
                             hovertemplate="机动=%{x}<br>partial rho=%{y:.4f}<br>n=%{customdata}<extra></extra>"), row=1, col=2)
        fig.add_hline(y=0, line_dash="dot", line_color="grey", row=1, col=2)
    else:
        fig.add_annotation(text="分层样本不足", x=0.5, y=0.5, showarrow=False, row=1, col=2)
    fig.update_yaxes(title_text="partial Spearman rho", row=1, col=1)
    fig.update_yaxes(title_text="partial Spearman rho", row=1, col=2)
    return _plotly_layout(fig, "扩展 3 · 去混杂后的模型误差关联", 470)


def _plotly_bag_stability_figure(alldf):
    from plotly import graph_objects as go
    err, control = acc_key("plan", "r", 10, "rmse"), "lat_err_rms"
    rows = []
    for bag_id, part in alldf.groupby("bag_id", sort=True):
        d = part[[err, control]].dropna()
        if len(d) >= 60:
            rows.append((int(bag_id), spearman(d[err], d[control]), len(d)))
    fig = go.Figure()
    if rows:
        labels, rhos, counts = zip(*rows)
        fig.add_trace(go.Bar(x=[f"bag {v}" for v in labels], y=rhos, customdata=counts, marker={"color": COLORS[0]},
                             hovertemplate="%{x}<br>rho=%{y:.4f}<br>n=%{customdata}<extra></extra>"))
        fig.add_hline(y=0, line_dash="dot", line_color="grey")
    else:
        fig.add_annotation(text="按 bag 样本不足", x=0.5, y=0.5, showarrow=False)
    fig.update_xaxes(title_text="数据包"); fig.update_yaxes(title_text="plan_r_10 × lat_err_rms Spearman rho")
    return _plotly_layout(fig, "扩展 4 · 关联在不同数据包中的稳定性", 470)


def _plotly_coverage_priority_figure(alldf):
    from plotly import graph_objects as go
    d = _coverage_priority(alldf)
    fig = go.Figure()
    if not d.empty:
        custom = np.column_stack([d["n"], d["model_rmse"], d["control_rms"], d["priority"]])
        fig.add_trace(go.Scatter3d(
            x=d["koopman_vel_u"], y=d["abs_r"], z=d["abs_steer"], mode="markers", customdata=custom,
            marker={"size": np.clip(5 + np.sqrt(d["n"]) * 0.55, 5, 18), "color": d["priority"],
                    "colorscale": "Viridis", "colorbar": {"title": "priority"}, "opacity": 0.86},
            hovertemplate="vx=%{x:.5g}<br>|r|=%{y:.5g}<br>|steer|=%{z:.5g}<br>n=%{customdata[0]:.0f}"
                          "<br>model rmse=%{customdata[1]:.5g}<br>control rms=%{customdata[2]:.5g}"
                          "<br>priority=%{customdata[3]:.3f}<extra></extra>"))
    else:
        fig.add_annotation(text="数据不足", x=0.5, y=0.5, showarrow=False)
    fig.update_layout(scene={"xaxis_title": "vx (m/s)", "yaxis_title": "|r| (rad/s)",
                             "zaxis_title": "|steer| (rad)"})
    return _plotly_layout(fig, "扩展 5 · 覆盖缺口风险三维排序", 620)


def _plotly_coverage_priority_top_figure(alldf):
    from plotly import graph_objects as go
    d = _coverage_priority(alldf).head(15).iloc[::-1]
    fig = go.Figure()
    if not d.empty:
        labels = [f"vx={r.koopman_vel_u:.2g} · r={r.abs_r:.2g} · steer={r.abs_steer:.2g}" for r in d.itertuples()]
        custom = np.column_stack([d["n"], d["model_rmse"], d["control_rms"]])
        fig.add_trace(go.Bar(y=labels, x=d["priority"], orientation="h", customdata=custom, marker={"color": COLORS[2]},
                             hovertemplate="%{y}<br>priority=%{x:.3f}<br>n=%{customdata[0]:.0f}"
                                           "<br>model rmse=%{customdata[1]:.5g}<br>control rms=%{customdata[2]:.5g}<extra></extra>"))
    else:
        fig.add_annotation(text="数据不足", x=0.5, y=0.5, showarrow=False)
    fig.update_xaxes(title_text="低频 × 高模型误差 × 高控制误差 priority")
    return _plotly_layout(fig, "扩展 6 · 最优先补充/回放的联合工况", 620)


def _plotly_event_timeline_figure(alldf):
    from plotly import graph_objects as go
    from plotly.subplots import make_subplots
    err = acc_key("plan", "r", 10, "rmse")
    fields = [err, "lat_err_rms", "abs_r", "abs_steer"]
    labels = ["plan_r_10 rmse", "lat_err rms", "|r|", "|steer|"]
    grid, profiles = _event_profiles(alldf, err, fields)
    fig = make_subplots(rows=2, cols=2, subplot_titles=labels)
    for pos, (field, label) in enumerate(zip(fields, labels)):
        row, col = divmod(pos, 2); series = profiles[field]
        if len(series):
            median = np.nanmedian(series, axis=0)
            p25, p75 = np.nanpercentile(series, [25, 75], axis=0)
            fig.add_trace(go.Scatter(x=grid, y=median, mode="lines", name=label,
                                     line={"color": COLORS[pos]}, customdata=np.column_stack([p25, p75]),
                                     hovertemplate="t=%{x:.2f}s<br>median=%{y:.5g}<br>P25=%{customdata[0]:.5g}<br>P75=%{customdata[1]:.5g}<extra></extra>"), row=row + 1, col=col + 1)
            fig.add_vline(x=0, line_dash="dot", line_color="grey", row=row + 1, col=col + 1)
        else:
            fig.add_annotation(text="事件不足", x=0.5, y=0.5, showarrow=False, row=row + 1, col=col + 1)
        fig.update_xaxes(title_text="相对事件峰值 (s)", row=row + 1, col=col + 1)
    return _plotly_layout(fig, "扩展 7 · 高模型误差事件的对齐时序", 720)


def _plotly_event_attribution_figure(alldf):
    from plotly import graph_objects as go
    err = acc_key("plan", "r", 10, "rmse")
    fields = ["koopman_vel_u", "abs_r", "abs_steer", "koopman_gain_lp", "koopman_health_score_total"]
    group_col = "segment_id" if "segment_id" in alldf else "segment"
    peaks = []
    for _, part in alldf.groupby(group_col, sort=False):
        d = part[["t", err] + fields].dropna(subset=["t", err])
        for _, _, peak_idx in nonoverlap_events(d, err, 0.90, DEFAULT_CONF["event_min_gap_sec"]):
            peaks.append(d.loc[peak_idx, fields])
    fig = go.Figure()
    if peaks:
        peak_df = pd.DataFrame(peaks)
        labels, effects, medians = [], [], []
        for field in fields:
            base = alldf[field].dropna()
            iqr = base.quantile(.75) - base.quantile(.25)
            effect = (peak_df[field].median() - base.median()) / max(float(iqr), 1e-12)
            labels.append(field); effects.append(effect); medians.append(peak_df[field].median())
        fig.add_trace(go.Bar(x=labels, y=effects, customdata=medians, marker={"color": COLORS[3]},
                             hovertemplate="%{x}<br>事件峰值相对全局中位 (IQR)=%{y:.3f}<br>事件中位=%{customdata:.5g}<extra></extra>"))
        fig.add_hline(y=0, line_dash="dot", line_color="grey")
    else:
        fig.add_annotation(text="事件不足", x=0.5, y=0.5, showarrow=False)
    fig.update_yaxes(title_text="高模型误差事件相对全局中位数（IQR 归一化）")
    return _plotly_layout(fig, "扩展 8 · 高模型误差事件的工况归因", 470)


def _plotly_health_calibration_figure(alldf):
    from plotly import graph_objects as go
    d = _risk_frame(alldf, 1.0)
    fig = go.Figure()
    if not d.empty:
        try:
            d = d.assign(bin=pd.qcut(d["koopman_health_score_total"], 8, duplicates="drop"))
            g = d.groupby("bin", observed=True)
            health = g["koopman_health_score_total"].median()
            risk = g["is_future_risk"].mean()
            count = g.size()
            fig.add_trace(go.Scatter(x=health, y=risk, mode="lines+markers", name="future risk rate", customdata=count,
                                     marker={"color": COLORS[1]},
                                     hovertemplate="health 中位=%{x:.3f}<br>未来风险率=%{y:.2%}<br>n=%{customdata}<extra></extra>"))
        except Exception:
            pass
    if not fig.data:
        fig.add_annotation(text="数据不足", x=0.5, y=0.5, showarrow=False)
    fig.update_xaxes(title_text="health score 中位数"); fig.update_yaxes(title_text="未来 1.0 s 大误差概率", tickformat=".0%")
    return _plotly_layout(fig, "扩展 9 · 健康分与未来控制风险标定", 470)


def _plotly_health_separation_figure(alldf):
    from plotly import graph_objects as go
    d = _risk_frame(alldf, 1.0)
    fig = go.Figure()
    if not d.empty:
        for label, flag, color in [("未来正常", False, COLORS[0]), ("未来大误差", True, COLORS[2])]:
            values = d.loc[d["is_future_risk"] == flag, "koopman_health_score_total"]
            fig.add_trace(go.Box(y=values, name=label, boxpoints=False, marker={"color": color}, line={"color": color},
                                 hovertemplate=f"{label}<br>health=%{{y:.3f}}<extra></extra>"))
    else:
        fig.add_annotation(text="数据不足", x=0.5, y=0.5, showarrow=False)
    fig.update_yaxes(title_text="health score")
    return _plotly_layout(fig, "扩展 10 · 健康分对未来风险的区分度", 470)


def _plotly_plan_exec_scatter_figure(alldf):
    from plotly import graph_objects as go
    from plotly.subplots import make_subplots
    fig = make_subplots(rows=2, cols=2, subplot_titles=[f"{state} · 10step" for state in STATES])
    for pos, state in enumerate(STATES):
        plan = acc_key("plan", state, 10, "rmse")
        executed = acc_key("executed", state, 10, "rmse")
        d = alldf[[plan, executed]].dropna()
        row, col = divmod(pos, 2)
        if len(d):
            x, y = downsample(d[plan].to_numpy(), d[executed].to_numpy(), DEFAULT_CONF["sample_plot"])
            limit = max(float(np.nanmax(x)), float(np.nanmax(y)))
            fig.add_trace(go.Scattergl(x=x, y=y, mode="markers", marker={"size": 4, "opacity": 0.32, "color": COLORS[pos]},
                                       hovertemplate="plan rmse=%{x:.5g}<br>executed rmse=%{y:.5g}<extra></extra>"), row=row + 1, col=col + 1)
            fig.add_trace(go.Scatter(x=[0, limit], y=[0, limit], mode="lines", line={"color": "#94a3b8", "dash": "dot"},
                                     showlegend=False, hoverinfo="skip"), row=row + 1, col=col + 1)
        fig.update_xaxes(title_text="plan rmse", row=row + 1, col=col + 1)
        fig.update_yaxes(title_text="executed rmse", row=row + 1, col=col + 1)
    return _plotly_layout(fig, "扩展 11 · plan 与 executed 预测误差一致性", 720)


def _plotly_plan_exec_delta_figure(alldf):
    from plotly import graph_objects as go
    err_plan = acc_key("plan", "r", 10, "rmse")
    err_exec = acc_key("executed", "r", 10, "rmse")
    d = alldf[["koopman_vel_u", "abs_r", err_plan, err_exec]].dropna().copy()
    d["delta"] = d[err_plan] - d[err_exec]
    res = heatmap_2d(d, "koopman_vel_u", "abs_r", "delta", 8, 8)
    fig = go.Figure()
    if res is not None:
        xc, yc, grid, count = res
        custom = np.empty(grid.shape + (3,), dtype=object)
        for iy in range(len(yc)):
            for ix in range(len(xc)):
                custom[iy, ix] = [xc[ix], yc[iy], count[iy, ix]]
        fig.add_trace(go.Heatmap(x=list(range(len(xc))), y=list(range(len(yc))), z=grid, customdata=custom,
                                 colorscale="RdBu", zmid=0, colorbar={"title": "plan − executed rmse"},
                                 hovertemplate="vx=%{customdata[0]:.5g}<br>|r|=%{customdata[1]:.5g}"
                                               "<br>delta=%{z:.5g}<br>n=%{customdata[2]:.0f}<extra></extra>"))
        fig.update_xaxes(tickmode="array", tickvals=list(range(len(xc))), ticktext=[f"{v:.3g}" for v in xc], title_text="vx (m/s)")
        fig.update_yaxes(tickmode="array", tickvals=list(range(len(yc))), ticktext=[f"{v:.3g}" for v in yc], title_text="|r| (rad/s)")
    else:
        fig.add_annotation(text="数据不足", x=0.5, y=0.5, showarrow=False)
    return _plotly_layout(fig, "扩展 12 · plan/executed 差异的工况分布", 520)


def _plotly_sections(bags_dfs):
    """按静态前端的页面与章节定义组织 Plotly 图，确保内容和卡片布局一一对应。"""
    alldf = _alldf(bags_dfs)
    err = acc_key("plan", "r", 10, "rmse")
    comparison = alldf.dropna(subset=["koopman_vel_u", "abs_r", "abs_steer", err])
    pages = {
        "Overview": _plotly_overview_figure(bags_dfs),
        "G1-1step": _plotly_g1_figure(alldf, 1),
        "G1-10step": _plotly_g1_figure(alldf, 10),
        "G1c": _plotly_g1c_page(comparison, err),
        "G2-lat_err_rms": _plotly_g2_figure(alldf, "lat_err_rms"),
        "G2-lat_err_p2p": _plotly_g2_figure(alldf, "lat_err_p2p"),
        "G2-lat_err_hfe": _plotly_g2_figure(alldf, "lat_err_hfe"),
        "G2-phi_err_rms": _plotly_g2_figure(alldf, "phi_err_rms"),
        "G2-phi_err_p2p": _plotly_g2_figure(alldf, "phi_err_p2p"),
        "G2-phi_err_hfe": _plotly_g2_figure(alldf, "phi_err_hfe"),
        "G3-相关性": _plotly_g3_transfer_figure(alldf),
        "G3-时域相关": _plotly_g3_temporal_figure(alldf),
        "G3-事件对照": _plotly_g3_event_figure(alldf),
        "G4-Score": _plotly_g4_figure(alldf),
    }
    sections = [(anchor, title, desc, [pages[page] for page in page_titles], layout)
                for anchor, title, desc, page_titles, layout in _SECTIONS]
    sections.append((
        "advanced", "扩展诊断",
        "风险预警、去混杂、覆盖缺口、事件根因、健康分标定和 plan/executed 差异；新增图卡均为一行两图。",
        [_plotly_risk_roc_pr_figure(alldf), _plotly_risk_threshold_figure(alldf),
         _plotly_deconfounded_figure(alldf), _plotly_bag_stability_figure(alldf),
         _plotly_coverage_priority_figure(alldf), _plotly_coverage_priority_top_figure(alldf),
         _plotly_event_timeline_figure(alldf), _plotly_event_attribution_figure(alldf),
         _plotly_health_calibration_figure(alldf), _plotly_health_separation_figure(alldf),
         _plotly_plan_exec_scatter_figure(alldf), _plotly_plan_exec_delta_figure(alldf)],
        "2col"))
    return sections


def build_frontend_interactive(bags_dfs, out_dir):
    """静态前端等版式的 Plotly 交互版：每页图可悬停，G1c 3D 可旋转。"""
    from plotly import io as pio
    os.makedirs(out_dir, exist_ok=True)
    sections = _plotly_sections(bags_dfs)
    alldf = _alldf(bags_dfs)
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
    n_frames = int(sum(c["_n_active"] for _, _, c in bags_dfs))
    n_segs = int(sum(df["segment"].nunique() if len(df) else 0 for _, df, _ in bags_dfs))
    nav = "\n  ".join('<a href="#%s">%s</a>' % (anchor, title) for anchor, title, _, _, _ in sections)
    include_js = True
    sec_html = []
    for anchor, title, desc, figures, layout in sections:
        cards = []
        for figure_index, figure in enumerate(figures):
            card_class = "card interactive-card wide-card" if anchor == "g3" and figure_index == 0 else "card interactive-card"
            cards.append(f'<div class="{card_class}">' + pio.to_html(
                figure, include_plotlyjs="inline" if include_js else False, full_html=False,
                config={"responsive": True, "scrollZoom": True, "displaylogo": False}) + '</div>')
            include_js = False
        grid_cls = "grid2 " if layout == "2col" else ""
        sec_html.append('<section class="section %s" id="%s"><div class="sec-head">'
                        '<span class="dot"></span><h2>%s</h2><span class="tag">%s</span></div>'
                        '<p class="sec-desc">%s</p><div class="%s">%s</div></section>'
                        % (CH_CLASS.get(anchor, "ch-z"), anchor, title, anchor.upper(), desc, grid_cls, ''.join(cards)))
    css = (_CSS + ".interactive-card{overflow:hidden}.interactive-card .plotly-graph-div{width:100%!important}"
           "@media(min-width:901px){.grid2 .wide-card{grid-column:1/-1}}")
    html = (f"<!doctype html><html lang='zh-CN'><head><meta charset='utf-8'>"
            f"<meta name='viewport' content='width=device-width, initial-scale=1'>"
            f"<title>Koopman 横向模型 · 三方分析（交互版）</title>"
            f"<style>{css}</style></head><body>"
            f"<header class='hero'><div class='wrap'>"
            f"<span class='eyebrow'><i></i>KOOPMAN LATERAL · PLOTLY INTERACTIVE</span>"
            f"<h1>Koopman 横向模型三方分析</h1>"
            f"<p class='lead'>工况 <code>Z</code> × 模型评分 <code>M</code> × 控制误差 <code>C</code>。"
            f"输入来自回放 bag 的 <code>/msd/endpoint/control_command</code> 每帧 JSON，全部在 "
            f"<code>koopman_analysis</code> 工具链内清洗、对齐、归因。</p>"
            f"<div class='meta'><span>{len(bags_dfs)} 数据包 · {n_frames:,} 有效帧</span>"
            f"<span>{n_segs} 个连续控制段</span><span>Plotly 离线交互</span></div>"
            f"<div class='stats'>{hero}</div></div></header>"
            f"<div class='tracks'><i></i><i></i><i></i></div>"
            f"<nav><div class='wrap inner'>{nav}</div></nav>"
            f"<main class='wrap'>{''.join(sec_html)}</main>"
            f"<div class='tracks' style='margin-top:34px'><i></i><i></i><i></i></div>"
            f"<footer>Koopman 三方分析 · Plotly 全交互版</footer></body></html>")
    idx = os.path.join(out_dir, "index.html")
    with open(idx, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"[OK] interactive -> {idx} ({sum(len(figures) for _, _, _, figures, _ in sections)} 页, Plotly 自包含)")
    return idx


def build_frontend(bags_dfs, out_dir, dpi=REPORT_DPI):
    """生成前端页面目录: index.html + 分页 PNG（hero 真实数据 + 章节导航 + 卡片）"""
    os.makedirs(out_dir, exist_ok=True)
    pages = build_report_pages(bags_dfs, out_dir, dpi=dpi)
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
            f"<h1>Koopman 横向模型三方分析</h1>"
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


def load_all(bags, conf, max_active_bags=None):
    """加载并清洗 bag；max_active_bags 用于快速生成有代表性的可分析子集。"""
    bags_dfs = []
    active_bags = 0
    for bag_index, b in enumerate(bags):
        print(f"[load] {b}")
        df, meta = load_bag(b, conf)
        cov = preflight(df, meta, conf)
        df = clean_and_segment(df, conf)
        # segment 名在每个 bag 内都从 S1 开始；分析跨包聚合前必须命名空间隔离。
        if not df.empty:
            df["bag_id"] = bag_index
            df["segment_id"] = "B" + str(bag_index) + "_" + df["segment"].astype(str)
            active_bags += 1
        bags_dfs.append((b, df, cov))
        print(f"  frames={meta['n_frame']} valid={cov['_n_active']} "
              f"segs={df['segment'].nunique() if len(df) else 0}")
        if max_active_bags is not None and active_bags >= max_active_bags:
            print(f"[load] reached --max-active-bags={max_active_bags}")
            break
    return bags_dfs


def main():
    ap = argparse.ArgumentParser(
        description="Koopman 三方分析 · 生成离线交互 HTML",
        epilog="运行后输出 output/index.html",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.parse_args()

    conf = dict(DEFAULT_CONF)
    list_path, out_dir = "analysis_list.txt", "output"
    if not os.path.exists(list_path):
        sys.exit(f"list 不存在: {list_path}")
    list_dir = os.path.dirname(os.path.abspath(list_path))
    bags = expand_bag_list(
        [l.strip() for l in open(list_path) if l.strip() and not l.strip().startswith("#")],
        [list_dir, os.path.dirname(list_dir)])
    if not bags:
        sys.exit("no bags found")
    bags_dfs = load_all(bags, conf)
    if not any(len(df) for _, df, _ in bags_dfs):
        sys.exit("no analysable Koopman frames: all bags lack active debug fields or valid segments")
    build_frontend_interactive(bags_dfs, out_dir)


if __name__ == "__main__":
    main()
