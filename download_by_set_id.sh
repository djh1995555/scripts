#!/usr/bin/env bash
# download_by_set_id.sh — 按 event_set_id 下载 bag，每 bag 一个子目录，下载后生成 report
#
# 用途: event.momenta.works(eql 搜索) 被墙时，走 ESS 绕行链路
#   (event_set_id → event_id → bag md5 → data.momenta.works 下载)
#
# 用法:
#   ./download_by_set_id.sh <set_id1> [set_id2 ...] --dir <目标目录> \
#       [--topics <t1> <t2>] [--token <MDI_TOKEN>] [--dry-run]
#
# 示例:
#   ./download_by_set_id.sh 6a9037d8b64938d2861b9f9e --dir bags/koopman_data \
#       --topics /msd/endpoint/control_command
#
# 输出结构:
#   <dir>/<bag名>/         每 bag 一个子目录
#     ├── <bag名>.bag      下载的 bag（含指定 topic）
#     └── <bag名>_report.html  report_generator 产物
#   <dir>/bagname_md5.json  bag 名→md5 映射（解析结果，供复用）
#
# 依赖: pyevents + pybagmining + rosbag（与 control_events_download_tool 相同）
# 绕行脚本: control_tools/data_process/control_events_download_tool/download_bags_by_event_set_id.py

set -e

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# debug 是 symlink -> /mnt/data/ws_djh/mf_system/debug; control_tools 是独立目录
_BYPASS_PY="/mnt/data/ws_djh/control_tools/data_process/control_events_download_tool/download_bags_by_event_set_id.py"
_REPORT_PY="${_SCRIPT_DIR}/report_generator/report_generator.py"
_DEFAULT_CONFIG="${_SCRIPT_DIR}/report_generator/config/target_signal_lat.yaml"
_DEFAULT_TOPIC="/msd/endpoint/control_command"

# ── 参数解析 ─────────────────────────────────────────────────────
_SET_IDS=()
_DIR=""
_TOPICS=()
_TOKEN=""
_DRY_RUN=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dir) _DIR="$2"; shift 2;;
        --topics) shift; while [[ $# -gt 0 && "$1" != --* ]]; do _TOPICS+=("$1"); shift; done;;
        --token) _TOKEN="$2"; shift 2;;
        --dry-run) _DRY_RUN=1; shift;;
        --*) echo "[download_by_set_id] 未知参数: $1" >&2; exit 1;;
        *) _SET_IDS+=("$1"); shift;;
    esac
done

if [[ ${#_SET_IDS[@]} -eq 0 ]]; then
    echo "用法: $0 <set_id1> [set_id2 ...] --dir <目录> [--topics ...] [--token ...] [--dry-run]" >&2
    exit 1
fi
if [[ -z "$_DIR" ]]; then
    echo "[download_by_set_id] 需要 --dir <目标目录>" >&2
    exit 1
fi
if [[ ${#_TOPICS[@]} -eq 0 ]]; then
    _TOPICS=("$_DEFAULT_TOPIC")
fi
[[ ! -f "$_BYPASS_PY" ]] && { echo "[download_by_set_id] 找不到绕行脚本: $_BYPASS_PY" >&2; exit 1; }
[[ ! -f "$_REPORT_PY" ]] && { echo "[download_by_set_id] 找不到 report_generator: $_REPORT_PY" >&2; exit 1; }

# 用绝对路径，避免 cwd 切换后相对路径错位
_DIR="$(cd "$_DIR" && pwd)"
mkdir -p "$_DIR"

# ── 1. 解析 event_set → bag 名+md5 ──────────────────────────────
echo ">>> [download_by_set_id] 解析 event_set: ${_SET_IDS[@]}"
_PYTHON_ARGS=(--dry-run -o "$_DIR")
[[ -n "$_TOKEN" ]] && _PYTHON_ARGS+=(--token "$_TOKEN")
python3 "$_BYPASS_PY" "${_PYTHON_ARGS[@]}" "${_SET_IDS[@]}" 2>&1 | tail -5

_BAG_JSON="$_DIR/bagname_md5.json"
[[ ! -f "$_BAG_JSON" ]] && { echo "[download_by_set_id] 解析失败，无 bagname_md5.json" >&2; exit 1; }

echo ">>> [download_by_set_id] 解析到 $(python3 -c "import json;print(len(json.load(open('$_BAG_JSON'))))") 个 bag"

# ── dry-run: 只解析不下，退出 ───────────────────────────────────
if [[ $_DRY_RUN -eq 1 ]]; then
    echo ">>> [download_by_set_id] --dry-run: 仅解析，不下载。bag 清单见 $_BAG_JSON"
    exit 0
fi

# ── 2. 逐 bag 下载到独立子目录 + 生成 report ────────────────────
python3 - "$_BAG_JSON" "$_DIR" "$_SCRIPT_DIR" "${_TOPICS[@]}" <<'PYEOF'
import json, os, sys, subprocess

BAG_JSON, OUT_DIR = sys.argv[1], os.path.abspath(sys.argv[2])
SCRIPT_DIR = sys.argv[3]
TOPICS = sys.argv[4:]

# import 绕行脚本与下载工具（同目录）
DL_TOOL_DIR = "/mnt/data/ws_djh/control_tools/data_process/control_events_download_tool"
sys.path.insert(0, DL_TOOL_DIR)
import download_bags_based_on_anything as dl

# 用 .mdi_conf 的 MDI token（download_bags_based_on_anything 默认是 mingcong 的, 无效）
try:
    import json as _j
    _conf = _j.load(open(os.path.expanduser("~/.mdi_conf")))
    if _conf.get("token"):
        dl.MDI_TOKEN = _conf["token"]
        print(f"使用 MDI token: {dl.MDI_TOKEN[:8]}...", flush=True)
except Exception as _e:
    print(f"读 ~/.mdi_conf 失败: {_e}", flush=True)

REPORT_PY = os.path.join(SCRIPT_DIR, "report_generator", "report_generator.py")
CONFIG = os.path.join(SCRIPT_DIR, "report_generator", "config", "target_signal_lat.yaml")

bmap = json.load(open(BAG_JSON))
print(f"开始下载 {len(bmap)} bag, topics={TOPICS}", flush=True)

for bag_name, bag_md5 in sorted(bmap.items()):
    base = os.path.splitext(bag_name)[0]
    bdir = os.path.join(OUT_DIR, base)
    os.makedirs(bdir, exist_ok=True)
    out_bag = os.path.join(bdir, bag_name)
    if os.path.exists(out_bag):
        print(f"skip existing: {bag_name}", flush=True)
    else:
        print(f"downloading {bag_name} ...", flush=True)
        try:
            dl.download_single_bag(bag_name, bag_md5, TOPICS, bdir)
        except Exception as ex:
            print(f"FAIL {bag_name}: {ex}", flush=True)
            continue
    # 生成 report（输出透传, 不 capture——后台无 tty 时 capture 会吞输出/卡住）
    print(f"  report: {base} ...", flush=True)
    try:
        r = subprocess.run(["python3", REPORT_PY, out_bag, "--config", CONFIG],
                           cwd=bdir, timeout=300)
        if r.returncode != 0:
            print(f"  report FAIL rc={r.returncode}: {base}", flush=True)
    except subprocess.TimeoutExpired:
        print(f"  report TIMEOUT: {base}", flush=True)
print("=== DONE ===")
PYEOF

echo ">>> [download_by_set_id] 完成，结果在 $_DIR"
