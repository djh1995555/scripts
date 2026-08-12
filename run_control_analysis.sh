#!/usr/bin/env bash
# run_control_analysis.sh — 控制分析批量运行：对单个 bag 依次执行
#                       cp/script/control_debug_scripts/ 下的所有 .py 分析脚本，
#                       每个脚本生成一个独立的 HTML 报告。
#
# Usage:
#   bash run_control_analysis.sh <bag_path> [output_dir]
#
# Examples:
#   bash run_control_analysis.sh /path/to/some.bag
#   bash run_control_analysis.sh /path/to/some.bag /path/to/out_dir
#
# 输出命名：<脚本名>.<bag名>.html，默认写入 bag 同目录（或用 output_dir 指定）。
# 单个脚本失败不会中断批量；最后汇总成功/失败并给出每份日志路径。

# 注意：不用 `set -e`，逐个脚本的失败由我们自行捕获处理（批量不应因单点失败中断）。
set -uo pipefail

# ── ROS overlay 环境（与 gen_report.sh 一致；rosbag 类脚本需要） ──────
# 若本脚本被 gen_report.sh 调用，这些变量已存在，重复导出无害（幂等）。
_ROS_OVERLAY=/mnt/data/docker/overlay2/b8bd618514aeb61db290b839f6f4dea3336b00d939ae08b8f9ca5ba966f4c52b/diff
if [[ -d "$_ROS_OVERLAY" ]]; then
    export LD_LIBRARY_PATH=\
"${_ROS_OVERLAY}/opt/ros/noetic/lib:\
${_ROS_OVERLAY}/root/ros_catkin_ws/devel_isolated/roslz4/lib:\
${LD_LIBRARY_PATH:-}"
    export PYTHONPATH=\
"${_ROS_OVERLAY}/opt/ros/noetic/lib/python3/dist-packages:\
${_ROS_OVERLAY}/root/ros_catkin_ws/devel_isolated/rosbag/lib/python3/dist-packages:\
${_ROS_OVERLAY}/root/ros_catkin_ws/devel_isolated/roslz4/lib/python3/dist-packages:\
${PYTHONPATH:-}"
fi

# ── 参数检查 ──────────────────────────────────────────────────────
if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <bag_path> [output_dir]"
    echo ""
    echo "  bag_path   : bag 文件路径（.bag / .mfbag / .plan）"
    echo "  output_dir : 可选，输出目录；默认输出到 bag 同目录"
    exit 1
fi

BAG_PATH="$1"

# 转为绝对路径
BAG_PATH="$(cd "$(dirname "$BAG_PATH")" && pwd)/$(basename "$BAG_PATH")"

if [[ ! -f "$BAG_PATH" ]]; then
    echo "Error: bag 文件不存在: $BAG_PATH"
    exit 1
fi

# 检查文件扩展名
BASENAME="$(basename "$BAG_PATH")"
if [[ ! "$BASENAME" =~ \.(bag|mfbag|plan)$ ]]; then
    echo "Error: 不是有效的 bag 文件（需要 .bag / .mfbag / .plan）: $BAG_PATH"
    exit 1
fi

BAG_STEM="${BAG_PATH%.*}"   # bag 全路径去扩展名（basename 用作输出名后缀）

# ── 输出目录 ──────────────────────────────────────────────────────
if [[ $# -ge 2 ]]; then
    OUT_DIR="$2"
    mkdir -p "$OUT_DIR"
    OUT_DIR="$(cd "$OUT_DIR" && pwd)"
else
    OUT_DIR="$(dirname "$BAG_PATH")"
fi

# 每脚本日志目录
LOG_DIR="$OUT_DIR/lat_analysis_logs"
mkdir -p "$LOG_DIR"

# ── 分析脚本目录 ──────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
# 优先用环境变量；其次 mf_system 内 cp/script/；最后回退到同级 cp_apa_simulation 仓库
if [[ -n "${CONTROL_DEBUG_SCRIPTS_DIR:-}" ]]; then
    SCRIPTS_DIR="$CONTROL_DEBUG_SCRIPTS_DIR"
elif [[ -d "$REPOS_DIR/cp/script/control_debug_scripts" ]]; then
    SCRIPTS_DIR="$REPOS_DIR/cp/script/control_debug_scripts"
else
    SCRIPTS_DIR="$REPOS_DIR/../cp_apa_simulation/script/control_debug_scripts"
fi

if [[ ! -d "$SCRIPTS_DIR" ]]; then
    echo "Error: 分析脚本目录不存在: $SCRIPTS_DIR"
    echo "       (可设环境变量 CONTROL_DEBUG_SCRIPTS_DIR 指向 control_debug_scripts 目录)"
    exit 1
fi

# 收集顶层 .py（排除 .bak 备份与子目录包），按名称排序保证顺序确定
mapfile -t ANALYSIS_SCRIPTS < <(find "$SCRIPTS_DIR" -maxdepth 1 -type f -name '*.py' | sort)

if [[ ${#ANALYSIS_SCRIPTS[@]} -eq 0 ]]; then
    echo "Error: 在 $SCRIPTS_DIR 下未找到任何 .py 分析脚本"
    exit 1
fi

# ── 批量执行 ──────────────────────────────────────────────────────
echo "=== 控制分析批量运行 ==="
echo "  bag 文件   : $BAG_PATH"
echo "  输出目录   : $OUT_DIR"
echo "  脚本数量   : ${#ANALYSIS_SCRIPTS[@]}"
echo "  日志目录   : $LOG_DIR"
echo ""

START_ALL=$(date +%s)
PER_SCRIPT_TIMEOUT=600   # 单脚本最长 600s，避免个别脚本卡死拖垮整批

OK_LIST=()
FAIL_LIST=()

for script in "${ANALYSIS_SCRIPTS[@]}"; do
    stem="$(basename "$script" .py)"
    bag_name="$(basename "$BAG_STEM")"          # bag 名（去路径去扩展名），放输出名末尾
    out_html="$OUT_DIR/${stem}.${bag_name}.html"  # <脚本名>.<bag名>.html
    log_file="$LOG_DIR/${stem}.log"

    printf '[%s] %s ... ' "$(date +%H:%M:%S)" "$stem"
    t0=$(date +%s)
    timeout "$PER_SCRIPT_TIMEOUT" python3 "$script" "$BAG_PATH" "$out_html" >"$log_file" 2>&1
    rc=$?
    t1=$(date +%s)

    if [[ $rc -eq 0 && -f "$out_html" && $(stat -c%s "$out_html" 2>/dev/null || echo 0) -gt 1000 ]]; then
        size_kb=$(( $(stat -c%s "$out_html") / 1024 ))
        printf 'OK   (%ds, %dKB)\n' "$((t1-t0))" "$size_kb"
        OK_LIST+=("$stem|$out_html")
    else
        printf 'FAIL (rc=%d)\n' "$rc"
        # 失败时打印日志末尾几行便于定位
        grep -iE 'error|exception|traceback' "$log_file" 2>/dev/null | tail -3 | sed 's/^/        /'
        FAIL_LIST+=("$stem|$log_file")
    fi
done

END_ALL=$(date +%s)
ELAPSED=$((END_ALL - START_ALL))

# ── 汇总 ──────────────────────────────────────────────────────────
echo ""
echo "=== 批量分析完成 ==="
echo "  总耗时     : ${ELAPSED}s"
echo "  成功       : ${#OK_LIST[@]} / ${#ANALYSIS_SCRIPTS[@]}"

if [[ ${#OK_LIST[@]} -gt 0 ]]; then
    echo "  报告列表   :"
    for item in "${OK_LIST[@]}"; do
        echo "    [OK]   ${item#*|}"
    done
fi

if [[ ${#FAIL_LIST[@]} -gt 0 ]]; then
    echo "  失败列表（日志）:"
    for item in "${FAIL_LIST[@]}"; do
        echo "    [FAIL] ${item%%|*}  ->  ${item#*|}"
    done
fi

if command -v xdg-open &>/dev/null && [[ ${#OK_LIST[@]} -gt 0 ]]; then
    echo ""
    echo "  可在浏览器打开任一报告，例如："
    echo "    xdg-open ${OK_LIST[0]#*|}"
fi

# 只要有一个成功即视为整体可用（返回 0）；全部失败才返回非 0
if [[ ${#OK_LIST[@]} -gt 0 ]]; then
    exit 0
else
    exit 1
fi
