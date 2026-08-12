#!/usr/bin/env bash
# gen_report.sh — wrapper for report_generator.py
# Sets up ROS / Python environment internally; no sourcing needed.

set -e

# ── ROS overlay paths ────────────────────────────────────────────────
_ROS_OVERLAY=/mnt/data/docker/overlay2/b8bd618514aeb61db290b839f6f4dea3336b00d939ae08b8f9ca5ba966f4c52b/diff

export LD_LIBRARY_PATH=\
"${_ROS_OVERLAY}/opt/ros/noetic/lib:\
${_ROS_OVERLAY}/root/ros_catkin_ws/devel_isolated/roslz4/lib:\
${LD_LIBRARY_PATH:-}"

export PYTHONPATH=\
"${_ROS_OVERLAY}/opt/ros/noetic/lib/python3/dist-packages:\
${_ROS_OVERLAY}/root/ros_catkin_ws/devel_isolated/rosbag/lib/python3/dist-packages:\
${_ROS_OVERLAY}/root/ros_catkin_ws/devel_isolated/roslz4/lib/python3/dist-packages:\
${PYTHONPATH:-}"

# ── script location ──────────────────────────────────────────────────
_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_REPORT_PY="${_SCRIPT_DIR}/report_generator/report_generator.py"
_DEFAULT_CONFIG="${_SCRIPT_DIR}/report_generator/config/target_signal_lat.yaml"

# 若用户没有传 --config，自动追加默认配置
_has_config=0
for arg in "$@"; do
    [[ "$arg" == "--config" ]] && _has_config=1 && break
done

if [[ $_has_config -eq 0 ]]; then
    set -- "$@" --config "${_DEFAULT_CONFIG}"
fi

# 用 tee 捕获 report_generator.py 输出：终端照常显示，同时落盘供下面提取 bag 路径
_REPORT_LOG="$(mktemp)"
python3 "${_REPORT_PY}" "$@" | tee "$_REPORT_LOG"
REPORT_RC=${PIPESTATUS[0]}   # python3 的退出码（管道最后一个命令是 tee）

# ── 控制性能批量分析 ─────────────────────────────────────────────
# 只对「本次 report_generator 实际处理的 bag」跑批量：
#   report_generator 每处理一个 bag 都会打印 "[1/5] Reading bag: <path>"，
#   直接从这里提取，既覆盖 Mode 1 单 bag，也精确覆盖 Mode 2 按 md5 下载的 bag，
#   不会扫到 output-dir 下的其它历史 bag。
if [[ $REPORT_RC -eq 0 ]]; then
    mapfile -t _bags_to_analyze < <(grep -oE 'Reading bag: .+\.(bag|mfbag)' "$_REPORT_LOG" \
                                    | sed -E 's#.*Reading bag: ##' | sort -u)

    if [[ ${#_bags_to_analyze[@]} -gt 0 ]]; then
        echo ">>> 生成控制性能分析 HTML（共 ${#_bags_to_analyze[@]} 个 bag）..."
        for _b in "${_bags_to_analyze[@]}"; do
            echo "    -> $(basename "$_b")"
            bash "${_SCRIPT_DIR}/run_control_analysis.sh" "$_b" || echo "       (该 bag 批量分析失败，已跳过)"
        done
    else
        echo ">>> 跳过控制性能分析（未找到本地 bag）"
    fi
else
    echo ">>> 跳过控制性能分析（report 失败）"
fi

rm -f "$_REPORT_LOG"
exit $REPORT_RC
