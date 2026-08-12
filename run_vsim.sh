#!/usr/bin/env bash
# run_vsim.sh — 批量 replay + 报告生成
#
# 先在 fpp 容器内编译 control,driving_control（参考 run_sim.sh），编译失败则终止；
# 然后读取 vsim_list.txt（每行 "task_id:bag_name"），为每一行依次：
#   1. 在 fpp_replay/<timestamp>/<bag_name> 下构造 output_dir
#   2. bash gen_replay.sh -t <task_id> -b <bag_name> -o <output_dir> --dc
#   3. bash <output_dir>/replay_local.sh              （跑仿真）
#   4. 对 <output_dir>/output/<bag_name>.bag 生成 report（经 gen_report.sh 调用
#      report_generator.py，自动配置 ROS 环境与默认 config）
#
# Usage: bash run_vsim.sh [vsim_list.txt]
#   默认列表文件：同目录下 vsim_list.txt

set -uo pipefail

DEBUG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPLAY_ROOT="$DEBUG_DIR/fpp_replay"

LIST_FILE="${1:-$DEBUG_DIR/vsim_list.txt}"
if [[ ! -f "$LIST_FILE" ]]; then
    echo "Error: list file not found: $LIST_FILE" >&2
    exit 1
fi

# 本次仿真时间戳目录
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$REPLAY_ROOT/$TIMESTAMP"
mkdir -p "$RUN_DIR"

# ── isim 认证预检（避免批量循环里每个 bag 都因 token 失效读到 EOF 而失败）──
# isim 在 access token 失效时会交互式读取 username；非交互批量运行读到 EOF 既失败
# 又会偷走 while 循环喂给 LIST_FILE 的 stdin。提前在编译前检测 + 尝试非交互刷新，
# 仍失败则明确引导 'isim auth login'，不浪费时间编译与逐 bag 试错。
echo "=== [auth] 检查 isim 认证 ==="
# isim auth status 在 token 失效时退出码非 0；run_vsim.sh 开头 set -o pipefail 会把
# 该非 0 码污染管道，使 `isim auth status | grep` 即便匹配到 expired 也判为假。
# 故先捕获输出到变量（|| true 兜住退出码），再用 echo | grep 独立判定。
_auth_out=$(isim auth status 2>&1 || true)
if echo "$_auth_out" | grep -qiE "expired|invalid"; then
    echo ">>> access token 失效，尝试非交互刷新 (isim auth refresh) ..."
    isim auth refresh </dev/null >/dev/null 2>&1 || true
    _auth_out=$(isim auth status 2>&1 || true)
    if echo "$_auth_out" | grep -qiE "expired|invalid"; then
        echo "Error: isim 认证不可用（refresh token 也已过期）。" >&2
        echo "       请先在终端交互登录：  isim auth login" >&2
        echo "       登录后重新运行：        bash $(basename "$0")${LIST_FILE:+ }${LIST_FILE:-}" >&2
        exit 1
    fi
    echo ">>> token 已刷新"
fi
echo "=== [auth] 认证可用 ==="
echo

echo "=== batch run start ==="
echo "list_file : $LIST_FILE"
echo "run_dir   :+ $RUN_DIR"
echo "timestamp : $TIMESTAMP"
echo

# ── 编译 driving_control（--dc 挂载本地 lib，需先编译产出）────────────
# 编译方法参考 run_sim.sh：在 fpp 容器内 ./sim fpp build -i control,driving_control
CONTAINER_NAME="fpp-container-mnt-data-ws_djh-mf_system"
echo "=== [build] 在容器内编译 control,driving_control ==="
if ! docker exec -i "$CONTAINER_NAME" bash -c \
    "export MFS_ROOT=/opt/mf_system PROJ_NAME=Devcar BENV_ID=devcar_with_cuda11.4 MFS_SYSTEM_CFG_YAML=config/Devcar/system.yaml && cd /opt/mf_system && ./sim fpp build -i control,driving_control"; then
    echo "Error: 编译失败，终止（不进入仿真）" >&2
    exit 1
fi
echo "=== [build] 编译完成 ==="
echo

# 去除首尾空白
trim() {
    local s="$1"
    s="${s#"${s%%[![:space:]]*}"}"   # 去前导空白
    s="${s%"${s##*[![:space:]]}"}"   # 去尾部空白
    printf '%s' "$s"
}

run_one() {
    local task_id="$1" bag_name="$2"
    local output_dir="$RUN_DIR/$bag_name"
    mkdir -p "$output_dir"

    local out_bag="$output_dir/output/${bag_name}.bag"
    # 兜底：若输出为 .mfbag
    if [[ ! -f "$out_bag" ]]; then
        local alt="$output_dir/output/${bag_name}.mfbag"
        [[ -f "$alt" ]] && out_bag="$alt"
    fi
    local report_html="${out_bag%.*}_report.html"

    echo "--------------------------------------------------"
    echo "[$bag_name] task_id=$task_id"
    echo "[$bag_name] output_dir=$output_dir"
    echo "--------------------------------------------------"

    # 1. 生成 replay_local.sh
    echo ">>> [$bag_name] gen_replay ..."
    if ! bash "$DEBUG_DIR/gen_replay.sh" -t "$task_id" -b "$bag_name" -o "$output_dir" --dc; then
        echo "!!! [$bag_name] gen_replay FAILED" >&2
        return 1
    fi

    # 2. 运行仿真
    echo ">>> [$bag_name] running simulation ..."
    if ! bash "$output_dir/replay_local.sh"; then
        echo "!!! [$bag_name] replay_local FAILED" >&2
        return 1
    fi

    # 3. 生成 report
    echo ">>> [$bag_name] generating report ..."
    if [[ ! -f "$out_bag" ]]; then
        echo "!!! [$bag_name] output bag not found: $out_bag" >&2
        return 1
    fi
    if ! bash "$DEBUG_DIR/gen_report.sh" "$out_bag"; then
        echo "!!! [$bag_name] report generation FAILED" >&2
        return 1
    fi
    echo ">>> [$bag_name] report: $report_html"

    # 4. 横向控制性能分析
    echo ">>> [$bag_name] generating lateral performance analysis ..."
    if ! bash "$DEBUG_DIR/run_control_analysis.sh" "$out_bag"; then
        echo "!!! [$bag_name] lateral analysis FAILED (non-blocking)" >&2
        # 不阻断流程，分析失败不影响主 pipeline
    fi
    return 0
}

TOTAL=0; OK=0; FAIL=0
declare -a RESULTS

while IFS= read -r line <&3 || [[ -n "$line" ]]; do
    # 去行内注释与首尾空白；空行跳过
    line="${line%%#*}"
    line="$(trim "$line")"
    [[ -z "$line" ]] && continue

    # 必须形如 task_id:bag_name
    if [[ "$line" != *:* ]]; then
        echo "Skip malformed line (no ':'): $line" >&2
        continue
    fi
    task_id="$(trim "${line%%:*}")"
    bag_name="$(trim "${line#*:}")"
    if [[ -z "$task_id" || -z "$bag_name" ]]; then
        echo "Skip empty field line: $line" >&2
        continue
    fi

    TOTAL=$((TOTAL + 1))
    bag_log_dir="$RUN_DIR/$bag_name/logs"
    mkdir -p "$bag_log_dir"
    bag_log="$bag_log_dir/batch.log"
    # 每个 bag 的调度日志落到该 bag 自己的 logs/batch.log
    run_one "$task_id" "$bag_name" 2>&1 | tee -a "$bag_log"
    rc=${PIPESTATUS[0]}
    if [[ $rc -eq 0 ]]; then
        OK=$((OK + 1)); RESULTS+=("OK   $bag_name  -> $bag_log")
    else
        FAIL=$((FAIL + 1)); RESULTS+=("FAIL $bag_name  -> $bag_log")
    fi
done 3< "$LIST_FILE"

echo
echo "================ SUMMARY ================"
echo "run_dir : $RUN_DIR"
echo "total   : $TOTAL"
echo "ok      : $OK"
echo "failed  : $FAIL"
if (( ${#RESULTS[@]} > 0 )); then
    for r in "${RESULTS[@]}"; do echo "  $r"; done
fi
echo "per-bag log : <run_dir>/<bag_name>/logs/batch.log"
echo "================ END ================"
