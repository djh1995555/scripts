#!/usr/bin/env bash
# run_sim.sh — 按照 sim_flow.md 执行仿真流程（支持单次 / --batch 批量）

set -euo pipefail

# ── 参数解析 ────────────────────────────────────────────────────────
usage() {
    cat <<EOF
Usage:
  单次: $0 <bag路径> <method> <car_name>
  批量: $0 --batch <list_file> <method> <car_name>

  bag路径   : bag 文件所在目录（相对脚本目录或绝对路径）
  list_file : 每行一个 bag 目录（相对脚本目录或绝对路径），# 开头注释 / 空行跳过
  method    : close-loop 或 open-loop
  car_name  : 车辆名称（如 PL567 / LC6-EC65130）

Example:
  $0 bags/横向失控1 close-loop PL567
  $0 --batch replay_list.txt close-loop LC6-EC65130
EOF
    exit 1
}

METHOD=""
CAR_NAME=""
BATCH_MODE=0
LIST_FILE=""
BAG_PATH=""

if [[ "${1:-}" == "--batch" ]]; then
    [[ $# -ne 4 ]] && usage
    BATCH_MODE=1
    LIST_FILE="$2"
    METHOD="$3"
    CAR_NAME="$4"
elif [[ $# -eq 3 ]]; then
    BAG_PATH="$1"
    METHOD="$2"
    CAR_NAME="$3"
else
    usage
fi

if [[ "$METHOD" != "close-loop" && "$METHOD" != "open-loop" ]]; then
    echo "Error: method 必须是 close-loop 或 open-loop，当前值: $METHOD" >&2
    exit 1
fi

# ── 路径计算 ────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 从 SCRIPT_DIR 向上找含 ./sim 的 mf_system 根，兼容 debug/ 在 mf_system 子树任意位置
# （既可从大盘 /mnt/data/ws_djh/mf_system/debug 运行，也可从仓库内 driving_control/debug 运行）
MF_SYSTEM_DIR="$SCRIPT_DIR"
while [[ "$MF_SYSTEM_DIR" != "/" && ! -x "$MF_SYSTEM_DIR/sim" ]]; do
    MF_SYSTEM_DIR="$(dirname "$MF_SYSTEM_DIR")"
done
if [[ ! -x "$MF_SYSTEM_DIR/sim" ]]; then
    echo "Error: 未找到 mf_system 根（含 ./sim），SCRIPT_DIR=$SCRIPT_DIR" >&2
    exit 1
fi
SIM_RESULT_DIR="$SCRIPT_DIR/sim_result"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"

CONTAINER_NAME="fpp-container-mnt-data-ws_djh-mf_system"

abs_bag_path() {
    local p="$1"
    if [[ "$p" != /* ]]; then p="$SCRIPT_DIR/$p"; fi
    printf '%s' "$p"
}

# 去行内注释 + 首尾空白
trim() {
    local s="$1"
    s="${s%%#*}"
    s="${s#"${s%%[![:space:]]*}"}"
    s="${s%"${s##*[![:space:]]}"}"
    printf '%s' "$s"
}

ensure_container() {
    if ! docker inspect "$CONTAINER_NAME" &>/dev/null; then
        echo "[prep] 容器不存在，自动创建（带 GPU）..."
        ( cd "$MF_SYSTEM_DIR" && ./sim fpp container start )
        echo "[prep] 容器已就绪"
    fi
}

build_once() {
    echo "[build] 在 docker 中编译 control,driving_control ..."
    docker exec -i "$CONTAINER_NAME" bash -c \
        "export MFS_ROOT=/opt/mf_system PROJ_NAME=Devcar BENV_ID=devcar_with_cuda11.4 MFS_SYSTEM_CFG_YAML=config/Devcar/system.yaml && cd /opt/mf_system&& ./sim fpp build -i control,driving_control"
}

# run_sim_only <bag_path_abs> <method> <car_name> <result_dir>
# 只做仿真 + 后处理；容器准备与编译由调用方在循环外完成一次
run_sim_only() {
    local bag_path="$1" method="$2" car_name="$3" result_dir="$4"
    local bag_dir_name; bag_dir_name="$(basename "$bag_path")"
    mkdir -p "$result_dir"
    echo "--------------------------------------------------"
    echo "[bag] $bag_dir_name"
    echo "[bag] bag路径  : $bag_path"
    echo "[bag] 结果目录 : $result_dir"
    echo "--------------------------------------------------"

    # 宿主机 /mnt/data/ws_djh/mf_system -> 容器内 /opt/mf_system
    local container_bag_path="${bag_path/\/mnt\/data\/ws_djh\/mf_system//opt/mf_system}"
    local sim_output_tmp="$result_dir/.sim_output.tmp"

    echo "[sim] 执行仿真..."
    docker exec -i "$CONTAINER_NAME" bash -c \
        "export MFS_ROOT=/opt/mf_system PROJ_NAME=Devcar BENV_ID=devcar_with_cuda11.4 MFS_SYSTEM_CFG_YAML=config/Devcar/system.yaml && cd /opt/mf_system&& ./sim fpp play -b $container_bag_path --product unp --modules-in-loop controller --$method --which-car $car_name" \
        2>&1 | tee "$sim_output_tmp" || true

    # 仿真是否成功以 .fpp.bag 是否生成为准
    if [[ ! -f "${bag_path}.fpp.bag" ]]; then
        echo "Error: 仿真失败，未生成 .fpp.bag (bag=$bag_dir_name)" >&2
        rm -f "$sim_output_tmp"
        return 1
    fi

    echo "[post] 仿真后处理..."
    # 立即提取 Mviz 链接（在 gen_report 之前保存）
    local mviz_txt="$result_dir/mviz_link.txt"
    local mviz_link; mviz_link="$(grep -o 'https://mviz\.momenta\.works[^ ]*' "$sim_output_tmp" 2>/dev/null | tail -1 || true)"
    rm -f "$sim_output_tmp"
    if [[ -n "$mviz_link" ]]; then
        echo "$mviz_link" > "$mviz_txt"
        echo "  Mviz 链接: $mviz_link"
        echo "  已保存至: $mviz_txt"
    else
        echo "  Warning: 未找到 Mviz 链接"
    fi

    # sim 输出 .fpp.bag 到 bag 目录同级，路径为 <bag目录>.fpp.bag
    local fpp_bag="${bag_path}.fpp.bag"
    [[ ! -f "$fpp_bag" ]] && fpp_bag=""
    if [[ -z "$fpp_bag" ]]; then
        echo "Warning: 未找到 .fpp.bag 文件，跳过 bag 移动和 report 生成"
    else
        echo "  移动仿真 bag: $fpp_bag -> $result_dir/"
        mv "$fpp_bag" "$result_dir/"
        local fpp_json="${bag_path}.json"
        if [[ -f "$fpp_json" ]]; then
            echo "  移动仿真 json: $fpp_json -> $result_dir/"
            mv "$fpp_json" "$result_dir/"
        fi
        local fpp_bag_dest="$result_dir/$(basename "$fpp_bag")"
        echo "  生成 report: $fpp_bag_dest"
        "$SCRIPT_DIR/gen_report.sh" "$fpp_bag_dest" || true
    fi

    local fpp_log="$MF_SYSTEM_DIR/logs/fpp.log"
    if [[ -f "$fpp_log" ]]; then
        echo "  复制 fpp.log -> $result_dir/"
        cp "$fpp_log" "$result_dir/"
    else
        echo "  Warning: 未找到 $fpp_log，跳过复制"
    fi
    return 0
}

# ── 主流程 ──────────────────────────────────────────────────────────
echo "=== 仿真配置 ==="
echo "  模式      : $([[ $BATCH_MODE -eq 1 ]] && echo "batch" || echo "单次")"
echo "  method    : $METHOD"
echo "  car_name  : $CAR_NAME"
echo "  mf_system : $MF_SYSTEM_DIR"
echo "  结果根    : $SIM_RESULT_DIR"
echo "  时间戳    : $TIMESTAMP"
echo ""

ensure_container
build_once

if [[ $BATCH_MODE -eq 1 ]]; then
    if [[ "$LIST_FILE" != /* ]]; then
        LIST_FILE="$SCRIPT_DIR/$LIST_FILE"
    fi
    if [[ ! -f "$LIST_FILE" ]]; then
        echo "Error: list file not found: $LIST_FILE" >&2
        exit 1
    fi
    mkdir -p "$SIM_RESULT_DIR/$TIMESTAMP"
    BATCH_LOG="$SIM_RESULT_DIR/$TIMESTAMP/batch.log"
    : > "$BATCH_LOG"
    echo "=== batch run start ==="
    echo "  list_file : $LIST_FILE"
    echo "  batch log : $BATCH_LOG"
    echo ""

    TOTAL=0; OK=0; FAIL=0
    declare -a RESULTS
    # 用 fd 3 读 list_file，避免 run_sim_only 内 docker exec -i 偷走 while 的 stdin
    while IFS= read -r line <&3 || [[ -n "$line" ]]; do
        line="$(trim "$line")"
        [[ -z "$line" ]] && continue
        bag_abs="$(abs_bag_path "$line")"
        bag_dir="$(basename "$bag_abs")"
        TOTAL=$((TOTAL + 1))
        rdir="$SIM_RESULT_DIR/$TIMESTAMP/$bag_dir"
        echo ">>> [$bag_dir] 开始 (TOTAL=$TOTAL) ..."
        if run_sim_only "$bag_abs" "$METHOD" "$CAR_NAME" "$rdir" 2>&1 | tee -a "$BATCH_LOG"; then
            OK=$((OK + 1)); RESULTS+=("OK   $bag_dir  -> $rdir")
        else
            FAIL=$((FAIL + 1)); RESULTS+=("FAIL $bag_dir  -> $rdir")
        fi
    done 3< "$LIST_FILE"

    echo ""
    echo "================ SUMMARY ================"
    echo "  时间戳目录 : $SIM_RESULT_DIR/$TIMESTAMP"
    echo "  total      : $TOTAL"
    echo "  ok         : $OK"
    echo "  failed     : $FAIL"
    if (( ${#RESULTS[@]} > 0 )); then
        for r in "${RESULTS[@]}"; do echo "  $r"; done
    fi
    echo "  batch log  : $BATCH_LOG"
    echo "================ END ================"
else
    bag_abs="$(abs_bag_path "$BAG_PATH")"
    rdir="$SIM_RESULT_DIR/$TIMESTAMP/$(basename "$bag_abs")"
    run_sim_only "$bag_abs" "$METHOD" "$CAR_NAME" "$rdir"
    echo ""
    echo "=== 仿真完成 ==="
    echo "结果保存在: $rdir"
fi
