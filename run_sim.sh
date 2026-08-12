#!/usr/bin/env bash
# run_sim.sh — 按照 sim_flow.md 执行仿真流程

set -euo pipefail

# ── 参数解析 ────────────────────────────────────────────────────────
usage() {
    echo "Usage: $0 <bag路径> <method> <car_name>"
    echo "  bag路径   : bag 文件所在目录（相对或绝对路径）"
    echo "  method    : close-loop 或 open-loop"
    echo "  car_name  : 车辆名称（如 PL567）"
    echo ""
    echo "Example: $0 bags/横向失控1 close-loop PL567"
    exit 1
}

[[ $# -ne 3 ]] && usage

BAG_PATH="$1"
METHOD="$2"
CAR_NAME="$3"

# 校验 method 参数
if [[ "$METHOD" != "close-loop" && "$METHOD" != "open-loop" ]]; then
    echo "Error: method 必须是 close-loop 或 open-loop，当前值: $METHOD"
    exit 1
fi

# ── 路径计算 ────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MF_SYSTEM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SIM_RESULT_DIR="$SCRIPT_DIR/sim_result"

# 将 bag 路径转为绝对路径
if [[ "$BAG_PATH" != /* ]]; then
    BAG_PATH="$SCRIPT_DIR/$BAG_PATH"
fi

# bag 目录名，结果目录结构：sim_result/<时间戳>/<bag目录名>
BAG_DIR_NAME="$(basename "$BAG_PATH")"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
RESULT_DIR="$SIM_RESULT_DIR/$TIMESTAMP/$BAG_DIR_NAME"

echo "=== 仿真配置 ==="
echo "  bag路径     : $BAG_PATH"
echo "  method      : $METHOD"
echo "  car_name    : $CAR_NAME"
echo "  mf_system   : $MF_SYSTEM_DIR"
echo "  结果目录    : $RESULT_DIR"
echo ""

# ── 创建结果目录 ────────────────────────────────────────────────────
mkdir -p "$RESULT_DIR"
echo "[1/4] 创建结果目录: $RESULT_DIR"

# ── 进入 mf_system 目录，执行 docker 内的编译和仿真 ─────────────────
cd "$MF_SYSTEM_DIR"

# 若容器不存在，自动以 --gpus all 创建（与原容器保持一致，仅增加 GPU 访问权限）
CONTAINER_NAME="fpp-container-mnt-data-ws_djh-mf_system"
if ! docker inspect "$CONTAINER_NAME" &>/dev/null; then
    echo "[prep] 容器不存在，自动创建（带 GPU）..."
    ./sim fpp container start
    echo "[prep] 容器已就绪"
fi

echo "[2/4] 在 docker 中编译..."
# 将宿主机路径转换为容器内路径（宿主机 /mnt/data/ws_djh/mf_system -> 容器内 /opt/mf_system）
CONTAINER_BAG_PATH="${BAG_PATH/\/mnt\/data\/ws_djh\/mf_system//opt/mf_system}"

docker exec -i fpp-container-mnt-data-ws_djh-mf_system bash -c \
    "export MFS_ROOT=/opt/mf_system PROJ_NAME=Devcar BENV_ID=devcar_with_cuda11.4 MFS_SYSTEM_CFG_YAML=config/Devcar/system.yaml && cd /opt/mf_system&& ./sim fpp build -i control,driving_control"

echo "[3/4] 执行仿真..."
SIM_OUTPUT_TMP="$RESULT_DIR/.sim_output.tmp"
docker exec -i fpp-container-mnt-data-ws_djh-mf_system bash -c \
    "export MFS_ROOT=/opt/mf_system PROJ_NAME=Devcar BENV_ID=devcar_with_cuda11.4 MFS_SYSTEM_CFG_YAML=config/Devcar/system.yaml && cd /opt/mf_system&& ./sim fpp play -b $CONTAINER_BAG_PATH --product unp --modules-in-loop controller --$METHOD --which-car $CAR_NAME -v" \
    2>&1 | tee "$SIM_OUTPUT_TMP" || true
# 仿真本身是否成功以 .fpp.bag 是否生成为准
if [[ ! -f "${BAG_PATH}.fpp.bag" ]]; then
    echo "Error: 仿真失败，未生成 .fpp.bag"
    exit 1
fi

# ── 仿真后处理 ──────────────────────────────────────────────────────
echo "[4/4] 仿真后处理..."

# 立即提取 Mviz 链接（仿真结束时已产生，在 gen_report 之前保存）
MVIZ_TXT="$RESULT_DIR/mviz_link.txt"
MVIZ_LINK="$(grep -o 'https://mviz\.momenta\.works[^ ]*' "$SIM_OUTPUT_TMP" 2>/dev/null | tail -1 || true)"
rm -f "$SIM_OUTPUT_TMP"
if [[ -n "$MVIZ_LINK" ]]; then
    echo "$MVIZ_LINK" > "$MVIZ_TXT"
    echo "  Mviz 链接: $MVIZ_LINK"
    echo "  已保存至: $MVIZ_TXT"
else
    echo "  Warning: 未找到 Mviz 链接"
fi

# sim 输出 .fpp.bag 到 bag 目录的同级，路径为 <bag目录>.fpp.bag
FPP_BAG="${BAG_PATH}.fpp.bag"
[[ ! -f "$FPP_BAG" ]] && FPP_BAG=""
if [[ -z "$FPP_BAG" ]]; then
    echo "Warning: 未找到 .fpp.bag 文件，跳过 bag 移动和 report 生成"
else
    echo "  移动仿真 bag: $FPP_BAG -> $RESULT_DIR/"
    mv "$FPP_BAG" "$RESULT_DIR/"

    # 移动 .json 文件
    FPP_JSON="${BAG_PATH}.json"
    if [[ -f "$FPP_JSON" ]]; then
        echo "  移动仿真 json: $FPP_JSON -> $RESULT_DIR/"
        mv "$FPP_JSON" "$RESULT_DIR/"
    fi

    # 生成 report
    FPP_BAG_DEST="$RESULT_DIR/$(basename "$FPP_BAG")"
    echo "  生成 report: $FPP_BAG_DEST"
    "$SCRIPT_DIR/gen_report.sh" "$FPP_BAG_DEST" || true
fi

# 复制 fpp.log
FPP_LOG="$MF_SYSTEM_DIR/logs/fpp.log"
if [[ -f "$FPP_LOG" ]]; then
    echo "  复制 fpp.log -> $RESULT_DIR/"
    cp "$FPP_LOG" "$RESULT_DIR/"
else
    echo "  Warning: 未找到 $FPP_LOG，跳过复制"
fi

echo ""
echo "=== 仿真完成 ==="
echo "结果保存在: $RESULT_DIR"
