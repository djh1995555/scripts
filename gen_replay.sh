      
#!/bin/bash
set -e

SIM_LAUNCH_BIN=/mnt/data/ws_djh/mf_system/workdir/Devcar/repos_dir/base/sim_launcher/common/build/install/sim_launch/bin
WORK_DIR=/mnt/data/ws_djh/mf_system/debug/fpp_replay/00000000/defalut

DC_LIB=/mnt/data/ws_djh/mf_system/workdir/Devcar/repos_dir/driving_control/build/install/lib
# driving_control 的 resource：install/resource 提供 config/mfr_nodes（无 model），
# deploy/.../resource/model 提供 koopman tar；两者互补，父子挂载（父 resource 在前、model 子在后）
DC_INSTALL_RESOURCE=/mnt/data/ws_djh/mf_system/workdir/Devcar/repos_dir/driving_control/build/install/resource
DC_DEPLOY_MODEL=/mnt/data/ws_djh/mf_system/workdir/Devcar/repos_dir/driving_control/build/deploy/modules/driving_control/resource/model

usage() {
    echo "Usage: $0 -t <task_id> -b <bag_name> [-s <stage>] [-o <output_dir>] [--sl] [--dc] [-c]"
    echo "  -t    Task ID (required)"
    echo "  -b    Bag name (required)"
    echo "  -s    Stage, default: FPP"
    echo "  -o    Output dir (WORK_DIR), default: $WORK_DIR"
    echo "  --sl  Use local sim_launch binary instead of the one in the image"
    echo "  --dc  Use local driving_control lib instead of the one in the image"
    echo "  -c    Run controller only (no GPU)"
    exit 1
}

STAGE=FPP
USE_LOCAL_SIM_LAUNCH=0
USE_LOCAL_DC=0
RUN_PLANNING=1
while [[ $# -gt 0 ]]; do
    case $1 in
        -t) TASK_ID=$2; shift 2 ;;
        -b) BAG_NAME=$2; shift 2 ;;
        -s) STAGE=$2; shift 2 ;;
        -o) WORK_DIR=$2; shift 2 ;;
        --sl) USE_LOCAL_SIM_LAUNCH=1; shift ;;
        --dc) USE_LOCAL_DC=1; shift ;;
        -c) RUN_PLANNING=0; shift ;;
        -h) usage ;;
        *) usage ;;
    esac
done

[ -z "$TASK_ID" ] && { echo "Error: -t task_id is required"; usage; }
[ -z "$BAG_NAME" ] && { echo "Error: -b bag_name is required"; usage; }

mkdir -p "$WORK_DIR"
cd "$WORK_DIR"
# 归一化为绝对路径：-o 可能传相对路径，cd 后 cwd 已变，
# 后续 $WORK_DIR/... 引用必须用绝对路径才不会重复拼接
WORK_DIR="$(pwd)"
# OUTPUT_SCRIPT 须在 WORK_DIR 归一化之后计算，否则仍是默认 fpp_replay2 路径
OUTPUT_SCRIPT="$WORK_DIR/replay_local.sh"

# 每次强制重新下载 input bag，避免复用不同 task 的旧 bag
rm -f "$WORK_DIR/input/${BAG_NAME}.bag"

# < /dev/null：isim 在 token 失效时会交互式读取 username，非交互批量运行下读到 EOF
# 既失败又会偷走 run_vsim.sh while 循环喂给 LIST_FILE 的 stdin（与下方 docker run
# < /dev/null 同因），重定向到空设备根治。
isim replay -t "$TASK_ID" -b "$BAG_NAME" --stage "$STAGE" --dry-run < /dev/null

# isim 退出码不可靠（token 失效读到 EOF 仍 exit 0），显式校验 replay.sh 是否真的生成
if [[ ! -s "$WORK_DIR/replay.sh" ]]; then
    echo "Error: isim replay 未能生成 replay.sh（token 失效或网络错误）" >&2
    echo "       提示：若 token 过期请先运行 'isim auth login' 或 'isim auth refresh'" >&2
    exit 1
fi

# 从 replay.sh 提取 docker run 那行，改造成 replay_local.sh
DOCKER_LINE=$(grep '^docker run' "$WORK_DIR/replay.sh" || true)
if [[ -z "$DOCKER_LINE" ]]; then
    echo "Error: $WORK_DIR/replay.sh 中未找到 'docker run' 行" >&2
    exit 1
fi

# 统一 input/output bag 文件名为 ${BAG_NAME}.bag：
# isim 下载的 input 可能带 _reindex_lz4 等后缀（bag_name=..._0 但实际文件=..._0_reindex_lz4.bag），
# 导致 run_vsim.sh 的 out_bag=${bag_name}.bag 找不到。这里 rename input + 改 DOCKER_LINE 的
# --input/--output 文件名，使仿真直接输出 ${BAG_NAME}.bag，run_vsim.sh 无需 glob 兜底。
ACTUAL_INPUT_BAG=$(ls "$WORK_DIR/input"/*.bag 2>/dev/null | head -1)
if [[ -n "$ACTUAL_INPUT_BAG" ]]; then
    ACTUAL_INPUT_NAME=$(basename "$ACTUAL_INPUT_BAG")
    if [[ "$ACTUAL_INPUT_NAME" != "${BAG_NAME}.bag" ]]; then
        mv "$ACTUAL_INPUT_BAG" "$WORK_DIR/input/${BAG_NAME}.bag"
        echo "Renamed input bag: $ACTUAL_INPUT_NAME -> ${BAG_NAME}.bag"
        # DOCKER_LINE 里 --input /data/input/X_reindex_lz4.bag 和 --output /data/output/X_reindex_lz4.bag
        # 文件名部分替换成 ${BAG_NAME}.bag（路径前缀 /data/input /data/output 不变）
        DOCKER_LINE="${DOCKER_LINE//$ACTUAL_INPUT_NAME/${BAG_NAME}.bag}"
        echo "Updated DOCKER_LINE --input/--output -> ${BAG_NAME}.bag"
    fi
fi

# 1. 按需加 --gpus all
if [ "$RUN_PLANNING" -eq 1 ]; then
    DOCKER_LINE="${DOCKER_LINE/docker run -i --rm/docker run -i --rm --gpus all}"
fi

# 2. 按需挂载本地 driving_control lib
if [ "$USE_LOCAL_DC" -eq 1 ]; then
    DOCKER_LINE="${DOCKER_LINE/-w\/deploy/-w\/deploy -v ${DC_LIB}:\/deploy\/modules\/driving_control\/lib -v ${DC_INSTALL_RESOURCE}:\/deploy\/modules\/driving_control\/resource -v ${DC_DEPLOY_MODEL}:\/deploy\/modules\/driving_control\/resource\/model}"
    echo "Using local driving_control lib:      $DC_LIB"
    echo "Using local driving_control resource: $DC_INSTALL_RESOURCE"
    echo "Using local driving_control model:    $DC_DEPLOY_MODEL"
fi

# 3. 按需挂载本地 sim_launch bin
if [ "$USE_LOCAL_SIM_LAUNCH" -eq 1 ]; then
    DOCKER_LINE="${DOCKER_LINE/-w\/deploy/-w\/deploy -v ${SIM_LAUNCH_BIN}:\/fpp_bin\/}"
    DOCKER_LINE=$(echo "$DOCKER_LINE" | sed 's|\(/deploy/common/bash/sim_launch\.sh .*\)|bash -c "cp /fpp_bin/sim_launch /deploy/common/bin/ \&\& \1"|')
    echo "Using local sim_launch: $SIM_LAUNCH_BIN"
fi

# 4. 透传固定 gain 实验开关：宿主设 KP_FIXED_GAIN=<值> 则注入容器，覆盖 koopman ONNX gain
if [ -n "${KP_FIXED_GAIN:-}" ]; then
    DOCKER_LINE="${DOCKER_LINE/-w\/deploy/-w\/deploy -e KP_FIXED_GAIN=${KP_FIXED_GAIN}}"
    echo "Using fixed koopman gain: KP_FIXED_GAIN=${KP_FIXED_GAIN}"
fi

# 5. 透传 koopman omega 权重补偿：宿主设 KP_OMEGA_W_SCALE=<系数> 则注入容器
if [ -n "${KP_OMEGA_W_SCALE:-}" ]; then
    DOCKER_LINE="${DOCKER_LINE/-w\/deploy/-w\/deploy -e KP_OMEGA_W_SCALE=${KP_OMEGA_W_SCALE}}"
    echo "Using koopman omega w scale: KP_OMEGA_W_SCALE=${KP_OMEGA_W_SCALE}"
fi

# 6. 透传 koopman Y(lat_err) 权重补偿：宿主设 KP_Y_W_SCALE=<系数> 则注入容器
if [ -n "${KP_Y_W_SCALE:-}" ]; then
    DOCKER_LINE="${DOCKER_LINE/-w\/deploy/-w\/deploy -e KP_Y_W_SCALE=${KP_Y_W_SCALE}}"
    echo "Using koopman Y w scale: KP_Y_W_SCALE=${KP_Y_W_SCALE}"
fi

OUTPUT_BAG="$WORK_DIR/output/${BAG_NAME}.bag"
LOG_FILE="$WORK_DIR/logs/${BAG_NAME}.log"

cat > "$OUTPUT_SCRIPT" <<SCRIPT
#!/bin/bash
set -e

mkdir -p "$WORK_DIR/logs"
echo "Logging to: $LOG_FILE"
# < /dev/null：docker run -i 默认继承宿主 stdin，会读走批量循环喂给
# while 的 LIST_FILE（导致后续 bag 被跳过）；重定向到空设备根治
$DOCKER_LINE < /dev/null 2>&1 > "$LOG_FILE"
cat "$LOG_FILE"

# mviz：用 glob 查找实际 output bag（应对 _reindex_lz4 等后缀，bag_name 与实际文件名不一致），
# 失败不阻断（|| true），mviz 失败不应中断 replay_local.sh（后续 gen_report/分析仍可继续）
ACTUAL_BAG=\$(ls "$WORK_DIR/output"/*.bag 2>/dev/null | head -1)
if [[ -n "\$ACTUAL_BAG" ]]; then
  isim mviz -i "\$ACTUAL_BAG" || true
else
  echo "Warning: no output bag found in $WORK_DIR/output, skip mviz"
fi
SCRIPT

chmod +x "$OUTPUT_SCRIPT"
echo "Generated: $OUTPUT_SCRIPT"
echo "Run with: bash $OUTPUT_SCRIPT"

    