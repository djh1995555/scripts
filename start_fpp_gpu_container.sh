#!/usr/bin/env bash
# 创建供 run_sim.sh 使用的本地 FPP GPU 容器。
#
# GPU 与 mount namespace 权限必须在创建阶段配置，不能用 docker exec 补齐。
# 使用 --replace 会删除同名容器本身，但工作区与 Conan 缓存均是宿主机 bind mount，
# 不会删除源码、构建产物或缓存。

set -euo pipefail

usage() {
    cat <<EOF
Usage: $0 [--replace] <container_name>

  --replace  删除同名容器后重建（GPU 或源码隔离权限不满足时）。
             会丢失容器可写层，请先确认没有运行任务并备份所需数据。
EOF
    exit 1
}

REPLACE=0
if [[ "${1:-}" == "--replace" ]]; then
    REPLACE=1
    shift
fi
[[ $# -eq 1 ]] || usage
CONTAINER_NAME="$1"

SCRIPT_DIR="$(cd -P "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/fpp_container_checks.sh"
MF_SYSTEM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CACHE_DIR="$MF_SYSTEM_DIR/.conan_data_cache"
IMAGE="${FPP_CONTAINER_IMAGE:-artifactory.momenta.works/docker-momenta/devcar_with_cuda11.4_fpp_base:maf5.0-v0.1.19}"

if ! nvidia-smi -L >/dev/null 2>&1; then
    echo "Error: 宿主机未检测到可用 NVIDIA GPU，无法运行 GPU 序列化的 Koopman 模型。" >&2
    exit 1
fi

if docker inspect "$CONTAINER_NAME" >/dev/null 2>&1; then
    if docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME" | grep -qx true &&
       docker exec "$CONTAINER_NAME" bash -lc 'nvidia-smi -L >/dev/null 2>&1 && test -x /opt/mf_system/sim' &&
       check_fpp_source_mounts "$CONTAINER_NAME"; then
        echo "FPP GPU 容器已就绪: $CONTAINER_NAME"
        exit 0
    fi
    if [[ "$REPLACE" -ne 1 ]]; then
        echo "Error: 同名容器已存在但未就绪: $CONTAINER_NAME" >&2
        echo "       已确认可重建时，请执行: $0 --replace $CONTAINER_NAME" >&2
        exit 1
    fi
    echo "[replace] 删除同名容器: $CONTAINER_NAME"
    docker rm -f "$CONTAINER_NAME" >/dev/null
fi

mkdir -p "$CACHE_DIR"
docker run -d --name "$CONTAINER_NAME" --network host --gpus all \
    --cap-add SYS_ADMIN \
    --security-opt seccomp=unconfined \
    --security-opt apparmor=unconfined \
    -e RUN_IN_BUILD_ENV=TRUE \
    -e RUN_IN_FPP_CONTAINER=TRUE \
    -e MF_SYSTEM_ROOT_DIR=/opt/mf_system \
    -e MFS_ROOT=/root/modo_mf_system \
    -e PROJ_NAME=FPP \
    -e BENV_ID=fpp \
    -e MFS_SYSTEM_CFG_YAML=config/Fpp/system.yaml \
    -v "$CACHE_DIR:/root/data_cache" \
    -v /dev/shm:/dev/shm \
    -v "$MF_SYSTEM_DIR:/opt/mf_system" \
    "$IMAGE" tail -f /dev/null >/dev/null

if ! docker exec "$CONTAINER_NAME" bash -lc 'nvidia-smi -L >/dev/null 2>&1 && test -x /opt/mf_system/sim' ||
   ! check_fpp_source_mounts "$CONTAINER_NAME"; then
    echo "Error: GPU FPP 容器创建后校验失败: $CONTAINER_NAME" >&2
    exit 1
fi
echo "FPP GPU 容器已创建（支持 --source-dir 隔离挂载）: $CONTAINER_NAME"
