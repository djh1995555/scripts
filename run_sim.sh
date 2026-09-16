#!/usr/bin/env bash
# run_sim.sh — 按照 sim_flow.md 执行仿真流程（支持单次 / --batch 批量）

set -euo pipefail

# ── 参数解析 ────────────────────────────────────────────────────────
usage() {
    cat <<EOF
Usage:
  单次: $0 --container <container_name> [--source-dir <目录>] [--skip-mviz-login] [--enable-mviz] <bag路径> <method> <car_name>
  批量: $0 --container <container_name> [--source-dir <目录>] [--skip-mviz-login] [--enable-mviz] --batch <list_file> <method> <car_name>

  container_name: 已启动的 FPP Docker 容器名（必填）
  bag路径   : bag 文件所在目录（相对脚本目录或绝对路径）
  list_file : 每行一个 bag 目录（相对脚本目录或绝对路径），# 开头注释 / 空行跳过
  method    : close-loop 或 open-loop
  car_name  : 车辆名称（如 PL567 / LC6-EC65130）
  --source-dir: driving_control 源码目录（相对当前工作目录或绝对路径，默认脚本目录下的 ../workdir/Devcar/repos_dir/driving_control）
  --skip-mviz-login: 不在开始时交互登录 Mviz（仅供已有认证态的自动化任务使用）
  --enable-mviz: 即使跳过登录也向 Mviz 上传（须已有有效认证态）

Example:
  $0 --container <container_name> bags/横向失控1 close-loop PL567
  $0 --container <container_name> --batch replay_list.txt close-loop LC6-EC65130
EOF
    exit 1
}

METHOD=""
CAR_NAME=""
BATCH_MODE=0
LIST_FILE=""
BAG_PATH=""
CONTAINER_NAME=""
SKIP_MVIZ_LOGIN=0
ENABLE_MVIZ=0
SOURCE_DIR=""

if [[ "${1:-}" != "--container" || -z "${2:-}" ]]; then
    echo "Error: 必须使用 --container 指定一个已启动的容器名。" >&2
    usage
fi
CONTAINER_NAME="$2"
shift 2

while [[ "${1:-}" == --* && "${1:-}" != "--batch" ]]; do
    case "$1" in
        --source-dir)
            [[ -n "${2:-}" && "$2" != --* ]] || usage
            SOURCE_DIR="$2"
            shift 2
            ;;
        --skip-mviz-login) SKIP_MVIZ_LOGIN=1; shift ;;
        --enable-mviz) ENABLE_MVIZ=1; shift ;;
        *) usage ;;
    esac
done

# Interactive login has always implied upload.  Keep that default while also
# allowing a batch to reuse an existing login with both flags above.
if [[ $SKIP_MVIZ_LOGIN -eq 0 ]]; then
    ENABLE_MVIZ=1
fi

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
# Resolve the physical directory: driving_control/debug is commonly a symlink
# to mf_system/debug, while the FPP container only mounts the latter path.
SCRIPT_DIR="$(cd -P "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/fpp_container_checks.sh"
# 从 SCRIPT_DIR 向上找含 ./sim 的 mf_system 根。
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
# driving_control 的模型依赖清单；在每个 case 结果目录中留档，便于追溯
# 本次仿真实际使用的 Koopman Conan 包版本。
DEFAULT_SOURCE_DIR="$SCRIPT_DIR/../workdir/Devcar/repos_dir/driving_control"
SOURCE_DIR="${SOURCE_DIR:-$DEFAULT_SOURCE_DIR}"
if [[ ! -d "$SOURCE_DIR" ]]; then
    echo "Error: 源码目录不存在: $SOURCE_DIR" >&2
    exit 1
fi
SOURCE_DIR="$(cd -P "$SOURCE_DIR" && pwd)"
if [[ "$SOURCE_DIR" != "$MF_SYSTEM_DIR/"* || ! -f "$SOURCE_DIR/Makefile" ]]; then
    echo "Error: 源码必须位于 $MF_SYSTEM_DIR 内，且包含 Makefile: $SOURCE_DIR" >&2
    exit 1
fi
CONTAINER_SOURCE_DIR="/opt/mf_system${SOURCE_DIR#"$MF_SYSTEM_DIR"}"
DEPENDS_JSON="$SOURCE_DIR/depends.json"

# 透传到容器的环境变量（ENV_OPTS 在 build_once / run_sim_only 的 docker exec 上注入）
LOWMU_V2="${LOWMU_V2:-0 }"
LOWMU_V2_MODEL="${LOWMU_V2_MODEL:-eclass_b30v8_4d_g2_s7}"
ENV_OPTS=(-e "LOWMU_V2=$LOWMU_V2" -e "LOWMU_V2_MODEL=$LOWMU_V2_MODEL")

# The deploy tree is reused by incremental FPP builds.  For Koopman validation
# runs, make the file read by sim_launch match the repository configuration
# after compilation and before the first replay.  This is opt-in so ordinary
# simulator users retain their existing workflow.
sync_koopman_runtime_config() {
    [[ "${REQUIRE_KOOPMAN_ACTIVE:-0}" == "1" ]] || return 0
    local dc_dir="$(dirname "$DEPENDS_JSON")"
    local source_config="$dc_dir/resource/config/controller.msquare.lc6.230228.json"
    local container_dc_dir="/opt/mf_system${dc_dir#"$MF_SYSTEM_DIR"}"
    local container_source_config="$container_dc_dir/resource/config/controller.msquare.lc6.230228.json"
    local container_deployed_config="/opt/mf_system/workdir/Devcar/build/host/Devcar/deploy_unstrip/modules/driving_control/resource/config/controller.msquare.lc6.230228.json"
    python3 - "$source_config" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as f:
    data = json.load(f)
found = {}
def walk(value):
    if isinstance(value, dict):
        for key, item in value.items():
            if key in {"use_koopman_lat_ctrl", "use_koopman_as_default"}:
                found[key] = item
            walk(item)
    elif isinstance(value, list):
        for item in value:
            walk(item)
walk(data)
if found.get("use_koopman_lat_ctrl") is not True or found.get("use_koopman_as_default") is not True:
    raise SystemExit(f"Koopman source config invalid: {found}")
PY
    if ! docker exec "$CONTAINER_NAME" test -f "$container_deployed_config"; then
        echo "Error: 未找到部署 Koopman 配置：$container_deployed_config" >&2
        return 1
    fi
    docker exec "$CONTAINER_NAME" cp "$container_source_config" "$container_deployed_config"
    docker exec "$CONTAINER_NAME" python3 - "$container_deployed_config" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as f:
    data = json.load(f)
found = {}
def walk(value):
    if isinstance(value, dict):
        for key, item in value.items():
            if key in {"use_koopman_lat_ctrl", "use_koopman_as_default"}:
                found[key] = item
            walk(item)
    elif isinstance(value, list):
        for item in value:
            walk(item)
walk(data)
if found.get("use_koopman_lat_ctrl") is not True or found.get("use_koopman_as_default") is not True:
    raise SystemExit(f"Koopman deployed config invalid: {found}")
print("[koopman] 部署配置已同步：feature/default 均开启")
PY
}

abs_bag_path() {
    local p="$1"
    if [[ "$p" != /* ]]; then p="$SCRIPT_DIR/$p"; fi
    printf '%s' "$p"
}

# rosbag reindex may retain the pre-indexed simulation artifact as
# <output>.orig.bag.  It is neither the replay input nor the usable
# simulation result, so retain only the indexed output bag in this run's
# result directory.
remove_sim_orig_bag() {
    local output_bag="$1"
    local orig_bag="${output_bag%.bag}.orig.bag"
    if [[ -f "$orig_bag" ]]; then
        rm -f -- "$orig_bag"
        echo "  清理仿真中间 bag: $orig_bag"
    fi
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
        echo "Error: 容器不存在: $CONTAINER_NAME" >&2
        exit 1
    fi
    if [[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME")" != "true" ]]; then
        echo "Error: 容器未运行: $CONTAINER_NAME" >&2
        exit 1
    fi
    if ! docker exec "$CONTAINER_NAME" test -x /opt/mf_system/sim; then
        echo "Error: 容器未挂载可用的 mf_system 到 /opt/mf_system: $CONTAINER_NAME" >&2
        exit 1
    fi

    # CUDA 是可选能力。无 GPU/无 CUDA 环境允许继续以 CPU 模式运行。
    if ! docker exec "$CONTAINER_NAME" bash -c 'ldconfig -p 2>/dev/null | grep -q "libcuda.so.1"' 2>/dev/null; then
        _HOST_LIBCUDA="$(ldconfig -p 2>/dev/null | grep -o '/[^ ]*libcuda\.so\.[0-9.]*$' | head -1 || true)"
        if [[ -n "$_HOST_LIBCUDA" ]]; then
            _DEST_DIR="$(docker exec "$CONTAINER_NAME" bash -c 'dirname "$(ldconfig -p 2>/dev/null | grep -o "/[^ ]*libc\.so\.6$" | head -1)"' 2>/dev/null)"
            [[ -z "$_DEST_DIR" ]] && _DEST_DIR="/usr/lib/x86_64-linux-gnu"
            echo "[prep] 容器缺 libcuda.so.1，从宿主机 $_HOST_LIBCUDA 补齐到 $_DEST_DIR/"
            docker cp "$_HOST_LIBCUDA" "$CONTAINER_NAME:$_DEST_DIR/$(basename "$_HOST_LIBCUDA")" 2>/dev/null || true
            docker exec "$CONTAINER_NAME" bash -c "ldconfig" 2>/dev/null || true
        else
            echo "[prep] 未检测到宿主机 CUDA；以无 GPU/无 CUDA 模式运行。"
        fi
    else
        echo "[prep] 容器检测到 libcuda.so.1；启用 CUDA 环境。"
    fi
}

# Mviz 云端上传依赖 sim 的交互认证。让 sim 直接占用终端读取凭据，
# 避免用户名或密码进入 shell 变量、命令行、环境变量或日志。
login_mviz() {
    if [[ $SKIP_MVIZ_LOGIN -eq 1 ]]; then
        echo "[mviz] 跳过交互登录（使用已有认证态）。"
        return 0
    fi
    if [[ ! -t 0 || ! -t 1 ]]; then
        echo "Error: Mviz 登录需要交互终端。请在终端运行，或仅在已有认证态时使用 --skip-mviz-login。" >&2
        exit 1
    fi
    echo "[mviz] 请在下方输入 Mviz 用户名和密码；凭据不会写入脚本或日志。"
    docker exec -it "$CONTAINER_NAME" bash -lc 'cd /opt/mf_system && ./sim login'
    echo "[mviz] 交互登录完成。"
}

build_once() {
    echo "[build] 在 docker 中编译 control,driving_control ..."
    echo "[build] driving_control 源码: $SOURCE_DIR"
    docker exec -i "${ENV_OPTS[@]}" "$CONTAINER_NAME" bash -c \
        'set -euo pipefail
         source_dir="$1"
         host_root="$2"
         canonical=/opt/mf_system/workdir/Devcar/repos_dir/driving_control
         test -f "$source_dir/Makefile"
         if [[ "$(readlink -f "$source_dir")" != "$(readlink -f "$canonical")" ]]; then
             # Only this process tree sees the alternate checkout.
             exec unshare --mount --propagation private bash -c '\''
                 set -euo pipefail
                 # Worktree .git files contain absolute host paths. Keep the
                 # original common Git directory accessible before overlaying.
                 mkdir -p "$3"
                 mount --bind /opt/mf_system "$3"
                 mount --bind "$1" "$2"
                 git -C "$2" rev-parse HEAD
                 export MFS_ROOT=/opt/mf_system PROJ_NAME=Devcar BENV_ID=devcar_with_cuda11.4 MFS_SYSTEM_CFG_YAML=config/Devcar/system.yaml
                 cd /opt/mf_system
                 ./sim fpp build -i control,driving_control
             '\'' bash "$source_dir" "$canonical" "$host_root"
         fi
         export MFS_ROOT=/opt/mf_system PROJ_NAME=Devcar BENV_ID=devcar_with_cuda11.4 MFS_SYSTEM_CFG_YAML=config/Devcar/system.yaml
         cd /opt/mf_system
         ./sim fpp build -i control,driving_control' bash "$CONTAINER_SOURCE_DIR" "$MF_SYSTEM_DIR"
    if [[ -n "${KOOPMAN_SIM_MINFER_LIB:-}" ]]; then
        [[ -f "$KOOPMAN_SIM_MINFER_LIB" ]] || { echo "Missing Minfer override" >&2; return 1; }
        local deployed_lib="$MF_SYSTEM_DIR/workdir/Devcar/build/host/Devcar/deploy_unstrip/common/lib/libminfer.so"
        [[ -f "${deployed_lib}.before_koopman_540" ]] || cp -p "$deployed_lib" "${deployed_lib}.before_koopman_540"
        cp "$KOOPMAN_SIM_MINFER_LIB" "$deployed_lib"
        echo "[runtime] Minfer override: $KOOPMAN_SIM_MINFER_LIB"
    fi
    sync_koopman_runtime_config
    echo "[build] ************************ 编译完成 ************************"
}

# run_sim_only <bag_path_abs> <method> <car_name> <result_dir>
# 只做仿真 + 后处理；容器准备与编译由调用方在循环外完成一次
run_sim_only() {
    local bag_path="$1" method="$2" car_name="$3" result_dir="$4"
    local bag_dir_name; bag_dir_name="$(basename "$bag_path")"

    mkdir -p "$result_dir"
    if [[ -f "$DEPENDS_JSON" ]]; then
        cp "$DEPENDS_JSON" "$result_dir/depends.json"
        echo "  依赖清单  : $result_dir/depends.json"
    else
        echo "Warning: 未找到 depends.json，跳过依赖清单留档: $DEPENDS_JSON" >&2
    fi
    echo "--------------------------------------------------"
    echo "[bag] $bag_dir_name"
    echo "[bag] bag路径  : $bag_path"
    echo "[bag] 结果目录 : $result_dir"
    echo "--------------------------------------------------"

    # The player accepts a file; batch entries can be directories. Preserve
    # the directory name for result grouping and select its replay input.
    if [[ -d "$bag_path" ]]; then
        bag_path="$(python3 - "$bag_path" <<'PYBAG'
from pathlib import Path
import sys
root = Path(sys.argv[1])
inputs = [p for p in root.glob('*.bag')
          if not p.name.endswith(('.fpp.bag', '.orig.bag'))]
if not inputs:
    raise SystemExit(f'No input bag in {root}')
if len({(p.stat().st_dev, p.stat().st_ino) for p in inputs}) != 1:
    raise SystemExit(f'Ambiguous input bags in {root}: {[p.name for p in inputs]}')
print(min(inputs, key=lambda p: (len(p.name), p.name)))
PYBAG
)" || return 1
    fi

    local output_bag="${bag_path%.bag}.fpp.bag"
    local output_json="${bag_path}.json"

    # 将当前工作区中的 bag 路径转换为容器挂载路径。
    # 不依赖特定宿主机目录，以支持不同 /mnt/data 工作区。
    local container_bag_path
    if [[ "$bag_path" == "$MF_SYSTEM_DIR" || "$bag_path" == "$MF_SYSTEM_DIR/"* ]]; then
        container_bag_path="/opt/mf_system${bag_path#"$MF_SYSTEM_DIR"}"
    else
        container_bag_path="$bag_path"
    fi
    local sim_output_tmp="$result_dir/.sim_output.tmp"
    # The caller may deliberately reuse a prior Mviz authentication state:
    # --skip-mviz-login --enable-mviz requests upload without another prompt.
    local mviz_arg=""
    if [[ $ENABLE_MVIZ -eq 1 ]]; then
        mviz_arg=" --mviz"
    fi

    echo "[sim] 执行仿真..."
    if ! docker exec -i "${ENV_OPTS[@]}" "$CONTAINER_NAME" bash -c \
        "export MFS_ROOT=/opt/mf_system PROJ_NAME=Devcar BENV_ID=devcar_with_cuda11.4 MFS_SYSTEM_CFG_YAML=config/Devcar/system.yaml && cd /opt/mf_system&& ./sim fpp play -b $container_bag_path --product unp --modules-in-loop controller --$method --which-car $car_name$mviz_arg" \
        2>&1 | tee "$sim_output_tmp"; then
        # A crashed player can leave a partially-written .fpp.bag behind.  Do
        # not let that artifact turn a failed simulation into a successful
        # batch entry merely because the file exists.
        echo "Error: 仿真进程非零退出 (bag=$bag_dir_name)" >&2
        local fpp_log="$MF_SYSTEM_DIR/logs/fpp.log"
        if [[ -f "$fpp_log" ]]; then
            cp "$fpp_log" "$result_dir/fpp.log"
        fi
        if [[ -f "$output_bag" ]]; then
            mv "$output_bag" "$result_dir/${bag_dir_name}.failed.fpp.bag"
        fi
        if [[ -f "$output_json" ]]; then
            mv "$output_json" "$result_dir/${bag_dir_name}.failed.json"
        fi
        rm -f "$sim_output_tmp"
        return 1
    fi

    # 仿真是否成功以 .fpp.bag 是否生成为准
    if [[ ! -f "$output_bag" ]]; then
        echo "Error: 仿真失败，未生成 .fpp.bag (bag=$bag_dir_name)" >&2
        [[ ! -f "$MF_SYSTEM_DIR/logs/fpp.log" ]] || cp "$MF_SYSTEM_DIR/logs/fpp.log" "$result_dir/fpp.log"
        rm -f "$sim_output_tmp"
        return 1
    fi

    # sim_launch may abort after creating its output file.  A mere file-exists
    # check would then incorrectly count a truncated, unindexed rosbag as a
    # successful replay.  Validate the rosbag index before post-processing.
    if ! rosbag info "$output_bag" >/dev/null 2>&1; then
        echo "Error: 仿真输出 bag 无法读取或未建立索引 (bag=$bag_dir_name)" >&2
        rm -f "$sim_output_tmp"
        if [[ -f "$output_bag" ]]; then
            mv "$output_bag" "$result_dir/${bag_dir_name}.failed.fpp.bag"
        fi
        if [[ -f "$output_json" ]]; then
            mv "$output_json" "$result_dir/${bag_dir_name}.failed.json"
        fi
        return 1
    fi

    echo "[post] 仿真后处理..."
    # sim fpp play --mviz may emit a local viewer URL, but not necessarily a
    # cloud URL.  Retain any direct URL here; a durable cloud upload happens
    # after the valid output bag has been moved below.
    local mviz_txt="$result_dir/mviz_link.txt"
    local mviz_link; mviz_link="$(grep -oE 'https://mviz\.momenta\.works[^[:space:]]*|http://[^[:space:]]+/local\?cpp_server=[^[:space:]]*' "$sim_output_tmp" 2>/dev/null | tail -1 || true)"
    if [[ -z "$mviz_link" ]]; then
        mviz_link="$(docker exec "$CONTAINER_NAME" bash -lc 'curl -fsS http://127.0.0.1:8000/mviz/link 2>/dev/null' | sed -n 's/.*"link":"\([^"]*\)".*/\1/p')"
    fi
    # daemon 会在 Mviz 服务未启动时也返回本地 URL；确认前端可达后再保存，
    # 避免输出一个打开后没有内容的假链接。
    if [[ "$mviz_link" =~ ^http://.+/local\?cpp_server= ]]; then
        local mviz_view_url="${mviz_link%%\?*}"
        if ! curl -fsS --connect-timeout 3 "$mviz_view_url" >/dev/null 2>&1; then
            echo "  Warning: Mviz 本地服务未启动，未保存不可用链接"
            mviz_link=""
        fi
    fi
    rm -f "$sim_output_tmp"
    if [[ -n "$mviz_link" ]]; then
        echo "$mviz_link" > "$mviz_txt"
        echo "  Mviz 链接: $mviz_link"
        echo "  已保存至: $mviz_txt"
    else
        echo "  未从 sim 输出取得 Mviz 链接；将在 bag 落盘后上传。"
    fi

    # sim 输出 .fpp.bag 到 bag 目录同级，路径为 <bag目录>.fpp.bag
    local fpp_bag="$output_bag"
    [[ ! -f "$fpp_bag" ]] && fpp_bag=""
    if [[ -z "$fpp_bag" ]]; then
        echo "Warning: 未找到 .fpp.bag 文件，跳过 bag 移动和 report 生成"
    else
        echo "  移动仿真 bag: $fpp_bag -> $result_dir/"
        mv "$fpp_bag" "$result_dir/"
        local fpp_json="$output_json"
        if [[ -f "$fpp_json" ]]; then
            echo "  移动仿真 json: $fpp_json -> $result_dir/"
            mv "$fpp_json" "$result_dir/"
        fi
        local fpp_bag_dest="$result_dir/$(basename "$fpp_bag")"
        if [[ $ENABLE_MVIZ -eq 1 ]]; then
            echo "  上传 Mviz: $fpp_bag_dest"
            local upload_output
            upload_output="$(isim mviz --input "$fpp_bag_dest" 2>&1)"
            printf '%s\n' "$upload_output"
            local uploaded_mviz_link
            uploaded_mviz_link="$(printf '%s\n' "$upload_output" | grep -oE 'https://mviz\.momenta\.works[^[:space:]]*' | tail -1 || true)"
            if [[ -n "$uploaded_mviz_link" ]]; then
                printf '%s\n' "$uploaded_mviz_link" > "$mviz_txt"
                echo "  Mviz 链接: $uploaded_mviz_link"
                echo "  已保存至: $mviz_txt"
            else
                echo "  Warning: Mviz 上传未返回链接" >&2
            fi
        fi
        echo "  生成 report: $fpp_bag_dest"
        "$SCRIPT_DIR/gen_report.sh" "$fpp_bag_dest" || true
        remove_sim_orig_bag "$fpp_bag_dest"
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
echo "  container : $CONTAINER_NAME"
echo "  Mviz 登录 : $([[ $SKIP_MVIZ_LOGIN -eq 1 ]] && echo "跳过" || echo "开始时交互登录")"
echo "  Mviz 上传 : $([[ $ENABLE_MVIZ -eq 1 ]] && echo "开启" || echo "关闭")"
echo "  LOWMU_V2 : $LOWMU_V2"
echo "  LOWMU_V2_MODEL: $LOWMU_V2_MODEL"
echo "  mf_system : $MF_SYSTEM_DIR"
echo "  源码目录  : $SOURCE_DIR"
echo "  结果根    : $SIM_RESULT_DIR"
echo "  时间戳    : $TIMESTAMP"
echo ""

ensure_container
if [[ "$SOURCE_DIR" != "$(cd -P "$DEFAULT_SOURCE_DIR" && pwd)" ]]; then
    if ! check_fpp_source_mounts "$CONTAINER_NAME"; then
        echo "Error: --source-dir 需要 SYS_ADMIN 及允许 unshare/mount 的 seccomp/AppArmor 配置。" >&2
        echo "       请用 $SCRIPT_DIR/start_fpp_gpu_container.sh 创建支持源码隔离的容器。" >&2
        echo "       已有容器需先备份可写层并确认无运行任务，再使用 --replace 重建。" >&2
        exit 1
    fi
fi
login_mviz
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
