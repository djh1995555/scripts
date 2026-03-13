#!/bin/bash

# =============================================================================
# 脚本名称: run_replay_workflow.sh
# 功能: 自动执行JIRA回放工作流
# 使用方法: bash run_replay_workflow.sh <jira-id> [jira-record_output_dir] [repo-output-dir]
# 示例1: bash run_replay_workflow.sh ADM2-112827
# 示例1: bash run_replay_workflow.sh ADM2-112827 /path/to/record /path/to/ws
# =============================================================================

# 设置脚本运行目录为当前工作目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_ROOT="$(pwd)"

# 创建日志文件路径
LOG_FILE="${SCRIPT_DIR}/workflow_replay.log"

# 删除旧的日志文件
rm -f "$LOG_FILE"

# 日志记录函数 - 同时输出到控制台和日志文件
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOG_FILE"
}

# 验证JIRA ID格式
validate_jira_id() {
    local jira_id="$1"
    if [[ ! "$jira_id" =~ ^[A-Z0-9]+-[0-9]+$ ]]; then
        return 1
    fi
    if [[ "$jira_id" == *"/"* ]] || [[ "$jira_id" == *".."* ]] || [[ "$jira_id" == *"\\"* ]]; then
        return 1
    fi
    return 0
}

# 检查输入参数
if [ $# -lt 1 ]; then
    log "错误: 请提供JIRA ID作为参数"
    log "使用方法: bash run_replay_workflow.sh <jira-id> [jira-record_output_dir] [repo-output-dir]"
    log "示例: bash run_replay_workflow.sh ADM2-112827"
    log "示例: bash run_replay_workflow.sh ADM2-112827 /path/to/record /path/to/ws"
    exit 1
fi

# 获取输入参数
JIRA_ID="$1"

# 验证JIRA ID格式
if ! validate_jira_id "$JIRA_ID"; then
    log "错误: 无效的JIRA ID格式 '${JIRA_ID}'"
    log "提示: JIRA ID应该是 'PROJECT-12345' 格式"
    exit 1
fi

# 设置jira-record_output_dir（默认为 ${pwd}/record/test_record/jira）
if [ -n "$2" ]; then
    JIRA_RECORD_OUTPUT_DIR="$2"
else
    JIRA_RECORD_OUTPUT_DIR="${SCRIPT_ROOT}/record/test_record/jira"
fi

# 设置repo输出目录（默认为 ${HOME}/ws）
if [ -n "$3" ]; then
    REPO_OUTPUT_DIR="$3"
else
    REPO_OUTPUT_DIR="${HOME}/ws"
fi

log "========================================"
log "开始执行REPLAY工作流"
log "JIRA ID: ${JIRA_ID}"
log "JIRA Record输出目录: ${JIRA_RECORD_OUTPUT_DIR}"
log "Repo输出目录: ${REPO_OUTPUT_DIR}"
log "日志文件: ${LOG_FILE}"
log "========================================"

# 步骤1: 下载jira-record
log "步骤1: 开始下载jira-record..."
log "执行: bash ${SCRIPT_DIR}/run_jira_workflow.sh ${JIRA_ID} ${JIRA_RECORD_OUTPUT_DIR}"
bash "${SCRIPT_DIR}/run_jira_workflow.sh" "$JIRA_ID" "$JIRA_RECORD_OUTPUT_DIR" >> "$LOG_FILE" 2>&1
if [ $? -ne 0 ]; then
    log "错误: 下载jira-record失败，请查看日志文件了解详情"
    exit 1
fi
log "jira-record下载完成"

# 步骤2: 获取JIRA版本信息
log "步骤2: 获取JIRA版本信息..."
log "执行: claude --dangerously-skip-permissions \"/mi-get-jira-manifest ${JIRA_ID}\""
claude --dangerously-skip-permissions "/mi-get-jira-manifest ${JIRA_ID}" >> "$LOG_FILE" 2>&1
if [ $? -ne 0 ]; then
    log "错误: 获取JIRA版本信息失败，请查看日志文件了解详情"
    exit 1
fi
log "JIRA版本信息获取完成"

# 步骤3: 读取manifest_name.txt
log "步骤3: 读取manifest_name.txt..."
MANIFEST_NAME_FILE="${SCRIPT_DIR}/manifest_name.txt"
if [ ! -f "$MANIFEST_NAME_FILE" ]; then
    log "错误: 找不到manifest_name.txt文件"
    exit 1
fi
MANIFEST_NAME=$(cat "$MANIFEST_NAME_FILE")
log "MANIFEST_NAME: ${MANIFEST_NAME}"

# 步骤4: 下载manifest.txt
log "步骤4: 下载manifest.txt..."
log "执行: claude --dangerously-skip-permissions \"/mi-manifest-download ${MANIFEST_NAME}, saved in ${SCRIPT_DIR}\""
claude --dangerously-skip-permissions "/mi-manifest-download ${MANIFEST_NAME}, saved in ${SCRIPT_DIR}" >> "$LOG_FILE" 2>&1
if [ $? -ne 0 ]; then
    log "错误: 下载manifest.txt失败，请查看日志文件了解详情"
    exit 1
fi
log "manifest.txt下载命令已执行"

# 步骤5: 等待manifest下载完成（循环60s，每10s检查一次）
log "步骤5: 等待manifest下载完成..."
WAIT_COUNT=0
MAX_WAIT=6  # 60s / 10s = 6次

while [ $WAIT_COUNT -lt $MAX_WAIT ]; do
    # 查找以manifest开头，revision_restore_script.txt结尾的文件
    MANIFEST_FILE=$(ls "${SCRIPT_DIR}"/manifest*revision_restore_script.txt 2>/dev/null | head -n 1)

    if [ -n "$MANIFEST_FILE" ] && [ -f "$MANIFEST_FILE" ]; then
        log "manifest下载成功: ${MANIFEST_FILE}"
        break
    fi

    WAIT_COUNT=$((WAIT_COUNT + 1))
    if [ $WAIT_COUNT -lt $MAX_WAIT ]; then
        log "等待中... ($WAIT_COUNT/$MAX_WAIT)"
        sleep 10
    fi
done

if [ $WAIT_COUNT -ge $MAX_WAIT ]; then
    log "错误: 等待manifest下载超时（60秒）"
    exit 1
fi

# 验证manifest文件存在
if [ ! -f "$MANIFEST_FILE" ]; then
    log "错误: manifest文件不存在: ${MANIFEST_FILE}"
    exit 1
fi
# MANIFEST_FILE=/home/mi/debug/scripts/manifest_mipilot_mbf_debug_v2_1162553_revision_restore_script.txt
log "MANIFEST_FILE: ${MANIFEST_FILE}"

# 步骤6: 下载repo并编译
log "步骤6: 下载repo并编译..."
log "执行: bash ${SCRIPT_DIR}/run_repo_workflow.sh ${MANIFEST_FILE} build"
bash "${SCRIPT_DIR}/run_repo_workflow.sh" "$MANIFEST_FILE" "build" >> "$LOG_FILE" 2>&1
if [ $? -ne 0 ]; then
    log "错误: 下载repo并编译失败，请查看日志文件了解详情"
    exit 1
fi
log "repo下载并编译完成"

# 从日志中提取repo路径
# 方式1: 从"最终路径: xxx"提取
REPO_DIR=$(grep -oP "最终路径: \K.*" "$LOG_FILE" | tail -n 1)

# 方式2: 从"发现已存在的文件夹: ~/ws/xxx"提取
if [ -z "$REPO_DIR" ]; then
    REPO_DIR=$(grep "发现已存在的文件夹:" "$LOG_FILE" | sed 's/.*: //' | xargs)
fi

# 方式3: 从"Repo已成功移动到: xxx"提取
if [ -z "$REPO_DIR" ]; then
    REPO_DIR=$(grep "Repo已成功移动到:" "$LOG_FILE" | sed 's/.*Repo已成功移动到: //')
fi

# 展开 ~ 为实际家目录路径
REPO_DIR="${REPO_DIR/#\~/$HOME}"

if [ ! -d "$REPO_DIR" ]; then
    log "错误: 找不到repo目录: ${REPO_DIR}"
    exit 1
fi
log "REPO_DIR: ${REPO_DIR}"

# 步骤7: bazel build iceoryx
log "步骤7: 执行bazel build..."
cd "$REPO_DIR"
log "执行: bazel build --config gcc-x86_64 @iceoryx//iceoryx_posh:iox-roudi"
bazel build --config gcc-x86_64 @iceoryx//iceoryx_posh:iox-roudi >> "$LOG_FILE" 2>&1
if [ $? -ne 0 ]; then
    log "错误: bazel build失败，请查看日志文件了解详情"
    exit 1
fi
log "bazel build完成"

# 步骤8: 启动roudi
log "步骤8: 启动roudi..."
# 先检查是否有旧的roudi进程在运行
if pgrep -f "iox-roudi" > /dev/null; then
    log "警告: 旧的roudi进程正在运行，先杀掉"
    pkill -f "iox-roudi"
    sleep 2
fi

log "执行: bazel-bin/external/iceoryx/iceoryx_posh/iox-roudi -m on -d ./"
nohup bazel-bin/external/iceoryx/iceoryx_posh/iox-roudi -m on -d ./ >> "$LOG_FILE" 2>&1 &
ROUDI_PID=$!
log "roudi已启动，PID: ${ROUDI_PID}"

# 等待roudi启动
sleep 5

# 检查roudi是否成功启动
if ! ps -p $ROUDI_PID > /dev/null 2>&1; then
    log "错误: roudi启动失败"
    exit 1
fi
log "roudi启动成功"

# 步骤9: 执行replay
log "步骤9: 执行回放..."
cd "$SCRIPT_DIR"
# 组成jira-dir路径
JIRA_DIR="${JIRA_RECORD_OUTPUT_DIR}/${JIRA_ID}"
if [ ! -d "$JIRA_DIR" ]; then
    log "错误: 找不到jira目录: ${JIRA_DIR}"
    kill $ROUDI_PID 2>/dev/null
    exit 1
fi
log "JIRA_DIR: ${JIRA_DIR}"
log "REPO_DIR: ${REPO_DIR}"

# 直接在当前终端执行replay
log "执行: bash ${SCRIPT_DIR}/start_replay.sh replay_single ${JIRA_DIR} ${REPO_DIR}"
bash "${SCRIPT_DIR}/start_replay.sh" "replay_single" "$JIRA_DIR" "$REPO_DIR" >> "$LOG_FILE" 2>&1
if [ $? -ne 0 ]; then
    log "错误: replay执行失败，请查看日志文件了解详情"
    kill $ROUDI_PID 2>/dev/null
    exit 1
fi
log "replay执行完成"

# 步骤10: 清理
log "步骤10: 清理临时文件..."

# 关闭roudi进程
if ps -p $ROUDI_PID > /dev/null 2>&1; then
    log "关闭roudi进程..."
    kill $ROUDI_PID 2>/dev/null
    sleep 2
    # 如果进程仍然存在，强制杀掉
    if ps -p $ROUDI_PID > /dev/null 2>&1; then
        kill -9 $ROUDI_PID 2>/dev/null
    fi
fi

删除manifest_name.txt和manifest.txt
if [ -f "$MANIFEST_NAME_FILE" ]; then
    rm -f "$MANIFEST_NAME_FILE"
    log "已删除: ${MANIFEST_NAME_FILE}"
fi

if [ -f "$MANIFEST_FILE" ]; then
    rm -f "$MANIFEST_FILE"
    log "已删除: ${MANIFEST_FILE}"
fi

# 工作流完成汇总
log "========================================"
log "REPLAY工作流执行完成"
log "JIRA ID: ${JIRA_ID}"
log "Repo目录: ${REPO_DIR}"
log "Replay记录目录: ${REPLAY_RECORD_DIR}"
log "日志文件: ${LOG_FILE}"
log "========================================"

exit 0
