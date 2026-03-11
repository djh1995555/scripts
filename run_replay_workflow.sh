#!/bin/bash

# =============================================================================
# 脚本名称: run_workflow3.sh
# 功能: 自动化执行完整的JIRA记录下载、代码仓库下载、编译和回灌测试流程
# 使用方法: bash run_workflow3.sh <jira-ids> <repo-name>
# 示例: bash run_workflow3.sh ADM2-112827 manifest_mipilot_mbf_2926_revision_restore_script.txt
# =============================================================================

# 设置脚本运行目录为当前工作目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 创建日志文件路径（使用时间戳命名）
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="${SCRIPT_DIR}/workflow_replay.log"

# 删除旧的日志文件
rm -f "$LOG_FILE"

# 日志记录函数 - 同时输出到控制台和日志文件
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOG_FILE"
}

# =============================================================================
# 安全验证函数
# =============================================================================

# 验证JIRA ID格式函数 - 防止路径遍历攻击
validate_jira_id() {
    local jira_id="$1"
    # JIRA ID格式: 项目前缀(大写字母+数字) + 连字符 + 数字
    # 例如: ADM2-112827, PROJ-12345, TEST-1
    if [[ ! "$jira_id" =~ ^[A-Z0-9]+-[0-9]+$ ]]; then
        return 1
    fi
    # 检查是否包含路径分隔符或危险字符
    if [[ "$jira_id" == *"/"* ]] || [[ "$jira_id" == *".."* ]] || [[ "$jira_id" == *"\\"* ]]; then
        return 1
    fi
    return 0
}

# 验证repo名称函数 - 防止路径遍历攻击
validate_repo_name() {
    local repo_name="$1"
    # 检查是否包含路径分隔符或危险字符
    if [[ "$repo_name" == *"/"* ]] || [[ "$repo_name" == *".."* ]] || [[ "$repo_name" == *"\\"* ]]; then
        return 1
    fi
    # 检查文件名是否合法（只允许字母、数字、下划线、点和连字符）
    if [[ ! "$repo_name" =~ ^[a-zA-Z0-9._-]+$ ]]; then
        return 1
    fi
    return 0
}

# 从repo名称中提取核心关键词
# 例如: manifest_mipilot_mbf_2926_revision_restore_script.txt -> mipilot_mbf_2926
extract_repo_keyword() {
    local repo_name="$1"
    # 提取manifest_和_revision之间的内容
    local keyword=$(echo "$repo_name" | sed -n 's/manifest_\(.*\)_revision.*/\1/p')
    echo "$keyword"
}

# 检查~/ws目录下是否存在包含指定关键词的文件夹
check_existing_repo() {
    local keyword="$1"
    local ws_dir="${HOME}/ws"

    # 如果~/ws目录不存在，直接返回未找到
    if [ ! -d "$ws_dir" ]; then
        return 1
    fi

    # 查找包含关键词的文件夹
    local found_dir=$(find "$ws_dir" -maxdepth 1 -type d -name "*${keyword}*" 2>/dev/null | head -n 1)

    if [ -n "$found_dir" ]; then
        echo "$found_dir"
        return 0
    fi
    return 1
}

# =============================================================================
# 清理函数 - 脚本退出时停止roudi
# =============================================================================
ROUDI_PID=""

cleanup() {
    log "========================================"
    log "执行清理操作..."
    if [ -n "$ROUDI_PID" ]; then
        log "停止roudi进程 (PID: $ROUDI_PID)..."
        kill "$ROUDI_PID" 2>/dev/null || true
        # 等待进程结束
        wait "$ROUDI_PID" 2>/dev/null || true
        log "roudi进程已停止"
    fi
    log "清理完成"
    log "========================================"
}

# 注册清理函数到脚本退出信号
trap cleanup EXIT INT TERM

# =============================================================================
# 主流程
# =============================================================================

# 检查输入参数
if [ $# -lt 2 ]; then
    log "错误: 参数不足"
    log "使用方法: bash run_workflow3.sh <jira-ids> <repo-name>"
    log "示例: bash run_workflow3.sh ADM2-112827 manifest_mipilot_mbf_2926_revision_restore_script.txt"
    exit 1
fi

# 获取输入参数
JIRA_IDS="$1"
REPO_NAME="$2"

# 验证JIRA ID格式
if ! validate_jira_id "$JIRA_IDS"; then
    log "错误: 无效的JIRA ID格式 '${JIRA_IDS}'"
    log "提示: JIRA ID应该是 'PROJECT-12345' 格式"
    exit 1
fi

# 验证repo名称
if ! validate_repo_name "$REPO_NAME"; then
    log "错误: 无效的repo名称 '${REPO_NAME}'"
    log "提示: repo名称不能包含路径分隔符或特殊字符"
    exit 1
fi

log "========================================"
log "开始执行REPLAY工作流"
log "JIRA IDs: ${JIRA_IDS}"
log "Repo名称: ${REPO_NAME}"
log "日志文件: ${LOG_FILE}"
log "========================================"

# 步骤1: 运行run_jira_workflow.sh
log ""
log "步骤1: 运行run_jira_workflow.sh下载JIRA记录..."

if [ ! -f "${SCRIPT_DIR}/run_jira_workflow.sh" ]; then
    log "错误: 找不到 run_jira_workflow.sh 文件"
    exit 1
fi

bash "${SCRIPT_DIR}/run_jira_workflow.sh" "$JIRA_IDS" >> "$LOG_FILE" 2>&1
JIRA_EXIT_CODE=$?
if [ $JIRA_EXIT_CODE -ne 0 ]; then
    log "错误: run_jira_workflow.sh 执行失败，退出码: ${JIRA_EXIT_CODE}"
    exit 1
fi
log "run_jira_workflow.sh 执行完成"

# 步骤2: 检查~/ws目录下是否已存在符合条件的文件夹
log ""
log "步骤2: 检查~/ws目录下是否已存在符合条件的文件夹..."

REPO_KEYWORD=$(extract_repo_keyword "$REPO_NAME")
REPO_DIR=""
SKIP_DOWNLOAD=false

if [ -n "$REPO_KEYWORD" ]; then
    log "提取的关键词: ${REPO_KEYWORD}"

    EXISTING_DIR=$(check_existing_repo "$REPO_KEYWORD")
    if [ $? -eq 0 ]; then
        log "发现已存在的文件夹: ${EXISTING_DIR}"
        log "跳过步骤3的repo下载"
        REPO_DIR="$EXISTING_DIR"
        SKIP_DOWNLOAD=true
    else
        log "未发现包含关键词 '${REPO_KEYWORD}' 的文件夹，将继续执行repo下载"
    fi
else
    log "无法从repo名称提取关键词，将继续执行repo下载"
fi

# 步骤3: 运行run_repo_workflow.sh（如果需要）
if [ "$SKIP_DOWNLOAD" = false ]; then
    log ""
    log "步骤3: 运行run_repo_workflow.sh下载代码仓库..."

    if [ ! -f "${SCRIPT_DIR}/run_repo_workflow.sh" ]; then
        log "错误: 找不到 run_repo_workflow.sh 文件"
        exit 1
    fi

    bash "${SCRIPT_DIR}/run_repo_workflow.sh" "$REPO_NAME" >> "$LOG_FILE" 2>&1
    REPO_EXIT_CODE=$?
    if [ $REPO_EXIT_CODE -ne 0 ]; then
        log "错误: run_repo_workflow.sh 执行失败，退出码: ${REPO_EXIT_CODE}"
        exit 1
    fi

    # 从日志中提取repo路径
    log "从日志中提取repo路径..."
    # run_repo_workflow.sh 的日志写入自己的 workflow_repo.log
    REPO_LOG_FILE="${SCRIPT_DIR}/workflow_repo.log"
    REPO_DIR=$(grep "最终路径:" "$REPO_LOG_FILE" | tail -n 1 | sed 's/.*最终路径: //')
    if [ -z "$REPO_DIR" ]; then
        log "错误: 无法从日志中提取repo路径"
        exit 1
    fi
    log "提取的repo路径: ${REPO_DIR}"
else
    log "步骤3: 跳过repo下载（已存在符合条件的文件夹）"
fi

# 验证repo目录是否存在
if [ ! -d "$REPO_DIR" ]; then
    log "错误: repo目录不存在: ${REPO_DIR}"
    exit 1
fi

# 步骤4: 切换到repo目录
log ""
log "步骤4: 切换到repo目录: ${REPO_DIR}"
cd "$REPO_DIR"
CD_EXIT_CODE=$?
if [ $CD_EXIT_CODE -ne 0 ]; then
    log "错误: 无法切换到repo目录，退出码: ${CD_EXIT_CODE}"
    exit 1
fi
log "当前工作目录: $(pwd)"

# 步骤5: 运行replay_adaption_revert.sh
log ""
log "步骤5: 运行replay_adaption_revert.sh..."

if [ ! -f "./replay_adaption_revert.sh" ]; then
    # 尝试从code目录复制
    if [ -f "${SCRIPT_DIR}/code/replay_adaption_revert.sh" ]; then
        log "从code目录复制replay_adaption_revert.sh..."
        cp "${SCRIPT_DIR}/code/replay_adaption_revert.sh" ./
    else
        log "错误: 找不到replay_adaption_revert.sh文件"
        exit 1
    fi
fi

bash ./replay_adaption_revert.sh >> "$LOG_FILE" 2>&1
REVERT_EXIT_CODE=$?
if [ $REVERT_EXIT_CODE -ne 0 ]; then
    log "警告: replay_adaption_revert.sh 执行失败或不完全成功，退出码: ${REVERT_EXIT_CODE}，继续执行..."
fi
log "replay_adaption_revert.sh 执行完成"

# 步骤6: 运行replay_adaption_apply.sh
log ""
log "步骤6: 运行replay_adaption_apply.sh..."

if [ ! -f "./replay_adaption_apply.sh" ]; then
    # 尝试从code目录复制
    if [ -f "${SCRIPT_DIR}/code/replay_adaption_apply.sh" ]; then
        log "从code目录复制replay_adaption_apply.sh..."
        cp "${SCRIPT_DIR}/code/replay_adaption_apply.sh" ./
    else
        log "错误: 找不到replay_adaption_apply.sh文件"
        exit 1
    fi
fi

bash ./replay_adaption_apply.sh >> "$LOG_FILE" 2>&1
APPLY_EXIT_CODE=$?
if [ $APPLY_EXIT_CODE -ne 0 ]; then
    log "错误: replay_adaption_apply.sh 执行失败，退出码: ${APPLY_EXIT_CODE}"
    exit 1
fi
log "replay_adaption_apply.sh 执行完成"

# 步骤7: 执行build.sh编译
log ""
log "步骤7: 执行build.sh编译..."

if [ ! -f "./build.sh" ]; then
    log "错误: 找不到build.sh文件"
    exit 1
fi

# 执行构建并捕获退出码
bash ./build.sh "mipilot/modules/parking/controller/..." >> "$LOG_FILE" 2>&1
BUILD_EXIT_CODE=$?
if [ $BUILD_EXIT_CODE -ne 0 ]; then
    log "错误: build.sh 执行失败，退出码: ${BUILD_EXIT_CODE}"
    exit 1
fi
log "build.sh 执行完成"

sleep 10

# 步骤8: 编译iceoryx roudi
log ""
log "步骤8: 编译iceoryx roudi..."

bazel build --config gcc-x86_64 @iceoryx//iceoryx_posh:iox-roudi >> "$LOG_FILE" 2>&1
BAZEL_EXIT_CODE=$?
if [ $BAZEL_EXIT_CODE -ne 0 ]; then
    log "错误: bazel build roudi 执行失败，退出码: ${BAZEL_EXIT_CODE}"
    exit 1
fi
log "bazel build roudi 执行完成"

# 步骤9: 启动roudi进程
log ""
log "步骤9: 启动roudi进程..."

ROUDI_PATH="./bazel-bin/external/iceoryx/iceoryx_posh/iox-roudi"
if [ ! -f "$ROUDI_PATH" ]; then
    log "错误: 找不到roudi可执行文件: ${ROUDI_PATH}"
    exit 1
fi

# 在后台启动roudi
$ROUDI_PATH -m on -d ./ &
ROUDI_PID=$!
log "roudi进程已启动，PID: ${ROUDI_PID}"

# 等待roudi启动
log "等待roudi启动..."
sleep 3

# 检查roudi是否仍在运行
if ! kill -0 "$ROUDI_PID" 2>/dev/null; then
    log "错误: roudi进程启动失败或已退出"
    exit 1
fi
log "roudi进程运行正常"

# 步骤10: 在新终端中执行start_replay.sh
log ""
log "步骤10: 在新终端中执行start_replay.sh进行回灌..."

# 定义JIRA记录目录（run_jira_workflow.sh下载的记录位置）
JIRA_RECORD_DIR="${SCRIPT_DIR}/record/test_record/jira/${JIRA_IDS}"

# 检查JIRA记录目录是否存在
if [ ! -d "$JIRA_RECORD_DIR" ]; then
    log "警告: JIRA记录目录不存在: ${JIRA_RECORD_DIR}"
    log "尝试查找其他可能的记录目录..."

    # 尝试查找最近创建的记录目录
    JIRA_BASE_DIR="${SCRIPT_DIR}/record/test_record/jira"
    if [ -d "$JIRA_BASE_DIR" ]; then
        JIRA_RECORD_DIR=$(find "$JIRA_BASE_DIR" -type d -name "${JIRA_IDS}" 2>/dev/null | head -1)
    fi

    if [ -z "$JIRA_RECORD_DIR" ] || [ ! -d "$JIRA_RECORD_DIR" ]; then
        log "错误: 找不到JIRA记录目录"
        exit 1
    fi
fi

log "JIRA记录目录: ${JIRA_RECORD_DIR}"

# 定义回灌输出目录
REPLAY_OUTPUT_DIR="${SCRIPT_DIR}/record/replay_record"

# 确保输出目录存在
mkdir -p "$REPLAY_OUTPUT_DIR"

# 检查start_replay.sh是否存在
if [ ! -f "${SCRIPT_DIR}/start_replay.sh" ]; then
    log "错误: 找不到start_replay.sh文件"
    exit 1
fi

# 在新终端中执行start_replay.sh
# 使用 gnome-terminal 或 xterm
log "尝试在新终端中启动start_replay.sh..."

# 检查是否有图形显示环境
if [ -z "$DISPLAY" ] && [ -z "$WAYLAND_DISPLAY" ]; then
    log "提示: 未检测到图形显示环境(DISPLAY/WAYLAND_DISPLAY未设置)"
    log "将在后台执行start_replay.sh，而非新终端..."
    USE_GUI_TERMINAL=false
else
    USE_GUI_TERMINAL=true
fi

# 根据环境选择执行方式
if [ "$USE_GUI_TERMINAL" = true ]; then
    # 检查可用的终端模拟器
    if command -v gnome-terminal &> /dev/null; then
        # 使用 --wait 选项让 gnome-terminal 等待命令执行完毕后再关闭
        gnome-terminal -- bash -c "cd '${SCRIPT_DIR}' && echo '工作目录: '\$(pwd) && bash start_replay.sh replay_single '${JIRA_RECORD_DIR}' '${REPO_DIR}'; echo '回灌完成，终端将自动关闭'" &
        log "已使用gnome-terminal启动新终端"
    elif command -v xterm &> /dev/null; then
        xterm -e "cd '${SCRIPT_DIR}' && echo '工作目录: '\$(pwd) && bash start_replay.sh replay_single '${JIRA_RECORD_DIR}' '${REPO_DIR}'; echo '回灌完成，终端将自动关闭'" &
        log "已使用xterm启动新终端"
    elif command -v konsole &> /dev/null; then
        konsole --workdir "${SCRIPT_DIR}" -e bash -c "echo '工作目录: '\$(pwd) && bash start_replay.sh replay_single '${JIRA_RECORD_DIR}' '${REPO_DIR}'; echo '回灌完成，终端将自动关闭'" &
        log "已使用konsole启动新终端"
    else
        log "警告: 未找到可用的终端模拟器(gnome-terminal/xterm/konsole)"
        USE_GUI_TERMINAL=false
    fi
fi

# 如果无法使用GUI终端，则在后台执行
if [ "$USE_GUI_TERMINAL" = false ]; then
    log "在当前终端后台执行start_replay.sh..."
    cd "$SCRIPT_DIR"
    bash start_replay.sh replay_single "$JIRA_RECORD_DIR" "${REPO_DIR}" > "$LOG_FILE" 2>&1 &
    REPLAY_PID=$!
    log "已在后台启动start_replay.sh，PID: ${REPLAY_PID}"
fi

log ""
log "========================================"
log "工作流3执行完成"
log "JIRA IDs: ${JIRA_IDS}"
log "Repo名称: ${REPO_NAME}"
log "Repo目录: ${REPO_DIR}"
log "JIRA记录目录: ${JIRA_RECORD_DIR}"
log "日志文件: ${LOG_FILE}"
log "========================================"
log ""
log "注意: roudi进程仍在后台运行 (PID: ${ROUDI_PID})"
if [ "$USE_GUI_TERMINAL" = true ]; then
    log "start_replay.sh已在新终端中启动，使用replay_single模式"
else
    log "start_replay.sh已在当前终端后台启动，PID: ${REPLAY_PID}，使用replay_single模式"
fi
log ""

# 步骤11: 等待并验证结果
log "步骤11: 等待回灌完成并验证结果..."
log "等待60秒让回灌生成文件..."
sleep 60

log ""
log "========================================"
log "验证结果"
log "========================================"

# 查找record/replay_record目录下的新文件夹
NEWEST_DIR=$(find "$REPLAY_OUTPUT_DIR" -type d -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2-)

if [ -n "$NEWEST_DIR" ]; then
    log "发现最新的目录: ${NEWEST_DIR}"

    # 递归查找.html文件
    HTML_FILES=$(find "$NEWEST_DIR" -name "*.html" -type f 2>/dev/null)

    if [ -n "$HTML_FILES" ]; then
        log "✅ 验证通过: 发现HTML文件:"
        echo "$HTML_FILES" | while read -r html_file; do
            log "  - ${html_file}"
        done
        log "========================================"
        log "测试通过！工作流3执行成功！"
        log "========================================"
        exit 0
    else
        log "⚠️  警告: 未发现HTML文件，回灌可能尚未完成"
        log "请稍后手动检查目录: ${REPLAY_OUTPUT_DIR}"
        log ""
        log "等待额外60秒再次检查..."
        sleep 60

        # 再次检查
        HTML_FILES=$(find "$NEWEST_DIR" -name "*.html" -type f 2>/dev/null)
        if [ -n "$HTML_FILES" ]; then
            log "✅ 验证通过: 发现HTML文件:"
            echo "$HTML_FILES" | while read -r html_file; do
                log "  - ${html_file}"
            done
            log "========================================"
            log "测试通过！工作流3执行成功！"
            log "========================================"
            exit 0
        else
            log "❌ 验证失败: 仍未发现HTML文件"
            log "请手动检查目录: ${REPLAY_OUTPUT_DIR}"
            exit 1
        fi
    fi
else
    log "❌ 验证失败: 未发现新创建的目录"
    log "请手动检查目录: ${REPLAY_OUTPUT_DIR}"
    exit 1
fi
