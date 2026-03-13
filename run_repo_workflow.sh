#!/bin/bash

# =============================================================================
# 脚本名称: run_repo_workflow.sh
# 功能: 下载代码仓库、复制代码文件并执行构建
# 使用方法: bash run_repo_workflow.sh <repo-name> <build>
# 示例: bash run_repo_workflow.sh manifest_mipilot_mbf_2926_revision_restore_script.txt build
# =============================================================================

# 设置脚本运行目录为当前工作目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 创建日志文件路径（使用时间戳命名）
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="${SCRIPT_DIR}/workflow_repo.log"

# 删除旧的日志文件
rm -f "$LOG_FILE"

# 日志记录函数 - 同时输出到控制台和日志文件
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOG_FILE"
}

# 验证repo名称函数 - 防止路径遍历攻击
validate_repo_name() {
    local repo_name="$1"
    # 检查是否包含路径分隔符或危险字符
    if [[ "$repo_name" == *"/"* ]] || [[ "$repo_name" == *".."* ]] || [[ "$repo_name" == *"\\"* ]]; then
        return 1
    fi
    # 检查文件名是否合法（只允许字母、数字、下划线和点）
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
        echo "$(basename "$found_dir")"
        return 0
    fi
    return 1
}

# 检查输入参数
if [ $# -lt 1 ]; then
    log "错误: 请提供repo名称或manifest路径作为参数"
    log "使用方法: bash run_repo_workflow.sh <repo-path> [build]"
    log "示例: bash run_repo_workflow.sh /home/mi/debug/scripts/manifest_mipilot_mbf_2926_revision_restore_script.txt"
    log "示例: bash run_repo_workflow.sh /home/mi/debug/scripts/manifest_mipilot_mbf_2926_revision_restore_script.txt build"
    exit 1
fi

# 是否需要编译（默认不需要）
NEED_BUILD=false
if [ "$2" = "build" ]; then
    NEED_BUILD=true
    log "将执行编译步骤"
fi

# 获取输入的参数（必须是完整路径）
INPUT_PATH="$1"

# 检查是否传入的是完整路径
if [[ "$INPUT_PATH" != *"/"* ]]; then
    log "错误: 请传入完整的文件路径，而不是文件名"
    log "示例: bash run_repo_workflow.sh /home/mi/debug/scripts/manifest_mipilot_mbf_2926_revision_restore_script.txt"
    exit 1
fi

REPO_NAME=$(basename "$INPUT_PATH")
REPO_PATH="$INPUT_PATH"

# 验证repo名称
if ! validate_repo_name "$REPO_NAME"; then
    log "错误: 无效的repo名称 '${REPO_NAME}'"
    log "提示: repo名称不能包含特殊字符"
    exit 1
fi

log "========================================"
log "开始执行REPO_DOWNLOAD工作流"
log "Repo名称: ${REPO_NAME}"
log "Repo路径: ${REPO_PATH}"
log "日志文件: ${LOG_FILE}"
log "========================================"

# 步骤0: 检查~/ws目录下是否已存在符合条件的文件夹
log "步骤0: 检查~/ws目录下是否已存在符合条件的文件夹..."

REPO_KEYWORD=$(extract_repo_keyword "$REPO_NAME")
if [ -n "$REPO_KEYWORD" ]; then
    log "提取的关键词: ${REPO_KEYWORD}"

    EXISTING_DIR=$(check_existing_repo "$REPO_KEYWORD")
    if [ $? -eq 0 ]; then
        log "发现已存在的文件夹: ~/ws/${EXISTING_DIR}"
        log "该repo已经下载过，无需重复操作，退出脚本"
        exit 0
    else
        log "未发现包含关键词 '${REPO_KEYWORD}' 的文件夹，继续执行..."
    fi
else
    log "无法从repo名称提取关键词，跳过此检查"
fi

# 步骤1: 检查repo文件是否存在
log "步骤1: 检查repo文件是否存在..."
if [ ! -f "$REPO_PATH" ]; then
    log "错误: 找不到repo文件: ${REPO_PATH}"
    exit 1
fi
log "repo文件存在: ${REPO_PATH}"

# 步骤2: 执行repo下载脚本
log "步骤2: 执行repo下载脚本..."
log "执行: bash ${REPO_PATH}"

# 执行脚本并捕获输出
DOWNLOAD_OUTPUT=$(bash "$REPO_PATH" 2>&1)
EXIT_CODE=$?

# 将输出写入日志
echo "$DOWNLOAD_OUTPUT" >> "$LOG_FILE"

# 检查执行是否成功
if [ $EXIT_CODE -ne 0 ]; then
    log "错误: repo下载脚本执行失败，退出码: ${EXIT_CODE}"
    exit 1
fi

log "repo下载脚本执行完成"

# 步骤3: 从输出中提取repo路径
log "步骤3: 从输出中提取repo路径..."

# 获取最后一行输出
LAST_LINE=$(echo "$DOWNLOAD_OUTPUT" | tail -n 1)
log "最后一行输出: ${LAST_LINE}"

# 提取路径（假设格式为 "...路径: /tmp/xxx"）
EXTRACTED_PATH=$(echo "$LAST_LINE" | sed -n 's/.*: \(.*\)/\1/p' | xargs)

# 如果没有匹配到，尝试直接使用最后一行
if [ -z "$EXTRACTED_PATH" ]; then
    EXTRACTED_PATH=$(echo "$LAST_LINE" | xargs)
fi

log "提取的repo路径: ${EXTRACTED_PATH}"

# 验证提取的路径是否存在
if [ ! -d "$EXTRACTED_PATH" ]; then
    log "错误: 提取的路径不存在或不是目录: ${EXTRACTED_PATH}"
    exit 1
fi

# 验证路径是否在/tmp目录下（安全检查）
if [[ ! "$EXTRACTED_PATH" =~ ^/tmp/ ]]; then
    log "警告: 提取的路径不在/tmp目录下: ${EXTRACTED_PATH}"
    log "继续执行..."
fi

# 步骤4: 将repo移动到~/ws
log "步骤4: 将repo移动到~/ws..."

WS_DIR="${HOME}/ws"

# 创建~/ws目录（如果不存在）
if [ ! -d "$WS_DIR" ]; then
    log "创建目录: ${WS_DIR}"
    mkdir -p "$WS_DIR"
fi

# 获取repo目录名
REPO_DIR_NAME=$(basename "$EXTRACTED_PATH")
TARGET_PATH="${WS_DIR}/${REPO_DIR_NAME}"

log "移动: ${EXTRACTED_PATH} -> ${TARGET_PATH}"

# 如果目标目录已存在，先删除
if [ -d "$TARGET_PATH" ]; then
    log "目标目录已存在，先删除: ${TARGET_PATH}"
    rm -rf "$TARGET_PATH"
fi

# 执行移动
mv "$EXTRACTED_PATH" "$TARGET_PATH"

if [ $? -ne 0 ]; then
    log "错误: 移动repo失败"
    exit 1
fi

log "repo已成功移动到: ${TARGET_PATH}"

# 步骤5: 复制code目录里的文件到repo根目录
log "步骤5: 复制code目录里的文件到repo根目录..."

CODE_DIR="${SCRIPT_DIR}/code"

if [ ! -d "$CODE_DIR" ]; then
    log "警告: code目录不存在: ${CODE_DIR}，跳过复制步骤"
else
    log "复制 ${CODE_DIR} 中的文件到 ${TARGET_PATH}/"
    # 使用cp -r将code目录中的所有文件/子目录复制到目标目录
    # 而不是复制整个code目录
    cp -r "${CODE_DIR}"/* "$TARGET_PATH/" 2>/dev/null
    COPY_EXIT=$?
    if [ $COPY_EXIT -eq 0 ]; then
        log "code目录中的文件复制成功"
    else
        log "警告: code目录中可能没有文件，或复制失败 (退出码: ${COPY_EXIT})"
    fi
fi

# 步骤6: 运行replay_adaption_revert.sh
log "步骤6: 运行replay_adaption_revert.sh..."

if [ ! -f "${TARGET_PATH}/replay_adaption_revert.sh" ]; then
    # 尝试从code目录复制
    if [ -f "${SCRIPT_DIR}/code/replay_adaption_revert.sh" ]; then
        log "从code目录复制replay_adaption_revert.sh..."
        cp "${SCRIPT_DIR}/code/replay_adaption_revert.sh" "${TARGET_PATH}/"
    else
        log "错误: 找不到replay_adaption_revert.sh文件"
        exit 1
    fi
fi

cd "$TARGET_PATH"
bash ./replay_adaption_revert.sh >> "$LOG_FILE" 2>&1
REVERT_EXIT_CODE=$?
if [ $REVERT_EXIT_CODE -ne 0 ]; then
    log "警告: replay_adaption_revert.sh 执行失败或不完全成功，退出码: ${REVERT_EXIT_CODE}，继续执行..."
fi
log "replay_adaption_revert.sh 执行完成"

# 步骤7: 运行replay_adaption_apply.sh
log "步骤7: 运行replay_adaption_apply.sh..."

if [ ! -f "${TARGET_PATH}/replay_adaption_apply.sh" ]; then
    # 尝试从code目录复制
    if [ -f "${SCRIPT_DIR}/code/replay_adaption_apply.sh" ]; then
        log "从code目录复制replay_adaption_apply.sh..."
        cp "${SCRIPT_DIR}/code/replay_adaption_apply.sh" "${TARGET_PATH}/"
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

# 如果不需要编译，执行到步骤7后直接退出
if [ "$NEED_BUILD" = false ]; then
    log "========================================"
    log "Repo工作流执行完成（未编译）"
    log "Repo名称: ${REPO_NAME}"
    log "最终路径: ${TARGET_PATH}"
    log "日志文件: ${LOG_FILE}"
    log "========================================"
    exit 0
fi

# 步骤8: 切换到repo根目录
log "步骤8: 切换到repo根目录..."
cd "$TARGET_PATH"
CURRENT_DIR=$(pwd)
log "当前工作目录: ${CURRENT_DIR}"

# 步骤9: 执行build.sh构建
log "步骤9: 执行build.sh构建..."

if [ ! -f "./build.sh" ]; then
    log "警告: build.sh不存在，跳过此步骤"
else
    log "执行: ./build.sh mipilot/modules/parking/controller/..."
    ./build.sh "mipilot/modules/parking/controller/..." >> "$LOG_FILE" 2>&1
    BUILD_EXIT=$?
    if [ $BUILD_EXIT -ne 0 ]; then
        log "错误: build.sh执行失败，退出码: ${BUILD_EXIT}"
        exit 1
    fi
    log "build.sh执行完成"
fi

# # 步骤10: 执行build_add3.sh构建
# log "步骤10: 执行build_add3.sh构建..."

# if [ ! -f "./build_add3.sh" ]; then
#     log "警告: build_add3.sh不存在，跳过此步骤"
# else
#     log "执行: ./build_add3.sh //mipilot/launch/modules/parking/controller/..."
#     ./build_add3.sh "//mipilot/launch/modules/parking/controller/..." >> "$LOG_FILE" 2>&1
#     BUILD_EXIT=$?
#     if [ $BUILD_EXIT -ne 0 ]; then
#         log "错误: build_add3.sh执行失败，退出码: ${BUILD_EXIT}"
#         exit 1
#     fi
#     log "build_add3.sh执行完成"
# fi

# 工作流完成汇总
log "========================================"
log "Repo工作流执行完成"
log "Repo名称: ${REPO_NAME}"
log "最终路径: ${TARGET_PATH}"
log "日志文件: ${LOG_FILE}"
log "========================================"

# 验证结果
log "验证结果..."
if [ -d "$TARGET_PATH" ]; then
    log "✅ 成功: repo目录存在于 ~/ws/"
    log "目录内容:"
    ls -la "$TARGET_PATH" 2>&1 | while read -r line; do
        log "  ${line}"
    done
else
    log "❌ 错误: repo目录不存在"
    exit 1
fi

exit 0
