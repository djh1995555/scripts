#!/bin/bash

# =============================================================================
# 脚本名称: run_jira_workflow.sh
# 功能: 根据JIRA ID自动下载记录文件并生成报告
# 使用方法: bash run_jira_workflow.sh <jira-ids> <output_dir:default>
# 示例: bash run_jira_workflow.sh ADM2-112827,ADM2-109165
# =============================================================================

# 设置脚本运行目录为当前工作目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 创建日志文件路径（使用时间戳命名）
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="${SCRIPT_DIR}/workflow_jira_download.log"

# 删除旧的日志文件
rm -f "$LOG_FILE"

# 日志记录函数 - 同时输出到控制台和日志文件
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOG_FILE"
}

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

# 检查输入参数
if [ $# -lt 1 ]; then
    log "错误: 请提供JIRA IDs作为参数"
    log "使用方法: bash run_jira_workflow.sh <jira-ids> [output_dir]"
    log "示例: bash run_jira_workflow.sh ADM2-112827,ADM2-109165"
    log "示例: bash run_jira_workflow.sh ADM2-112827 /custom/output/path"
    exit 1
fi

# 获取输入的JIRA IDs（逗号分隔）
JIRA_IDS="$1"

# 设置输出目录（支持自定义，默认为 ${SCRIPT_DIR}/record/test_record/jira）
if [ -n "$2" ]; then
    OUTPUT_DIR="$2"
else
    OUTPUT_DIR="${SCRIPT_DIR}/record/test_record/jira"
fi

log "========================================"
log "开始执行JIRA_DOWNLOAD工作流"
log "输入的JIRA IDs: ${JIRA_IDS}"
log "输出目录: ${OUTPUT_DIR}"
log "日志文件: ${LOG_FILE}"
log "========================================"

# 步骤1: 下载记录文件
log "步骤1: 开始下载JIRA记录文件..."

# 检查jira_download.py是否存在
if [ ! -f "${SCRIPT_DIR}/jira_download.py" ]; then
    log "错误: 找不到 jira_download.py 文件"
    exit 1
fi

# 执行下载命令，所有输出重定向到日志文件
log "执行: python jira_download.py --jira-ids ${JIRA_IDS} -o ${OUTPUT_DIR}"
python "${SCRIPT_DIR}/jira_download.py" --jira-ids "${JIRA_IDS}" -o "${OUTPUT_DIR}" >> "$LOG_FILE" 2>&1

# 检查下载是否成功
if [ $? -ne 0 ]; then
    log "错误: 下载记录文件失败，请查看日志文件了解详情"
    exit 1
fi

log "记录文件下载完成"

# 步骤2: 生成报告
log "步骤2: 开始生成报告..."

# 检查start_generate_report.sh是否存在
if [ ! -f "${SCRIPT_DIR}/start_generate_report.sh" ]; then
    log "错误: 找不到 start_generate_report.sh 文件"
    exit 1
fi

# 将逗号分隔的JIRA IDs转换为数组
IFS=',' read -ra JIRA_ARRAY <<< "$JIRA_IDS"

# 遍历每个JIRA ID，生成报告
TOTAL_COUNT=${#JIRA_ARRAY[@]}
CURRENT=0

for JIRA_ID_RAW in "${JIRA_ARRAY[@]}"; do
    # 去除可能的空白字符（使用参数扩展，比xargs更安全）
    JIRA_ID="${JIRA_ID_RAW// /}"
    CURRENT=$((CURRENT + 1))

    # 验证JIRA ID格式
    if ! validate_jira_id "$JIRA_ID"; then
        log "错误: 无效的JIRA ID格式 '${JIRA_ID}'，跳过处理"
        log "提示: JIRA ID应该是 'PROJECT-12345' 格式"
        continue
    fi

    # 构建单个JIRA目录路径（使用绝对路径，防止路径遍历）
    SINGLE_JIRA_DIR="${OUTPUT_DIR}/${JIRA_ID}"
    # 确保解析后的路径仍在输出目录内
    RESOLVED_DIR=$(cd "${OUTPUT_DIR}" 2>/dev/null && cd "$(dirname "${JIRA_ID}/.")" 2>/dev/null && pwd)
    if [[ ! "$RESOLVED_DIR" =~ ^${OUTPUT_DIR} ]]; then
        log "错误: JIRA ID '${JIRA_ID}' 尝试访问输出目录之外的区域，已阻止"
        continue
    fi

    log "处理进度: ${CURRENT}/${TOTAL_COUNT} - JIRA ID: ${JIRA_ID}"

    # 检查目录是否存在
    if [ ! -d "$SINGLE_JIRA_DIR" ]; then
        log "警告: 目录不存在，跳过: ${SINGLE_JIRA_DIR}"
        continue
    fi

    # 检查目录中是否有记录文件（.record, .mcap, 或 full_record.*）
    HAS_RECORD=false
    if ls "${SINGLE_JIRA_DIR}"/*.record 1> /dev/null 2>&1; then
        HAS_RECORD=true
    elif ls "${SINGLE_JIRA_DIR}"/*.mcap 1> /dev/null 2>&1; then
        HAS_RECORD=true
    elif ls "${SINGLE_JIRA_DIR}"/full_record.* 1> /dev/null 2>&1; then
        HAS_RECORD=true
    fi

    if [ "$HAS_RECORD" = false ]; then
        log "警告: 目录中没有找到记录文件，跳过: ${SINGLE_JIRA_DIR}"
        continue
    fi

    log "正在为 ${JIRA_ID} 生成报告..."

    # 执行报告生成脚本，所有输出重定向到日志文件
    bash "${SCRIPT_DIR}/start_generate_report.sh" "$SINGLE_JIRA_DIR" >> "$LOG_FILE" 2>&1

    # 检查报告生成是否成功
    if [ $? -eq 0 ]; then
        log "成功为 ${JIRA_ID} 生成报告"
    else
        log "警告: 为 ${JIRA_ID} 生成报告时发生错误"
    fi
done

# 工作流完成汇总
log "========================================"
log "工作流执行完成"
log "JIRA IDs 处理总数: ${TOTAL_COUNT}个"
log "输出目录: ${OUTPUT_DIR}"
log "日志文件: ${LOG_FILE}"
log "========================================"

# 列出结果目录中的内容
log "结果目录概览:"
ls -la "$OUTPUT_DIR" 2>&1 | tee -a "$LOG_FILE"

# 检查每个JIRA目录中的记录文件
log "各JIRA目录中的记录文件详情:"
for JIRA_ID_RAW in "${JIRA_ARRAY[@]}"; do
    # 去除空白字符
    JIRA_ID="${JIRA_ID_RAW// /}"
    # 跳过无效的JIRA ID
    if ! validate_jira_id "$JIRA_ID"; then
        continue
    fi
    JIRA_DIR="${OUTPUT_DIR}/${JIRA_ID}"
    if [ -d "$JIRA_DIR" ]; then
        log "  ${JIRA_ID} 目录内容:"
        ls -la "$JIRA_DIR" 2>&1 | while read -r line; do
            log "    ${line}"
        done
    else
        log "  警告: ${JIRA_ID} 目录不存在"
    fi
done

exit 0
