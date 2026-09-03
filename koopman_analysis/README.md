# Koopman 三方数据分析工具链

Z(输入工况) × M(模型在线评分) × C(控制误差) 的相关性与传导分析，单文件工具链，
输出单文件 Plotly 离线交互 HTML 页面。

## 目录结构

```
koopman_analysis/
├── README.md                    # 本文件
├── KOOPMAN_DATA_ANALYSIS_PLAN.md  # 分析方案（已按 adversarial review 修正）
├── analysis_list.txt            # 输入：数据包列表（每行一个场景目录/bag 路径，# 注释）
├── koopman_analysis.py          # 唯一脚本（逻辑层 + 报告层 + 前端 + 准入，全部在此）
└── output/                      # 输出产物
```

数据包在共享目录 `../bags/`（回放系统使用，勿移动）。analysis_list.txt 中
`bags/score/score_1` 与 `../bags/score/score_1` 两种写法均可（相对 list 目录及上级自动解析）。

## 使用

```bash
cd koopman_analysis
python3 koopman_analysis.py  # 生成 output/index.html
```

依赖：python3 + rosbag + pandas + numpy + scipy + matplotlib + Pillow + plotly。
中文字体：脚本自动注册系统 Noto CJK，缺失时回退 DejaVu。

## 报告内容（交互 HTML）

| 章节 | 分析 | 目标 |
|---|---|---|
| Overview | 数据包汇总表 + health/lat_err/gain 时序 | 总览 |
| G1 | 4 状态 × 3 工况的分箱 rmse 曲线 | 模型薄弱工况 |
| G1c | (vx,|r|,|steer|) 联合分位网格及其二维投影 | 训练/回放覆盖度；分位区间而非连续坐标 |
| G2 | 控制误差派生量(rms/p2p/高频能量) × 工况 | 场景难度基线 |
| G3 | 全局/机动×速度分层 Spearman + 时域趋势 + 非重叠事件对照 | 传导关联 |
| G4 | horizon 相关 + 分量外部效度 + gain 先导 + 双权重仲裁 | 评分校验 |
| 扩展诊断（交互版） | 风险预警、去混杂、覆盖缺口、事件根因、健康分标定、plan/executed 差异 | 12 张图卡，均为两列排版 |

## 关键清洗规则

- **"本帧 Koopman 在算"以 `koopman_vel_u` 字段存在为准**——停车/暂停段 health 输出冻结假值（如恒 100），必须剔除
- 时间 gap > 1s 切断 segment；C 派生量、时滞相关和事件检测都只在 segment 内计算
- `koopman_history_warmup=0` 帧剔除；旧包若缺字段会在准入报告明确标出
- M 为事后已兑现误差，G3 结论只称**关联**不称因果（见方案文档 G3 节）

## 数据包规格建议（正式分析）

≥10 段多工况回放（低/中/高速 × 直道/缓弯/急弯/变道，含 OOD 边界工况），每段 ≥5min。
示例包（bags/score）的三个已知限制：全程 OOD=IN 无变异、速度单一、事件数不足。

## 交互报告

脚本仅生成 Plotly 单文件离线 HTML：所有报告图均可悬停查看数据、缩放和通过图例筛选，G1c 三维覆盖图还可拖拽旋转。Plotly 脚本内嵌在 `output/index.html`，打开报告时不需要网络。
