# Koopman 三方数据分析工具链

Z(输入工况) × M(模型在线评分) × C(控制误差) 的相关性与传导分析，单文件工具链，
输出 matplotlib 静态长图 / 多页 PDF / frontend-design 风格前端页面。

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
python3 koopman_analysis.py                                  # 长图 → output/koopman_report.png
python3 koopman_analysis.py --out report.pdf                 # 多页 PDF
python3 koopman_analysis.py --frontend [dir]                 # 前端页面(index.html+分页PNG)
python3 koopman_analysis.py --preflight ../bags/score/score_3   # 新包字段准入检查
python3 koopman_analysis.py --preflight a.bag,b.bag --preflight-max-frames 0  # 全量准入
```

依赖：python3 + rosbag + pandas + numpy + scipy + matplotlib + Pillow。
中文字体：脚本自动注册系统 Noto CJK，缺失时回退 DejaVu。

## 报告内容（7 章节 / 15 页）

| 章节 | 分析 | 目标 |
|---|---|---|
| Overview | 数据包汇总表 + health/lat_err/gain 时序 | 总览 |
| G1 | 4 状态 × 3 工况的分箱 rmse 曲线 | 模型薄弱工况 |
| G1c | (vx,|r|) 误差热力图 + OOD 覆盖度热力图 | 覆盖度/OOD 有效性 |
| G2 | 控制误差派生量(rms/p2p/高频能量) × 工况 | 场景难度基线 |
| G3 | 分层 Spearman(机动) + 时滞互相关 + 非重叠事件 + M-C 散点 | 传导关联 |
| G4 | horizon 相关 + 分量外部效度 + gain 先导 + 双权重仲裁 | 评分校验 |
| G5 | 字段覆盖率准入 + segment 明细 | 数据质量 |

## 关键清洗规则

- **"本帧 Koopman 在算"以 `koopman_vel_u` 字段存在为准**——停车/暂停段 health 输出冻结假值（如恒 100），必须剔除
- 时间 gap > 1s 切断 segment；C 派生量只在 segment 内计算
- warmup=0 帧剔除
- M 为事后已兑现误差，G3 结论只称**关联**不称因果（见方案文档 G3 节）

## 数据包规格建议（正式分析）

≥10 段多工况回放（低/中/高速 × 直道/缓弯/急弯/变道，含 OOD 边界工况），每段 ≥5min。
示例包（bags/score）的三个已知限制：全程 OOD=IN 无变异、速度单一、事件数不足。

## 为什么不用 bokeh/交互式

bokeh 2.4 在大文档（Tabs+90 图）下有间歇性 JS `unreachable code` 导致整页空白。
故全静态渲染（matplotlib 出图 + 纯 HTML），从根上无空白问题。前端页面仍保留章节锚点导航。
