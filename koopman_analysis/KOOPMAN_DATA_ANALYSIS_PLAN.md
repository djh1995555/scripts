# Koopman 三方数据分析方案（Z 工况 × M 模型评分 × C 控制误差）

> 状态：待评审。数据源：`debug/bags/score` 回放 bag；产物：`debug/` 下脚本 + 单文件 HTML 报告。
> 背景文档：`KOOPMAN_MONITOR_IMPROVEMENTS.md`（在线评价提升路线）、`.claude/rules/LAT_CONTROL.md` §9（架构）。

## 0. 数据链路（已验证）

- 数据包 = 回放输出 bag（rosbag/mfbag），debug 数据在 `/msd/endpoint/control_command` topic 的 `msg.extra.json` 字段（每帧一个 JSON，含全部 `JSON_DEBUG_VALUE` 键值对）
- 现有解析模式参考 `cp_apa_simulation/script/control_debug_scripts/control_lat_performance_analysis.py`（bokeh HTML 报告）
- 样例包：`debug/bags/score/score_1`、`score_2`（2026-08-21 录制，含 .bag 与 .orig.bag）

## 1. 输入数据定义

**Z 工况（ONNX 输入状态量，直取 debug 字段）**：
`koopman_vel_u`（clamp 后）、`vel_by`、`mpc_init_dyawrate`、`koopman_wheel_angle_meas`、`w_fl/fr/rl/rr_mps_slip`、`mpc_init_dy/dphi`、`koopman_health_ood_level`

**M 模型评分（在线评价输出）**：
`koopman_acc_{plan|executed}_{y|phi|vy|r}_{1|10|25}step_{error|rmse}`、`koopman_health_score_total` 及 5 分量（plan_accuracy/stability、ood、executed_error/jump_p99）、`koopman_gain_raw/lp`、`koopman_history_warmup`

**C 控制误差（暂定两个直取量）**：`lat_err`、`phi_err`

**C 派生量（脚本内计算）**：
| 派生 | 算法 | 表征 |
|---|---|---|
| 滑窗 RMS / mean / std | 0.5s 滑窗 | 平滑误差水平/波动 |
| 峰峰值 | 滑窗 max−min | 震荡幅度 |
| 过零率 | 滑窗内符号变化次数 | 震荡频率 |
| 高频能量 | 一阶差分序列滑窗能量 | 修正频繁度 |

**Z 派生量**：|r|、|δ|、a_y≈vx·r、机动类型自动分段（按 |r|/|δ| 时序形态分直道/缓弯/急弯/变道，阈值可配）

## 2. 分析目标（G1-G5）

| 目标 | 问题 | 路径 |
|---|---|---|
| G1 模型薄弱工况 | 模型在哪些工况失真？外推特性？OOD 判定有效吗？ | Z→M |
| G2 场景难度基线 | 哪些工况 lat_err 天然大（对照组，防误归因）？ | Z→C |
| G3 传导分析 ⭐ | 模型误差是否/何时/多大程度传导为控制误差？ | M→C（控制 Z） |
| G4 评分体系校验 | 100 分有外部效度吗？horizon 权重用页面版还是代码版？ | M↔C 互验 |
| G5 数据质量 | 缺失/warmup/invalid 占比，结论可信度边界 | 全部 |

## 3. 分析方法

- **G1**：各 Z 量分位分箱 × M rmse 的 P50/P95 曲线（4 状态 × 3 horizon 分面）；(vx,|r|) 误差热力图叠 OOD 分档；ood_level 分组误差单调性检验
- **G2**：同一套 Z 分箱 × C 派生量（与 G1 对照形成"模型差 vs 场景难"的区分）
- **G3（核心三件套）**：
  1. 分层 Spearman：机动类型 × 速度段层内算 M-C 相关（层内显著 → 传导稳健）
  2. 时滞互相关：M(t)×C(t+Δ)，扫 Δ∈[0,3s]，峰值位置=传导时滞、峰值符号验证因果方向
  3. 事件对照：M top10% 段 vs bottom10% 段之后 2.5s 内 C 分布对比（效应量 + Mann-Whitney）
- **G4**：两种 horizon 加权（代码 {0.4,0.3,0.3} vs 设计页 {0.2,0.3,0.5}/{0.5,0.3,0.2}）各算 total'，比谁与 C 相关高（仲裁页面/代码分歧）；M 分量与 C 相关条形图；gain 对 M/C 的先导曲线
- **G5**：字段缺失率（含 40KiB buffer 降级损失）、warmup/invalid 帧占比、时间戳连续性

**已知局限（无 A/B 对照时的妥协）**：单组数据无法完全分离场景难度与模型误差的混淆，靠分层+回归近似净效应；结论为相关性证据，非因果。

## 4. 可视化方案（单文件 HTML · bokeh 分页）

```
Tab1 Overview ── 每包一行汇总表（时长/帧数/有效占比/health 均值·P05/
                 lat_err RMS/事件计数）+ 多包关键指标横向对比条
Tab2 Timeline ── 单包多轨时间线：Z(速度/横摆/转角) / M(health+分量堆叠+
                 rmse 曲线) / C(lat_err/phi_err+震荡带) 三轨对齐
Tab3 Model ───── G1：分箱曲线矩阵 + (vx,|r|) 热力图(叠OOD) + 机动分组箱线
Tab4 Control ─── G2：Z-C 基线分箱 + C 派生量分布
Tab5 Transfer ── G3：M-C 散点(机动类型着色) + 分层相关热图 +
                 时滞互相关曲线(峰值标注) + 高低M段C对比
Tab6 Score ───── G4：两种加权 total 对比 + 分量-C 相关 + gain 先导性
Tab7 Events ──── M top 段/C 超标段列表 + 每事件前后±5s 三线 sparkline
```

## 5. 脚本架构

```bash
python3 koopman_health_report.py --bags pkg1,pkg2,... [--out report.html] [--conf conf.json]
```

```
koopman_health_report.py（放 debug/ 目录）
├── loader    bag→DataFrame：read_messages("/msd/endpoint/control_command")
│             → json.loads(msg.extra.json) → 字段映射
├── cleaner   剔 warmup=0/health invalid/暂停段；时间戳对齐；缺失统计
├── features  C 派生量 + Z 派生量 + 机动分段（阈值进 conf）
├── analysis  G1-G5 纯函数（DataFrame→结果对象）
├── report    bokeh 组装 7-Tab HTML
└── main      CLI + 多包循环聚合
```

依赖：python3 + rosbag/mfbagpy + pandas + numpy + scipy + bokeh（现有环境同款）。字段清单/阈值/机动分段规则放 conf.json。

## 6. 实施步骤

```
1. 样例包验证（半天）：跑通 loader，确认 Z/M/C 全字段在 bags/score 包中存在
   风险：bags/score 是 08-21 录制，100 分制新字段是否在内待验证
2. cleaner + features + 单包 Timeline/Overview（1 天）
3. G1-G2 分析与 Tab3/4（1 天）
4. G3-G4 分析与 Tab5/6（1-2 天）
5. 多包聚合 + Events 页 + 打磨（1 天）
```
