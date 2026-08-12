# Report Generator

从 ROS bag 文件中读取 `/msd/endpoint/control_command` 的 `msg.extra.json`，生成交互式 HTML 报告。

> **推荐使用方式**：通过根目录的 `gen_report` 命令（已封装好环境变量），无需手动设置 Python 路径。详见 [../README.md](../README.md)。

---

## 文件结构

```
report_generator/
├── report_generator.py        # 主程序
├── json_fields_map.py         # JSON 字段分组映射（勿手动编辑）
├── config/
│   └── target_signal_lat.yaml # 特别关注信号配置示例
├── utils/
│   └── report_plotter.py      # Plotly 封装工具
└── README.md
```

---

## 依赖

运行前需要设置 ROS 环境（提供 `rosbag` Python 模块）：

```bash
ROS_PKG=/mnt/data/docker/overlay2/b8bd618514aeb61db290b839f6f4dea3336b00d939ae08b8f9ca5ba966f4c52b/diff
export LD_LIBRARY_PATH=$ROS_PKG/opt/ros/noetic/lib:$ROS_PKG/root/ros_catkin_ws/devel_isolated/roslz4/lib:$LD_LIBRARY_PATH
export PYTHONPATH=$ROS_PKG/opt/ros/noetic/lib/python3/dist-packages:$ROS_PKG/root/ros_catkin_ws/devel_isolated/rosbag/lib/python3/dist-packages:$ROS_PKG/root/ros_catkin_ws/devel_isolated/roslz4/lib/python3/dist-packages:$PYTHONPATH
```

Python 包依赖：

```bash
pip3 install pandas plotly pyyaml lz4
```

---

## 使用方式

### Mode 1：从本地 bag 文件生成报告

```bash
python3 report_generator.py <bag路径> [<bag路径> ...] [--config <config路径>]
```

**参数说明：**

| 参数 | 说明 |
|------|------|
| `bag路径` | 一个或多个 `.bag` / `.mfbag` 文件路径，空格分隔 |
| `--config` | 特别关注信号配置文件路径（可选，默认使用 `config/target_signal_lat.yaml`） |

**输出：** 每个 bag 的报告保存在其同目录下，文件名为 `<bag文件名>_report.html`。多个 bag 依次处理，单个失败不影响其余。

**示例：**

```bash
# 单个 bag
python3 report_generator.py /data/bags/test.bag

# 多个 bag
python3 report_generator.py /data/bags/a.bag /data/bags/b.bag /data/bags/c.bag

# 指定 config
python3 report_generator.py /data/bags/test.bag --config config/target_signal_lat.yaml
```

---

### Mode 2：通过 MD5 批量下载并生成报告

```bash
python3 report_generator.py --md5 <md5> [<md5> ...] [--dir-name <名> [<名> ...]] --output-dir <dir> [--config <config路径>]
python3 report_generator.py --md5-file <file> --output-dir <dir> [--config <config路径>]
```

**参数说明：**

| 参数 | 说明 |
|------|------|
| `--md5 <md5> ...` | 一个或多个 MD5 值，空格分隔 |
| `--dir-name <名> ...` | 每个 MD5 对应的目录名，顺序与 `--md5` 一致；省略则使用 bag 文件名 |
| `--md5-file <file>` | MD5 列表文件，每行格式为 `<md5>` 或 `<md5>:<目录名>` |
| `--output-dir <dir>` | bag 下载和报告输出的根目录（默认当前目录） |
| `--config` | 特别关注信号配置文件路径（可选） |

**流程：** 对每个 MD5：
1. 在 `output-dir` 下执行 `mdi fetch <md5>` 下载 bag
2. 在 `output-dir/<目录名>/` 下创建子目录（目录名由 `--dir-name` 或文件中的 `:<名>` 指定，否则使用 bag 文件名）
3. 将 bag 移入该子目录
4. 在该子目录下生成报告

**示例：**

```bash
# 不指定目录名，使用 bag 文件名
python3 report_generator.py --md5 abc123 def456 --output-dir /data/reports

# 指定目录名
python3 report_generator.py --md5 abc123 def456 --dir-name 直道测试 变道测试 --output-dir /data/reports

# 从文件读取
python3 report_generator.py --md5-file md5_list.txt --output-dir /data/reports
```

`md5_list.txt` 格式（两种写法可混用）：

```
abc123def456
789xyz000111:直道测试
aabbccdd1234:变道压力测试
```

输出目录结构：

```
reports/
├── 直道测试/
│   ├── foo.bag
│   └── foo_report.html
├── 变道压力测试/
│   ├── bar.bag
│   └── bar_report.html
├── baz_bag_name/          # 未指定目录名时使用 bag 文件名
│   ├── baz_bag_name.bag
│   └── baz_bag_name_report.html
```

---

## 配置文件说明

配置文件为 YAML 格式，定义**特别关注信号**，这些信号会在报告最顶部单独展示。

**格式：**

```yaml
target_panel:
  <panel名称>:
    "<显示名称>": "<字段名>"
    ...
  <panel名称2>:
    "<显示名称>": "<字段名>"
    ...
```

- 每个 `panel` 对应报告中的一个 subplot，所有 panel 合并在同一个图内，共享 x 轴
- `字段名` 直接填写 JSON debug 字段名（如 `ctrl_osqp_mpc`），程序会自动在全量数据中查找

**示例（`config/target_signal_lat.yaml`）：**

```yaml
target_panel:
  LatCmd:
    "ctrl_osqp_mpc": "ctrl_osqp_mpc"
    "wheel_angle_comp": "wheel_angle_comp"
    "wheel_angle_cmd": "wheel_angle_cmd"
    "lat_raw_steering_angle_cmd": "lat_raw_steering_angle_cmd"
    "lat_steering_angle_cmd": "lat_steering_angle_cmd"
  LatCmd2:
    "steering_angle_bias_deg": "steering_angle_bias_deg"
    "lat_steering_angle_cmd_bias_fix": "lat_steering_angle_cmd_bias_fix"
    "alpha_shared_control": "alpha_shared_control"
    "steering_angle_cmd_with_share_control": "steering_angle_cmd_with_share_control"
    "steering_angle_cmd": "steering_angle_cmd"
```

---

## 报告结构

生成的 HTML 报告分为两部分：

**1. 特别关注信号**（仅在提供 `--config` 时出现）
- 所有 target panel 合并为一个图，内含多个 subplot，共享 x 轴
- 页面打开时立即渲染

**2. 全量数据库**
- 按 `doc/control_debug_signals.md` 的大节分组（逆序展示，#18 在上，#1 在下）
- 分组粒度：子节函数级别（如 `ControlPreprocess::UpdateVelocity`、`ControlLoop::SharedControl`）
- **懒渲染**：滚动到视口附近时才渲染，避免浏览器卡顿

**交互功能：**
- 所有 panel 的 x 轴联动：在任意 panel 缩放/拖拽，其余 panel 同步更新
- 双击任意 panel 恢复全局视图
- 点击 legend 中的信号名可单独显示/隐藏该曲线

---

## 字段分组说明

`json_fields_map.py` 由 `doc/control_debug_signals.md` 自动生成，定义了 1222 个字段到 215 个子节的映射，分组与顺序与 md 完全一致。大节如下：

| 大节 | 内容 |
|------|------|
| #1. IO 编排 | driving_task.cc |
| #2. 算法总控·输入与编排 | control_module.h |
| #3. 门面层·初始化/参数 | control_interface.cpp |
| #4. 感知融合 | control_preprocess.cpp |
| #5. 状态机 | control_preprocess.cpp / lon_fsm.cpp |
| #6. 参考生成 | control_preprocess.cpp |
| #7. 总览打印（预处理） | control_preprocess.cpp PrintJsonLog |
| #8. 门面层·主更新 | control_interface.cpp |
| #9. 在线辨识 | control_loop.cpp |
| #10. 抖动检测 | vehicle_shake_optimization.cpp |
| #11. 控制求解·纵向 | control_loop.cpp LonOcpControl |
| #12. 横向 MPC | control_loop.cpp LateralAcadoMpcControl |
| #13. 横向执行层 | control_loop.cpp |
| #14. 总览打印（控制后） | control_loop.cpp PrintJsonLog |
| #15. 算法总控·执行器/指令组装 | control_module.h / control_module.cpp |
| #16. 规划回放 | hack_planning.cpp |
| #17. 故障诊断 | fault_diagnose_isolation.cpp |
| #18. 车辆模型 | vehicle_model.cpp |
| Other | 未在映射表中的字段 |

未在映射表中的字段归入 `Other` 分组，显示在全量数据库末尾（逆序时为最顶部）。
