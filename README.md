# scripts

## gen_report — 控制调试报告生成工具

一键生成交互式 HTML 报告，从 ROS bag 中解析控制器 JSON debug 信号。

### 首次配置

将以下内容加入 `~/.bashrc`（已自动配置，新环境需手动执行一次）：

```bash
alias gen_report='/mnt/data/ws/scripts/gen_report.sh'
```

然后使 alias 生效：

```bash
source ~/.bashrc
```

---

### Mode 1：从本地 bag 生成报告

```bash
gen_report <bag路径> [<bag路径> ...] [--config <config路径>]
```

- 支持同时传入多个 bag，逐个处理，单个失败不影响其余
- 支持相对路径和绝对路径
- 报告输出在每个 bag 文件的同目录下，文件名为 `<bag文件名>_report.html`
- 未指定 `--config` 时自动使用默认配置 `report_generator/config/target_signal_lat.yaml`

**示例：**

```bash
# 单个 bag，使用默认 config
gen_report bags/test/test.bag

# 多个 bag
gen_report bags/scene1.bag bags/scene2.bag bags/scene3.bag

# 指定 config
gen_report /data/bags/foo.bag /data/bags/bar.bag --config report_generator/config/target_signal_lat.yaml
```

---

### Mode 2：通过 MD5 批量下载并生成报告

每个 MD5 对应一个 bag，脚本自动执行 `mdi fetch <md5>` 下载，在 `--output-dir` 下创建子目录，并在其中生成报告。

```bash
# 命令行传入 MD5，可选指定目录名（与 --md5 一一对应）
gen_report --md5 <md5> [<md5> ...] [--dir-name <名> [<名> ...]] --output-dir <输出目录>

# 从文件读取，每行格式为 <md5> 或 <md5>:<目录名>
gen_report --md5-file <md5列表文件> --output-dir <输出目录>

# 两者混用
gen_report --md5 abc123 --dir-name 场景1 --md5-file md5_list.txt --output-dir <输出目录>
```

**参数说明：**

| 参数 | 说明 |
|------|------|
| `--md5` | 一个或多个 MD5 值 |
| `--dir-name` | 每个 MD5 对应的目录名，顺序与 `--md5` 一致；省略则使用 bag 文件名 |
| `--md5-file` | MD5 列表文件，每行 `<md5>` 或 `<md5>:<目录名>` |
| `--output-dir` | 根输出目录（默认当前目录） |
| `--config` | 特别关注信号配置文件（可选，省略则使用默认 config） |

**示例：**

```bash
# 不指定目录名，使用 bag 文件名
gen_report --md5 abc123def456 789xyz --output-dir ./reports

# 指定目录名
gen_report --md5 abc123def456 789xyz --dir-name 直道测试 变道测试 --output-dir ./reports

# 从文件读取
gen_report --md5-file md5_list.txt --output-dir /data/reports
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

详细说明见 [`report_generator/README.md`](report_generator/README.md)。
