# 从跑完到晋升

在 M5-4 目录执行。需 Python 3.10+，读取 zstd/LZ4 压缩块需要系统的 libzstd1/liblz4-1。

```bash
python -m pip install -r requirements.txt
time python scripts/quick_review.py --mcap data/run_01.mcap --logs data/logs --switch-log data/switch_log.csv

python scripts/exp.py new --name replay_base --set nav_speed=0.8 --desc '验证基线回放' --expect '复盘正常'
python scripts/exp.py run --name replay_base --runs 1 --replay-dir data
python scripts/exp.py new --name replay_candidate --set nav_speed=0.9 --desc '验证候选配置回放' --expect '生成对比表'
python scripts/exp.py run --name replay_candidate --runs 3 --replay-dir data
python scripts/exp.py compare --names replay_base,replay_candidate --metric nav_time,success_rate,cpu_peak
python scripts/gen_baseline.py --exp replay_candidate --out outputs/baseline_package_demo.tar.gz

verify_dir=$(mktemp -d)
tar -xzf outputs/baseline_package_demo.tar.gz -C "$verify_dir"
"$verify_dir/verify_baseline.sh"
```

样本回放用于验收工具链。两个实验使用同一批预录数据的不同范围，数据差异无法证明车速修改的效果。相邻的 compare HTML 可离线打开，切换指标。重复创建实验时使用新的名称。

## 现场运行与晋升

1. 在场景 YAML 的 runner 中填写真实的 hot_load 和 run 命令数组。命令可引用 {config}、{run_dir}、{run_index}，由程序直接执行。hot_load 应校验实际节点已加载生效，run 完成时将 blackbox.mcap、logs/、task_report.json 和 switch_log.csv 写到 {run_dir}。runner 负责逐把复位环境。
2. new 记录基线哈希、目的、预期指标和配置差异。合并时映射递归合并、列表和标量替换。基线有变更时，按当前基线重新创建实验。
3. run 不带 --replay-dir 时调用现场 runner。批次记录 Git HEAD、跟踪文件变更状态以及配置和模型哈希。统计各指标均值、总体方差和有效样本数。整把成功率的分母为完成批次的请求把数，分子只统计 SUCCESS。命令失败会中断批次，逐把报告仍保留。
4. 根据对比结果选定实验，执行 `python scripts/exp.py promote --name <实验名> --to scene_race.yaml`。晋升验证已跑配置和模型内容，写入模型 SHA-256，并只提交目标场景。目标场景有本地改动时拒绝写入。纯回放演练可显式传入 --allow-replay，此命令会真正修改并提交目标场景。
5. gen_baseline.py 也接受 --scene scene_race.yaml。验包要求模型数精确匹配、每个模型至少 1 字节，且所有模型和参数文件 SHA-256 一致。验包需要 shell、GNU coreutils 和 findutils。模型推理由上车流程验收。打包时显示字节数，样本包约 1.3 KB，生产包大小取决于模型。

### 复用 M5-3 的 ROS 参数服务

现场也可用下面的配置替代自定义 hot_load。先在 M5-3 终端配置的 parameter_targets 中填入实际 ROS 节点名。该终端配置应作为现场输入文件，由 runner.ros_hot_load.terminal 引用。路径相对于 M5-4。

```yaml
runner:
  ros_hot_load:
    terminal: /path/to/terminal.yaml
    bindings:
      navigation:
        max_velocity: nav_speed
  run: ["/path/to/runner", "{config}", "{run_dir}"]
```

bindings 按「M5-3 节点别名 → ROS 参数名 → M5-4 完整场景字段」映射。需要设置模型路径时，可绑定到 models.<模型名>.path，运行时会转换为绝对路径。参数服务的写入和回读由 M5-3 执行，并在每把目录生成 switch_log.csv 和审计日志。runner 应保留这些日志。模型推理就绪仍由现场 runner 确认。启动时先加载 ROS 环境，并使用对应的系统 Python。

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 scripts/exp.py run --name <已创建的实验名> --runs 3
```

现场计时应使用考官原始数据。开发验证对有块索引的 321 MiB 构造文件测得 0.05 秒，跳过 41 个无关块。此耗时取决于块索引、无关数据的分布及关键块的压缩情况。

### 在本机验证 ROS 参数回读

在 M5-4 目录执行下面整段。测试节点只发布参数服务，使用独立的 DDS 域，不启动控制器。验收会检查实际节点参数、M5-3 切换 CSV 和审计日志。

```bash
(
set -e
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}" PYTHONDONTWRITEBYTECODE=1
export M54_TEST_DIR="$(mktemp -d)"
export ROS_LOG_DIR="$M54_TEST_DIR/ros"
trap 'if [ -n "${node_pid:-}" ]; then kill "$node_pid" 2>/dev/null || true; wait "$node_pid" 2>/dev/null || true; fi' EXIT
/usr/bin/python3 - <<'PY' &
import os
from pathlib import Path
import rclpy

rclpy.init()
node = rclpy.create_node("m54_test_navigation")
node.declare_parameter("max_velocity", 0.2)
(Path(os.environ["M54_TEST_DIR"]) / "ready").touch()
rclpy.spin(node)
PY
node_pid=$!
for i in {1..50}; do
    [ -f "$M54_TEST_DIR/ready" ] && break
    sleep 0.1
done
test -f "$M54_TEST_DIR/ready"
/usr/bin/python3 - <<'PY'
import csv
import os
from pathlib import Path
from exp import ROOT, hot_load_ros, read_config, write_config

directory = Path(os.environ["M54_TEST_DIR"])
terminal = read_config(ROOT.parent / "M5-3/config/terminal.yaml")
terminal["hot_switch"]["parameter_targets"]["navigation"] = "/m54_test_navigation"
terminal["hot_switch"]["timeout_s"] = 8.0
path = directory / "terminal.yaml"
write_config(path, terminal)
hot_load_ros(
    {"nav_speed": 0.35},
    {"terminal": str(path), "bindings": {"navigation": {"max_velocity": "nav_speed"}}},
    directory, "m54_ros_acceptance",
)
with (directory / "switch_log.csv").open() as stream:
    rows = list(csv.DictReader(stream))
assert any(row["op"] == "SCENE_SWITCH" and row["result"] == "ok" for row in rows), rows
assert list((directory / "logs").glob("audit_log_*.jsonl"))
PY
ros2 param get /m54_test_navigation max_velocity | grep -Fx 'Double value is: 0.35'
printf 'ROS hot-load PASS. Evidence: %s\n' "$M54_TEST_DIR"
)
```

无索引的 zstd/LZ4 构造 MCAP 也已验证能解码 ROS 2 String CDR。人为改错数据块 CRC 时，两种编码均返回可读错误。
