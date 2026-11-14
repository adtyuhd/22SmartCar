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
