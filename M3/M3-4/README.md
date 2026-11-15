# M3-4 动态障碍下的局部规划与避障

在 M3-2 地图和 M3-3 导航栈上，对比阿克曼小车在临时障碍出现后的避障表现。障碍物在导航开始后延迟生成，由激光雷达检测。项目使用 RPP 的两套前视参数进行对照，并对长前视方案进行调优。

## 运行环境

- Ubuntu 22.04、ROS2 Humble、Gazebo Classic 11、Nav2。
- 系统 Python 3，安装 PyYAML、Pillow、matplotlib。
- 已构建的 M3-1 车型工作空间：`../M3-1/m3_ws/install/`。
- M3-2 地图与场景：`../M3-2/maps/`、`../M3-2/worlds/`。
- M3-3 导航启动文件及配置：`../M3-3/`。
- 本机使用的 TF2 覆盖层：`../M3-3-deps/install/`。

评测脚本自动加载 ROS 环境、启动 Gazebo 和 Nav2。运行时需有可用的桌面显示环境。默认使用 ROS 域 33、Gazebo 端口 11346，同一时间只运行一组实验。

## 目录结构

```text
M3-4/
├── scripts/
│   ├── inject_obstacle.py        # 延迟生成障碍物
│   ├── run_benchmark.py          # 批量实验与数据采集
│   └── eval_metrics.py           # 指标计算与绘图
├── config/
│   ├── planner_a_params.yaml     # 短前视方案
│   ├── planner_b_params.yaml     # 长前视方案
│   └── planner_b_tuned.yaml      # 长前视方案调优
├── results/
│   ├── benchmark.csv             # A、B各10次实验数据
│   ├── tuning_log.md             # 参数调优记录
│   ├── failure_boundary.md       # 失效条件与实验结果
│   └── *.png                     # 过程截图、轨迹图和指标图
├── report.md                     # 实验分析报告
└── README.md
```

## 运行方法

### 一键复现

```bash
cd ~/22SmartCar/M3/M3-4
python3 scripts/run_benchmark.py --campaign
```

程序依次采集无障碍参考、运行 A/B 各10次、运行调优方案10次，再完成三组边界实验并生成图表和报告。每次实验重新启动仿真。新一轮结果保存在 `results/` 下的独立目录中。

默认目标为地图坐标 `(5.5, 0)`，朝向 `0°`；障碍物位于世界坐标 `(2.5, -5.05)`，边长 `0.6 m`，在目标接受后约3秒出现。

### 分别运行对照与调优

使用新的输出目录，先测无障碍参考，再运行避障实验：

```bash
python3 scripts/run_benchmark.py --baseline --planners A,B,B_tuned --runs 1 --out results/new-run

python3 scripts/run_benchmark.py --planners A,B --runs 10 \
  --obstacle-at 2.5,-5.05 --delay 3.0 --out results/new-run

python3 scripts/run_benchmark.py --planners B_tuned --runs 10 --out results/new-run
```

无障碍参考与避障实验应使用相同的目标和规划参数，用于计算绕行额外路径。

### 指定障碍物位置

```bash
python3 scripts/run_benchmark.py --planners A --runs 2 \
  --obstacle-at 2.8,-5.05 --obstacle-size 0.6 --delay 2.5 \
  --goal 5.5,0,0 --out results/demo
```

`--obstacle-at` 使用 **Gazebo世界坐标**，`--goal` 使用 **地图坐标**，格式为 `x,y,yaw`，其中朝向单位为度。本地图初始世界坐标 `y≈-5.05` 对应地图坐标 `y≈0`。障碍物应放在导航路径上，延迟范围为2～4仿真秒。

### 绘制新一轮实验结果

```bash
PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 MPLCONFIGDIR="$PWD/results/mpl" \
  /usr/bin/python3 scripts/eval_metrics.py results/new-run
```

将 `results/new-run` 替换为本轮实验的实际输出目录。

## 规划方案与结果

|方案|配置文件|前视距离|成功次数|
|---|---|---|---:|
|A|`planner_a_params.yaml`|随速度变化，0.30～0.50 m|9/10|
|B|`planner_b_params.yaml`|固定1.50 m|0/10|
|B调优后|`planner_b_tuned.yaml`|固定0.40 m|8/10|

调优仅缩短 B 的前视距离。三个方案的最大线速度均为0.22 m/s，最小转弯半径0.60 m，膨胀半径0.40 m。

实验记录避障成功率、成功试次平均导航耗时、绕行额外路径和路径平滑度。额外路径为实际轨迹长度减去无障碍参考长度；平滑度为单位路程内转向角变化总量，单位 rad/m。

完整数据和结论见 [实验报告](report.md)、[调优记录](results/tuning_log.md) 和 [失效边界实验](results/failure_boundary.md)。
