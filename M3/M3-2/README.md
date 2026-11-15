# M3-2：Gazebo 仿真巡场与 SLAM 建图

## 一、任务概述

本项目使用 ROS 2 Humble、Gazebo Classic 和 `slam_toolbox`，在自行构建的 Gazebo 场景中运行车辆巡场控制程序，使用激光雷达 `/scan`、里程计 `/odom` 及 TF 进行在线建图与路线反馈。最终保存二维栅格地图、巡场日志、ROS 2 rosbag 和地图质量评估结果。

巡场程序包含 **475 个路线点**，直线速度 **0.16 m/s**，转弯及窄通道速度 **0.08 m/s**。不带 `--run` 时只预览路线，带 `--run` 会实际驱动车辆。

## 二、运行环境与目录

- Ubuntu / ROS 2 Humble
- Gazebo Classic（通过项目内 ROS 2 仿真启动文件启动）
- `slam_toolbox`
- `nav2_map_server`（`map_saver_cli`）
- Ubuntu 系统 Python 3.10（`/usr/bin/python3`；`scripts/eval_map.py` 仅使用标准库）
- RViz2（查看地图、雷达和坐标系）
- M3-1 已构建的工作空间：`M3-1/m3_ws/install/setup.bash`

若终端启用了 Conda `(base)`，巡场脚本必须使用 `/usr/bin/python3`（Ubuntu 系统 Python 3.10）。Conda 的 Python 3.14 无法加载 ROS 2 Humble 的 `rclpy` 二进制扩展；仅 source ROS 环境不会切换 Python 解释器。

仿真启动文件已为 `spawn_entity.py` 显式指定 `/usr/bin/python3`，避免 Conda 导致 `No module named 'lxml'`、小车生成失败和 `/scan` 无发布者。

下列命令均假设仓库位于 `~/22SmartCar`，在 `~/22SmartCar/M3` 下执行。

```text
M3-2/
├── README.md
├── worlds/scene.world
├── launch/sim_m3_2.launch.py
├── config/slam_params.yaml
├── maps/
│   ├── map.pgm
│   └── map.yaml
├── bags/
│   └── m3_2_full_run_20261010_044458/
│       ├── metadata.yaml
│       └── m3_2_full_run_20261010_044458_0.db3  # 原始数据，必须随交付另行提供
├── scripts/
│   ├── drive_route.py
│   ├── eval_map.py
│   ├── audit_project.py            # 只读地图/录包审计，可准备过滤后的回放包
│   └── audit_posegraph.cpp         # 读取 Karto 位姿图结构
└── results/
    ├── full_route.log
    ├── full_route_*.log            # 新巡场使用时间戳，避免覆盖旧日志
    ├── quality_report.md
    ├── map_overview.png
    ├── wall_straightness.png
    ├── narrow_passage.png
    └── audit/                     # 验收报告、量化数据、轨迹图及回放证据
```

**重要：** `metadata.yaml` 不能替代 `.db3`。如果仅通过 Git 提交本项目，必须另外提供 `.db3` 原始文件及其下载位置；否则无法完整回放 rosbag。

## 三、运行流程（四个终端，RViz另开终端）

启动一轮新实验前，先在上一轮的巡场、录包、SLAM 和 Gazebo 终端依次按 `Ctrl+C` 结束对应程序，再从头启动。不要同时运行多套仿真或多个建图节点。各终端应使用相同的 `ROS_DOMAIN_ID`（没有手动设置时保持默认即可）。

### 终端一：启动 Gazebo 仿真

```bash
cd ~/22SmartCar/M3
source /opt/ros/humble/setup.bash
source M3-1/m3_ws/install/setup.bash
ros2 launch "$PWD/M3-2/launch/sim_m3_2.launch.py"
```

确认 Gazebo 中已经生成小车，而不只是空场景，再开启终端二。正常启动会显示 `Successfully spawned entity [smart_car]`，两个控制器应加载并激活。

### 终端二：启动在线 SLAM

```bash
cd ~/22SmartCar/M3
source /opt/ros/humble/setup.bash
source M3-1/m3_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:="$PWD/M3-2/config/slam_params.yaml" \
  use_sim_time:=true
```

这个命令只启动建图后台，不会自动打开页面。初始地图只覆盖出生点附近，巡场后才会逐步扩展。

### 可视化：另开终端启动 RViz2

```bash
cd ~/22SmartCar/M3
source /opt/ros/humble/setup.bash
source M3-1/m3_ws/install/setup.bash
rviz2 --ros-args -p use_sim_time:=true
```

在 RViz 左侧配置：

1. `Global Options → Fixed Frame`：`map`。
2. `Add → Map`，`Topic`：`/map`。
3. `Add → LaserScan`，**点击 Topic 右侧的值并填入 `/scan`**，`Reliability Policy`：`Best Effort`。只展开 Topic 或修改 QoS 不会自动选择话题。
4. 可添加 `TF`；需要俯视地图时，将右侧 `Views → Type` 改成 `TopDownOrtho`。

正常状态下 Global Status、Map、LaserScan 应为绿色；地图宽高不为0。若 `map` 坐标系不存在，先检查激光和底盘数据，见第八节。

### 终端三：录制原始 rosbag

如果要重新录制实验数据，先开始录制，再启动车辆巡场。

```bash
cd ~/22SmartCar/M3
source /opt/ros/humble/setup.bash
source M3-1/m3_ws/install/setup.bash
mkdir -p M3-2/bags
ros2 bag record \
  -o "$PWD/M3-2/bags/m3_2_full_run_$(date +%Y%m%d_%H%M%S)" \
  /scan /odom /tf /tf_static /clock /cmd_vel /map
```

等待开始录制提示，保持终端三运行。**巡场结束后先保存地图，再在本终端按一次 `Ctrl+C`**，使保存地图时刻也位于录包范围内，并正常写入元数据。记录本轮 bag 路径及其对应的地图、日志路径。

### 终端四：执行巡场

```bash
cd ~/22SmartCar/M3
source /opt/ros/humble/setup.bash
source M3-1/m3_ws/install/setup.bash
mkdir -p M3-2/results
set -o pipefail
/usr/bin/python3 M3-2/scripts/drive_route.py
/usr/bin/python3 M3-2/scripts/drive_route.py --run \
  2>&1 | tee "M3-2/results/full_route_$(date +%Y%m%d_%H%M%S).log"
```

第一条 Python 命令只做预览，第二条会驱动车辆。正常完成时，日志应出现“已完成一圈巡场路线”和“已发送停车指令”；若出现“自动驾驶停止”，应先查看后面的原因，不能仅凭命令退出就认定完成。`pipefail` 防止 `tee` 掩盖管道前端的非零退出，但仍需核对驾驶脚本的结束日志。

## 四、保存地图

巡场停车后，保持 Gazebo、SLAM 和 rosbag 录制运行，在新的终端执行。新地图使用时间戳前缀，保留原有交付地图：

```bash
cd ~/22SmartCar/M3
source /opt/ros/humble/setup.bash
source M3-1/m3_ws/install/setup.bash
mkdir -p M3-2/maps
MAP_NAME="map_$(date +%Y%m%d_%H%M%S)"
MAP_PREFIX="$PWD/M3-2/maps/$MAP_NAME"
ros2 run nav2_map_server map_saver_cli \
  -f "$MAP_PREFIX" \
  --ros-args -p use_sim_time:=true
```

生成 `maps/map_时间戳.pgm` 和同名 YAML。确认保存成功后，停止录包，并记下对应路径。若正式交付需要命名为 `map.pgm`/`map.yaml`，整理时还必须同步修改 YAML 的 `image` 字段，并重新加载验证；不要只重命名 PGM。

原有 `maps/map.pgm` 分辨率为 **0.05 m/像素**，尺寸 **327 × 247 像素**；这是历史文件的测量值，新运行的尺寸可能不同。

## 五、地图质量评估

在刚保存地图的同一终端，沿用 `MAP_NAME`、`MAP_PREFIX`：

```bash
/usr/bin/python3 M3-2/scripts/eval_map.py \
  --map "$MAP_PREFIX.pgm" \
  --yaml "$MAP_PREFIX.yaml" \
  --out-dir "$PWD/M3-2/results/$MAP_NAME"
```

如果只想复算原交付地图，执行：

```bash
cd ~/22SmartCar/M3
/usr/bin/python3 M3-2/scripts/eval_map.py --out-dir /tmp/m3_2_map_check
```

脚本生成三张 PNG 和首次质量报告；它**不生成 `quality_metrics.json`**。已有质量报告默认不覆盖，需要覆盖时使用 `--overwrite-report`。ROI 使用相对图像位置，适配原地图方向；新地图若旋转或边界变化，需要检查ROI是否仍对应实际墙面和窄通道，不能直接套用历史结论。自动生成报告中的运行描述也需依据本轮真实日志核实。

评估内容包括：

1. **墙面直线度：** 对指定中央墙体区域拟合直线，记录平均偏差和最大偏差。
2. **回环一致性：** 检查最终地图是否存在明显重复墙线或错位迹象；但单张最终地图无法独立证明回环优化事件发生。
3. **地图可用性：** 统计栅格占用情况、自由空间连通性，以及窄通道的自由宽度。

此次地图测得中央北墙与南墙平均直线偏差均约 **0.006 m**；最大偏差分别约 **0.016 m** 和 **0.020 m**。窄通道测得自由宽度约 **1.05 m**。具体算法、区域选择及局限性见 `results/quality_report.md`。

上述数值针对原交付地图。脚本的局部双影筛查不能证明回环优化，墙线拟合误差也不是绝对建图精度。原报告中关于 JSON 输出和 rosbag 状态的部分描述已滞后，以代码及 [审计报告](results/audit/acceptance_review.md) 为准。

## 六、rosbag 检查与回放

当前已完成的录制文件夹为：

```text
M3-2/bags/m3_2_full_run_20261010_044458/
```

录制验证结果：

- 时长：**430.33 秒**（约 7 分 10 秒）
- 消息总数：**89,694**
- 体积：**61.7 MiB**
- 话题：`/scan`、`/odom`、`/tf`、`/tf_static`、`/clock`、`/cmd_vel`、`/map`

检查：

```bash
cd ~/22SmartCar
source /opt/ros/humble/setup.bash
ros2 bag info M3/M3-2/bags/m3_2_full_run_20261010_044458
```

**查看历史地图：** 以下在独立 ROS 域回放录包，不启动 Gazebo 或新 SLAM。只回放显示所需话题，避免旧 `/cmd_vel` 进入正在运行的车辆：

```bash
cd ~/22SmartCar
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=175
ros2 bag play M3/M3-2/bags/m3_2_full_run_20261010_044458 \
  --topics /scan /odom /tf /tf_static /clock /map
```

查看该回放的 RViz 终端也设置 `export ROS_DOMAIN_ID=175`，再启动带 `use_sim_time:=true` 的 RViz。

包中已经有 `/clock`，**不要再加 `--clock`**。原存储时间是系统时间，消息内部使用仿真时间，两套时钟混发会导致时间异常。

**用原始数据重新跑 SLAM：** 必须过滤掉旧 SLAM 的 `map→odom`，也不回放旧 `/map`、`/cmd_vel`。本项目审计脚本可准备输入包：

```bash
cd ~/22SmartCar/M3
source /opt/ros/humble/setup.bash
/usr/bin/python3 M3-2/scripts/audit_project.py \
  --prepare-replay /tmp/m3_2_filtered_replay
```

目标目录必须不存在；该脚本当前读取项目中找到的第一个 `.db3`，适用于原有单文件包，新增多个实验或分卷包后需先明确选择输入，不能盲用。它不修改原包。

在两个新终端中都 source ROS 并设置相同独立域 `export ROS_DOMAIN_ID=176`：一个终端按第三节启动SLAM；另一个运行 `ros2 bag play /tmp/m3_2_filtered_replay --rate 1.0`。不要加 `--clock`，不要在此域再启动 Gazebo。需要 RViz 时也使用域176。

> 数据交付说明：`.db3` 是可回放的关键数据文件。若仓库中没有该大文件，需要通过附件、共享盘或明确的下载链接同时交付，不能只提供 `metadata.yaml`。

## 七、方法说明与结果边界

- **建图输入：** `/scan`、`/odom`、相关 TF 和配置文件中的 SLAM 参数。
- **巡场反馈：** `drive_route.py` 使用 SLAM TF 与 `/scan` 激光纠偏、防撞；不使用 Gazebo ground truth 修正 SLAM 地图或正式巡场位置。
- **安全行为：** 程序具有障碍物距离检查、定位超时停车和结束停车行为；具体阈值以脚本及对应运行日志为准。
- **回环相关结论：** 审计回放的位姿图有246个节点、246条边，其中一条连接起点节点0与近终点节点243，日志有优化收敛证据。它支持本次回放发生闭环优化；不替代原PGM来源证明，也不等同于同一面墙的重影间距测量。没有正式回环关闭对照实验。
- **评估限制：** 地图直线度和通道宽度是基于特定栅格图的测量，不代表车辆真实定位精度或严格的地面真值误差。

### 场景与参数依据

- 东侧长走廊连续8 m、净宽2.1 m，中央障碍四周构成回环；北侧局部净宽1.15 m。窄通道用于检查遮挡、转向扰动和栅格化后的自由宽度，实际问题及预期对比仍需在本轮质量报告中明确记录。
- 0.05 m栅格使设计窄通道约23格宽；原地图测得21格。M3-3当前导航footprint宽0.30 m，对应6格。分辨率兼顾通行空间表达和地图规模，不代表定位精度达到5 cm。
- 雷达噪声标准差0.01 m；直行0.16 m/s、10 Hz对应1.6 cm/帧。后端移动门槛0.10 m会进一步筛选扫描，不是每帧都成为图节点。
- `max_laser_range=7.8` m低于实际8 m上限。实际模型/录包为360束、角度0～6.28 rad、约1.002°/束，frame为`lidar_link`；应读取消息角度，不假设起点一定为-π。
- `map.yaml`的0.65/0.25是地图加载占据/自由阈值，与SLAM内部的`occupancy_threshold`属于不同阶段。

