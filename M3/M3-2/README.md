# M3-2：Gazebo 仿真巡场与 SLAM 建图

## 一、任务概述

本项目使用 ROS 2 Humble、Gazebo Classic 和 `slam_toolbox`，在自行构建的 Gazebo 场景中运行车辆巡场控制程序，使用激光雷达 `/scan`、里程计 `/odom` 及 TF 进行在线建图与路线反馈。最终保存二维栅格地图、巡场日志、ROS 2 rosbag 和地图质量评估结果。

已验证的巡场程序共包含 **475 个路线点**。正式录包运行日志记录了“已完成一圈巡场路线”和“已发送停车指令”。

> 注意：跑完一圈与 `slam_toolbox` 实际触发回环优化不是同一个结论。当前结果验证了路线执行及建图产出，但尚无独立日志或回环开关对照实验能够严格证明发生过回环优化；详见 `results/quality_report.md`。

## 二、运行环境与目录

- Ubuntu / ROS 2 Humble
- Gazebo Classic（通过项目内 ROS 2 仿真启动文件启动）
- `slam_toolbox`
- `nav2_map_server`（`map_saver_cli`）
- Python 3（`scripts/eval_map.py` 仅使用标准库）
- M3-1 已构建的工作空间：`M3-1/m3_ws/install/setup.bash`

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
│   └── eval_map.py
└── results/
    ├── full_route.log
    ├── full_route_bag.log
    ├── quality_report.md
    ├── map_overview.png
    ├── wall_straightness.png
    └── narrow_passage.png
```

**重要：** `metadata.yaml` 不能替代 `.db3`。如果仅通过 Git 提交本项目，必须另外提供 `.db3` 原始文件及其下载位置；否则无法完整回放 rosbag。

## 三、复现流程（四个终端）

### 终端一：启动 Gazebo 仿真

```bash
cd ~/22SmartCar/M3
source /opt/ros/humble/setup.bash
source M3-1/m3_ws/install/setup.bash
ros2 launch "$PWD/M3-2/launch/sim_m3_2.launch.py"
```

确认仿真和机器人已正常生成后，再开启终端二。

### 终端二：启动在线 SLAM

```bash
cd ~/22SmartCar/M3
source /opt/ros/humble/setup.bash
source M3-1/m3_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:="$PWD/M3-2/config/slam_params.yaml" \
  use_sim_time:=true
```

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

等待开始录制提示，保持终端三运行。待巡场结束后，在本终端按一次 `Ctrl+C`，使 rosbag 正常保存元数据。

### 终端四：执行巡场

```bash
cd ~/22SmartCar/M3
source /opt/ros/humble/setup.bash
source M3-1/m3_ws/install/setup.bash
mkdir -p M3-2/results
python3 M3-2/scripts/drive_route.py --run \
  2>&1 | tee M3-2/results/full_route_bag.log
```

正常完成时，日志应出现“已完成一圈巡场路线”和“已发送停车指令”。执行新的一次完整实验前，建议重启仿真与 SLAM，使车辆和 SLAM 地图都从初始状态开始。

## 四、保存地图

保持 Gazebo 与 SLAM 运行，在新的终端执行：

```bash
cd ~/22SmartCar/M3
source /opt/ros/humble/setup.bash
source M3-1/m3_ws/install/setup.bash
mkdir -p M3-2/maps
ros2 run nav2_map_server map_saver_cli \
  -f "$PWD/M3-2/maps/map" \
  --ros-args -p use_sim_time:=true
```

生成 `maps/map.pgm` 和 `maps/map.yaml`。已保存地图的分辨率为 **0.05 m/像素**，尺寸 **327 × 247 像素**。

## 五、地图质量评估

```bash
cd ~/22SmartCar/M3
python3 M3-2/scripts/eval_map.py
```

该脚本读取已有 `maps/map.pgm`、`maps/map.yaml`，并生成或更新 `results/` 下的证据图。已有质量报告不会被无意覆盖；需要重新生成报告时，请先查看脚本帮助与覆盖参数。

评估内容包括：

1. **墙面直线度：** 对指定中央墙体区域拟合直线，记录平均偏差和最大偏差。
2. **回环一致性：** 检查最终地图是否存在明显重复墙线或错位迹象；但单张最终地图无法独立证明回环优化事件发生。
3. **地图可用性：** 统计栅格占用情况、自由空间连通性，以及窄通道的自由宽度。

此次地图测得中央北墙与南墙平均直线偏差均约 **0.006 m**；最大偏差分别约 **0.016 m** 和 **0.020 m**。窄通道测得自由宽度约 **1.05 m**。具体算法、区域选择及局限性见 `results/quality_report.md`。

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

回放原始话题（在适当的独立 ROS 2 环境中进行，避免与正在运行的仿真重复发布 `/clock`、`/tf` 等话题）：

```bash
cd ~/22SmartCar
source /opt/ros/humble/setup.bash
ros2 bag play M3/M3-2/bags/m3_2_full_run_20261010_044458 --clock
```

> 数据交付说明：`.db3` 是可回放的关键数据文件。若仓库中没有该大文件，需要通过附件、共享盘或明确的下载链接同时交付，不能只提供 `metadata.yaml`。

## 七、方法说明与结果边界

- **建图输入：** `/scan`、`/odom`、相关 TF 和配置文件中的 SLAM 参数。
- **巡场反馈：** `drive_route.py` 使用 SLAM TF 与 `/scan` 激光纠偏、防撞；不使用 Gazebo ground truth 修正 SLAM 地图或正式巡场位置。
- **安全行为：** 程序具有障碍物距离检查、定位超时停车和结束停车行为；具体阈值以脚本及对应运行日志为准。
- **回环相关结论：** 已完成闭合路线的巡场；**未证明** SLAM 后端发生回环优化。当前没有正式的关闭回环对照地图 `map_with_loop_off.pgm`，因此不做“开启与关闭回环改善百分比”的结论。
- **评估限制：** 地图直线度和通道宽度是基于特定栅格图的测量，不代表车辆真实定位精度或严格的地面真值误差。

## 八、实验过程 Git 记录

- `0c35533` — `M3-2: complete SLAM loop route and initial map quality evaluation`
- `3eb62e8` — `M3-2: record full-route SLAM rosbag metadata and run log`

这两次阶段提交均限定在 `M3/M3-2`，未将其他模块的未完成修改并入提交。
