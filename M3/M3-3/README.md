# M3-3｜自主导航与调优

基于 **M3-1 阿克曼小车 + M3-2 原地图与场景**，使用 **Smac Hybrid-A* + RPP + AMCL** 完成自主导航。一条 launch 命令启动 Gazebo、定位、Nav2 和 RViz。

| 正式试跑 | 单次首条路径 | 累计路径转角 | 平均耗时 |
| :---: | :---: | :---: | :---: |
| **3/3 成功** | **8.29 m** | **113.9°** | **41.33 s** |

三次使用相同配置、相同起点和目标，耗时分别为 41.27、41.82、40.90 s。旧 MPPI 组的 2/3 结果及失败记录完整保留；有限次试跑不代表任意目标都能成功。

[运行结果与控制平滑性](results/runs.md) · [三个参数的调优对照](results/tuning_log.md) · [原始证据](../M3-3-evidence/results/optimization_20261114/)

## 1. 环境与目录

- Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11、系统 Python。
- Nav2 Smac/RPP、Gazebo ROS/ros2_control、Ackermann 控制器、CycloneDDS、RViz，以及 `python3-lxml`、`python3-matplotlib`。
- 已构建的 `../M3-1/m3_ws/`、M3-2 原地图与场景、本机已构建的 `../M3-3-deps/` TF2 覆盖层。

```text
M3/
├── M3-1/m3_ws/             # 小车模型与驱动
├── M3-2/                  # 原地图 maps/map.yaml、场景 worlds/scene.world
├── M3-3/                  # 本项目
│   ├── config/            # Nav2、AMCL 参数与注释
│   ├── launch/            # 一键启动
│   ├── scripts/           # 发目标、记录和评估
│   └── results/           # 调参记录、运行结果、四张 PNG
├── M3-3-evidence/          # 正式试跑、失败记录、话题数据与配置快照
└── M3-3-deps/              # TF2 0.25.20 源码及构建产物
```

**交付时须保留原始证据和上述依赖关系。** 单独复制 `M3-3/` 无法完整复核运行结果。控制器配置、行为树、曲率限制节点和 RViz 预设内嵌于 `launch/bringup.launch.py`，启动时写入临时目录。

## 2. 一键启动

### 加载环境（每个终端执行一次）

```bash
cd ~/22SmartCar/M3/M3-3
export PATH=/usr/bin:/bin:/usr/sbin:/sbin:$PATH
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
export ROS_DOMAIN_ID=33 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GAZEBO_MASTER_URI=http://127.0.0.1:11346
export ROS_LOG_DIR=/tmp/m3-3-ros-log
source /opt/ros/humble/setup.bash
source ../M3-1/m3_ws/install/setup.bash
source ../M3-3-deps/install/local_setup.bash
```

`PYTHONNOUSERSITE=1` 用于隔离用户 Python 包，避免 NumPy/Matplotlib 版本冲突。固定 domain 33 与 Gazebo 端口 11346；同一 domain 不要同时运行两个本项目仿真。

### 启动全栈

```bash
ros2 launch "$PWD/launch/bringup.launch.py" map:="$(realpath ../M3-2/maps/map.yaml)" use_sim_time:=true
```

无桌面时追加 `gui:=false rviz:=false`。项目通过 launch 文件路径启动，无需单独构建 M3-3 包。

日志出现 `Managed nodes are active` 后，检查控制器状态、RViz 中激光与地图是否重合，以及 TF 是否连续更新：

```bash
ros2 control list_controllers
ros2 run tf2_ros tf2_echo map base_footprint
```

## 3. 发目标与复现实验

另开终端，先执行上面的环境加载命令，再发目标：

```bash
python3 scripts/goto.py --x 7 --y 2.5 --yaw 90 --runs 3 --out ../M3-3-evidence/new-runs/
```

`x/y` 是地图坐标（米），`yaw` 为朝向（度），现场可直接替换。第 2、3 次前脚本会等待确认：在启动终端按 Ctrl-C 关闭全栈，重新运行同一条 launch 命令，确认车辆与 AMCL 回到相同起点后，再按回车继续。

每次运行记录 action 结果、路径、两张代价地图、速度、TF、里程计、AMCL 与独立真值。耗时采用墙钟；action 结束后额外采集 1 秒停车尾段，不计入导航耗时。超时会先取消目标并等待终止确认。

重算已有正式结果并导出四张图：

```bash
python3 scripts/eval_run.py ../M3-3-evidence/results/optimization_20261114/formal \
  --out /tmp/m3-3-evaluation/runs.md --plot-run 1
```

评估新实验时，将输入目录换成 `goto.py` 实际输出的 `session-*` 路径即可。评估支持单会话和独立试跑根目录。

| 图 | 内容 |
| --- | --- |
| [global_costmap.png](results/global_costmap.png) | 全局障碍与膨胀代价带 |
| [local_costmap.png](results/local_costmap.png) | 局部代价地图 |
| [path_and_trajectory.png](results/path_and_trajectory.png) | 全局规划与实际 TF 轨迹、独立真值 |
| [cmd_vel.png](results/cmd_vel.png) | 最终线速度、角速度命令与停车尾段 |

三次命令均满足 `0 ≤ v ≤ 0.22 m/s`、`|ω| ≤ v/0.60`。曲线存在起步转向振荡和终点附近短暂再加速，平滑性仍有改进空间；分段分析、量化指标与改进取舍见 [运行结果](results/runs.md#控制平滑性分析)。

## 4. 方案与关键约束

### 规划、控制与车辆几何

```text
Smac Hybrid-A* → RPP /cmd_vel_nav → 速度平滑器
             → /cmd_vel_smoothed_raw → 曲率限制 /cmd_vel → Ackermann 控制器
```

- **全局规划**：Hybrid-A* 使用 DUBIN 前进搜索；最小转弯半径 0.60 m。
- **局部控制**：RPP 关闭原地对准与倒车，目标前 0.80 m 开始减速，最低接近速度 0.03 m/s。
- **安全约束**：矩形 footprint 为 0.50 × 0.30 m；最终命令限制 `0 ≤ v ≤ 0.22`、`|ω| ≤ v/0.60`，输入失效 0.5 s 后发布零速度。恢复行为仅清代价地图、等待。
- **选型依据**：NavFn 栅格折线可能不满足转弯约束；原 MPPI 在本场景出现终点收敛失败，延长预测时域也未改善，因此采用本组实测更稳定的 RPP。对照详见调优记录。

内嵌控制器配置与原 URDF 的轴距 **0.28 m**、轮距 **0.18 m**、轮半径 **0.05 m**、转向限位 **±0.5236 rad** 对应。考虑内轮限位，后轴半径下限约为 `0.28/tan(0.5236) + 0.18/2 ≈ 0.575 m`，规划与最终命令均取 0.60 m。RPP 的半径减速阈值不构成硬曲率限制，因此另设最终限幅节点。

### 定位与 TF

AMCL 根据原地图、`/scan`（`lidar_link`）和轮式里程计发布 `map→odom`；M3-1 仅提供轮式里程计。控制器按右、左顺序读取关节，TF 链为：

```text
map → odom → rear_axle → base_footprint → base_link
```

`rear_axle→base_footprint` 固定前移 0.14 m，将后轴积分转换到车身中心；`base_footprint→base_link` 固定高度差为 0.05 m。地面投影用于导航 footprint 与 RSSI 二维位置，后轴参考点用于车辆运动学。

Gazebo 真值仅用于独立评估，不输入 AMCL、TF 发布或控制计算。ModelStates 无 Header，使用回调接收的仿真时间与墙钟记录，定位误差配对允许时间差不超过 0.15 s。

### 窄门与膨胀半径

原地图门宽约 **1.05 m**，world 几何开口 **1.15 m**；车身 footprint 宽 **0.30 m**，正向居中时单侧余量分别约 0.375/0.425 m。`inflation_radius=0.65 m` 会使两侧衰减代价带重叠，但不等于致命障碍堵死。

最终组合已通过窄门并到达 `(0, 9.6, 180°)`：路径 22.24 m、耗时 104.31 s、0 次恢复，真值轨迹确认穿越门段。完整证据见 [运行结果](results/runs.md)。

## 5. 换机准备：TF2 覆盖层

本机历史实测中，系统 TF2 0.25.23 曾出现 Nav2 内部 TF 冻结，使用固定 **0.25.20** 源码覆盖层规避。当前机器已构建；换机时先准备第 1 节依赖，再执行以下一次性构建，不修改系统 ROS：

```bash
cd ~/22SmartCar/M3
export PATH=/usr/bin:/bin:/usr/sbin:/sbin:$PATH
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
source /opt/ros/humble/setup.bash
mkdir -p M3-3-deps/src
# 已有源码时跳过 clone；随后核对固定提交。
test -d M3-3-deps/src/geometry2 || \
  git clone --depth 1 --branch 0.25.20 https://github.com/ros2/geometry2.git M3-3-deps/src/geometry2
test "$(git -C M3-3-deps/src/geometry2 rev-parse HEAD)" = 518e0801d419ba1bcb24cfbe039a99903fd2d710
colcon build \
  --base-paths M3-3-deps/src \
  --build-base M3-3-deps/build \
  --install-base M3-3-deps/install \
  --packages-select tf2 tf2_ros tf2_geometry_msgs tf2_sensor_msgs tf2_py tf2_ros_py \
  --allow-overriding tf2 tf2_ros tf2_geometry_msgs tf2_sensor_msgs tf2_py tf2_ros_py \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
```
