# M3-2：SLAM 建图

ROS 2 Humble + Gazebo Classic + `slam_toolbox`。自建场景包含 8 m 长、2.1 m 宽的走廊、环形路线和 1.15 m 窄通道。巡场速度：直行 0.16 m/s，转弯及窄通道 0.08 m/s。

## 1. 环境

依赖：已编译的 `M3-1/m3_ws`、`gazebo_ros`、`slam_toolbox`、`nav2_map_server`、RViz2。Python 使用 `/usr/bin/python3`；对照分析另需系统 `matplotlib`、`numpy`、`g++` 和 Karto 开发库。

## 2. 交付文件

| 路径 | 内容 |
|---|---|
| `worlds/scene.world` | 自建世界 |
| `config/slam_params.yaml` | 开启回环的 SLAM 配置 |
| `config/slam_params_loop_off.yaml` | 关闭回环的对照配置 |
| `maps/map.pgm`、`maps/map.yaml` | 正式地图，回环开启 |
| `maps/map_with_loop_off.pgm`、同名 YAML | 回环关闭地图 |
| `bags/m3_2_full_run_20261010_044458/` | 原始 rosbag，430.33 s；交付时包含 `.db3` 和 `metadata.yaml` |
| `results/quality_report.md` | 质量报告、图及数据对比 |
| `results/loop_comparison/` | 参数快照、位姿图、CSV、对照图及量化结果 |
| `maps/archive/` | 更换前的地图备份 |

正式地图为已有 rosbag 的重建结果，分辨率 0.05 m，尺寸 330 × 251。两组地图均通过 `map_server` 加载验证。

| 指标 | 开启回环 | 关闭回环 |
|---|---:|---:|
| 重访墙拟合间距 | 0.0054 m | 0.0218 m |
| 窄通道最小自由宽度 | 1.10 m | 1.05 m |
| 后端节点 / 边 | 246 / 246 | 246 / 245 |

测量方法及导航适用范围见 [质量报告](results/quality_report.md)。

## 3. 终端初始化

后续每个新终端先执行：

```bash
cd ~/22SmartCar/M3/M3-2
source /opt/ros/humble/setup.bash
source ../M3-1/m3_ws/install/setup.bash
export PATH=/usr/bin:$PATH
```

在线建图各终端使用相同的 `ROS_DOMAIN_ID`。开始新一轮前，结束上一轮 Gazebo、SLAM、巡场及录包进程。

## 4. 启动建图

**终端一：仿真。** 等小车生成、控制器激活后继续。

```bash
ros2 launch "$PWD/launch/sim_m3_2.launch.py"
```

**终端二：SLAM。**

```bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:="$PWD/config/slam_params.yaml" use_sim_time:=true
```

**终端三：录包。** 开始录制后再巡场。

```bash
ros2 bag record -o "$PWD/bags/run_$(date +%Y%m%d_%H%M%S)" \
  /scan /odom /tf /tf_static /clock /cmd_vel /map
```

**终端四：巡场。** 不带 `--run` 只预览路线。

```bash
set -o pipefail
/usr/bin/python3 scripts/drive_route.py --run \
  2>&1 | tee "results/full_route_$(date +%Y%m%d_%H%M%S).log"
```

完成标志：日志出现“已完成一圈巡场路线”和“已发送停车指令”。

**RViz（另开终端，可选）：**

```bash
rviz2 --ros-args -p use_sim_time:=true
```

设置 Fixed Frame 为 `map`；添加 Map `/map`、LaserScan `/scan`，LaserScan 的 Reliability 设为 `Best Effort`。

## 5. 保存与评估地图

巡场停车后，保持仿真、SLAM 和录包运行，在新终端初始化后执行：

```bash
MAP_NAME="map_$(date +%Y%m%d_%H%M%S)"
MAP_PREFIX="$PWD/maps/$MAP_NAME"
ros2 run nav2_map_server map_saver_cli -f "$MAP_PREFIX" \
  --ros-args -p use_sim_time:=true
/usr/bin/python3 scripts/eval_map.py \
  --map "$MAP_PREFIX.pgm" --yaml "$MAP_PREFIX.yaml" \
  --out-dir "$PWD/results/$MAP_NAME"
```

保存成功后，在录包终端按 `Ctrl+C`。新地图按时间戳保存；若改名，同时修改 YAML 的 `image` 字段。评估 ROI 适用于当前地图方向，新图旋转后需重新检查。

## 6. 查看原始录包

```bash
ros2 bag info bags/m3_2_full_run_20261010_044458
export ROS_DOMAIN_ID=175
ros2 bag play bags/m3_2_full_run_20261010_044458 \
  --topics /scan /odom /tf /tf_static /clock /map
```

RViz 使用同一域 175。包内已有 `/clock`，无需添加 `--clock`；这里显示的是录包中的历史地图。

## 7. 复算现有对照结果

```bash
/usr/bin/python3 -s scripts/analyze_loop.py
```

读取已保存的位姿图和地图，更新 `results/loop_comparison/metrics.json`、两张对照图及分组评估图；不覆盖正式质量报告。

## 8. 重建回环对照实验

同一录包、同步 SLAM 后端、2 倍速回放，仅切换 `do_loop_closing`。脚本自动过滤旧 `/map`、`/cmd_vel` 和 `map→odom`，使用独立 ROS 域 181。

```bash
/usr/bin/python3 scripts/compare_loop.py \
  --bag "$PWD/bags/m3_2_full_run_20261010_044458" \
  --out-dir /tmp/m3_2_loop_repeat
/usr/bin/python3 -s scripts/analyze_loop.py \
  --out-dir /tmp/m3_2_loop_repeat \
  --map-dir /tmp/m3_2_loop_repeat/maps
```

输出目录须不存在。两套新地图、参数、位姿图及对照数据保存在该目录，现有交付文件不覆盖。
