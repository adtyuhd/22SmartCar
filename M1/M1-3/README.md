# M1-3 时间与坐标：让两个传感器“说到一起去”

本项目基于 ROS 2 Humble，实现二维激光雷达 `LaserScan` 与相机图像的近似时间同步、TF2 坐标变换、针孔相机投影以及调试图像输出。

主要数据流：

```text
/scan ------------------+
                         |
                         +--> ApproximateTimeSynchronizer
                         |              |
/camera/image_raw -------+              |
                                        v
                              LaserScan polar points
                                        |
                                        v
                                   laser_link
                                        |
                                       TF2
                                        |
                                        v
                                   camera_link
                                        |
                              body -> optical axes
                                        |
                                        v
                              pinhole projection
                                        |
                                        v
                           /projection/debug_image
                                        |
                                        v
                              projection_*.png
```

---

## 1. 环境

- Ubuntu
- ROS 2 Humble
- Python 3
- `ament_python`
- `rclpy`
- `sensor_msgs`
- `geometry_msgs`
- `tf2_ros`
- `message_filters`
- `PyYAML`
- `Pillow`

---

## 2. 输入与输出

### 输入

```text
/scan
sensor_msgs/msg/LaserScan

/camera/image_raw
sensor_msgs/msg/Image

/camera/camera_info
sensor_msgs/msg/CameraInfo
```

### 输出

```text
/projection/debug_image
sensor_msgs/msg/Image
encoding: rgb8
```

静态坐标变换发布到：

```text
/tf_static
tf2_msgs/msg/TFMessage
```

程序还会保存：

```text
output/projection_001.png
output/projection_002.png
output/projection_003.png
```

---

## 3. 参数

`projection_node` 使用以下参数：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `use_sim_time` | `false` | 是否使用 ROS simulated time |
| `extrinsics_file` | `extrinsics.yaml` | 外参和相机内参文件 |
| `output_dir` | `.` | PNG 输出目录 |
| `max_images` | `3` | 最多保存的调试图片数量 |
| `slop` | `0.02` | ApproximateTimeSynchronizer 时间窗口，单位秒 |
| `queue_size` | `50` | ApproximateTimeSynchronizer 队列大小 |

在 rosbag 回放时使用：

```text
use_sim_time:=true
```

并通过：

```bash
ros2 bag play sample_bag --clock
```

发布 `/clock`。

---

## 4. TF 树

本项目发布以下静态 TF：

```text
map
 |
 v
odom
 |
 v
base_link
 |       \
 |        \
 v         v
laser_link camera_link
```

其中：

```text
map -> odom
odom -> base_link
```

在本任务中使用单位变换。

传感器外参：

```text
base_link -> laser_link
base_link -> camera_link
```

从 `extrinsics.yaml` 读取，不在代码中硬编码。

默认生成数据中：

```text
base_link -> laser_link
xyz = [0.15, 0.00, 0.20]

base_link -> camera_link
xyz = [0.10, 0.00, 0.30]
```

因此：

```text
laser_link -> camera_link

translation ≈ [0.05, 0.00, -0.10]
```

TF2 查询时需要注意参数顺序：

```python
lookup_transform(
    target_frame,
    source_frame,
    time,
)
```

本项目需要把激光点变换到相机坐标，因此使用：

```text
target = camera_link
source = laser_link
```

---

## 5. LaserScan 极坐标转换

二维 `LaserScan` 的第 `i` 根 beam 的角度为：

```text
theta_i = angle_min + i * angle_increment
```

距离为：

```text
r = ranges[i]
```

二维雷达扫描平面中：

```text
z = 0
```

因此激光点在 `laser_link` 中为：

```text
x = r * cos(theta)
y = r * sin(theta)
z = 0
```

无效的 `NaN`、`Inf` 和超出：

```text
range_min <= r <= range_max
```

范围的测量会被丢弃。

---

## 6. TF2 坐标变换

激光点首先位于：

```text
laser_link
```

程序根据该 `LaserScan` 的：

```text
scan_msg.header.stamp
```

查询：

```text
laser_link -> camera_link
```

变换。

对点应用：

```text
P_camera = R * P_laser + t
```

其中旋转来自 TF 四元数，平移来自 TF translation。

默认标定下实测：

```text
TF translation:
[0.050, 0.000, -0.100]

TF quaternion:
[0.000000, 0.000000, 0.000000, 1.000000]
```

例如中央附近的一根激光：

```text
laser_link:
(6.152, -0.054, 0.000)

camera_link:
(6.202, -0.054, -0.100)
```

与默认外参一致。

---

## 7. camera_link 与 optical 坐标

`camera_link` 使用常见 ROS body frame：

```text
+x forward
+y left
+z up
```

针孔投影使用 optical 坐标：

```text
+Z forward
+X right
+Y down
```

因此本项目使用：

```text
Xopt = -Ycamera
Yopt = -Zcamera
Zopt =  Xcamera
```

只投影：

```text
Zopt > 0
```

的点。

---

## 8. 针孔相机投影

相机内参：

```text
fx
fy
cx
cy
```

从 `extrinsics.yaml` 读取。

投影公式：

```text
u = fx * Xopt / Zopt + cx
v = fy * Yopt / Zopt + cy
```

然后只保留：

```text
0 <= u < image_width
0 <= v < image_height
```

的像素。

默认参数：

```text
fx = 500
fy = 500
cx = 320
cy = 240
```

例如：

```text
camera_link:
(6.202, -0.054, -0.100)
```

转换为：

```text
optical:
(0.054, 0.100, 6.202)
```

得到约：

```text
u ≈ 324
v ≈ 248
```

图像主点为：

```text
(320, 240)
```

因此该激光点位于图像中心稍右、稍下的位置。

由于默认情况下相机比雷达高 `0.10 m`，激光扫描平面在相机下方，因此有效投影整体位于图像中心线附近偏下的位置。

---

## 9. 距离颜色

投影点按照 LaserScan 距离进行线性着色。

归一化：

```text
alpha =
    (range - range_min)
    /
    (range_max - range_min)
```

然后：

```text
R = 255 * (1 - alpha)
G = 0
B = 255 * alpha
```

因此：

```text
near -> red
far  -> blue
```

输出图像保持：

```text
encoding = rgb8
```

---

## 10. ApproximateTimeSynchronizer

激光频率约：

```text
10 Hz
```

相机频率约：

```text
27 Hz
```

两个传感器还带有独立 timestamp jitter，因此不能要求时间戳完全相等。

本项目使用：

```python
ApproximateTimeSynchronizer
```

而不是：

```python
TimeSynchronizer
```

队列大小：

```text
queue_size = 50
```

---

## 11. slop 实验

为了研究 `slop` 对匹配结果的影响，对同一个 rosbag 离线执行 `ApproximateTimeSynchronizer`。

离线读取 bag 的原因是：如果同时在实时 DDS 流上挂很多 synchronizer，实验节点本身可能造成消息处理压力和丢帧，从而污染实验结果。

最终实验数据：

| slop (ms) | matched pairs | median |dt| (ms) |
|---:|---:|---:|
| 20 | 40 | 9.645 |
| 30 | 40 | 11.738 |
| 40 | 40 | 18.500 |
| 50 | 40 | 18.500 |
| 60 | 40 | 18.500 |
| 70 | 40 | 18.500 |
| 80 | 40 | 18.500 |
| 90 | 40 | 18.500 |
| 100 | 40 | 18.500 |
| 110 | 40 | 18.500 |
| 120 | 40 | 18.500 |
| 130 | 40 | 18.500 |
| 140 | 40 | 18.500 |
| 150 | 40 | 18.500 |

对于本次使用的：

```text
--frames 40
--seed 7
```

数据，`20 ms` 已经能够匹配全部：

```text
40 / 40
```

LaserScan，同时得到最低的 median time difference。

因此正式投影节点使用：

```text
slop = 0.02 s
```

作为本数据集的默认实验选择。

这并不表示 `20 ms` 对所有传感器组合都是全局最优值；它是当前数据集上的实验结果。

### 为什么 slop 越大 median 不一定越小？

`ApproximateTimeSynchronizer` 不是离线全局最近邻搜索。

它是在线、基于队列的一对一近似匹配。

因此更大的 `slop` 表示允许更大的匹配窗口，但不保证它会等待未来出现的“更近”时间戳。

一个消息被配对后不会无限重复用于其他 pair。

所以：

```text
larger slop
```

并不等价于：

```text
smaller timestamp difference
```

本实验中相机约为 `27 Hz`：

```text
period ≈ 37 ms
```

其半周期约：

```text
18.5 ms
```

这与较大 slop 时观察到约 `18.5 ms` 的 median 差值相符。

完整实验结果保存在：

```text
sync_slop_results.csv
```

---

## 12. use_sim_time 实验

使用：

```bash
ros2 bag play sample_bag --clock
```

播放 rosbag。

实验节点比较：

```text
node.get_clock().now()
```

与：

```text
scan.header.stamp
```

### use_sim_time=true

实测稳定阶段例如：

```text
node_clock=0.096143
scan_stamp=0.100299
difference=-0.004156

node_clock=0.196192
scan_stamp=0.199726
difference=-0.003534

node_clock=0.296188
scan_stamp=0.299109
difference=-0.002922
```

节点 ROS clock 与 bag sensor timestamp 位于同一个时间域，稳定后差值为几毫秒量级。

第一帧可能存在较大的启动阶段差异，这是 `/clock`、bag 消息发布以及 callback 调度启动时序造成的。

### use_sim_time=false

实测：

```text
node_clock=1790412062.575565
scan_stamp=1.500695
difference=1790412061.074870
```

后续差值保持在约：

```text
1.790412061e9 s
```

说明：

```text
node clock -> system / wall time
sensor stamp -> recorded bag time
```

二者属于完全不同的时间域。

因此 rosbag 使用 `--clock` 回放时，本项目节点应使用：

```text
use_sim_time:=true
```

### ATS 与 use_sim_time 的区别

`ApproximateTimeSynchronizer` 比较的是：

```text
scan.header.stamp
image.header.stamp
```

因此即使 `use_sim_time=false`，只要两个消息 header 本身处于同一个记录时间域，ATS 仍可能正常匹配。

`use_sim_time` 解决的是：

```text
node ROS clock
```

与：

```text
bag time
```

之间的时间域一致性。

这对于动态 TF、ROS timer 以及其他依赖 ROS clock 的操作尤其重要。

---

## 13. 非默认标定参数验收

为了验证程序没有把默认传感器标定参数硬编码到代码中，重新生成测试数据：

```bash
python3 make_bag.py \
  --frames 40 \
  --seed 7 \
  --laser-z 0.45 \
  --camera-z 0.75 \
  --fx 420
```

生成的 YAML 中：

```text
laser z  = 0.45
camera z = 0.75

fx = 420
fy = 500
cx = 320
cy = 240
```

投影节点启动时实际读取：

```text
fx=420.000
fy=500.000
cx=320.000
cy=240.000
```

TF2 实测：

```text
camera_link <- laser_link

Translation:
[0.050, 0.000, -0.300]
```

这与：

```text
0.45 - 0.75 = -0.30
```

一致。

默认配置下一帧约有：

```text
66
```

个 LaserScan 点落入图像范围。

非默认 `fx=420` 配置下一帧约有：

```text
76
```

个点落入图像范围。

降低 `fx` 后，相同视角对应的水平像素偏移减小，因此更多激光点进入 `640 x 480` 图像范围，这与针孔模型预测一致。

同时相机与雷达的高度差：

```text
0.10 m -> 0.30 m
```

因此投影点在图像中明显向下移动，也符合：

```text
v = fy * Yopt / Zopt + cy
```

的预测。

该实验验证了传感器外参与相机内参来自 YAML/TF，而不是依赖默认硬编码值。

---

## 14. 编译

从 M1-3 工作目录执行：

```bash
cd ~/22SmartCar/M1/M1-3

source /opt/ros/humble/setup.bash

colcon build \
  --packages-select m1_3_projection \
  --symlink-install

source ~/22SmartCar/M1/M1-3/install/setup.bash
```

---

## 15. 运行

建议使用三个终端。

### Terminal A：静态 TF

```bash
cd ~/22SmartCar/M1/M1-3

source /opt/ros/humble/setup.bash
source ~/22SmartCar/M1/M1-3/install/setup.bash

ros2 run m1_3_projection tf_static_publisher \
  --ros-args \
  -p use_sim_time:=true \
  -p extrinsics_file:=/home/adtyuhd/22SmartCar/M1/M1-3/extrinsics.yaml
```

### Terminal B：投影节点

```bash
cd ~/22SmartCar/M1/M1-3

source /opt/ros/humble/setup.bash
source ~/22SmartCar/M1/M1-3/install/setup.bash

ros2 run m1_3_projection projection_node \
  --ros-args \
  -p use_sim_time:=true \
  -p extrinsics_file:=/home/adtyuhd/22SmartCar/M1/M1-3/extrinsics.yaml \
  -p output_dir:=/home/adtyuhd/22SmartCar/M1/M1-3/output \
  -p max_images:=3 \
  -p slop:=0.02 \
  -p queue_size:=50
```

### Terminal C：rosbag

```bash
cd ~/22SmartCar/M1/M1-3

source /opt/ros/humble/setup.bash

ros2 bag play sample_bag --clock
```

---

## 16. 验证输出 topic

检查：

```bash
source /opt/ros/humble/setup.bash
source ~/22SmartCar/M1/M1-3/install/setup.bash

ros2 topic info /projection/debug_image
```

预期：

```text
Type: sensor_msgs/msg/Image
Publisher count: 1
```

检查 encoding：

```bash
ros2 topic echo \
  /projection/debug_image \
  --field encoding \
  --once
```

预期：

```text
rgb8
```

检查 header：

```bash
ros2 topic echo \
  /projection/debug_image \
  --field header \
  --once
```

输出 frame 应为：

```text
camera_link
```

---

## 17. 验证 PNG

```bash
cd ~/22SmartCar/M1/M1-3

ls -lh output/projection_*.png
```

应得到：

```text
projection_001.png
projection_002.png
projection_003.png
```

验证文件：

```bash
cd ~/22SmartCar/M1/M1-3

python3 - <<'PY'
from pathlib import Path
from PIL import Image

paths = sorted(
    Path("output").glob("projection_*.png")
)

print(
    f"Found {len(paths)} projection images"
)

for path in paths:
    with Image.open(path) as image:
        print(
            f"{path}: "
            f"size={image.size}, "
            f"mode={image.mode}, "
            f"format={image.format}"
        )
        image.verify()

print(
    "PNG verification complete."
)
PY
```

本次最终测试结果：

```text
Found 3 projection images

size=(640, 480)
mode=RGB
format=PNG

PNG verification complete.
```

---

## 18. 项目中的实验节点

除了正式投影节点，还保留了几个用于理解和验证过程的节点。

### `tf_static_publisher`

从 YAML 读取传感器外参并发布静态 TF。

### `sync_test_node`

观察单一 `slop` 下 ATS 的实时匹配结果和 timestamp difference。

### `sync_sweep_node`

离线读取 rosbag，对 `20 ms ~ 150 ms` 的 slop 做可重复实验。

### `transform_test_node`

将若干代表性 LaserScan beam 从极坐标转换到 `laser_link`，再通过 TF2 转换到 `camera_link`，用于验证空间变换。

### `time_test_node`

比较 node ROS clock 与 LaserScan header timestamp，用于验证 `use_sim_time=true/false` 的区别。

### `projection_node`

正式完成同步、TF2、坐标轴转换、针孔投影、距离着色、ROS Image 发布以及 PNG 保存。

---

## 19. 最终结果

本项目完成：

- `/scan` 与 `/camera/image_raw` 的近似时间同步
- `queue_size >= 50`
- `20 ms ~ 150 ms` slop 实验
- `map -> odom -> base_link -> sensor` TF 树
- 从 `extrinsics.yaml` 读取 sensor extrinsics
- 从 `extrinsics.yaml` 读取 camera intrinsics
- LaserScan 极坐标到 3D Cartesian 转换
- `laser_link -> camera_link` TF2 转换
- camera body frame 到 optical frame 转换
- 针孔模型 3D -> 2D 投影
- 图像范围过滤
- 近红、远蓝距离可视化
- `/projection/debug_image` 的 `rgb8` 输出
- 3 张不同时间的 `projection_*.png`
- `use_sim_time=true/false` 对比实验
- 非默认 `laser-z/camera-z/fx` 验收测试
- Git 分阶段记录开发过程