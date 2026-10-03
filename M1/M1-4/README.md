# M1-4 系统调优：从“能跑”到“跑得稳、跑得快”

## 1. 任务目标

本任务优化一条 ROS 2 图像处理全链路：

```text
camera_pub.py
    ↓
/camera/image_raw
    ↓
image_proc.py
    ↓
/perception/result
    +
/control/cmd
```

接口要求：

| 话题 | 类型 | 要求 |
| :-- | :-- | :-- |
| `/camera/image_raw` | `sensor_msgs/Image` | 640×480，30 Hz |
| `/perception/result` | `sensor_msgs/Image` | 640×480，`mono8`，30 Hz |
| `/control/cmd` | `geometry_msgs/Twist` | 20 Hz |

图像处理语义保持为：

```text
输入图像
→ 灰度
→ 提亮 +20
→ 对比度增强 ((v - 128) * 3 / 2 + 128)
→ 限制到 [0, 255]
→ mono8 输出
```

同时要求：

```text
输出图像 header.stamp 必须沿用输入图像的 header.stamp
```

不能重新打当前时间戳。

---

## 2. 运行环境问题

本机用户目录中安装了 NumPy 2.2.6，而 ROS Humble 当前的 `cv_bridge` 与该版本不兼容。

直接运行：

```bash
python3 camera_pub.py
```

会出现：

```text
AttributeError: _ARRAY_API not found

ImportError: numpy.core.multiarray failed to import
```

本实验通过：

```bash
PYTHONNOUSERSITE=1
```

让 Python 使用系统 NumPy 1.21.5。

因此实际运行命令为：

### 终端 1

```bash
cd ~/22SmartCar/M1/M1-4
PYTHONNOUSERSITE=1 python3 camera_pub.py
```

### 终端 2

```bash
cd ~/22SmartCar/M1/M1-4
PYTHONNOUSERSITE=1 python3 image_proc.py
```

---

## 3. 基线性能

题目给出的基线约为：

| 版本 | 输入 | `/perception/result` | `/control/cmd` | P95 延迟 |
| :-- | --: | --: | --: | --: |
| 基线 | ≈15.6 Hz | ≈2.3 Hz | ≈2.3 Hz | ≈2443 ms |

我实际开始调试时，相机端测到：

```text
[CAMERA] 已发布 46 帧, 平均 14.95 Hz (目标 30.0 Hz)
[CAMERA] 已发布 92 帧, 平均 15.10 Hz (目标 30.0 Hz)
[CAMERA] 已发布 138 帧, 平均 15.16 Hz (目标 30.0 Hz)
```

处理端在相机优化后，原始处理代码仍只有约：

```text
3.24 Hz
```

说明发布端和处理端分别存在独立瓶颈。

---

# 4. 发布端优化

## 4.1 原始问题

原始 `camera_pub.py` 每生成一帧都执行 Python 双重循环：

```python
for y in range(IMAGE_HEIGHT):
    for x in range(IMAGE_WIDTH):
        ...
```

640×480 一帧共有：

```text
307200
```

个像素。

相机目标为 30 Hz，因此单帧总时间预算只有：

```text
1 / 30 ≈ 33.3 ms
```

Python 逐像素循环占用了过多时间，导致实际发布频率只有约 15 Hz。

---

## 4.2 优化方式

原像素公式可以整理为：

```text
value = (x + y * 13 + seq) & 0xFF
```

因此将与帧序号无关的部分提前使用 NumPy 计算：

```python
self.base_pattern = (
    (x + y * 13) & 0xFF
).astype(np.uint8)
```

图像内存也只初始化一次：

```python
self.frame = np.empty(
    (IMAGE_HEIGHT, IMAGE_WIDTH, 3),
    dtype=np.uint8,
)
```

每帧使用 NumPy 向量化：

```python
np.add(
    self.base_pattern,
    seq_offset,
    out=self.frame[:, :, 0],
)
```

这样保留了原来的图像生成规律，但避免 Python 对 307200 个像素逐个解释执行。

---

## 4.3 优化结果

最终测量：

```text
[CAMERA] 最近 3.00 秒发布 90 帧, 30.00 Hz (目标 30.0 Hz)
[CAMERA] 最近 3.00 秒发布 90 帧, 30.00 Hz (目标 30.0 Hz)
[CAMERA] 最近 3.00 秒发布 90 帧, 30.00 Hz (目标 30.0 Hz)
[CAMERA] 最近 3.00 秒发布 90 帧, 30.00 Hz (目标 30.0 Hz)
```

发布端：

```text
约 15.1 Hz
→
30.00 Hz
```

---

# 5. 处理端优化

## 5.1 删除无意义的中间放大

原处理代码会：

```text
640×480
→ resize 到 1280×720
→ 处理
→ resize 回 640×480
```

题目并没有要求在 1280×720 分辨率上进行处理。

1280×720 有：

```text
921600
```

个像素，是 640×480 的约 3 倍。

删除中间放大和缩小后，处理频率从：

```text
≈3.24 Hz
```

提升到：

```text
≈9.7 Hz
```

说明中间 resize 确实消耗了大量无意义计算。

---

## 5.2 将逐像素处理改为 NumPy 向量化

原代码的提亮和对比度增强都是 Python 双重循环。

原来的逻辑类似：

```python
for y in range(height):
    for x in range(width):
        value = int(image[y, x]) + 20
```

以及：

```python
for y in range(height):
    for x in range(width):
        value = (
            (int(image[y, x]) - 128)
            * 3
            // 2
            + 128
        )
```

最终改成数组运算：

```python
work = np.array(
    gray,
    dtype=np.int16,
    copy=True,
)

work += 20

np.clip(
    work,
    0,
    255,
    out=work,
)

work -= 128
work *= 3
work //= 2
work += 128

np.clip(
    work,
    0,
    255,
    out=work,
)

payload = work.astype(np.uint8)
```

### 为什么使用 int16

图像原始类型是：

```text
uint8
```

范围只有：

```text
0 ~ 255
```

例如：

```text
250 + 20 = 270
```

如果直接使用 `uint8` 运算可能发生溢出。

所以计算阶段先转换为：

```text
int16
```

最后使用：

```python
np.clip(...)
```

将结果限制到：

```text
0 ~ 255
```

再转回 `uint8`。

---

# 6. QoS：限制队列积压

原订阅 QoS 使用：

```python
depth=30
```

这意味着处理端最多可以保存很多尚未处理的旧图像。

对于实时视觉链路，如果处理速度低于输入速度，大队列可能形成：

```text
最新帧
 ↓
[旧帧][旧帧][旧帧][旧帧]...
```

即使每一帧最终都被处理，机器人看到的也可能是很久以前的数据。

因此图像输入改成：

```python
depth=1
```

即：

```text
KEEP_LAST + depth=1
```

只保留很少的待处理消息，避免历史帧持续积压。

在调试过程中，修改 `depth=1` 后曾测到：

```text
average rate: 29.986
average rate: 29.989
average rate: 29.991
average rate: 29.998
```

并且帧间隔变得稳定：

```text
min: 0.021s
max: 0.046s
```

---

# 7. 执行器优化

原程序使用：

```python
SingleThreadedExecutor
```

图像节点和控制节点都放在同一个单线程执行器中。

后续调试时，我在节点内部测得：

```text
单帧图像 callback 总时间 ≈ 1 ms
```

但是实际图像回调有时只有：

```text
2~7 Hz
```

说明当时的问题已经不是图像算法本身算得慢，而是执行器调度表现异常。

将：

```python
SingleThreadedExecutor
```

改成：

```python
MultiThreadedExecutor(
    num_threads=2,
)
```

后，内部统计立即恢复到：

```text
[PROC] 29.99 Hz
[PROC] 30.01 Hz
[PROC] 30.00 Hz
[PROC] 30.00 Hz
```

单帧处理时间仍然只有约：

```text
1.0 ~ 1.5 ms
```

因此最终使用双 worker 多线程执行器。

---

# 8. 控制链路

控制话题要求：

```text
/control/cmd = 20 Hz
```

控制循环使用独立定时器：

```python
self.timer = self.create_timer(
    1.0 / CONTROL_HZ,
    self.on_timer,
)
```

其中：

```python
CONTROL_HZ = 20.0
```

最终正式测量：

```text
average rate: 19.997
        min: 0.047s max: 0.053s std dev: 0.00137s window: 103

average rate: 19.995
        min: 0.047s max: 0.053s std dev: 0.00129s window: 123

average rate: 19.999
        min: 0.047s max: 0.053s std dev: 0.00122s window: 144

average rate: 19.998
        min: 0.047s max: 0.053s std dev: 0.00117s window: 165
```

即：

```text
≈20.00 Hz
```

---

# 9. 时间戳与端到端延迟

输出消息严格沿用输入时间戳：

```python
out.header.stamp = msg.header.stamp
```

没有使用：

```python
self.get_clock().now().to_msg()
```

重新生成时间戳。

因此：

```bash
ros2 topic delay /perception/result
```

测到的是从相机采集/发布时间开始的真实端到端延迟，而不是处理完成后重新计时造成的“假低延迟”。

最终正式测量：

```text
average delay: 0.007
        min: 0.003s max: 0.017s std dev: 0.00457s window: 89

average delay: 0.008
        min: 0.003s max: 0.018s std dev: 0.00529s window: 148

average delay: 0.009
        min: 0.003s max: 0.019s std dev: 0.00582s window: 209

average delay: 0.010
        min: 0.003s max: 0.019s std dev: 0.00599s window: 239
```

本次测量窗口中的最大值只有：

```text
19 ms
```

因此该测量窗口中的：

```text
P95 <= 19 ms
```

满足：

```text
P95 < 200 ms
```

---

# 10. 输出格式验证

执行：

```bash
ros2 topic echo /perception/result --once | head -n 15
```

结果：

```text
header:
  stamp:
    sec: 1790993570
    nanosec: 381988298
  frame_id: camera_link
height: 480
width: 640
encoding: mono8
is_bigendian: 0
step: 640
```

说明最终输出满足：

```text
640×480
mono8
```

接口要求。

---

# 11. 最终性能结果

最终长时间测量 `/perception/result`：

```text
average rate: 29.991
        min: 0.018s max: 0.047s std dev: 0.00467s window: 885

average rate: 29.990
        min: 0.018s max: 0.047s std dev: 0.00469s window: 915

average rate: 30.000
        min: 0.018s max: 0.047s std dev: 0.00465s window: 976

average rate: 29.991
        min: 0.018s max: 0.047s std dev: 0.00466s window: 1006
```

综合结果：

| 版本 | 输入 | `/perception/result` | `/control/cmd` | P95 延迟 |
| :-- | --: | --: | --: | --: |
| 基线 | ≈15.6 Hz | ≈2.3 Hz | ≈2.3 Hz | ≈2443 ms |
| 优化后 | 30.00 Hz | ≈30.00 Hz | ≈20.00 Hz | ≤19 ms（本次窗口） |

---

# 12. 我牺牲了什么

本次优化没有牺牲以下内容：

```text
没有降低输入分辨率
没有降低输出分辨率
没有降低相机目标频率
没有直接转发原始图像
没有修改处理语义
没有伪造 header.stamp
```

输入仍为：

```text
640×480 bgr8
```

输出仍为：

```text
640×480 mono8
```

真正的主要取舍是：

## 12.1 用队列完整性换实时性

图像订阅：

```python
depth=1
```

意味着系统过载时，不追求长期保存大量尚未处理的旧图像。

相比：

```text
“每一帧最终都必须处理”
```

本实现更重视：

```text
“当前处理的数据必须尽量新”
```

即：

```text
freshness > completeness
```

对于机器人实时感知链路，我认为这是合理的取舍。

## 12.2 使用更多执行线程

最终使用：

```python
MultiThreadedExecutor(
    num_threads=2,
)
```

相比单线程执行器，会使用额外的线程资源。

换来的效果是图像处理与控制相关回调不再全部依赖一个执行线程进行调度。

## 12.3 使用额外内存保存预计算数据

相机端提前生成：

```python
self.base_pattern
```

以及复用：

```python
self.frame
```

因此使用了一小部分额外常驻内存。

但避免了每帧大量重复计算和重复内存分配。

---

# 13. 测量命令

## 输入真实频率

按题目要求，直接观察相机自己的：

```text
[CAMERA]
```

日志。

不使用：

```bash
ros2 topic hz /camera/image_raw
```

作为输入真值。

---

## 输出频率

```bash
ros2 topic hz /perception/result
```

正式记录：

```text
measurements/final_result_hz.txt
```

---

## 控制频率

```bash
ros2 topic hz /control/cmd
```

正式记录：

```text
measurements/final_cmd_hz.txt
```

---

## 端到端延迟

```bash
ros2 topic delay /perception/result
```

正式记录：

```text
measurements/final_delay.txt
```

---

## 输出格式

```bash
ros2 topic echo /perception/result --once | head -n 15
```

正式记录：

```text
measurements/final_result_format.txt
```

---

# 14. 最终结论

本次优化分别处理了发布端、算法实现、消息队列以及执行器调度问题。

最终：

```text
camera_pub.py
    30 Hz
      ↓
/camera/image_raw
      ↓
image_proc.py
      ↓
/perception/result ≈ 30 Hz

/control/cmd ≈ 20 Hz

端到端延迟：
本次测量 max = 19 ms
P95 <= 19 ms
```

满足本题性能要求。