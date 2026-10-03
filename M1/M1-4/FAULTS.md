# 任务二：故障记录手册

本文件记录 M1-4 图像链路优化过程中实际遇到的故障。

所有记录均来自本次实际调试过程，包括失败尝试、实际命令和测量结果，不额外编造故障。

---

## 故障 1：NumPy 2.2.6 与 ROS Humble 的 cv_bridge 不兼容，节点无法启动

### ① 现象

第一次直接运行：

```bash
python3 camera_pub.py
```

程序没有正常启动，而是出现：

```text
A module that was compiled using NumPy 1.x cannot be run in
NumPy 2.2.6 as it may crash. To support both 1.x and 2.x
versions of NumPy, modules must be compiled with NumPy 2.0.

AttributeError: _ARRAY_API not found

ImportError: numpy.core.multiarray failed to import
```

报错栈指向：

```text
from cv_bridge import CvBridge
```

以及：

```text
self.bridge = CvBridge()
```

预期是相机节点正常启动并发布 `/camera/image_raw`，实际情况是节点在初始化阶段直接退出。

### ② 定位过程

一开始我没有修改代码，而是先确认当前 Python 实际加载的是哪个 NumPy。

执行：

```bash
python3 -c "import numpy; print('version =', numpy.__version__); print('path =', numpy.__file__)"
```

输出：

```text
version = 2.2.6
path = /home/adtyuhd/.local/lib/python3.10/site-packages/numpy/__init__.py
```

这说明当前 Python 优先加载的是用户目录中的 NumPy 2.2.6。

然后执行：

```bash
python3 -m pip show numpy
```

确认 NumPy 位于：

```text
/home/adtyuhd/.local/lib/python3.10/site-packages
```

接下来没有直接卸载 NumPy，而是先进行无损测试，暂时关闭 Python 用户级 site-packages：

```bash
PYTHONNOUSERSITE=1 python3 -c "import numpy; print('numpy =', numpy.__version__); print('path =', numpy.__file__); from cv_bridge import CvBridge; print('cv_bridge OK')"
```

输出：

```text
numpy = 1.21.5
path = /usr/lib/python3/dist-packages/numpy/__init__.py
cv_bridge OK
```

因此排除了以下可能：

- `cv_bridge` 没有安装；
- ROS Humble 环境完全损坏；
- `camera_pub.py` 本身有 Python 语法错误。

问题只在使用 NumPy 2.2.6 时出现。

### ③ 根因

当前 ROS Humble 中安装的 `cv_bridge` 二进制模块是针对 NumPy 1.x ABI 构建的。

但是用户目录：

```text
/home/adtyuhd/.local/lib/python3.10/site-packages
```

中安装了 NumPy 2.2.6，并且它的加载优先级高于：

```text
/usr/lib/python3/dist-packages
```

中的系统 NumPy 1.21.5。

因此 `cv_bridge` 加载底层二进制扩展时遇到了 NumPy ABI 不兼容，最终出现：

```text
AttributeError: _ARRAY_API not found
```

以及：

```text
ImportError: numpy.core.multiarray failed to import
```

### ④ 修复

我没有直接卸载 NumPy 2.2.6，因为本机还有其他软件可能依赖它。

本实验运行 ROS 节点时使用：

```bash
PYTHONNOUSERSITE=1 python3 camera_pub.py
```

处理端同样使用：

```bash
PYTHONNOUSERSITE=1 python3 image_proc.py
```

这样 Python 会忽略用户目录里的 NumPy 2.2.6，转而加载系统 NumPy 1.21.5。

### ⑤ 验证

修复前：

```text
NumPy: 2.2.6
cv_bridge: 导入失败
camera_pub.py: 无法启动
```

修复后测试：

```bash
PYTHONNOUSERSITE=1 python3 -c "import numpy; print('numpy =', numpy.__version__); print('path =', numpy.__file__); from cv_bridge import CvBridge; print('cv_bridge OK')"
```

输出：

```text
numpy = 1.21.5
path = /usr/lib/python3/dist-packages/numpy/__init__.py
cv_bridge OK
```

随后：

```bash
PYTHONNOUSERSITE=1 python3 camera_pub.py
```

相机节点可以正常启动。

| 指标 | 修复前 | 修复后 |
| :-- | :-- | :-- |
| NumPy | 2.2.6 | 1.21.5 |
| cv_bridge 导入 | 失败 | 成功 |
| camera_pub.py | 无法启动 | 正常运行 |

**遗留问题**：

当前只是通过 `PYTHONNOUSERSITE=1` 隔离 ROS 环境，没有删除用户目录中的 NumPy 2.2.6。

因此如果直接运行：

```bash
python3 camera_pub.py
```

仍有可能重新遇到相同问题。

---

## 故障 2：相机发布端只有约 15 Hz，达不到目标 30 Hz

### ① 现象

解决 Python 环境问题之后，启动原始相机程序：

```bash
PYTHONNOUSERSITE=1 python3 camera_pub.py
```

相机自己打印：

```text
[CAMERA] 已发布 46 帧, 平均 14.95 Hz (目标 30.0 Hz)
[CAMERA] 已发布 92 帧, 平均 15.10 Hz (目标 30.0 Hz)
[CAMERA] 已发布 138 帧, 平均 15.16 Hz (目标 30.0 Hz)
[CAMERA] 已发布 183 帧, 平均 15.13 Hz (目标 30.0 Hz)
```

目标是 30 Hz，但实际只有约 15.1 Hz。

### ② 定位过程

首先确认相机定时器本身配置没有写错。

代码中的目标频率是：

```python
CAMERA_HZ = 30.0
```

定时器：

```python
self.timer = self.create_timer(
    1.0 / CAMERA_HZ,
    self.on_timer,
)
```

因此定时器目标确实是 30 Hz。

然后检查每次定时器触发后执行的 `capture()`。

原实现中每帧都会执行：

```python
frame = np.zeros(
    (IMAGE_HEIGHT, IMAGE_WIDTH, 3),
    dtype=np.uint8,
)

for y in range(IMAGE_HEIGHT):
    row = frame[y]
    off = (y * 13 + base) & 0xFF

    for x in range(IMAGE_WIDTH):
        value = (x + off) & 0xFF
        row[x, 0] = value
        row[x, 1] = value
        row[x, 2] = 60
```

640×480 一帧共有：

```text
640 × 480 = 307200
```

个像素。

也就是说每帧都要经过 30 多万次 Python 内层循环。

30 Hz 的时间预算只有：

```text
1 / 30 ≈ 33.3 ms
```

因此重点怀疑每帧造图时间超过了实时周期预算。

### ③ 根因

根因是相机测试图生成采用 Python 双重循环逐像素计算。

Python 解释器不适合执行大量高频逐像素操作。

原程序每发布一帧，都需要：

1. 新建一张 640×480×3 图像；
2. Python 遍历 480 行；
3. 每行再遍历 640 个像素；
4. 对三个颜色通道逐项赋值。

虽然每个像素计算本身很简单，但是大量 Python 循环的解释器开销使整个 `on_timer()` 无法在约 33.3 ms 内完成。

最终定时器来不及按 30 Hz 调度，实际只能发布约 15 Hz。

### ④ 修复

保留原来的图像数学规律，但是改为 NumPy 向量化。

先在初始化阶段生成基础二维图案：

```python
x = np.arange(
    IMAGE_WIDTH,
    dtype=np.uint16,
)

y = np.arange(
    IMAGE_HEIGHT,
    dtype=np.uint16,
).reshape(-1, 1)

self.base_pattern = (
    (x + y * 13) & 0xFF
).astype(np.uint8)
```

图像缓冲区也只创建一次：

```python
self.frame = np.empty(
    (
        IMAGE_HEIGHT,
        IMAGE_WIDTH,
        3,
    ),
    dtype=np.uint8,
)

self.frame[:, :, 2] = 60
```

每帧只执行：

```python
seq_offset = np.uint8(
    self.seq & 0xFF
)

np.add(
    self.base_pattern,
    seq_offset,
    out=self.frame[:, :, 0],
)

self.frame[:, :, 1] = self.frame[:, :, 0]
```

也就是说，计算语义没有变化，只是把逐像素循环交给 NumPy 底层实现。

### ⑤ 验证

优化前：

```text
约 15.1 Hz
```

优化后运行：

```bash
PYTHONNOUSERSITE=1 python3 camera_pub.py
```

得到：

```text
[CAMERA] 最近 3.00 秒发布 90 帧, 30.01 Hz (目标 30.0 Hz)
[CAMERA] 最近 3.00 秒发布 90 帧, 30.00 Hz (目标 30.0 Hz)
[CAMERA] 最近 3.00 秒发布 90 帧, 30.00 Hz (目标 30.0 Hz)
[CAMERA] 最近 3.00 秒发布 90 帧, 29.99 Hz (目标 30.0 Hz)
[CAMERA] 最近 3.00 秒发布 90 帧, 30.01 Hz (目标 30.0 Hz)
```

最终正式记录：

```text
[CAMERA] 最近 3.00 秒发布 90 帧, 30.00 Hz (目标 30.0 Hz)
```

| 指标 | 修复前 | 修复后 |
| :-- | :-- | :-- |
| `/camera/image_raw` | ≈15.1 Hz | ≈30.0 Hz |
| 分辨率 | 640×480 | 640×480 |
| 编码 | bgr8 | bgr8 |

说明本次没有通过降低分辨率或降低目标频率来换性能。

---

## 故障 3：图像处理只有约 3 Hz，逐像素处理和无意义放大严重拖慢链路

### ① 现象

相机已经恢复到约 30 Hz 后，启动原始处理节点：

```bash
PYTHONNOUSERSITE=1 python3 image_proc.py
```

然后测量：

```bash
ros2 topic hz /perception/result
```

得到：

```text
average rate: 3.300
        min: 0.298s max: 0.315s std dev: 0.00361s window: 17

average rate: 3.258
        min: 0.298s max: 0.315s std dev: 0.00461s window: 41

average rate: 3.241
        min: 0.298s max: 0.321s std dev: 0.00475s window: 61
```

输出只有约：

```text
3.24 Hz
```

而目标是 30 Hz。

### ② 定位过程

首先检查图像处理代码。

发现输入图像本来是：

```text
640 × 480
```

但代码首先将图像放大为：

```python
WORK_WIDTH = 1280
WORK_HEIGHT = 720
```

并执行：

```python
large = cv2.resize(
    gray,
    (WORK_WIDTH, WORK_HEIGHT),
    interpolation=cv2.INTER_LINEAR,
)
```

然后在 1280×720 图像上执行两遍 Python 双重循环。

第一遍：

```python
for y in range(height):
    for x in range(width):
        value = int(large[y, x]) + 20
```

第二遍：

```python
for y in range(height):
    for x in range(width):
        value = (
            (int(brightened[y, x]) - 128)
            * 3
            // 2
            + 128
        )
```

单遍像素数：

```text
1280 × 720 = 921600
```

两遍就是超过：

```text
1,843,200
```

次 Python 像素循环。

随后还要再缩回：

```text
640 × 480
```

我先只删除了无意义的 1280×720 放大和最后缩小，但仍保留双重循环。

重新测量：

```bash
ros2 topic hz /perception/result
```

结果提升到：

```text
约 9.7 Hz
```

例如：

```text
average rate: 9.720
average rate: 9.743
average rate: 9.765
```

这说明中间放大确实浪费了大量计算，但仍然没有达到 30 Hz。

然后我只向量化第一遍 `+20`，第二遍对比度循环仍然保留。

结果仍然约：

```text
9.7 Hz
```

说明单独剩下一遍 Python 双重循环，就足以成为主要瓶颈。

最后将第二遍对比度也改成 NumPy 向量化。

### ③ 根因

根因有两个。

第一，中间分辨率被无意义地从：

```text
640×480
```

放大到：

```text
1280×720
```

像素数量从：

```text
307200
```

增加到：

```text
921600
```

但题目并没有要求在更高分辨率上处理，最终输出仍然是 640×480。

所以这一放大和缩小只增加了计算量，没有增加任务需要的有效信息。

第二，亮度和对比度增强都采用 Python 双重循环逐像素实现。

这种操作本质上非常适合数组向量化，不应该放在 Python 解释器中逐像素执行。

### ④ 修复

首先删除：

```text
640×480
→ 1280×720
→ 处理
→ 640×480
```

改为直接在原始 640×480 图像上处理。

随后将：

```python
for y in range(height):
    for x in range(width):
        ...
```

改成 NumPy 整图运算。

最终核心处理：

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

payload = work.astype(
    np.uint8
)
```

这里使用 `int16` 是为了避免 `uint8` 运算溢出。

例如一个像素值：

```text
250 + 20
```

数学结果是：

```text
270
```

如果始终使用 `uint8`，就可能发生模 256 溢出。

所以先转为 `int16`，完成运算后再裁剪到：

```text
0~255
```

最后转回 `uint8`。

### ⑤ 验证

原始版本：

```text
约 3.24 Hz
```

删除无意义 resize 后：

```text
约 9.7 Hz
```

向量化第一遍、仍保留第二遍 Python 循环时：

```text
仍约 9.7 Hz
```

两遍全部向量化后：

```text
约 27 Hz 以上
```

后续配合 QoS 和执行器优化，最终正式测量：

```bash
ros2 topic hz /perception/result
```

得到：

```text
average rate: 29.919
        min: 0.020s max: 0.046s std dev: 0.00701s window: 152

average rate: 29.948
        min: 0.020s max: 0.046s std dev: 0.00661s window: 183

average rate: 29.962
        min: 0.020s max: 0.046s std dev: 0.00679s window: 244
```

| 阶段 | `/perception/result` |
| :-- | --: |
| 原始版本 | ≈3.24 Hz |
| 删除 1280×720 中间 resize | ≈9.7 Hz |
| 仅向量化一部分 | ≈9.7 Hz |
| 最终版本 | ≈29.96 Hz |

---

## 故障 4：单线程执行器下图像回调频率异常，多线程执行器恢复稳定 30 Hz

### ① 现象

图像算法完成向量化之后，曾经出现一个比较反常的现象：

单帧图像处理本身非常快，但是 `image_proc` 实际处理到的图像频率却只有几 Hz。

为了排除 `ros2 topic hz` 自己读数异常，我在 `image_proc.py` 内部增加了处理频率统计。

其中一组结果：

```text
[PROC] 2.00 Hz | convert 0.28 ms | process 0.56 ms | build 0.14 ms | publish 0.13 ms | total 1.12 ms

[PROC] 4.33 Hz | convert 0.27 ms | process 0.57 ms | build 0.15 ms | publish 0.14 ms | total 1.14 ms

[PROC] 4.00 Hz | convert 0.34 ms | process 0.66 ms | build 0.15 ms | publish 0.14 ms | total 1.30 ms
```

这说明一个明显矛盾：

```text
单帧 callback ≈ 1 ms
```

但实际收到并处理的图像却只有：

```text
约 2~7 Hz
```

如果真的是算法吞吐量不足，那么单帧处理时间应该接近几百毫秒，而不是 1 ms。

### ② 定位过程

首先怀疑 CPU 是否被其他程序占满。

执行：

```bash
ps -eo pid,comm,%cpu,%mem,args --sort=-%cpu | head -n 20
```

结果中：

```text
python3 image_proc.py
```

CPU 占用只有个位数百分比，没有出现 CPU 满载。

因此排除了“整个 CPU 被打满”的可能。

然后进一步测量处理过程的各阶段耗时：

```text
convert ≈ 0.2~0.5 ms
process ≈ 0.5~0.8 ms
build ≈ 0.1~0.3 ms
publish ≈ 0.1~0.2 ms
total ≈ 1~2 ms
```

因此也排除了：

- cv_bridge 转换过慢；
- NumPy 图像增强过慢；
- ROS Image 构造过慢；
- `publish()` 本身每帧阻塞几百毫秒。

接着检查执行器。

原代码使用：

```python
from rclpy.executors import SingleThreadedExecutor
```

并且：

```python
executor = SingleThreadedExecutor()
```

图像节点和控制节点都被加入同一个单线程执行器。

于是进行单变量实验，只修改执行器：

```python
from rclpy.executors import MultiThreadedExecutor
```

并改为：

```python
executor = MultiThreadedExecutor(
    num_threads=2,
)
```

其他图像算法和 QoS 不修改。

### ③ 根因

在本机实际运行环境中，`SingleThreadedExecutor` 下图像订阅回调、统计 timer 和控制 timer 全部依赖同一个执行线程调度。

虽然单次图像处理只有约 1 ms，但是单线程执行器的整体调度表现不稳定，图像订阅回调没有稳定获得 30 Hz 的执行机会。

因此问题不是：

```text
一帧算得慢
```

而是：

```text
回调没有被及时调度
```

换成两个 worker 的 `MultiThreadedExecutor` 后，控制定时器和图像相关回调不再全部绑定到唯一执行线程。

这里我只能根据实验确认“单线程执行器是触发条件，多线程执行器可以解决”；更底层的 DDS/执行器内部调度细节没有继续深入到源码级分析。

### ④ 修复

将：

```python
from rclpy.executors import SingleThreadedExecutor
```

改成：

```python
from rclpy.executors import MultiThreadedExecutor
```

原来：

```python
executor = SingleThreadedExecutor()
```

修改为：

```python
executor = MultiThreadedExecutor(
    num_threads=2,
)
```

两个节点继续加入同一个 executor：

```python
for node in nodes:
    executor.add_node(node)
```

### ⑤ 验证

修改前内部统计曾出现：

```text
[PROC] 2.00 Hz
[PROC] 3.00 Hz
[PROC] 4.33 Hz
```

但单帧处理：

```text
约 1~2 ms
```

修改执行器后：

```text
[PROC] 29.99 Hz | convert 0.29 ms | process 0.49 ms | build 0.15 ms | publish 0.08 ms | total 1.01 ms

[PROC] 30.01 Hz | convert 0.41 ms | process 0.60 ms | build 0.21 ms | publish 0.14 ms | total 1.36 ms

[PROC] 30.00 Hz | convert 0.40 ms | process 0.58 ms | build 0.20 ms | publish 0.13 ms | total 1.32 ms

[PROC] 30.00 Hz | convert 0.38 ms | process 0.58 ms | build 0.20 ms | publish 0.13 ms | total 1.30 ms
```

外部测量：

```bash
ros2 topic hz /perception/result
```

得到：

```text
average rate: 29.998
average rate: 30.001
average rate: 30.000
```

控制话题：

```bash
ros2 topic hz /control/cmd
```

得到：

```text
average rate: 19.997
average rate: 19.999
average rate: 19.998
```

| 指标 | SingleThreadedExecutor 异常状态 | MultiThreadedExecutor |
| :-- | --: | --: |
| image_proc 内部频率 | 约 2~7 Hz | ≈30 Hz |
| 单帧处理时间 | ≈1~2 ms | ≈1~1.5 ms |
| `/perception/result` | 不稳定 | ≈30 Hz |
| `/control/cmd` | 目标 20 Hz | ≈20 Hz |

这说明瓶颈不是单帧图像计算，而是执行器调度。

---

# 最终验收结果

最终相机端：

```text
[CAMERA] 最近 3.00 秒发布 90 帧, 30.00 Hz (目标 30.0 Hz)
```

最终 `/perception/result`：

```text
average rate: 29.962
        min: 0.020s max: 0.046s std dev: 0.00679s window: 244
```

最终 `/control/cmd`：

```text
average rate: 19.998
        min: 0.047s max: 0.053s std dev: 0.00117s window: 165
```

最终端到端延迟：

```text
average delay: 0.010
        min: 0.003s max: 0.019s std dev: 0.00599s window: 239
```

因此：

| 指标 | 基线 | 最终结果 | 目标 |
| :-- | --: | --: | --: |
| 输入 `/camera/image_raw` | ≈15.6 Hz | 30.00 Hz | ≥30 Hz |
| `/perception/result` | ≈2.3 Hz | ≈29.96 Hz | ≈30 Hz |
| `/control/cmd` | ≈2.3 Hz | ≈20.00 Hz | ≥20 Hz |
| 端到端延迟 | P95 ≈2443 ms | 本次观测最大 19 ms | P95 < 200 ms |

由于最终延迟测试中观测到的最大值只有：

```text
19 ms
```

所以该测量窗口中的 P95 必然不大于 19 ms，满足 P95 < 200 ms 的要求。

---

# 本次调优的主要经验

本次实际调试过程中，我最开始容易把所有“频率低”问题都理解成 CPU 算力不足，但实际发现不同阶段的原因并不相同。

发布端的主要问题是：

```text
Python 逐像素造图
```

处理端最初的主要问题是：

```text
无意义放大图像 + Python 逐像素处理
```

而后面出现的低频异常又证明：

```text
单帧计算很快
```

并不代表：

```text
整个 ROS 2 链路一定能稳定调度
```

必须分别测量：

- 发布频率；
- 订阅处理频率；
- 单帧 callback 时间；
- 输出频率；
- 控制频率；
- 端到端延迟。

这样才能区分：

```text
算得慢
```

和：

```text
排队 / 调度 / 通信导致的慢
```

这也是本次任务中最重要的收获。