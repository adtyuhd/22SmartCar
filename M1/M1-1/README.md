# M1-1 三节点链路 + 动态配参

## 功能说明

本项目实现一个虚拟超声波测距链路：

```text
Node A
  |
  | /sensor_data
  v
Node B
  |
  | /processed_distance
  v
Node C
  |
  | /trigger_alarm
  v
Node A Alarm Service
```

- Node A：以 10 Hz 发布模拟距离数据，并提供报警服务
- Node B：对有效距离数据进行一阶低通滤波
- Node C：检测距离是否低于报警阈值，并异步调用报警服务
- Node C 在报警服务 1 秒无响应时输出超时警告

## 自定义接口

### SensorData.msg

```text
float32 distance
string unit
builtin_interfaces/Time stamp
uint8 status
```

`status` 定义：

- `0`：正常
- `1`：超出量程
- `2`：传感器错误

### TriggerAlarm.srv

```text
float32 distance
---
bool success
string message
```

## Topic 和 Service

| 名称 | 类型 | 方向 |
|---|---|---|
| `/sensor_data` | `sensor_interfaces/msg/SensorData` | Node A -> Node B |
| `/processed_distance` | `std_msgs/msg/Float32` | Node B -> Node C |
| `/trigger_alarm` | `sensor_interfaces/srv/TriggerAlarm` | Node C -> Alarm Server |

## 参数

Node B：

| 参数 | 类型 | 默认值 |
|---|---|---|
| `alpha` | double | `0.3` |

Node C：

| 参数 | 类型 | 默认值 |
|---|---|---|
| `alarm_threshold` | double | `0.3` |
| `service_timeout` | double | `1.0` |

低通滤波公式：

```text
y_k = alpha * x_k + (1 - alpha) * y_(k-1)
```

当输入数据 `status != 0` 时，Node B 输出警告并忽略该数据。

## 编译

在 `M1/M1-1` 目录执行：

```bash
colcon build
source install/setup.bash
```

## 运行

Terminal 1：

```bash
source install/setup.bash
ros2 run sensor_chain node_a_sensor
```

Terminal 2：

```bash
source install/setup.bash
ros2 run sensor_chain node_b_filter
```

Terminal 3：

```bash
source install/setup.bash
ros2 run sensor_chain node_c_alarm
```

## 检查节点

```bash
ros2 node list
```

应包含：

```text
/node_a_sensor
/node_b_filter
/node_c_alarm
```

## 检查 Topic

```bash
ros2 topic list
ros2 topic type /sensor_data
ros2 topic type /processed_distance
```

检查处理后数据频率：

```bash
ros2 topic hz /processed_distance
```

正常情况下频率约为 10 Hz。

## 动态修改 alpha

查看参数：

```bash
ros2 param get /node_b_filter alpha
```

动态修改：

```bash
ros2 param set /node_b_filter alpha 0.9
```

再次查看：

```bash
ros2 param get /node_b_filter alpha
```

Node B 会在运行过程中使用新的 `alpha` 进行低通滤波。

## 测试无效传感器数据

停止 Node A 后执行：

```bash
ros2 topic pub --once /sensor_data sensor_interfaces/msg/SensorData "{distance: 9.9, unit: 'm', status: 1}"
```

Node B 应输出警告，并忽略该数据。

## 手动测试报警服务

```bash
ros2 service call /trigger_alarm sensor_interfaces/srv/TriggerAlarm "{distance: 0.2}"
```

服务端应输出：

```text
[ALARM] 距离过近！
```

并返回成功响应。

## 测试报警超时

运行 Node B 和 Node C，但停止提供 `/trigger_alarm` 服务的 Node A。

向 `/sensor_data` 连续发布低距离数据：

```bash
ros2 topic pub -r 10 /sensor_data sensor_interfaces/msg/SensorData "{distance: 0.1, unit: 'm', status: 0}"
```

当 `/processed_distance` 低于报警阈值后，Node C 会异步请求报警服务。

服务端无响应达到 `service_timeout` 后，Node C 应输出：

```text
[WARN] [node_c_alarm]: 报警服务超时
```

Node C 在等待服务响应期间不会阻塞或崩溃。