# M1-2 Launch 编排

本项目基于 M1-1 的三个 ROS 2 节点，实现 Python Launch 一键启动、YAML 参数管理、多 namespace 实例隔离、启动依赖控制和参数文件错误处理。

## 节点

系统包含三个节点：

- `node_a_sensor`：发布 `sensor_data`，并提供 `trigger_alarm` 服务
- `node_b_filter`：订阅 `sensor_data`，进行滤波后发布 `processed_distance`
- `node_c_alarm`：订阅 `processed_distance`，低于报警阈值时异步调用 `trigger_alarm`

所有 Topic 和 Service 均使用相对名称，因此可以通过 namespace 自动隔离。

## 编译

```bash
cd ~/22SmartCar/M1/M1-2
colcon build
source install/setup.bash
```

## Launch 参数

`bringup.launch.py` 支持以下参数：

- `namespace`：节点所在 namespace，默认 `robot1`
- `params_file`：YAML 参数文件路径，默认空
- `use_sim_time`：是否使用仿真时间，默认 `false`

## 启动 robot1

```bash
cd ~/22SmartCar/M1/M1-2
source install/setup.bash

ros2 launch sensor_chain bringup.launch.py \
  namespace:=robot1 \
  params_file:=$HOME/22SmartCar/M1/M1-2/params_robot1.yaml
```

`params_robot1.yaml` 中：

```yaml
/**/node_b_filter:
  ros__parameters:
    alpha: 0.3

/**/node_c_alarm:
  ros__parameters:
    alarm_threshold: 0.3
    service_timeout: 1.0
```

验证参数：

```bash
ros2 param get /robot1/node_b_filter alpha
ros2 param get /robot1/node_c_alarm alarm_threshold
```

预期：

```text
Double value is: 0.3
Double value is: 0.3
```

## 同时启动 robot1 和 robot2

终端 1：

```bash
cd ~/22SmartCar/M1/M1-2
source install/setup.bash

ros2 launch sensor_chain bringup.launch.py \
  namespace:=robot1 \
  params_file:=$HOME/22SmartCar/M1/M1-2/params_robot1.yaml
```

终端 2：

```bash
cd ~/22SmartCar/M1/M1-2
source install/setup.bash

ros2 launch sensor_chain bringup.launch.py \
  namespace:=robot2 \
  params_file:=$HOME/22SmartCar/M1/M1-2/params_robot2.yaml
```

## 验证节点隔离

```bash
ros2 node list
```

预期可以看到：

```text
/robot1/node_a_sensor
/robot1/node_b_filter
/robot1/node_c_alarm
/robot2/node_a_sensor
/robot2/node_b_filter
/robot2/node_c_alarm
```

## 验证参数隔离

```bash
ros2 param get /robot1/node_c_alarm alarm_threshold
ros2 param get /robot2/node_c_alarm alarm_threshold
```

预期：

```text
Double value is: 0.3
Double value is: 0.5
```

其中：

- `robot1` 报警阈值为 `0.3 m`
- `robot2` 报警阈值为 `0.5 m`

## 验证 Topic 隔离

```bash
ros2 topic list
```

可以看到两套独立 Topic：

```text
/robot1/sensor_data
/robot1/processed_distance
/robot2/sensor_data
/robot2/processed_distance
```

也可以分别查看两组数据：

```bash
ros2 topic echo /robot1/processed_distance
```

```bash
ros2 topic echo /robot2/processed_distance
```

## 启动依赖

`bringup.launch.py` 使用：

```python
RegisterEventHandler(
    OnProcessStart(
        target_action=node_b,
        on_start=[node_c]
    )
)
```

Node A 和 Node B 首先启动。

Node C 由 `OnProcessStart` 事件触发，在 Node B 进程启动后再启动，从而实现启动顺序控制。

## 参数文件错误处理

如果传入不存在的 YAML 文件：

```bash
ros2 launch sensor_chain bringup.launch.py params_file:=/nope.yaml
```

Launch 会输出清晰的错误信息并退出，不产生 Python traceback。

如果没有提供 `params_file`：

```bash
ros2 launch sensor_chain bringup.launch.py
```

Launch 同样会提示需要提供参数文件并退出。

## use_sim_time

默认：

```text
use_sim_time:=false
```

也可以通过 Launch 参数修改：

```bash
ros2 launch sensor_chain bringup.launch.py \
  namespace:=robot1 \
  params_file:=$HOME/22SmartCar/M1/M1-2/params_robot1.yaml \
  use_sim_time:=true
```

## 文件结构

```text
M1/M1-2/
├── params_robot1.yaml
├── params_robot2.yaml
├── README.md
└── src/
    ├── sensor_interfaces/
    └── sensor_chain/
        ├── launch/
        │   └── bringup.launch.py
        ├── sensor_chain/
        │   ├── node_a_sensor.py
        │   ├── node_b_filter.py
        │   └── node_c_alarm.py
        ├── package.xml
        └── setup.py
```