cmake_minimum_required(VERSION 3.8)
project(sensor_interfaces)

# ROS2 的 CMake 构建系统
find_package(ament_cmake REQUIRED)

# 用来根据 .msg / .srv 生成接口代码
find_package(rosidl_default_generators REQUIRED)

# 因为 SensorData.msg 使用了 builtin_interfaces/Time
find_package(builtin_interfaces REQUIRED)

# 告诉 ROS2：需要生成哪些接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorData.msg"
  "srv/TriggerAlarm.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()