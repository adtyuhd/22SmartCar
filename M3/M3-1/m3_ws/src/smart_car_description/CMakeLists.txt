cmake_minimum_required(VERSION 3.8)
project(smart_car_description)

# 查找 ROS 2 的 ament_cmake 构建系统
find_package(ament_cmake REQUIRED)

# 安装本包的资源文件
install(
  DIRECTORY
    urdf
    config
    launch
    worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()