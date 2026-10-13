cmake_minimum_required(VERSION 3.8)

project(m3_3_navigation)

# 查找 ROS 2 的 ament_cmake 构建系统
find_package(ament_cmake REQUIRED)

# 安装 launch 和 config 目录
# 这样 ros2 launch 才能从功能包中找到启动文件
install(
  DIRECTORY
    launch
    config
  DESTINATION
    share/${PROJECT_NAME}
)

# 注册 ROS 2 功能包
ament_package()