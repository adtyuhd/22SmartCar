#!/usr/bin/env python3

"""
M3-3 一键自主导航启动文件

复用：
    M3-1：smart_car_description 阿克曼小车
    M3-2：sim_m3_2.launch.py、scene.world、map.yaml

启动：
    1. Gazebo + 阿克曼车 + 雷达 + 里程计
    2. Map Server + AMCL
    3. Nav2 规划器、控制器、行为树
    4. Lifecycle Manager
    5. RViz2

不使用 Gazebo 真值作为定位输入。
"""

import os
from pathlib import Path

from ament_index_python.packages import (
    get_package_share_directory,
)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from nav2_common.launch import RewrittenYaml
from launch_ros.descriptions import ParameterFile


def generate_launch_description():

    # ==================================================
    # 1. 找到 M3-1、M3-2、M3-3 的实际文件
    # ==================================================

    launch_file = Path(__file__).resolve()
    m3_dir = launch_file.parents[2]

    # 复用 M3-2 的仿真启动文件
    sim_launch = (
        m3_dir
        / "M3-2"
        / "launch"
        / "sim_m3_2.launch.py"
    )

    # 只能使用 M3-2 的地图
    default_map = (
        m3_dir
        / "M3-2"
        / "maps"
        / "map.yaml"
    )

    # 获取 M3-3 已安装功能包的位置
    package_share = Path(
        get_package_share_directory("m3_3_navigation")
    )

    nav2_params = (
        package_share
        / "config"
        / "nav2_params.yaml"
    )

    amcl_params = (
        package_share
        / "config"
        / "amcl_params.yaml"
    )

    behavior_tree = (
        package_share
        / "config"
        / "navigate_to_pose.xml"
    )

    # 检查启动所需的文件
    required_files = [
        sim_launch,
        default_map,
        nav2_params,
        amcl_params,
        behavior_tree,
    ]

    for file_path in required_files:
        if not file_path.is_file():
            raise FileNotFoundError(
                f"M3-3 必需文件不存在: {file_path}"
            )

    # ==================================================
    # 2. 声明启动参数
    # ==================================================

    map_arg = DeclareLaunchArgument(
        "map",
        default_value=str(default_map),
        description="M3-2 map.yaml 的完整路径",
    )

    sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="是否使用 Gazebo 仿真时间",
    )

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="是否同时启动 RViz2",
    )

    map_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("rviz")

    sim_time_bool = ParameterValue(
        use_sim_time,
        value_type=bool,
    )

    # ==================================================
    # 3. 配置 Nav2 参数文件
    # ==================================================

    configured_nav2_params = ParameterFile(
        RewrittenYaml(
            source_file=str(nav2_params),
            param_rewrites={
                "use_sim_time": use_sim_time,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    # ==================================================
    # 4. 复用 M3-2 的 Gazebo 仿真
    # ==================================================

    gazebo_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(sim_launch)
        )
    )

    # ==================================================
    # 5. Map Server 地图服务器
    # ==================================================

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            configured_nav2_params,
            {
                "yaml_filename": map_file,
                "use_sim_time": sim_time_bool,
            },
        ],
    )

    # ==================================================
    # 6. AMCL 定位
    # ==================================================

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            str(amcl_params),
            {
                "use_sim_time": sim_time_bool,
            },
        ],
    )

    # ==================================================
    # 7. Nav2 全局规划器
    # ==================================================

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[configured_nav2_params],
    )

    # ==================================================
    # 8. Nav2 局部控制器
    # ==================================================

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[configured_nav2_params],

        # MPPI 的输出先送给速度平滑器
        remappings=[
            ("cmd_vel", "cmd_vel_nav"),
        ],
    )

    # ==================================================
    # 9. 路径平滑服务
    # ==================================================

    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[configured_nav2_params],
    )

    # ==================================================
    # 10. 恢复行为服务
    # ==================================================

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[configured_nav2_params],
    )

    # ==================================================
    # 11. 行为树导航器
    # ==================================================

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            configured_nav2_params,
            {
                # 使用自己的阿克曼行为树
                # 避免默认的 Spin / BackUp 恢复行为
                "default_nav_to_pose_bt_xml":
                    str(behavior_tree),
            },
        ],
    )

    # ==================================================
    # 12. 速度平滑器
    # ==================================================

    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[configured_nav2_params],

        remappings=[
            # 接收 MPPI 的速度指令
            ("cmd_vel", "cmd_vel_nav"),

            # 输出到 M3-1 阿克曼控制器的 cmd_vel
            ("cmd_vel_smoothed", "cmd_vel"),
        ],
    )

    # ==================================================
    # 13. 路点跟随器
    # ==================================================

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[configured_nav2_params],
    )

    # ==================================================
    # 14. 生命周期管理器
    # ==================================================

    lifecycle_nodes = [
        "map_server",
        "amcl",
        "planner_server",
        "controller_server",
        "smoother_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "use_sim_time": sim_time_bool,
                "autostart": True,
                "node_names": lifecycle_nodes,
            },
        ],
    )

    # ==================================================
    # 15. 修复 RViz 的 Snap 图形环境冲突
    # ==================================================

    # os.environ 是当前进程的环境变量集合。
    # copy() 创建独立副本，不修改原来的启动环境。
    rviz_environment = os.environ.copy()

    # 从 RViz 的专用环境中排除已验证的冲突变量。
    # pop(..., None) 表示变量不存在时也不会报错。
    snap_graphics_variables = [
        "GTK_PATH",
        "GTK_EXE_PREFIX",
        "GIO_EXTRA_MODULES",
        "GIO_MODULE_DIR",
    ]

    for variable in snap_graphics_variables:
        rviz_environment.pop(variable, None)

    # ==================================================
    # 16. RViz2 可视化
    # ==================================================

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",

        condition=IfCondition(use_rviz),

        parameters=[
            {
                "use_sim_time": sim_time_bool,
            },
        ],

        # 关键修复：
        # 仅 RViz 使用过滤后的环境变量。
        # Gazebo、AMCL、Nav2 保持原来的运行环境。
        env=rviz_environment,
    )

    # ==================================================
    # 17. 组合完整的一键启动流程
    # ==================================================

    return LaunchDescription([
        map_arg,
        sim_time_arg,
        rviz_arg,

        gazebo_simulation,

        map_server,
        amcl,

        planner_server,
        controller_server,
        smoother_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        waypoint_follower,

        lifecycle_manager,
        rviz,
    ])