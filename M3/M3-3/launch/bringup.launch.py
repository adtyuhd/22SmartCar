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

    # resolve() 跟随 --symlink-install 创建的符号链接
    # 从源码 launch 文件向上两级得到 M3 目录
    launch_file = Path(__file__).resolve()
    m3_dir = launch_file.parents[2]

    # M3-2 已经完成的仿真启动文件
    sim_launch = (
        m3_dir
        / "M3-2"
        / "launch"
        / "sim_m3_2.launch.py"
    )

    # 只使用 M3-2 已经建立好的地图
    default_map = (
        m3_dir
        / "M3-2"
        / "maps"
        / "map.yaml"
    )

    # M3-3 已安装的 ROS 2 功能包目录
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

    # 预先检查必需文件，避免路径错误时启动半套系统
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
    # 2. 声明命令行启动参数
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

    # 将 launch 的字符串 true/false 转成 ROS bool 参数
    sim_time_bool = ParameterValue(
        use_sim_time,
        value_type=bool,
    )

    # ==================================================
    # 3. 配置 Nav2 参数文件
    # ==================================================

    # 保留原始 YAML，不修改磁盘上的 nav2_params.yaml
    # 启动时统一替换其中的 use_sim_time
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
    # 5. 地图服务器
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
    # 7. Nav2 全局规划器与局部控制器
    # ==================================================

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[configured_nav2_params],
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[configured_nav2_params],

        # MPPI 输出到速度平滑器，而非直接驱动车轮
        remappings=[
            ("cmd_vel", "cmd_vel_nav"),
        ],
    )

    # ==================================================
    # 8. 路径平滑与恢复行为
    # ==================================================

    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[configured_nav2_params],
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[configured_nav2_params],
    )

    # ==================================================
    # 9. 行为树导航器
    # ==================================================

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            configured_nav2_params,

            # 明确加载 M3-3 的阿克曼行为树
            # 避免使用含 Spin / BackUp 的默认行为树
            {
                "default_nav_to_pose_bt_xml":
                    str(behavior_tree),
            },
        ],
    )

    # ==================================================
    # 10. 速度平滑器
    # ==================================================

    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[configured_nav2_params],

        remappings=[
            # 接收 MPPI 的原始速度
            ("cmd_vel", "cmd_vel_nav"),

            # 平滑后送到 M3-1 阿克曼控制器
            ("cmd_vel_smoothed", "cmd_vel"),
        ],
    )

    # ==================================================
    # 11. 路点跟随器
    # ==================================================

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[configured_nav2_params],
    )

    # ==================================================
    # 12. 生命周期管理器
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
    # 13. RViz2 可视化
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
    )

    # ==================================================
    # 14. 返回完整启动描述
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