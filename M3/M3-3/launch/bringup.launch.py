
#!/usr/bin/env python3
"""M3-3: one-command Gazebo + localization + Nav2 + saved RViz view.

Reuses M3-1 smart_car_description and M3-2 scene/map.
Does not use /gazebo/model_states as a localization input.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # -- Locate existing project assets (do not modify M3-1 or M3-2) --
    m3_dir = Path(__file__).resolve().parents[2]
    sim_launch = m3_dir / "M3-2" / "launch" / "sim_m3_2.launch.py"
    default_map = m3_dir / "M3-2" / "maps" / "map.yaml"

    package_share = Path(get_package_share_directory("m3_3_navigation"))
    nav2_params = package_share / "config" / "nav2_params.yaml"
    amcl_params = package_share / "config" / "amcl_params.yaml"
    behavior_tree = package_share / "config" / "navigate_to_pose.xml"
    through_poses_tree = package_share / "config" / "navigate_through_poses.xml"
    rviz_config = package_share / "config" / "navigation.rviz"

    for path in (
        sim_launch,
        default_map,
        nav2_params,
        amcl_params,
        behavior_tree,
        through_poses_tree,
        rviz_config,
    ):
        if not path.is_file():
            raise FileNotFoundError(f"Missing M3-3 dependency: {path}")

    # -- Command-line options: map, use_sim_time, rviz --
    map_arg = DeclareLaunchArgument(
        "map", default_value=str(default_map),
        description="Absolute path to the M3-2 map.yaml",
    )
    sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use Gazebo simulation time",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true",
        description="Start RViz with the saved navigation.rviz view",
    )

    map_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("rviz")
    sim_time_bool = ParameterValue(use_sim_time, value_type=bool)

    # Rewrites temporary launch parameters, never edits the source YAML.
    configured_nav2_params = ParameterFile(
        RewrittenYaml(
            source_file=str(nav2_params),
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # -- Gazebo: directly reuse the existing M3-2 simulator launch --
    gazebo_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sim_launch))
    )

    # -- Map and localization --
    map_server = Node(
        package="nav2_map_server", executable="map_server",
        name="map_server", output="screen",
        parameters=[
            configured_nav2_params,
            {"yaml_filename": map_file, "use_sim_time": sim_time_bool},
        ],
    )
    amcl = Node(
        package="nav2_amcl", executable="amcl",
        name="amcl", output="screen",
        parameters=[str(amcl_params), {"use_sim_time": sim_time_bool}],
    )

    # -- Navigation servers --
    planner_server = Node(
        package="nav2_planner", executable="planner_server",
        name="planner_server", output="screen",
        parameters=[configured_nav2_params],
    )
    controller_server = Node(
        package="nav2_controller", executable="controller_server",
        name="controller_server", output="screen",
        parameters=[configured_nav2_params],
        remappings=[("cmd_vel", "cmd_vel_nav")],
    )
    smoother_server = Node(
        package="nav2_smoother", executable="smoother_server",
        name="smoother_server", output="screen",
        parameters=[configured_nav2_params],
    )
    behavior_server = Node(
        package="nav2_behaviors", executable="behavior_server",
        name="behavior_server", output="screen",
        parameters=[configured_nav2_params],
    )
    bt_navigator = Node(
        package="nav2_bt_navigator", executable="bt_navigator",
        name="bt_navigator", output="screen",
        parameters=[
            configured_nav2_params,
            {
                "default_nav_to_pose_bt_xml": str(behavior_tree),
                "default_nav_through_poses_bt_xml": str(through_poses_tree),
            },
        ],
    )
    velocity_smoother = Node(
        package="nav2_velocity_smoother", executable="velocity_smoother",
        name="velocity_smoother", output="screen",
        parameters=[configured_nav2_params],
        remappings=[
            ("cmd_vel", "cmd_vel_nav"),
            ("cmd_vel_smoothed", "cmd_vel"),
        ],
    )
    waypoint_follower = Node(
        package="nav2_waypoint_follower", executable="waypoint_follower",
        name="waypoint_follower", output="screen",
        parameters=[configured_nav2_params],
    )

    # Lifecycle Manager configures and activates Nav2 components.
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_navigation", output="screen",
        parameters=[{
            "use_sim_time": sim_time_bool,
            "autostart": True,
            "node_names": [
                "map_server", "amcl", "planner_server", "controller_server",
                "smoother_server", "behavior_server", "bt_navigator",
                "waypoint_follower", "velocity_smoother",
            ],
        }],
    )

    # -- RViz only: remove Snap/VS Code GTK variables that caused GLIBC crash --
    # Copying avoids modifying the Gazebo/Nav2 child processes' environment.
    rviz_environment = os.environ.copy()
    for variable in (
        "GTK_PATH", "GTK_EXE_PREFIX", "GIO_EXTRA_MODULES", "GIO_MODULE_DIR"
    ):
        rviz_environment.pop(variable, None)

    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        output="screen", condition=IfCondition(use_rviz),
        parameters=[{"use_sim_time": sim_time_bool}],
        arguments=["-d", str(rviz_config)],  # Restore displays on every launch.
        env=rviz_environment,
    )

    return LaunchDescription([
        map_arg, sim_time_arg, rviz_arg,
        gazebo_simulation,
        map_server, amcl,
        planner_server, controller_server,
        smoother_server, behavior_server, bt_navigator,
        velocity_smoother, waypoint_follower,
        lifecycle_manager, rviz,
    ])