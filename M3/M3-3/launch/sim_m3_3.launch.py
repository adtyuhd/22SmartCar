#!/usr/bin/env python3
"""M3-3 simulation: M3-2 world + M3-1 robot geometry, fixed wheel ordering.

The only override is the controller YAML stored in M3-3/config.
This does not edit M3-1, M3-2, or publish Gazebo truth into Nav2.
"""
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    m3_dir = Path(__file__).resolve().parents[2]
    world_file = m3_dir / "M3-2" / "worlds" / "scene.world"
    robot_share = Path(get_package_share_directory("smart_car_description"))
    xacro_file = robot_share / "urdf" / "smart_car.urdf.xacro"
    local_share = Path(get_package_share_directory("m3_3_navigation"))
    controller_yaml = local_share / "config" / "ackermann_controllers.yaml"

    for path in (world_file, xacro_file, controller_yaml):
        if not path.is_file():
            raise FileNotFoundError(f"M3-3 simulation dependency missing: {path}")

    robot_xml = xacro.process_file(str(xacro_file))
    # Patch only the gazebo_ros2_control plugin parameter path in memory.
    # Wheel geometry, LiDAR, frames, and world stay identical to M3-1/M3-2.
    replaced = 0
    for plugin in robot_xml.getElementsByTagName("plugin"):
        if "gazebo_ros2_control" not in plugin.getAttribute("filename"):
            continue
        for params in plugin.getElementsByTagName("parameters"):
            if params.firstChild is not None:
                params.firstChild.nodeValue = str(controller_yaml)
            else:
                params.appendChild(robot_xml.createTextNode(str(controller_yaml)))
            replaced += 1
    if replaced != 1:
        raise RuntimeError(f"Expected 1 Gazebo controller configuration; got {replaced}")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory("gazebo_ros")) /
                "launch" / "gazebo.launch.py")
        ),
        launch_arguments={"world": str(world_file)}.items(),
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_xml.toxml()},
            {"use_sim_time": True},
        ],
    )
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-entity", "smart_car",
            "-x", "0.0", "-y", "-5.05", "-z", "0.06", "-Y", "0.0",
        ],
    )
    joint_states = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    ackermann_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackermann_steering_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    return LaunchDescription([
        gazebo, robot_state_publisher, spawn_robot, joint_states, ackermann_controller,
    ])
