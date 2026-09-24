from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os
import xacro


def generate_launch_description():

    package_name = 'smart_car_description'

    package_share = get_package_share_directory(package_name)

    # 1. 找到小车的 Xacro
    xacro_file = os.path.join(
        package_share,
        'urdf',
        'smart_car.urdf.xacro'
    )

    # 2. Xacro -> robot_description
    robot_description = xacro.process_file(xacro_file).toxml()

    # 3. 启动 robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description}
        ]
    )

    # 4. 找到我们自己的 Gazebo world
    world_file = os.path.join(
        package_share,
        'worlds',
        'square.world'
    )

    # 5. 找到 gazebo_ros 自带的 gazebo.launch.py
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_ros_share,
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': world_file
        }.items()
    )
    # 6. 把 robot_description 中的小车生成到 Gazebo
    spawn_car = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'smart_car',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.15'
        ],
        output='screen'
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    ackermann_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'ackermann_steering_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )
    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_car,
        joint_state_broadcaster_spawner,
        ackermann_controller_spawner
    ])
