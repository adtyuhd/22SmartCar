import os

import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    # 找到 smart_car_description 的 share 目录
    package_share = get_package_share_directory(
        'smart_car_description'
    )

    # 世界文件
    world_file = os.path.join(
        package_share,
        'worlds',
        'square.world'
    )

    # Xacro 文件
    xacro_file = os.path.join(
        package_share,
        'urdf',
        'smart_car.urdf.xacro'
    )

    # Xacro -> URDF
    robot_description_config = xacro.process_file(
        xacro_file
    )

    robot_description = {
        'robot_description':
            robot_description_config.toxml()
    }

    # 启动 Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': world_file
        }.items()
    )

    # 发布机器人 TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {
                'use_sim_time': True
            }
        ]
    )

    # 把机器人放进 Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic',
            'robot_description',
            '-entity',
            'smart_car',
            '-x',
            '0.0',
            '-y',
            '0.0',
            '-z',
            '0.06'
        ]
    )

    # 启动 joint_state_broadcaster
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

    # 启动阿克曼控制器
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
        gazebo,
        robot_state_publisher,
        spawn_robot,
        joint_state_broadcaster_spawner,
        ackermann_controller_spawner
    ])