
import os

import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    # ==========================================
    # 1. 获取 M3-2 自身的路径
    # ==========================================

    # 当前 Python 文件所在目录：M3-2/launch
    launch_dir = os.path.dirname(
        os.path.abspath(__file__)
    )

    # launch 的上一级目录：M3-2
    m3_2_dir = os.path.dirname(launch_dir)

    # M3-2 自己的 Gazebo 世界
    world_file = os.path.join(
        m3_2_dir,
        "worlds",
        "scene.world"
    )

    # ==========================================
    # 2. 获取已有小车功能包
    # ==========================================

    # 复用 M3-1 已经编译的小车模型
    package_share = get_package_share_directory(
        "smart_car_description"
    )

    xacro_file = os.path.join(
        package_share,
        "urdf",
        "smart_car.urdf.xacro"
    )

    # ==========================================
    # 3. 将 Xacro 转换为 URDF
    # ==========================================

    robot_description_config = xacro.process_file(
        xacro_file
    )

    robot_description = {
        "robot_description":
            robot_description_config.toxml()
    }

    # ==========================================
    # 4. 启动 Gazebo，加载 M3-2 世界
    # ==========================================

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            )
        ),
        launch_arguments={
            "world": world_file
        }.items()
    )

    # ==========================================
    # 5. 发布机器人 TF
    # ==========================================

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {
                "use_sim_time": True
            }
        ]
    )

    # ==========================================
    # 6. 将小车放入 Gazebo
    # ==========================================

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",

            "-entity",
            "smart_car",

            # 南侧走廊中心
            "-x",
            "0.0",

            "-y",
            "-5.05",

            # 与原模型一致的出生高度
            "-z",
            "0.06",

            # 车头朝向 X 轴正方向
            "-Y",
            "0.0"
        ]
    )

    # ==========================================
    # 7. 启动关节状态广播器
    # ==========================================

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    # ==========================================
    # 8. 启动阿克曼控制器
    # ==========================================

    ackermann_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ackermann_steering_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    # ==========================================
    # 9. 组合所有启动动作
    # ==========================================

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        joint_state_broadcaster_spawner,
        ackermann_controller_spawner
    ])