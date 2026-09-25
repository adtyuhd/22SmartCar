import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogError,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessStart
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')

    if not params_file:
        return [
            LogError(
                msg='params_file is required. '
                    'Please provide params_file:=/path/to/params.yaml'
            ),
            EmitEvent(
                event=Shutdown(
                    reason='Missing parameter file'
                )
            ),
        ]

    params_file = os.path.abspath(
        os.path.expanduser(params_file)
    )

    if not os.path.isfile(params_file):
        return [
            LogError(
                msg=f'Parameter file does not exist: {params_file}'
            ),
            EmitEvent(
                event=Shutdown(
                    reason='Parameter file not found'
                )
            ),
        ]

    node_a = Node(
        package='sensor_chain',
        executable='node_a_sensor',
        name='node_a_sensor',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    node_b = Node(
        package='sensor_chain',
        executable='node_b_filter',
        name='node_b_filter',
        namespace=namespace,
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    node_c = Node(
        package='sensor_chain',
        executable='node_c_alarm',
        name='node_c_alarm',
        namespace=namespace,
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    start_node_c = RegisterEventHandler(
        OnProcessStart(
            target_action=node_b,
            on_start=[
                node_c
            ]
        )
    )

    return [
        node_a,
        node_b,
        start_node_c,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='robot1'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=''
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'
        ),
        OpaqueFunction(function=launch_setup),
    ])