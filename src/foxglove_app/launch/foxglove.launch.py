from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _configure_nodes(context, *args, **kwargs):
    port = LaunchConfiguration('port').perform(context)
    address = LaunchConfiguration('address').perform(context)
    layout_path = LaunchConfiguration('layout').perform(context)

    bridge_params = {
        'port': int(port),
        'address': address,
        'use_compression': False,
    }

    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[bridge_params],
    )

    info = LogInfo(msg=f"[foxglove] Load layout in Studio: {layout_path}")
    return [info, foxglove]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8765', description='WebSocket port for foxglove_bridge.'),
        DeclareLaunchArgument('address', default_value='0.0.0.0', description='Bind address for foxglove_bridge.'),
        DeclareLaunchArgument(
            'layout',
            default_value=PathJoinSubstitution([
                FindPackageShare('fishbot_foxglove'),
                'layouts',
                'multi_robot_navigation.json',
            ]),
            description='Default Foxglove layout to load from Studio.',
        ),
        OpaqueFunction(function=_configure_nodes),
    ])
