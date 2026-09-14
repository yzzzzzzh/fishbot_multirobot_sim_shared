import os
from math import ceil, cos, pi, sin, sqrt

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('gazebo_sim')

    declare_args = [
        DeclareLaunchArgument('robot', default_value='fishbot_v2_3d', description='Robot blueprint folder name.'),
        DeclareLaunchArgument('world_name', default_value='default', description='Gazebo world name to target.'),
        DeclareLaunchArgument('count', default_value='3', description='Number of robots to spawn.'),
        DeclareLaunchArgument('name_prefix', default_value='bot', description='Prefix for spawned robot names.'),
        DeclareLaunchArgument('start_index', default_value='1', description='Starting index appended to the name prefix.'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('pattern', default_value='matrix', description='Spawn pattern: line, matrix, or circle.'),
        DeclareLaunchArgument('spacing', default_value='1', description='Step size (line/matrix) or radius (circle).'),
        DeclareLaunchArgument('spawn_delay', default_value='5.0', description='Seconds to wait before issuing spawn requests.'),
    ]

    def launch_setup(context, *args, **kwargs):
        robot = LaunchConfiguration('robot').perform(context)
        count = int(LaunchConfiguration('count').perform(context))
        world_name = LaunchConfiguration('world_name').perform(context)
        name_prefix = LaunchConfiguration('name_prefix').perform(context)
        start_index = int(LaunchConfiguration('start_index').perform(context))
        base_x = float(LaunchConfiguration('x').perform(context))
        base_y = float(LaunchConfiguration('y').perform(context))
        base_z = float(LaunchConfiguration('z').perform(context))
        pattern = LaunchConfiguration('pattern').perform(context).lower()
        spacing = float(LaunchConfiguration('spacing').perform(context))
        spawn_delay = float(LaunchConfiguration('spawn_delay').perform(context))

        xacro_path = os.path.join(pkg_share, 'robots', robot, 'urdf', f'{robot}.xacro')
        config_path = os.path.join(pkg_share, 'robots', robot, 'config', f'{robot}.yaml')

        rsp_nodes = []
        spawn_nodes = []
        bridge_nodes = []

        def pose_for_index(idx: int) -> tuple[float, float, float, float]:
            """Compute spawn pose per index for the selected pattern."""
            if pattern == 'circle':
                angle = (2 * pi * idx / max(count, 1))
                x = base_x + spacing * cos(angle)
                y = base_y + spacing * sin(angle)
                yaw = angle  # face outward
            elif pattern in ('matrix', 'grid'):
                cols = max(1, ceil(sqrt(count)))
                rows = ceil(count / cols)
                row = idx // cols
                col = idx % cols
                x = base_x + spacing * (col - (cols - 1) / 2)
                y = base_y + spacing * (row - (rows - 1) / 2)
                yaw = 0.0
            else:  # line
                x = base_x + spacing * idx
                y = base_y
                yaw = 0.0
            return x, y, base_z, yaw

        tf_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='tf_bridge',
            output='screen',
            arguments=[
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            ],
        )
        bridge_nodes.append(tf_bridge)

        for i in range(count):
            name = f'{name_prefix}{start_index + i}'
            robot_description = Command([ 'xacro ', xacro_path,
                ' robot_namespace:=', name,
                ' config_file:=', config_path,
            ])

            rsp_nodes.append(
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    namespace=name,
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'frame_prefix': f'{name}/',
                        'robot_description': robot_description,
                    }],
                )
            )

            spawn_x, spawn_y, spawn_z, spawn_yaw = pose_for_index(i)

            spawn_nodes.append(
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name=f'{name}_spawner',
                    output='screen',
                    arguments=[
                        '--world', world_name,
                        '--name', name,
                        '--allow_renaming=false',
                        '--string', robot_description,
                        '--x', str(spawn_x),
                        '--y', str(spawn_y),
                        '--z', str(spawn_z),
                        '--Y', str(spawn_yaw),
                    ],
                )
            )

            bridge_nodes.append(
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name=f'{name}_bridge',
                    output='screen',
                    arguments=[
                        f'/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                        f'/{name}/wheel_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                        f'/{name}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                        f'/{name}/lidar_points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                    ],
                )
            )

        if not rsp_nodes:
            return []

        spawn_after_delay = TimerAction(period=spawn_delay, actions=spawn_nodes)
        return rsp_nodes + bridge_nodes + [spawn_after_delay]

    ld = LaunchDescription()
    for action in declare_args:
        ld.add_action(action)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
