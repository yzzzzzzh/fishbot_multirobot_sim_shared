import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _launch_setup(context, *args, **kwargs):
    root_id = int(LaunchConfiguration("root_id").perform(context))
    uav_ids_str = LaunchConfiguration("uav_ids").perform(context)
    use_sim_time_raw = LaunchConfiguration("use_sim_time").perform(context)

    use_sim_time = str(use_sim_time_raw).lower() in ("true", "1", "yes", "on")
    uav_ids = [int(x) for x in uav_ids_str.replace(" ", "").split(",") if x]

    pkg_share = get_package_share_directory("map_fusion")
    default_config = os.path.join(pkg_share, "config", "nvblox_esdf.yaml")
    nvblox_config = LaunchConfiguration("nvblox_config").perform(context) or default_config

    # 1) Map fusion node (your Python node)
    map_fusion_node = Node(
        package="map_fusion",
        executable="global_map_node.py",
        name="global_map_node",
        output="screen",
        parameters=[{
            "root_id": root_id,
            "uav_ids": uav_ids,
            "use_sim_time": use_sim_time,
            "pointcloud_topic": "/global_downsampled_map",
            "origin_frame_id": "map_origin",
        }],
    )

    global_pose_node = Node(
        package="map_fusion",
        executable="global_pose_publisher.py",
        name="global_pose_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "global_frame": "map_origin",   # map_origin -> bot<i>/base_link
            "bot_prefix": "bot",            # matches bot1/..., bot2/...
            "publish_rate_hz": 10.0,
            "pose_topic_suffix": "global_pose",
        }],
    )

    # # 2) NVBlox ESDF server node
    # # Choose package name according to what you have installed:
    # #   - "isaac_ros_nvblox"  (Isaac ROS)
    # #   - "nvblox_ros"        (older NVLabs ROS2 wrapper)
    # nvblox_pkg = "nvblox_ros"

    # esdf_node = Node(
    #     package=nvblox_pkg,
    #     executable="nvblox_node",
    #     name="nvblox_node",
    #     output="screen",
    #     parameters=[
    #         nvblox_config,
    #         {
    #             "use_sim_time": use_sim_time,
    #             "global_frame": "map_origin",
    #         },
    #     ],
    #     remappings=[
    #         ("pointcloud", "/global_downsampled_map"),
    #     ],
    # )

    return [map_fusion_node, global_pose_node]
    # return [map_fusion_node, esdf_node]


def generate_launch_description():
    pkg_share = get_package_share_directory("map_fusion")
    default_nvblox_config = os.path.join(pkg_share, "config", "nvblox_esdf.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("root_id", default_value="1", description="Root robot ID"),
        DeclareLaunchArgument("uav_ids", default_value="1,2,3,4", description="Comma-separated UAV IDs"),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time"),
        DeclareLaunchArgument("nvblox_config",default_value=default_nvblox_config, description="Path to NVBlox config YAML"),
        OpaqueFunction(function=_launch_setup),
    ])
