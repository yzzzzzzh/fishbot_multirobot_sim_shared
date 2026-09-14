#!/usr/bin/env python3
"""Publish a saved RACER/Swarm-LIO2 NPZ run for two RViz2 views."""

import os
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


PREFIX = os.environ.get(
    "REPLAY_PREFIX",
    "/workspace/artifacts/racer_6uav_official/"
    "short_v41_no_progress_guard/run",
)
WORLD = os.environ.get(
    "REPLAY_WORLD",
    "/workspace/src/gazebo_sim/worlds/drone_racer.world",
)
COLORS = (
    (0.12, 0.47, 0.71),
    (1.00, 0.50, 0.05),
    (0.17, 0.63, 0.17),
    (0.84, 0.15, 0.16),
    (0.58, 0.40, 0.74),
    (0.55, 0.34, 0.29),
)


def latched():
    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    qos.history = HistoryPolicy.KEEP_LAST
    return qos


def cloud_msg(points, frame, stamp):
    points = np.asarray(points, dtype=np.float32)
    values = np.zeros(
        len(points),
        dtype=[("x", "f4"), ("y", "f4"), ("z", "f4"), ("intensity", "f4")],
    )
    values["x"], values["y"], values["z"] = points.T
    values["intensity"] = points[:, 2]
    message = PointCloud2()
    message.header.frame_id = frame
    message.header.stamp = stamp
    message.height = 1
    message.width = len(points)
    message.fields = [
        PointField(
            name=name,
            offset=offset,
            datatype=PointField.FLOAT32,
            count=1,
        )
        for name, offset in (("x", 0), ("y", 4), ("z", 8), ("intensity", 12))
    ]
    message.point_step = 16
    message.row_step = 16 * len(points)
    message.data = values.tobytes()
    message.is_dense = True
    return message


def path_msg(points, frame, stamp):
    message = Path()
    message.header.frame_id = frame
    message.header.stamp = stamp
    for point in points[::3]:
        pose = PoseStamped()
        pose.header = message.header
        pose.pose.position.x = float(point[0])
        pose.pose.position.y = float(point[1])
        pose.pose.position.z = float(point[2])
        pose.pose.orientation.w = 1.0
        message.poses.append(pose)
    return message


def environment_markers(world, frame, stamp):
    root = ET.parse(world).getroot()
    result = MarkerArray()
    for marker_id, link in enumerate(
        root.findall(".//model[@name='racer_env']/link")
    ):
        size_text = link.findtext("collision/geometry/box/size")
        if not size_text:
            continue
        center = [float(value) for value in link.findtext(
            "pose", default="0 0 0 0 0 0"
        ).split()[:3]]
        size = [float(value) for value in size_text.split()[:3]]
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = stamp
        marker.ns = "gazebo_ground_truth"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = center
        marker.pose.orientation.w = 1.0
        marker.scale.x, marker.scale.y, marker.scale.z = size
        marker.color.r = 0.42
        marker.color.g = 0.49
        marker.color.b = 0.60
        marker.color.a = 0.72
        result.markers.append(marker)
    return result


class ReplayPublisher(Node):
    def __init__(self):
        super().__init__("racer_v41_dual_rviz_replay")
        data = np.load(PREFIX + ".npz", allow_pickle=False)
        robot_ids = [int(value) for value in data["robot_ids"]]
        voxels = np.asarray(data["voxel_xyz"], dtype=np.float32)
        owners = np.asarray(data["voxel_robot"], dtype=np.int16)
        qos = latched()
        stamp = self.get_clock().now().to_msg()

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.frame_id = "map"
        transform.child_frame_id = "replay"
        transform.header.stamp = stamp
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)

        self.publishers_and_messages = []
        for index, robot_id in enumerate(robot_ids):
            cloud = voxels[owners == robot_id]
            cloud_publisher = self.create_publisher(
                PointCloud2,
                f"/racer_replay/v41/bot{robot_id}/observed_cloud",
                qos,
            )
            path_publisher = self.create_publisher(
                Path,
                f"/racer_replay/v41/bot{robot_id}/truth_path",
                qos,
            )
            self.publishers_and_messages.extend(
                (
                    (cloud_publisher, cloud_msg(cloud, "map", stamp)),
                    (
                        path_publisher,
                        path_msg(data[f"truth_{robot_id}"], "map", stamp),
                    ),
                )
            )
            self.get_logger().info(
                f"bot{robot_id}: {len(cloud)} first-observed map voxels"
            )

        marker_publisher = self.create_publisher(
            MarkerArray,
            "/racer_replay/v41/environment_ground_truth",
            qos,
        )
        markers = environment_markers(WORLD, "map", stamp)
        self.publishers_and_messages.append((marker_publisher, markers))
        self.get_logger().info(
            f"environment: {len(markers.markers)} exact Gazebo box obstacles"
        )
        self.publish_all()
        self.timer = self.create_timer(2.0, self.publish_all)

    def publish_all(self):
        for publisher, message in self.publishers_and_messages:
            publisher.publish(message)


def main():
    rclpy.init()
    node = ReplayPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
