#!/usr/bin/env python3
from __future__ import annotations

from typing import Dict, List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


def quat_to_rot_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = x*x + y*y + z*z + w*w
    if n < 1e-12: return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x*x*s, y*y*s, z*z*s
    xy, xz, yz = x*y*s, x*z*s, y*z*s
    wx, wy, wz = w*x*s, w*y*s, w*z*s
    return np.array([
        [1.0 - (yy + zz),       xy - wz,       xz + wy],
        [      xy + wz, 1.0 - (xx + zz),       yz - wx],
        [      xz - wy,       yz + wx, 1.0 - (xx + yy)],
    ], dtype=float)


class MapFusionNode(Node):
    def __init__(self) -> None:
        super().__init__('global_map_node')

        # use_sim_time may already be declared by launch parameters; ignore if so
        try:
            self.declare_parameter('use_sim_time', True)
        except ParameterAlreadyDeclaredException:
            pass
        self.declare_parameter('uav_ids', [1, 2, 3, 4])

        # TF naming to match your TF-only publisher
        self.declare_parameter('origin_frame_id', 'map_origin')   # parent
        self.declare_parameter('robot_prefix', 'bot')             # botX
        self.declare_parameter('robot_frame_suffix', 'world')     # botX/world
        self.declare_parameter('pointcloud_topic', '/global_downsampled_map')

        self.uav_ids: List[int] = [int(x) for x in self.get_parameter('uav_ids').value]
        self.origin_frame_id: str = str(self.get_parameter('origin_frame_id').value).lstrip('/')
        self.robot_prefix: str = str(self.get_parameter('robot_prefix').value)
        self.robot_frame_suffix: str = str(self.get_parameter('robot_frame_suffix').value).lstrip('/')
        self.pointcloud_topic: str = str(self.get_parameter('pointcloud_topic').value)

        qos_map = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        qos_default = QoSProfile(depth=10)

        self.local_maps: Dict[int, Tuple[np.ndarray, float]] = {}

        # TF buffer/listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriptions to each bot's downsampled map
        self.map_subs = []
        for uid in self.uav_ids:
            topic = f'/bot{uid}/downsampled_map'
            self.map_subs.append(
                self.create_subscription(
                    PointCloud2,
                    topic,
                    lambda msg, uid=uid: self._map_callback(uid, msg),
                    qos_map,
                )
            )

        self.global_pub = self.create_publisher(PointCloud2, self.pointcloud_topic, qos_default)
        self.create_timer(0.5, self._publish_global_map)

        self.get_logger().info(
            f'global_map_node TF-fusion started. uav_ids={self.uav_ids}, '
            f'global_frame={self.origin_frame_id}, pointcloud_topic={self.pointcloud_topic}, '
            f'tf children=/{self.robot_prefix}X/{self.robot_frame_suffix}'
        )

    def _map_callback(self, uid: int, msg: PointCloud2) -> None:
        pts = self._extract_xyz(msg)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.local_maps[uid] = (pts, stamp)

    def _extract_xyz(self, msg: PointCloud2) -> np.ndarray:
        """Return Nx3 float32 array of xyz points; be robust to malformed input."""
        try:
            raw = np.fromiter(
                pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True),
                dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)],
            )
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().warn(f"Failed to parse PointCloud2: {exc}")
            return np.empty((0, 3), dtype=np.float32)

        if raw.size == 0:
            return np.empty((0, 3), dtype=np.float32)

        pts = np.stack((raw['x'], raw['y'], raw['z']), axis=1).astype(np.float32, copy=False)
        return pts

    def _child_frame(self, uid: int) -> str:
        return f'{self.robot_prefix}{uid}/{self.robot_frame_suffix}'.lstrip('/')

    def _lookup_extrinsic(self, uid: int) -> Tuple[np.ndarray, np.ndarray] | None:
        child = self._child_frame(uid)
        try:
            # latest available transform
            tf = self.tf_buffer.lookup_transform(
                self.origin_frame_id,  # target (parent)
                child,                 # source (child)
                rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

        t = tf.transform.translation
        q = tf.transform.rotation

        R = quat_to_rot_matrix(q.x, q.y, q.z, q.w)
        trans = np.array([t.x, t.y, t.z], dtype=float)
        return R, trans

    def _publish_global_map(self) -> None:
        if self.global_pub.get_subscription_count() == 0:
            return

        merged_pts: List[List[float]] = []
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        for uid in self.uav_ids:
            map_entry = self.local_maps.get(uid)
            if map_entry is None:
                continue
            pts, _ = map_entry
            if pts.size == 0:
                continue

            extr = self._lookup_extrinsic(uid)
            if extr is None:
                continue
            R, t = extr

            base_int = float(100 * (uid - 1))
            transformed = (R @ pts.T).T + t
            intensities = np.full((transformed.shape[0], 1), base_int, dtype=np.float32)
            combined = np.hstack((transformed.astype(np.float32), intensities))
            merged_pts.extend(combined.tolist())

        if not merged_pts:
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.origin_frame_id
        cloud = pc2.create_cloud(header, fields, merged_pts)
        self.global_pub.publish(cloud)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
