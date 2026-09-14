#!/usr/bin/env python3
from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.exceptions import ParameterAlreadyDeclaredException


@dataclass
class BotPub:
    bot_id: int
    base_frame: str
    pub: any


class MultiBotGlobalPose(Node):
    def __init__(self) -> None:
        super().__init__("global_pose_publisher")

        try:
            self.declare_parameter('use_sim_time', True)
        except ParameterAlreadyDeclaredException:
            pass
        self.declare_parameter("global_frame", "map_origin")
        self.declare_parameter("bot_prefix", "bot")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("pose_topic_suffix", "global_pose")

        self.global_frame: str = str(self.get_parameter("global_frame").value).lstrip("/")
        self.bot_prefix: str = str(self.get_parameter("bot_prefix").value)
        self.pose_topic_suffix: str = str(self.get_parameter("pose_topic_suffix").value)
        rate_hz: float = float(self.get_parameter("publish_rate_hz").value)

        # Match a TF frame path segment like "<bot_prefix><digits>" (e.g., "bot12" in "map_origin/bot12/base_link")
        # and capture the numeric id; require segment boundaries (start or '/' before, '/' or end after) to avoid
        # accidental matches inside longer names.
        self._bot_re = re.compile(rf"(?:^|/){re.escape(self.bot_prefix)}(\d+)(?:/|$)")
        self._bots: Dict[int, BotPub] = {}

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        qos_tf = QoSProfile(
            depth=100,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        qos_tf_static = QoSProfile(
            depth=100,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(TFMessage, "/tf", self._on_tf_msg, qos_tf)
        self.create_subscription(TFMessage, "/tf_static", self._on_tf_msg, qos_tf_static)

        self._timer = self.create_timer(1.0 / max(rate_hz, 0.1), self._tick)

        self.get_logger().info(
            f"Publishing global poses in '{self.global_frame}' for discovered frames like "
            f"'{self.bot_prefix}<id>/base_link'."
        )

    def _bot_id_from_frame(self, frame_id: str) -> Optional[int]:
        f = frame_id.lstrip("/")
        m = self._bot_re.search(f)
        return int(m.group(1)) if m else None

    def _register_bot(self, bot_id: int) -> None:
        if bot_id is None or bot_id in self._bots:
            return

        base_frame = f"{self.bot_prefix}{bot_id}/base_link"
        topic = f"{self.bot_prefix}{bot_id}/{self.pose_topic_suffix}"

        pub = self.create_publisher(PoseStamped, topic, 10)
        self._bots[bot_id] = BotPub(bot_id=bot_id, base_frame=base_frame, pub=pub)

        self.get_logger().info(f"Discovered {self.bot_prefix}{bot_id}: publishing on '{topic}'")

    def _on_tf_msg(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            self._register_bot(self._bot_id_from_frame(t.header.frame_id))
            self._register_bot(self._bot_id_from_frame(t.child_frame_id))

    def _tick(self) -> None:
        now = rclpy.time.Time()  # latest
        timeout = Duration(seconds=0.05)

        for bot in list(self._bots.values()):
            try:
                tf = self._tf_buffer.lookup_transform(
                    self.global_frame,
                    bot.base_frame,
                    now,
                    timeout=timeout,
                )
            except (LookupException, ConnectivityException, ExtrapolationException):
                continue

            out = PoseStamped()
            out.header.frame_id = self.global_frame
            out.header.stamp = tf.header.stamp
            out.pose.position.x = tf.transform.translation.x
            out.pose.position.y = tf.transform.translation.y
            out.pose.position.z = tf.transform.translation.z
            out.pose.orientation = tf.transform.rotation

            bot.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = MultiBotGlobalPose()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
