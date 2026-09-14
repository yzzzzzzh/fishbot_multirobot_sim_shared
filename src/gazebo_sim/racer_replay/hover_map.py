#!/usr/bin/env python3
"""Hover bot4/bot6 in place and accumulate their cloud_registered scans into a
dense local point-cloud map (voxel-deduplicated). A slow yaw sweep lets the
non-repetitive Mid360 pattern fill in the room. Saves /tmp/map_<bot>.npy in the
LIO world frame (add the original spawn afterwards to reach the Gazebo frame).
"""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

BOTS = ["bot4", "bot6"]
VOX = 0.15
DUR = 40.0     # wall-clock seconds


class Hover(Node):
    def __init__(self):
        super().__init__("hover_map")
        # wall-clock timer (do NOT use sim time) so the save fires reliably even
        # if the GT bridge is down; cloud accumulation needs no clock.
        self.vox = {b: {} for b in BOTS}
        self.seen = 0
        for b in BOTS:
            self.create_subscription(PointCloud2, f"/{b}/cloud_registered",
                                     lambda m, bb=b: self.on_cloud(m, bb), 5)
        self.cmd = {b: self.create_publisher(Twist, f"/{b}/cmd_vel", 10) for b in BOTS}
        self.arm = {b: self.create_publisher(Bool, f"/{b}/enable", 10) for b in BOTS}
        self.timer = self.create_timer(0.1, self.step)
        self.t0 = time.time()

    def on_cloud(self, msg, b):
        self.seen += 1
        d = self.vox[b]
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            k = (round(p[0] / VOX), round(p[1] / VOX), round(p[2] / VOX))
            if k not in d:
                d[k] = (float(p[0]), float(p[1]), float(p[2]))

    def step(self):
        for b in BOTS:                       # arm + hover (0 vel) + slow yaw sweep
            self.arm[b].publish(Bool(data=True))
            m = Twist()
            m.angular.z = 0.45
            self.cmd[b].publish(m)
        el = time.time() - self.t0
        if el % 5 < 0.1:
            self.get_logger().info(
                "t=%.0fs  scans=%d  bot4=%d bot6=%d voxels"
                % (el, self.seen, len(self.vox['bot4']), len(self.vox['bot6'])))
        if el > DUR:
            for b in BOTS:
                arr = np.array(list(self.vox[b].values())) if self.vox[b] else np.zeros((0, 3))
                np.save(f"/tmp/map_{b}.npy", arr)
                self.get_logger().info(f"{b}: saved {len(arr)} voxels")
            self.timer.cancel()
            raise SystemExit(0)


def main():
    rclpy.init()
    n = Hover()
    try:
        rclpy.spin(n)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
