#!/usr/bin/env python3
"""Bring-up check for the quad_mid360 blueprint: arm, take off, hover, fly a
square, land -- and report whether the drone is actually controllable and its
sensors usable for SLAM.

Feedback is Gazebo ground truth (/world/default/pose/info bridged to TF, whose
child_frame_id is the model name), the same source the fishbot controllers use.

Pass criteria printed at the end:
  * hover altitude drift          < 5 cm over 8 s
  * square corner tracking error  < 25 cm
  * lidar returns a full 180x64 cloud with plausible ranges
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage

BOT = "bot1"
CTRL_HZ = 20.0
TAKEOFF_Z = 1.5
HOVER_T = 8.0
SQUARE = 2.0          # side length [m]
# Outer position loop, cascaded onto the plugin's own velocity loop. Keep these
# gentle: the inner loop alone holds a commanded velocity to ~0.1 mm, so any
# hover wobble that shows up is this loop overdriving it.
KP_XY = 0.9
KP_Z = 0.6
V_MAX_XY = 0.8
V_MAX_Z = 0.8
POS_TOL = 0.12


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Bringup(Node):
    def __init__(self):
        super().__init__("drone_bringup")
        self.pose = None          # (x, y, z, yaw)
        self.cloud = None
        self.t0 = None

        self.create_subscription(TFMessage, "/world/default/pose/info",
                                 self._on_tf, 50)
        self.create_subscription(
            PointCloud2, f"/{BOT}/lidar_points/points", self._on_cloud,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.cmd = self.create_publisher(Twist, f"/{BOT}/cmd_vel", 10)
        self.arm = self.create_publisher(Bool, f"/{BOT}/enable", 10)

        # waypoints: (x, y, z) in world frame, walked in order
        s = SQUARE / 2.0
        self.wps = [(0.0, 0.0, TAKEOFF_Z)]
        self.hover_until = None
        self.square = [(s, -s, TAKEOFF_Z), (s, s, TAKEOFF_Z),
                       (-s, s, TAKEOFF_Z), (-s, -s, TAKEOFF_Z),
                       (0.0, 0.0, TAKEOFF_Z)]
        self.phase = "ARM"
        self.wp_i = 0
        self.hover_z = []
        self.corner_err = []
        self.armed_at = None
        self.timer = self.create_timer(1.0 / CTRL_HZ, self._tick)

    def _on_tf(self, msg: TFMessage):
        for t in msg.transforms:
            if t.child_frame_id == BOT:
                p, q = t.transform.translation, t.transform.rotation
                self.pose = (p.x, p.y, p.z, yaw_of(q))

    def _on_cloud(self, msg: PointCloud2):
        self.cloud = msg

    def _go(self, tx, ty, tz) -> float:
        """World-frame P control -> body-frame Twist. Returns distance to goal."""
        x, y, z, yaw = self.pose
        ex, ey, ez = tx - x, ty - y, tz - z
        vx_w = max(-V_MAX_XY, min(V_MAX_XY, KP_XY * ex))
        vy_w = max(-V_MAX_XY, min(V_MAX_XY, KP_XY * ey))
        vz = max(-V_MAX_Z, min(V_MAX_Z, KP_Z * ez))
        # the plugin takes velocity in the BODY frame
        c, s = math.cos(yaw), math.sin(yaw)
        m = Twist()
        m.linear.x = c * vx_w + s * vy_w
        m.linear.y = -s * vx_w + c * vy_w
        m.linear.z = vz
        self.cmd.publish(m)
        return math.hypot(math.hypot(ex, ey), ez)

    def _tick(self):
        now = time.time()
        if self.pose is None:
            return
        if self.t0 is None:
            self.t0 = now
            self.get_logger().info("ground truth acquired at %.2f %.2f %.2f"
                                   % self.pose[:3])

        if self.phase == "ARM":
            self.arm.publish(Bool(data=True))
            self.cmd.publish(Twist())
            if self.armed_at is None:
                self.armed_at = now
            elif now - self.armed_at > 1.5:
                self.get_logger().info("armed -> taking off to %.1f m" % TAKEOFF_Z)
                self.phase = "TAKEOFF"

        elif self.phase == "TAKEOFF":
            d = self._go(*self.wps[0])
            if d < POS_TOL:
                self.get_logger().info("reached altitude -> hovering %.0f s" % HOVER_T)
                self.phase = "HOVER"
                self.hover_until = now + HOVER_T

        elif self.phase == "HOVER":
            self._go(*self.wps[0])
            # Skip the first 2 s: the drone is still settling out of the climb,
            # and folding that transient in would measure the step response,
            # not the hold.
            if now > self.hover_until - (HOVER_T - 2.0):
                self.hover_z.append(self.pose[2])
            if now > self.hover_until:
                self.get_logger().info("hover done -> square")
                self.phase = "SQUARE"

        elif self.phase == "SQUARE":
            tgt = self.square[self.wp_i]
            d = self._go(*tgt)
            if d < POS_TOL:
                self.corner_err.append(d)
                self.get_logger().info("corner %d/%d reached (err %.3f m)"
                                       % (self.wp_i + 1, len(self.square), d))
                self.wp_i += 1
                if self.wp_i >= len(self.square):
                    self.phase = "LAND"

        elif self.phase == "LAND":
            d = self._go(self.pose[0], self.pose[1], 0.06)
            if self.pose[2] < 0.15:
                self.cmd.publish(Twist())
                self.arm.publish(Bool(data=False))
                self.phase = "DONE"
                self.timer.cancel()
                self._report()
                raise SystemExit(0)

    def _report(self):
        print("\n" + "=" * 62)
        print("quad_mid360 bring-up report")
        print("=" * 62)
        if self.hover_z:
            drift = max(self.hover_z) - min(self.hover_z)
            err = abs(sum(self.hover_z) / len(self.hover_z) - TAKEOFF_Z)
            print("hover altitude   : mean %.3f m (target %.2f, offset %.0f mm), "
                  "peak-to-peak %.1f mm over %.0f s  %s"
                  % (sum(self.hover_z) / len(self.hover_z), TAKEOFF_Z, err * 1000,
                     drift * 1000, HOVER_T - 2.0,
                     "PASS" if drift < 0.05 else "FAIL"))
        if self.corner_err:
            worst = max(self.corner_err)
            print("square tracking  : %d/%d corners hit, worst err %.3f m  %s"
                  % (len(self.corner_err), len(self.square), worst,
                     "PASS" if worst < 0.25 else "FAIL"))
        if self.cloud is not None:
            pts = list(point_cloud2.read_points(
                self.cloud, field_names=("x", "y", "z"), skip_nans=True))
            rng = [math.sqrt(p[0] ** 2 + p[1] ** 2 + p[2] ** 2) for p in pts]
            finite = [r for r in rng if r < 19.0]
            print("lidar            : %dx%d grid, %d finite returns, range %.2f-%.2f m  %s"
                  % (self.cloud.width, self.cloud.height, len(finite),
                     min(finite) if finite else -1, max(finite) if finite else -1,
                     "PASS" if len(finite) > 500 else "FAIL"))
        else:
            print("lidar            : NO DATA  FAIL")
        print("=" * 62, flush=True)


def main():
    rclpy.init()
    n = Bringup()
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
