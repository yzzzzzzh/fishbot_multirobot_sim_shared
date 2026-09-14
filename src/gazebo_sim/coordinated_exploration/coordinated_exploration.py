#!/usr/bin/env python3
"""
Coordinated multi-robot exploration of fishbot2.world.

For each of the 4 fishbots this node:
  * plans a collision-free, curvature-bounded coverage tour with A* (arena.py),
    routing bot2/bot4 through the central tunnels into the right-hand room while
    bot1/bot3 sweep the left-hand room;
  * tracks the path with a closed-loop pure-pursuit controller, using the
    Gazebo ground-truth pose (bridged pose/info -> TF) as feedback;
  * yields to higher-priority (lower-id) neighbours when they are close and
    ahead, so robots never collide with each other.

While the robots drive, Swarm-LIO2 (per-robot LIO) and map_fusion keep running.
When every tour finishes (or a global timeout hits) the node stops the robots
and writes, to ./output/:
  * global_map.ply          - the fused point cloud  (/global_downsampled_map)
  * traj_botN.csv           - t, ground-truth xy, SLAM-estimated xy (world frame)
  * ate_report.txt          - absolute trajectory error (RMSE) per robot
"""
import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage

from arena import Arena

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")

# Spawn poses in the world frame (circle pattern, base=(-3,0), r=1, count=4).
SPAWN = {
    "bot1": (-2.0, 0.0, 0.0),
    "bot2": (-3.0, 1.0, math.pi / 2),
    "bot3": (-4.0, 0.0, math.pi),
    "bot4": (-3.0, -1.0, -math.pi / 2),
}

# Coverage tours (world-frame waypoints). A* fills in the collision-free path.
TOURS = {
    # left room, upper lobe (around the pillars)
    "bot1": [(-3.5, 2.8), (-4.8, 1.6), (-4.6, 3.8), (-2.2, 3.9)],
    # right room, upper lobe (through the y=+1.1 tunnel)
    "bot2": [(0.0, 1.1), (2.2, 1.6), (4.0, 2.6), (4.6, 4.0), (2.2, 4.6)],
    # left room, lower lobe
    "bot3": [(-3.5, -2.8), (-4.8, -1.6), (-4.6, -3.8), (-2.2, -3.9)],
    # right room, lower lobe (through the y=-1.1 tunnel)
    "bot4": [(0.0, -1.1), (2.2, -1.6), (4.0, -2.6), (4.6, -4.0), (2.2, -4.6)],
}
BOTS = ["bot1", "bot2", "bot3", "bot4"]

# --- controller gains ---------------------------------------------------------
V_MAX = 0.35          # m/s cruise
W_MAX = 1.2           # rad/s
LOOKAHEAD = 0.45      # m
GOAL_TOL = 0.22       # m
SLOW_RADIUS = 0.6     # m, decelerate near final goal
ROTATE_THRESH = 1.1   # rad, rotate in place when target is this far off heading
CTRL_HZ = 20.0
GLOBAL_TIMEOUT = 220.0  # s

# --- inter-robot safety -------------------------------------------------------
SAFE_DIST = 0.55      # m, yield when a higher-priority neighbour is this close
AHEAD_DOT = 0.2       # neighbour counts as "ahead" when heading dot > this


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Explorer(Node):
    def __init__(self):
        super().__init__("coordinated_exploration")
        os.makedirs(OUT_DIR, exist_ok=True)

        self.get_logger().info("Planning collision-free tours (A*) ...")
        arena = Arena()
        self.paths = {}
        for b in BOTS:
            sx, sy, _ = SPAWN[b]
            path = arena.plan_tour((sx, sy), TOURS[b])
            self.paths[b] = path
            self.get_logger().info(f"  {b}: {len(path)} pts -> {path[-1] if path else None}")

        # runtime state
        self.gt = {}            # bot -> (x, y, yaw)  ground truth
        self.slam = {}          # bot -> (x, y)       SLAM estimate in world frame
        self.idx = {b: 0 for b in BOTS}
        self.done = {b: (not self.paths[b]) for b in BOTS}
        self.traj = {b: [] for b in BOTS}   # list of (t, gtx, gty, slamx, slamy)
        self.cloud = None
        self.t0 = time.time()

        # publishers / subscribers
        self.cmd = {b: self.create_publisher(Twist, f"/{b}/cmd_vel", 10) for b in BOTS}

        best = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(TFMessage, "/world/default/pose/info",
                                 self.on_gt, best)
        for b in BOTS:
            self.create_subscription(
                Odometry, f"/{b}/lidar_slam/odom",
                lambda m, bb=b: self.on_slam(m, bb), best)
        self.create_subscription(PointCloud2, "/global_downsampled_map",
                                 self.on_cloud, best)

        self.timer = self.create_timer(1.0 / CTRL_HZ, self.control_step)
        self.get_logger().info("Controller running. Robots are moving.")

    # --- callbacks ------------------------------------------------------------
    def on_gt(self, msg: TFMessage):
        for tr in msg.transforms:
            name = tr.child_frame_id
            if name in SPAWN:
                t = tr.transform.translation
                q = tr.transform.rotation
                self.gt[name] = (t.x, t.y, yaw_from_quat(q.x, q.y, q.z, q.w))

    def on_slam(self, msg: Odometry, bot):
        sx, sy, syaw = SPAWN[bot]
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        c, s = math.cos(syaw), math.sin(syaw)
        self.slam[bot] = (sx + c * px - s * py, sy + s * px + c * py)

    def on_cloud(self, msg: PointCloud2):
        self.cloud = msg

    # --- control --------------------------------------------------------------
    def control_step(self):
        now = time.time() - self.t0

        for b in BOTS:
            if b not in self.gt:
                continue
            x, y, th = self.gt[b]
            # log paired ground-truth / SLAM sample
            if b in self.slam:
                sx, sy = self.slam[b]
                self.traj[b].append((now, x, y, sx, sy))

            if self.done[b]:
                self.cmd[b].publish(Twist())
                continue

            path = self.paths[b]
            # advance progress index to the closest point ahead (monotonic)
            best_i, best_d = self.idx[b], float("inf")
            for i in range(self.idx[b], len(path)):
                d = math.hypot(path[i][0] - x, path[i][1] - y)
                if d < best_d:
                    best_d, best_i = d, i
            self.idx[b] = best_i

            # goal check
            gx, gy = path[-1]
            dgoal = math.hypot(gx - x, gy - y)
            if best_i >= len(path) - 1 and dgoal < GOAL_TOL:
                self.done[b] = True
                self.cmd[b].publish(Twist())
                self.get_logger().info(f"{b} reached goal.")
                continue

            # pick lookahead target
            ti = best_i
            while ti < len(path) - 1 and math.hypot(path[ti][0] - x, path[ti][1] - y) < LOOKAHEAD:
                ti += 1
            tx, ty = path[ti]

            alpha = wrap(math.atan2(ty - y, tx - x) - th)

            if abs(alpha) > ROTATE_THRESH:
                v = 0.0
                w = max(-W_MAX, min(W_MAX, 2.0 * alpha))
            else:
                v = V_MAX * max(0.25, 1.0 - abs(alpha) / ROTATE_THRESH)
                if dgoal < SLOW_RADIUS:
                    v *= max(0.2, dgoal / SLOW_RADIUS)
                curvature = 2.0 * math.sin(alpha) / LOOKAHEAD
                w = max(-W_MAX, min(W_MAX, v * curvature))

            # inter-robot safety: yield to closer, higher-priority neighbours
            if self._must_yield(b, x, y, th):
                v, w = 0.0, 0.0

            msg = Twist()
            msg.linear.x = v
            msg.angular.z = w
            self.cmd[b].publish(msg)

        if all(self.done[b] for b in BOTS) or now > GLOBAL_TIMEOUT:
            self.finish()

    def _must_yield(self, b, x, y, th):
        prio = BOTS.index(b)
        for other in BOTS[:prio]:          # only lower-id (higher priority) bots
            if other not in self.gt or self.done[other]:
                continue
            ox, oy, _ = self.gt[other]
            dx, dy = ox - x, oy - y
            dist = math.hypot(dx, dy)
            if dist < SAFE_DIST:
                # ahead of me?
                if (math.cos(th) * dx + math.sin(th) * dy) / (dist + 1e-6) > AHEAD_DOT:
                    return True
        return False

    # --- shutdown / reporting -------------------------------------------------
    def finish(self):
        self.timer.cancel()
        for b in BOTS:                      # make sure everyone stops
            for _ in range(5):
                self.cmd[b].publish(Twist())
        self.get_logger().info("Tours complete. Saving results ...")
        self._save_cloud()
        self._save_traj_and_ate()
        self.get_logger().info(f"Done. Outputs in {OUT_DIR}")
        rclpy.shutdown()

    def _save_cloud(self):
        if self.cloud is None:
            self.get_logger().warn("No fused map received; skipping map save.")
            return
        pts = list(point_cloud2.read_points(
            self.cloud, field_names=("x", "y", "z"), skip_nans=True))
        path = os.path.join(OUT_DIR, "global_map.ply")
        with open(path, "w") as f:
            f.write("ply\nformat ascii 1.0\n")
            f.write(f"element vertex {len(pts)}\n")
            f.write("property float x\nproperty float y\nproperty float z\n")
            f.write("end_header\n")
            for p in pts:
                f.write(f"{float(p[0])} {float(p[1])} {float(p[2])}\n")
        self.get_logger().info(f"Saved fused map: {len(pts)} points -> {path}")

    def _save_traj_and_ate(self):
        lines = ["Absolute Trajectory Error (SLAM estimate vs Gazebo ground truth)",
                 "=" * 62]
        for b in BOTS:
            rows = self.traj[b]
            csv = os.path.join(OUT_DIR, f"traj_{b}.csv")
            with open(csv, "w") as f:
                f.write("t,gt_x,gt_y,slam_x,slam_y\n")
                for t, gx, gy, sx, sy in rows:
                    f.write(f"{t:.3f},{gx:.4f},{gy:.4f},{sx:.4f},{sy:.4f}\n")
            errs = [math.hypot(gx - sx, gy - sy) for _, gx, gy, sx, sy in rows]
            if errs:
                ate = math.sqrt(sum(e * e for e in errs) / len(errs))
                path_len = sum(
                    math.hypot(rows[i][1] - rows[i - 1][1], rows[i][2] - rows[i - 1][2])
                    for i in range(1, len(rows)))
                lines.append(f"{b}: ATE(RMSE)={ate:.3f} m  max={max(errs):.3f} m  "
                             f"path_len={path_len:.1f} m  samples={len(errs)}")
            else:
                lines.append(f"{b}: no paired samples")
        report = "\n".join(lines) + "\n"
        with open(os.path.join(OUT_DIR, "ate_report.txt"), "w") as f:
            f.write(report)
        self.get_logger().info("\n" + report)


def main():
    rclpy.init()
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        for b in BOTS:
            node.cmd[b].publish(Twist())
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
