#!/usr/bin/env python3
"""Single-drone SLAM run through the maze -- the fishbot flow, flown.

Same shape as maze_exploration.py: plan a collision-free A* tour over the maze
walls, track it closed-loop against Gazebo ground truth while Swarm-LIO2 runs,
then report ATE and dump the fused map.

Drone-specific differences:
  * holonomic -- yaw is held at the spawn value for the whole flight, so the
    footprint never rotates and there is no rotate-in-place or reversing.
  * inflation is 0.45 m, not the fishbot's 0.25: the quad's circumscribed
    radius is 0.397 m (rotor tips at +-0.235 x, +-0.320 y) vs the fishbot's
    0.15. That leaves a 0.50 m free channel in the 1.40 m corridors.
  * altitude follows a sine so the trajectory is genuinely 3D, and ATE is
    reported in 3D as well as in the fishbot's 2D form.
  * the flight controller must be armed (Bool true on /<ns>/enable) or cmd_vel
    is ignored.

Outputs (output/): global_map.ply, traj_bot1.csv, ate_report.txt
"""
import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "maze_exploration"))
from gridmap import GridMap  # noqa: E402

OUT_DIR = os.path.join(HERE, "output")
DEF = json.load(open(os.path.join(HERE, "..", "maze_exploration", "maze_def.json")))

BOT = "bot1"
# docker-compose.drone.yml spawns count:=1 pattern:=circle x:=0 y:=0 spacing:=0.7,
# which puts bot1 at angle 0 -> (0.7, 0.0), yaw 0.
SPAWN_XY = tuple(DEF["spawns"]["bot1"][:2])
SPAWN_YAW = DEF["spawns"]["bot1"][2]

INFLATION = 0.45
RES = 0.05

CTRL_HZ = 20.0
V_MAX = 0.35             # matches the fishbot's cruise, for comparable tracking
V_MAX_Z = 0.5
KP_XY = 0.9
KP_Z = 0.6               # gentle: the plugin's inner loop holds to ~0.1 mm
LOOKAHEAD = 0.45
GOAL_TOL = 0.18
SLOW_RADIUS = 0.6

INIT_HOLD = 12.0         # sit disarmed on the ground: Swarm-LIO2 needs a still IMU
TAKEOFF_TOL = 0.10
GLOBAL_TIMEOUT = 480.0

# Altitude profile. Walls are 1.0 m and the lidar sits 0.15 m above base_link,
# so cruising at 0.50 m keeps the lidar at 0.65 m -- looking at wall, not over it.
Z_BASE = 0.50
Z_AMP = 0.12
Z_PERIOD = 20.0
Z_LAND = 0.06


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def cell(c, r):
    return (DEF["x0"] + c * DEF["pitch"], DEF["y0"] + r * DEF["pitch"])


class DroneExplorer(Node):
    def __init__(self):
        super().__init__("drone_exploration")
        os.makedirs(OUT_DIR, exist_ok=True)

        self.gt = None            # (x, y, z, yaw) world, ground truth
        self.slam_raw = None      # (x, y, z) in the SLAM start frame
        self.cloud = None
        self.origin = None        # gt pose captured before takeoff == SLAM frame origin
        self.traj = []
        self.t0 = time.time()
        self.phase = "INIT"
        self.idx = 0
        self.armed_at = None

        # Plan the tour: loop the four corners, come home. Same planner the
        # fishbots use, only the inflation differs.
        gm = GridMap(DEF["boxes"], DEF["bounds"], inflation=INFLATION, res=RES)
        cc = DEF["corner_cells"]
        tour = [cell(*cc["tr"]), cell(*cc["tl"]), cell(*cc["bl"]), cell(*cc["br"]),
                SPAWN_XY]
        self.path = gm.plan_tour(SPAWN_XY, tour)
        if not self.path:
            raise SystemExit("planner found no tour -- is the drone too wide?")
        plen = sum(math.hypot(self.path[i][0] - self.path[i - 1][0],
                              self.path[i][1] - self.path[i - 1][1])
                   for i in range(1, len(self.path)))
        self.get_logger().info("planned tour: %d pts, %.1f m, inflation %.2f m"
                               % (len(self.path), plen, INFLATION))

        best = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(TFMessage, "/world/default/pose/info", self.on_gt, 50)
        self.create_subscription(Odometry, f"/{BOT}/lidar_slam/odom", self.on_slam, 20)
        self.create_subscription(PointCloud2, "/global_downsampled_map", self.on_cloud, best)
        self.cmd = self.create_publisher(Twist, f"/{BOT}/cmd_vel", 10)
        self.arm = self.create_publisher(Bool, f"/{BOT}/enable", 10)
        self.timer = self.create_timer(1.0 / CTRL_HZ, self.step)

    # ---------------- callbacks ----------------
    def on_gt(self, msg):
        for tr in msg.transforms:
            if tr.child_frame_id == BOT:
                t, q = tr.transform.translation, tr.transform.rotation
                self.gt = (t.x, t.y, t.z, yaw_from_quat(q.x, q.y, q.z, q.w))

    def on_slam(self, msg):
        p = msg.pose.pose.position
        self.slam_raw = (p.x, p.y, p.z)

    def on_cloud(self, msg):
        self.cloud = msg

    def slam_world(self):
        """SLAM pose -> world, using the pre-takeoff ground-truth pose as the
        SLAM frame origin (that is exactly what Swarm-LIO2 initialises to)."""
        if self.slam_raw is None or self.origin is None:
            return None
        ox, oy, oz, oyaw = self.origin
        px, py, pz = self.slam_raw
        c, s = math.cos(oyaw), math.sin(oyaw)
        return (ox + c * px - s * py, oy + s * px + c * py, oz + pz)

    # ---------------- control ----------------
    def target_z(self, t):
        return Z_BASE + Z_AMP * math.sin(2 * math.pi * t / Z_PERIOD)

    def go(self, tx, ty, tz, vcap=V_MAX):
        x, y, z, yaw = self.gt
        ex, ey, ez = tx - x, ty - y, tz - z
        d = math.hypot(ex, ey)
        v = min(vcap, KP_XY * d)
        if d > 1e-6:
            vx_w, vy_w = v * ex / d, v * ey / d
        else:
            vx_w = vy_w = 0.0
        vz = max(-V_MAX_Z, min(V_MAX_Z, KP_Z * ez))
        c, s = math.cos(yaw), math.sin(yaw)
        m = Twist()
        m.linear.x = c * vx_w + s * vy_w      # plugin takes BODY-frame velocity
        m.linear.y = -s * vx_w + c * vy_w
        m.linear.z = vz
        # hold the spawn heading: holonomic, so there is never a need to turn
        eyaw = math.atan2(math.sin(SPAWN_YAW - yaw), math.cos(SPAWN_YAW - yaw))
        m.angular.z = max(-0.5, min(0.5, 1.0 * eyaw))
        self.cmd.publish(m)
        return d

    def step(self):
        if self.gt is None:
            return
        now = time.time() - self.t0

        if self.phase == "INIT":
            if self.origin is None:
                self.origin = self.gt
                self.get_logger().info(
                    "SLAM frame origin (pre-takeoff truth): %.3f %.3f %.3f yaw %.3f"
                    % self.origin)
            self.cmd.publish(Twist())
            if now > INIT_HOLD:
                self.get_logger().info("IMU init window done -> arming")
                self.phase = "ARM"
            return

        if self.phase == "ARM":
            self.arm.publish(Bool(data=True))
            self.cmd.publish(Twist())
            if self.armed_at is None:
                self.armed_at = now
            elif now - self.armed_at > 1.5:
                self.get_logger().info("armed -> climbing to %.2f m" % Z_BASE)
                self.phase = "TAKEOFF"
            return

        self._log()

        if self.phase == "TAKEOFF":
            self.go(*SPAWN_XY, Z_BASE)
            if abs(self.gt[2] - Z_BASE) < TAKEOFF_TOL:
                self.get_logger().info("at cruise altitude -> flying the tour")
                self.phase = "TOUR"

        elif self.phase == "TOUR":
            x, y = self.gt[0], self.gt[1]
            # monotonic carrot: the tour ends where it starts, so a global
            # nearest-point search would declare "arrived" at t=0
            i = self.idx
            while i < len(self.path) - 1 and \
                    math.hypot(self.path[i][0] - x, self.path[i][1] - y) < LOOKAHEAD:
                i += 1
            self.idx = i
            tx, ty = self.path[i]
            dgoal = math.hypot(self.path[-1][0] - x, self.path[-1][1] - y)
            vcap = V_MAX * min(1.0, max(0.25, dgoal / SLOW_RADIUS)) \
                if i >= len(self.path) - 1 else V_MAX
            self.go(tx, ty, self.target_z(now), vcap)
            if (i >= len(self.path) - 1 and dgoal < GOAL_TOL) or now > GLOBAL_TIMEOUT:
                self.get_logger().info("tour done (%.0f s) -> landing" % now)
                self.phase = "LAND"

        elif self.phase == "LAND":
            self.go(self.gt[0], self.gt[1], Z_LAND, 0.15)
            if self.gt[2] < 0.15:
                self.cmd.publish(Twist())
                self.arm.publish(Bool(data=False))
                self.timer.cancel()
                self.finish()
                raise SystemExit(0)

    def _log(self):
        sw = self.slam_world()
        if sw is None:
            return
        t = time.time() - self.t0
        self.traj.append((t, self.gt[0], self.gt[1], self.gt[2], sw[0], sw[1], sw[2]))

    # ---------------- outputs ----------------
    def finish(self):
        self._save_cloud()
        self._save_traj_ate()
        print("Outputs in %s" % OUT_DIR, flush=True)

    def _save_cloud(self):
        if self.cloud is None:
            print("WARNING: no fused map received on /global_downsampled_map "
                  "-- is map_fusion up?", flush=True)
            return
        pts = list(point_cloud2.read_points(
            self.cloud, field_names=("x", "y", "z"), skip_nans=True))
        p = os.path.join(OUT_DIR, "global_map.ply")
        with open(p, "w") as f:
            f.write("ply\nformat ascii 1.0\nelement vertex %d\n" % len(pts))
            f.write("property float x\nproperty float y\nproperty float z\nend_header\n")
            for q in pts:
                f.write("%f %f %f\n" % (float(q[0]), float(q[1]), float(q[2])))
        print("saved fused map: %d points -> %s" % (len(pts), p), flush=True)

    def _save_traj_ate(self):
        rows = self.traj
        with open(os.path.join(OUT_DIR, "traj_bot1.csv"), "w") as f:
            f.write("t,gt_x,gt_y,gt_z,slam_x,slam_y,slam_z\n")
            for r in rows:
                f.write("%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n" % r)
        if not rows:
            print("no trajectory logged -- did /bot1/lidar_slam/odom ever publish?",
                  flush=True)
            return
        e2 = [math.hypot(r[1] - r[4], r[2] - r[5]) for r in rows]
        e3 = [math.sqrt((r[1] - r[4]) ** 2 + (r[2] - r[5]) ** 2 + (r[3] - r[6]) ** 2)
              for r in rows]
        ez = [abs(r[3] - r[6]) for r in rows]
        plen = sum(math.sqrt((rows[i][1] - rows[i - 1][1]) ** 2 +
                             (rows[i][2] - rows[i - 1][2]) ** 2 +
                             (rows[i][3] - rows[i - 1][3]) ** 2)
                   for i in range(1, len(rows)))
        rms = lambda v: math.sqrt(sum(x * x for x in v) / len(v))
        back = math.hypot(rows[-1][1] - SPAWN_XY[0], rows[-1][2] - SPAWN_XY[1])
        lines = [
            "Single-drone SLAM in the maze (quad_mid360, Swarm-LIO2)",
            "=" * 60,
            "samples        : %d over %.0f s" % (len(rows), rows[-1][0]),
            "path flown     : %.1f m (3D)" % plen,
            "altitude       : %.2f - %.2f m" % (min(r[3] for r in rows),
                                                max(r[3] for r in rows)),
            "ATE 3D         : RMS %.3f m  max %.3f m" % (rms(e3), max(e3)),
            "ATE 2D (xy)    : RMS %.3f m  max %.3f m   <- comparable to the fishbot runs"
            % (rms(e2), max(e2)),
            "ATE z only     : RMS %.3f m  max %.3f m" % (rms(ez), max(ez)),
            "back to spawn  : %.3f m" % back,
        ]
        with open(os.path.join(OUT_DIR, "ate_report.txt"), "w") as f:
            f.write("\n".join(lines) + "\n")
        print("\n".join(lines), flush=True)


def main():
    rclpy.init()
    n = DroneExplorer()
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
