#!/usr/bin/env python3
"""Replay the RACER-Lite trajectories with 6 quads while Swarm-LIO2 estimates.

bot1..bot6 == RACER R0,R1,R2,R3,R6,R7. Drones spawn at spread-out,
collision-checked 3D world positions (no stacked columns, no wall contact). INIT then
brings each drone to its reference start in two de-conflicted sub-steps:
CLIMB straight up at its own spawn (x,y) to the ref-start altitude (altitudes
differ by 3 m between drones sharing a column), then SLIDE horizontally to the
ref start. REPLAY tracks the voxel-centre, arc-length-re-timed refs. Set
RACER_REPLAY_DURATION to run only a prefix without changing the saved source
trajectory, and RACER_OUTPUT_DIR to keep short validation runs separate.

Records per drone: ground truth (Gazebo), SLAM estimate, RACER reference, and
the voxel-deduplicated accumulated SLAM point cloud (cloud_registered) for the
map-vs-obstacles visualisation.
"""
import json, math, os
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.environ.get("RACER_OUTPUT_DIR", os.path.join(HERE, "output"))
if not os.path.isabs(OUT):
    OUT = os.path.join(HERE, OUT)
os.makedirs(OUT, exist_ok=True)

BOTS = ["bot%d" % i for i in range(1, 7)]
RACER_ID = {"bot1": 0, "bot2": 1, "bot3": 2, "bot4": 3, "bot5": 6, "bot6": 7}

_r = np.load(os.path.join(HERE, "racer_refs.npz"))
REF_T = {b: _r["t_%d" % RACER_ID[b]] for b in BOTS}
REF_P = {b: _r["p_%d" % RACER_ID[b]] for b in BOTS}
START = {b: REF_P[b][0].astype(float) for b in BOTS}   # ref start (in the air)
FULL_DURATION = float(REF_T["bot1"][-1])
DURATION = min(
    FULL_DURATION,
    max(1.0, float(os.environ.get("RACER_REPLAY_DURATION", FULL_DURATION))),
)

CTRL_HZ = 20.0
KP = 1.2
V_MAX = 1.0                  # HARD drone speed limit (user spec: max 1 m/s);
                             # refs cruise at <=0.9 so corrections fit under it
INIT_T_MIN = 25.0            # earliest REPLAY start (climb+slide budget)
INIT_MAX = 60.0              # hard INIT timeout
GATHER_TOL = 0.8             # all drones this close to ref starts -> REPLAY
HARD_STOP = 1.0              # safety net during REPLAY
MAP_VOX = 0.15               # accumulated-cloud voxel size


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class RacerReplay(Node):
    def __init__(self):
        super().__init__("racer_replay")
        # sim time: Gazebo RTF ~0.3, wall-clock refs would outrun the drones 3x
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.gt = {}
        self.slam_raw = {}
        self.origin = {}
        self.rec = {b: [] for b in BOTS}
        self.vox = {b: {} for b in BOTS}      # accumulated SLAM cloud (LIO frame)
        self.t0 = None
        self.phase = "INIT"

        self.create_subscription(TFMessage, "/world/default/pose/info", self.on_gt, 50)
        for b in BOTS:
            self.create_subscription(Odometry, "/%s/lidar_slam/odom" % b,
                                     lambda m, bb=b: self.on_slam(m, bb), 20)
            self.create_subscription(PointCloud2, "/%s/cloud_registered" % b,
                                     lambda m, bb=b: self.on_cloud(m, bb), 5)
        self.cmd = {b: self.create_publisher(Twist, "/%s/cmd_vel" % b, 10) for b in BOTS}
        self.arm = {b: self.create_publisher(Bool, "/%s/enable" % b, 10) for b in BOTS}
        self.timer = self.create_timer(1.0 / CTRL_HZ, self.step)

    # ---------------- callbacks ----------------
    def on_gt(self, msg):
        for tr in msg.transforms:
            if tr.child_frame_id in RACER_ID:
                t, q = tr.transform.translation, tr.transform.rotation
                self.gt[tr.child_frame_id] = np.array(
                    [t.x, t.y, t.z, yaw_from_quat(q.x, q.y, q.z, q.w)])

    def on_slam(self, msg, b):
        p = msg.pose.pose.position
        self.slam_raw[b] = np.array([p.x, p.y, p.z])

    def on_cloud(self, msg, b):
        d = self.vox[b]
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            k = (round(p[0] / MAP_VOX), round(p[1] / MAP_VOX), round(p[2] / MAP_VOX))
            if k not in d:
                d[k] = (float(p[0]), float(p[1]), float(p[2]))

    # ---------------- helpers ----------------
    def slam_world(self, b):
        if b not in self.slam_raw or b not in self.origin:
            return None
        ox, oy, oz, oyaw = self.origin[b]
        px, py, pz = self.slam_raw[b]
        c, s = math.cos(oyaw), math.sin(oyaw)
        return np.array([ox + c * px - s * py, oy + s * px + c * py, oz + pz])

    def ref(self, b, t):
        T, P = REF_T[b], REF_P[b]
        t = min(max(t, 0.0), T[-1])
        pos = np.array([np.interp(t, T, P[:, k]) for k in range(3)])
        dt = 0.1
        p2 = np.array([np.interp(min(t + dt, T[-1]), T, P[:, k]) for k in range(3)])
        return pos, (p2 - pos) / dt

    def send(self, b, world_v):
        n = float(np.linalg.norm(world_v))
        if n > V_MAX:
            world_v = world_v * (V_MAX / n)
        yaw = self.gt[b][3]
        c, s = math.cos(yaw), math.sin(yaw)
        m = Twist()
        m.linear.x = c * world_v[0] + s * world_v[1]
        m.linear.y = -s * world_v[0] + c * world_v[1]
        m.linear.z = float(world_v[2])
        m.angular.z = max(-0.5, min(0.5, 1.0 * (0.0 - yaw)))
        self.cmd[b].publish(m)

    def goto(self, b, target, ff=np.zeros(3)):
        self.send(b, ff + KP * (target - self.gt[b][:3]))

    def yield_hold(self, b):
        for o in BOTS:
            if o == b or o not in self.gt:
                continue
            if np.linalg.norm(self.gt[b][:3] - self.gt[o][:3]) < HARD_STOP \
                    and BOTS.index(o) < BOTS.index(b):
                return True
        return False

    def init_target(self, b):
        """CLIMB at spawn (x,y) to ref-start z, then SLIDE to the ref start.
        Climb altitudes differ 3 m between column-mates, slide paths were
        collision-checked against the map and each other."""
        ox, oy = self.origin[b][0], self.origin[b][1]
        sz = START[b][2]
        if abs(self.gt[b][2] - sz) > 0.5:
            return np.array([ox, oy, sz])          # CLIMB
        return START[b]                            # SLIDE

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------------- main loop ----------------
    def step(self):
        if len(self.gt) < len(BOTS):
            return
        if self.t0 is None:
            self.t0 = self.now_s()
        now = self.now_s() - self.t0
        for b in BOTS:
            self.arm[b].publish(Bool(data=True))

        if self.phase == "INIT":
            if not self.origin:
                for b in BOTS:
                    self.origin[b] = self.gt[b].copy()
                self.get_logger().info("3D spawn origins captured; climb+gather")
            for b in BOTS:
                self.goto(b, self.init_target(b))
            gaps = [np.linalg.norm(self.gt[b][:3] - START[b]) for b in BOTS]
            if (now > INIT_T_MIN and max(gaps) < GATHER_TOL) or now > INIT_MAX:
                self.get_logger().info(
                    "init done (max gap %.2f m) -> REPLAY %.0fs" % (max(gaps), DURATION))
                self.phase = "REPLAY"
                self.t_replay = now
            return

        if self.phase == "REPLAY":
            trep = now - self.t_replay
            for b in BOTS:
                pos, vel = self.ref(b, trep)
                if self.yield_hold(b):
                    self.send(b, np.zeros(3))
                else:
                    self.goto(b, pos, ff=vel)
                sw = self.slam_world(b)
                g = self.gt[b]
                self.rec[b].append((trep, g[0], g[1], g[2],
                                    *(sw if sw is not None else (np.nan,) * 3),
                                    pos[0], pos[1], pos[2]))
            if trep > DURATION:
                self.get_logger().info("replay done -> STOP")
                self.phase = "STOP"
            return

        if self.phase == "STOP":
            for b in BOTS:
                self.send(b, np.zeros(3))
            self.timer.cancel()
            self.save()
            raise SystemExit(0)

    # ---------------- outputs ----------------
    def save(self):
        import csv
        lines = ["Per-drone SLAM accuracy on RACER replay (est vs Gazebo truth)", "=" * 60]
        for b in BOTS:
            rows = self.rec[b]
            with open(os.path.join(OUT, "rec_%s.csv" % b), "w") as f:
                w = csv.writer(f)
                w.writerow(["t", "gt_x", "gt_y", "gt_z", "sl_x", "sl_y", "sl_z",
                            "ref_x", "ref_y", "ref_z"])
                w.writerows(rows)
            arr = np.array([r for r in rows if not math.isnan(r[4])])
            m = np.array(list(self.vox[b].values())) if self.vox[b] else np.zeros((0, 3))
            np.save(os.path.join(OUT, "map_%s.npy" % b), m)
            if len(arr) < 5:
                lines.append("%s (R%d): NO SLAM ODOM  (map %d pts)" % (b, RACER_ID[b], len(m)))
                continue
            e = np.linalg.norm(arr[:, 1:4] - arr[:, 4:7], axis=1)
            trk = np.linalg.norm(arr[:, 1:4] - arr[:, 7:10], axis=1)
            lines.append("%s (R%d): ATE=%.3f m  max=%.3f m  |  track-vs-ref=%.3f m  |  map %d pts"
                         % (b, RACER_ID[b], math.sqrt((e ** 2).mean()), e.max(),
                            math.sqrt((trk ** 2).mean()), len(m)))
        json.dump({b: [float(v) for v in self.origin[b]] for b in BOTS},
                  open(os.path.join(OUT, "origins.json"), "w"), indent=1)
        txt = "\n".join(lines)
        open(os.path.join(OUT, "ate_report.txt"), "w").write(txt + "\n")
        print(txt, flush=True)
        print("Outputs in %s" % OUT, flush=True)


def main():
    rclpy.init()
    n = RacerReplay()
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
