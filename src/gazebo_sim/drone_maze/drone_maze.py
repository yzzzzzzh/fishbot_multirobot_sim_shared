#!/usr/bin/env python3
"""5-drone coordinated SLAM in the TALL-WALL maze (drone_maze.world, 3 m walls).

Derived from drone_swarm.py; differences:
  * world has 3 m walls so drones cannot see or fly over the maze
  * tours add a per-drone phase-shifted z sine (mild 3D motion)
  * lidar v-FOV fixed to +52 deg (see quad_mid360.yaml) before this run

Every design choice here is a direct answer to something an earlier run got
wrong. In order of how much they cost:

1. FUSION IS DECIDED BY THE WARM-UP, NOT THE TOUR.
   Swarm-LIO2 identifies a teammate by trajectory matching, gated on the SECOND
   singular value of the observed position scatter (traj_matching_start_thresh
   = 8.0, MultiUAV.cpp), and the FIRST match that fits is locked forever
   (`iter_tracker != end() -> continue`). So the warm-up must make each drone
   simultaneously *observable* and *unambiguous*, before any chance of a wrong
   lock. Staggered solo loops did that best for the fishbots (3/4, zero
   mis-assignments); this is that, with its one failure fixed.

2. THE LOOP MUST CLOSE.
   fishbot bot5 got sigma2 = 1.6 and was never estimated. Not for lack of
   points: it drove only 35 % of its circle, and a short arc is nearly
   collinear, so sigma2 collapses even though sigma1 is fine. Here each loop is
   time-parameterised to complete exactly one full circle inside its slot, so
   the scatter is isotropic by construction: sigma2 ~ N*r^2/2 >= 28 for every
   drone (gate is 8). That model reproduces the fishbots' measured 13.0/21.9/
   31.8 to ~35 %.

3. LAST SLOT GETS THE SMALLEST LOOP.
   bot5 held the last slot *and* the largest loop, so it ran out of time. Radii
   here descend 1.00 -> 0.60 with slot order, and the slot is sized for the
   largest loop, not the smallest.

4. DISTINCT RADII.
   Five identical circles made the fishbots swap identities outright: the paths
   were indistinguishable to trajectory matching.

5. VERTICAL SEPARATION DURING THE WARM-UP.
   A quad is 0.64 m across; five of them in a 3.9 m chamber cannot loop past
   each other horizontally. So the drone whose slot it is climbs to 0.70 m
   while the rest hold at 0.35 m. 0.35 m of vertical clearance against 0.11 m
   of body makes a warm-up collision geometrically impossible, and both stay
   under the 1.0 m wall tops so the lidar still sees wall. Ground robots had no
   such option.

6. INFLATION 0.45, NOT 0.25. Quad circumscribed radius is 0.397 m vs the
   fishbot's 0.15. Corridors are 1.40 m -> a 0.50 m free channel.

7. COVERAGE, BECAUSE OF THE LEVER ARM.
   The single-drone run mapped the NW corridor from up to 5.5 m away and it
   landed 0.26 m out -- not from position drift (only 0.10 m at the time) but
   from ~1.8 deg of yaw amplified by range. Each drone here gets a sector it
   flies *through*, so walls are mapped close up.

8. SAVE THE INTENSITY CHANNEL. map_fusion encodes the source robot in it
   (base_int = 100*(uid-1), global_map_node.py). Dropping it, as the fishbot
   scripts did, throws away the only way to tell whose points are whose.

Outputs (output/): global_map.ply (x,y,z,intensity), traj_botN.csv,
ate_report.txt, extrinsics_report.txt, dist_pairs.csv
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
import tf2_ros

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "maze_exploration"))
from gridmap import GridMap  # noqa: E402

OUT_DIR = os.path.join(HERE, "output")  # drone_maze/output
DEF = json.load(open(os.path.join(HERE, "..", "maze_exploration", "maze_def.json")))

N_BOTS = 5
BOTS = ["bot%d" % (i + 1) for i in range(N_BOTS)]
SPACING = 1.2            # must match DRONE_SPACING in docker-compose.drone.yml
SPAWN_Z = 0.055

# spawn_robots.launch.py circle pattern: angle = 2*pi*i/count, yaw = angle
SPAWN = {}
for i, b in enumerate(BOTS):
    a = 2 * math.pi * i / N_BOTS
    SPAWN[b] = (SPACING * math.cos(a), SPACING * math.sin(a), a)

INFLATION = 0.45
RES = 0.05

CTRL_HZ = 20.0
KP_XY = 0.9
KP_Z = 0.6               # gentle: the inner loop holds velocity to ~0.1 mm
V_MAX = 0.35
V_MAX_Z = 0.5
LOOKAHEAD = 0.45
GOAL_TOL = 0.20
SLOW_RADIUS = 0.6

PARK_R = 1.55            # parked ring radius; spawn ring is 1.2. Keeps the
                         # mover's loop >=0.55 m clear of every parked drone,
                         # outside their trackers' 0.7 m predict regions.
PARK = {b: (PARK_R * math.cos(SPAWN[b][2]), PARK_R * math.sin(SPAWN[b][2]))
        for b in BOTS}
PARK_Z = 0.35            # lidar at 0.50 m
SOLO_Z = 0.65            # lidar at 0.85 m, still under the 1.0 m wall tops
CRUISE_Z = 0.45
Z_AMP = 0.12             # tour z sine amplitude: mild 3D motion
Z_PERIOD = 11.0          # s; per-bot phase offset decorrelates altitudes
LAND_Z = 0.06

# --- schedule -------------------------------------------------------------
T_INIT = 12.0            # disarmed on the ground: Swarm-LIO2 wants a still IMU
T_ARM = T_INIT + 2.0
T_CLIMB = T_ARM + 6.0
SOLO_SLOT = 26.0
LOOP_T = 16.0            # one full circle per slot, always closed
T_WARM_END = T_CLIMB + N_BOTS * SOLO_SLOT
# Point-2 fix: cap the tour so we reach the REGROUP re-observation sooner
# (tours mostly finish ~210 s; the tail is just return-gridlock).
TOUR_MAX = 230.0
# REGROUP: after the tours, repeat a compressed staggered-solo pass in the open
# chamber. This is the "periodic rendezvous": every drone is re-observed by the
# others in line-of-sight, so the ESIKF (which keeps refining the extrinsic
# whenever a teammate is seen -- MultiUAV/laserMapping, NOT gated by
# found_all_teammates) can pull a starved extrinsic (bot3) back to truth.
REGROUP_SLOT = 17.0
REGROUP_LOOP_T = 11.0
GLOBAL_TIMEOUT = T_WARM_END + TOUR_MAX + N_BOTS * REGROUP_SLOT + 40.0

# slot i -> this drone, with a strictly decreasing radius so the LAST slot has
# the SMALLEST loop (the fishbot bot5 failure, inverted)
LOOP_R = {"bot1": 1.00, "bot2": 0.90, "bot3": 0.80, "bot4": 0.70, "bot5": 0.60}

# --- inter-drone avoidance ------------------------------------------------
# a quad is 0.397 m circumscribed, so two touching is 0.79 m
PRIO = {"bot5": 0, "bot1": 1, "bot2": 2, "bot3": 3, "bot4": 4}  # tour length desc
YIELD_DIST = 1.30
HARD_STOP = 0.95
COLLISION = 0.80
ENCOUNTER_TH = 1.60
DEADLOCK_T = 5.0
BACKOFF_T = 2.5
BACKOFF_V = 0.15


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def cell(c, r):
    return (DEF["x0"] + c * DEF["pitch"], DEF["y0"] + r * DEF["pitch"])


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Swarm(Node):
    def __init__(self):
        super().__init__("drone_maze")
        os.makedirs(OUT_DIR, exist_ok=True)

        self.gt = {}
        self.slam_raw = {}
        self.origin = {}
        self.cloud = None
        self.traj = {b: [] for b in BOTS}
        self.dist_rows = []
        self.t0 = time.time()
        self.phase = "INIT"
        self.t_regroup = None            # set when REGROUP begins
        self.idx = {b: 0 for b in BOTS}
        self.done = {b: False for b in BOTS}
        self.blocked_since = {b: None for b in BOTS}
        self.backoff_until = {b: 0.0 for b in BOTS}
        self.encounters = 0
        self._enc_open = set()

        gm = GridMap(DEF["boxes"], DEF["bounds"], inflation=INFLATION, res=RES)
        cc = DEF["corner_cells"]
        # one sector each, all distinct, every corridor flown through up close
        tours = {
            "bot1": [cell(5, 3), cell(*cc["tr"]), cell(3, 5)],
            "bot2": [cell(3, 5), cell(*cc["tl"]), cell(1, 3)],
            "bot3": [cell(1, 3), cell(*cc["bl"]), cell(3, 1)],
            "bot4": [cell(3, 1), cell(*cc["br"]), cell(5, 3)],
            "bot5": [cell(*cc["tr"]), cell(*cc["tl"]), cell(*cc["bl"]), cell(*cc["br"])],
        }
        self.path = {}
        for b in BOTS:
            s = SPAWN[b][:2]
            p = gm.plan_tour(s, tours[b] + [s])
            if not p:
                raise SystemExit("no tour for %s" % b)
            self.path[b] = p
            plen = sum(math.hypot(p[i][0] - p[i-1][0], p[i][1] - p[i-1][1])
                       for i in range(1, len(p)))
            self.get_logger().info("%s tour: %d pts, %.1f m" % (b, len(p), plen))

        best = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(TFMessage, "/world/default/pose/info", self.on_gt, 50)
        for b in BOTS:
            self.create_subscription(
                Odometry, "/%s/lidar_slam/odom" % b,
                lambda m, bb=b: self.on_slam(m, bb), 20)
        self.create_subscription(PointCloud2, "/global_downsampled_map", self.on_cloud, best)
        self.cmd = {b: self.create_publisher(Twist, "/%s/cmd_vel" % b, 10) for b in BOTS}
        self.armp = {b: self.create_publisher(Bool, "/%s/enable" % b, 10) for b in BOTS}

        self.tfbuf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfbuf, self)
        self.timer = self.create_timer(1.0 / CTRL_HZ, self.step)

    # ---------------- callbacks ----------------
    def on_gt(self, msg):
        for tr in msg.transforms:
            if tr.child_frame_id in SPAWN:
                t, q = tr.transform.translation, tr.transform.rotation
                self.gt[tr.child_frame_id] = (t.x, t.y, t.z,
                                              yaw_from_quat(q.x, q.y, q.z, q.w))

    def on_slam(self, msg, b):
        p = msg.pose.pose.position
        self.slam_raw[b] = (p.x, p.y, p.z)

    def on_cloud(self, msg):
        self.cloud = msg

    def slam_world(self, b):
        if b not in self.slam_raw or b not in self.origin:
            return None
        ox, oy, oz, oyaw = self.origin[b]
        px, py, pz = self.slam_raw[b]
        c, s = math.cos(oyaw), math.sin(oyaw)
        return (ox + c * px - s * py, oy + s * px + c * py, oz + pz)

    # ---------------- low level ----------------
    def go(self, b, tx, ty, tz, vcap=V_MAX):
        x, y, z, yaw = self.gt[b]
        ex, ey, ez = tx - x, ty - y, tz - z
        d = math.hypot(ex, ey)
        v = min(vcap, KP_XY * d)
        vx_w, vy_w = (v * ex / d, v * ey / d) if d > 1e-6 else (0.0, 0.0)
        self.pub_body(b, vx_w, vy_w, max(-V_MAX_Z, min(V_MAX_Z, KP_Z * ez)))
        return d

    def pub_body(self, b, vx_w, vy_w, vz):
        x, y, z, yaw = self.gt[b]
        c, s = math.cos(yaw), math.sin(yaw)
        m = Twist()
        m.linear.x = c * vx_w + s * vy_w       # plugin takes BODY-frame velocity
        m.linear.y = -s * vx_w + c * vy_w
        m.linear.z = vz
        # hold the spawn heading: holonomic, so it strafes and never needs to turn
        m.angular.z = max(-0.6, min(0.6, 1.0 * wrap(SPAWN[b][2] - yaw)))
        self.cmd[b].publish(m)

    def hold(self, b, z):
        px, py = PARK[b]
        self.go(b, px, py, z, 0.4)

    def blocker(self, b, x, y):
        """Nearest drone ahead-ish that we must not hit. Holonomic, so 'ahead'
        is the direction of travel, not the heading."""
        best, bd = None, 1e9
        for o in BOTS:
            if o == b or o not in self.gt:
                continue
            ox, oy = self.gt[o][0], self.gt[o][1]
            d = math.hypot(ox - x, oy - y)
            if d > YIELD_DIST or d > bd:
                continue
            higher = PRIO[o] < PRIO[b]
            if d < HARD_STOP or higher:
                best, bd = o, d
        return best

    # ---------------- main loop ----------------
    def step(self):
        if len(self.gt) < N_BOTS:
            return
        now = time.time() - self.t0

        if self.phase == "INIT":
            if not self.origin:
                for b in BOTS:
                    self.origin[b] = self.gt[b]
                self.get_logger().info("SLAM origins captured (pre-takeoff truth)")
            for b in BOTS:
                self.cmd[b].publish(Twist())
            if now > T_INIT:
                self.get_logger().info("IMU init done -> arming all")
                self.phase = "ARM"
            return

        if self.phase == "ARM":
            for b in BOTS:
                self.armp[b].publish(Bool(data=True))
                self.cmd[b].publish(Twist())
            if now > T_ARM:
                self.get_logger().info("armed -> climbing to park altitude %.2f" % PARK_Z)
                self.phase = "CLIMB"
            return

        self.log(now)

        if self.phase == "CLIMB":
            for b in BOTS:
                self.hold(b, PARK_Z)
            if now > T_CLIMB:
                self.get_logger().info(
                    "WARM-UP: staggered solo loops, %.0f s each, radii %s"
                    % (SOLO_SLOT, [LOOP_R[b] for b in BOTS]))
                self.phase = "WARMUP"
            return

        if self.phase == "WARMUP":
            slot = int((now - T_CLIMB) // SOLO_SLOT)
            tau = (now - T_CLIMB) - slot * SOLO_SLOT
            for i, b in enumerate(BOTS):
                if i != slot:
                    self.hold(b, PARK_Z)      # parked low; mover is 0.35 m above
                    continue
                self.solo(b, tau)
            if now > T_WARM_END:
                self.get_logger().info("warm-up done -> tours")
                self.phase = "TOUR"
            return

        if self.phase == "TOUR":
            self.tour(now)
            if all(self.done.values()) or now > T_WARM_END + TOUR_MAX:
                self.t_regroup = now
                self.get_logger().info(
                    "tours done (%.0f s) -> REGROUP (re-observe in chamber)" % now)
                self.phase = "REGROUP"
            return

        if self.phase == "REGROUP":
            # compressed staggered-solo pass: one drone loops high in the chamber
            # while the rest park low, so every drone is cleanly re-observed.
            tr = now - self.t_regroup
            slot = int(tr // REGROUP_SLOT)
            tau = tr - slot * REGROUP_SLOT
            for i, b in enumerate(BOTS):
                if i == slot:
                    self.solo(b, tau, loop_t=REGROUP_LOOP_T)
                else:
                    self.hold(b, PARK_Z)
            if slot >= N_BOTS or now > GLOBAL_TIMEOUT:
                self.get_logger().info("regroup done (%.0f s) -> landing" % now)
                self.phase = "LAND"
            return

        if self.phase == "LAND":
            for b in BOTS:
                x, y, z, _ = self.gt[b]
                self.go(b, x, y, LAND_Z, 0.15)
            if all(self.gt[b][2] < 0.15 for b in BOTS):
                for b in BOTS:
                    self.cmd[b].publish(Twist())
                    self.armp[b].publish(Bool(data=False))
                self.timer.cancel()
                self.finish()
                raise SystemExit(0)

    def solo(self, b, tau, loop_t=LOOP_T):
        """This drone's slot. Climb high, fly ONE closed circle about the
        chamber centre, come back, drop. Everyone else is 0.35 m below."""
        sa = SPAWN[b][2]
        px, py = PARK[b]
        r = LOOP_R[b]
        if tau < 3.0:                      # climb in place, clear of the others
            self.go(b, px, py, SOLO_Z, 0.4)
        elif tau < 5.0:                    # slide in to the circle, on own spoke
            self.go(b, r * math.cos(sa), r * math.sin(sa), SOLO_Z, 0.5)
        elif tau < 5.0 + loop_t:           # the loop -- time-parameterised so it CLOSES
            s = (tau - 5.0) / loop_t
            a = sa + 2 * math.pi * s
            self.go(b, r * math.cos(a), r * math.sin(a), SOLO_Z, 0.6)
        elif tau < 5.0 + loop_t + 2.5:     # back out to the park spoke
            self.go(b, px, py, SOLO_Z, 0.5)
        else:                              # drop back to the parked layer
            self.go(b, px, py, PARK_Z, 0.4)

    def tour(self, now):
        for b in BOTS:
            x, y, z, yaw = self.gt[b]
            # phase-shifted z sine: mild 3D motion, decorrelated across drones
            cz = CRUISE_Z + Z_AMP * math.sin(
                2 * math.pi * now / Z_PERIOD + BOTS.index(b) * 2 * math.pi / N_BOTS)
            if self.done[b]:
                self.hold(b, 1.10)   # high above the cruise band; walls are 3 m
                continue
            p = self.path[b]
            i = self.idx[b]
            while i < len(p) - 1 and math.hypot(p[i][0] - x, p[i][1] - y) < LOOKAHEAD:
                i += 1
            self.idx[b] = i                 # monotonic carrot: the tour ends where
            tx, ty = p[i]                   # it starts, so nearest-point would say
            dgoal = math.hypot(p[-1][0] - x, p[-1][1] - y)   # "arrived" at t=0
            if i >= len(p) - 1 and dgoal < GOAL_TOL:
                self.done[b] = True
                self.get_logger().info("%s finished its tour at %.0f s" % (b, now))
                continue

            if now < self.backoff_until[b]:
                ex, ey = tx - x, ty - y
                d = math.hypot(ex, ey) or 1.0
                self.pub_body(b, -BACKOFF_V * ex / d, -BACKOFF_V * ey / d,
                              KP_Z * (cz - z))
                continue
            blk = self.blocker(b, x, y)
            if blk is not None:
                self.pub_body(b, 0.0, 0.0, KP_Z * (cz - z))
                if self.blocked_since[b] is None:
                    self.blocked_since[b] = now
                elif (now - self.blocked_since[b] > DEADLOCK_T
                      and (PRIO[blk] < PRIO[b] or self.done[blk])):
                    self.backoff_until[b] = now + BACKOFF_T
                    self.blocked_since[b] = None
                continue
            self.blocked_since[b] = None
            vcap = V_MAX * min(1.0, max(0.25, dgoal / SLOW_RADIUS)) \
                if i >= len(p) - 1 else V_MAX
            self.go(b, tx, ty, cz, vcap)

    # ---------------- logging ----------------
    def log(self, now):
        for b in BOTS:
            sw = self.slam_world(b)
            if sw is None:
                continue
            g = self.gt[b]
            self.traj[b].append((now, g[0], g[1], g[2], sw[0], sw[1], sw[2]))
        row = [now]
        for i in range(N_BOTS):
            for j in range(i + 1, N_BOTS):
                a, c = self.gt[BOTS[i]], self.gt[BOTS[j]]
                d = math.sqrt((a[0]-c[0])**2 + (a[1]-c[1])**2 + (a[2]-c[2])**2)
                row.append(d)
                key = (i, j)
                if d < ENCOUNTER_TH and key not in self._enc_open:
                    self._enc_open.add(key)
                    self.encounters += 1
                elif d > ENCOUNTER_TH * 1.2 and key in self._enc_open:
                    self._enc_open.discard(key)
        self.dist_rows.append(row)

    # ---------------- outputs ----------------
    def finish(self):
        self.save_cloud()
        self.save_traj()
        self.save_extrinsics()
        self.save_dist()
        print("Outputs in %s" % OUT_DIR, flush=True)

    def save_cloud(self):
        if self.cloud is None:
            print("WARNING: no fused map on /global_downsampled_map", flush=True)
            return
        # KEEP the intensity: map_fusion writes base_int = 100*(uid-1) there, so
        # it is the only record of which drone contributed which point.
        fields = [f.name for f in self.cloud.fields]
        want = ("x", "y", "z", "intensity") if "intensity" in fields else ("x", "y", "z")
        pts = list(point_cloud2.read_points(self.cloud, field_names=want, skip_nans=True))
        p = os.path.join(OUT_DIR, "global_map.ply")
        with open(p, "w") as f:
            f.write("ply\nformat ascii 1.0\nelement vertex %d\n" % len(pts))
            f.write("property float x\nproperty float y\nproperty float z\n")
            if len(want) == 4:
                f.write("property float intensity\n")
            f.write("end_header\n")
            for q in pts:
                f.write(" ".join("%f" % float(v) for v in q) + "\n")
        print("saved fused map: %d points (%s) -> %s" % (len(pts), ",".join(want), p),
              flush=True)
        if len(want) == 4:
            from collections import Counter
            cnt = Counter(int(round(float(q[3]) / 100.0)) + 1 for q in pts)
            print("  per-drone contributions: " +
                  "  ".join("bot%d=%d" % (k, v) for k, v in sorted(cnt.items())),
                  flush=True)

    def save_traj(self):
        lines = ["Per-drone SLAM accuracy (LIO, independent of fusion)", "=" * 62]
        for b in BOTS:
            rows = self.traj[b]
            with open(os.path.join(OUT_DIR, "traj_%s.csv" % b), "w") as f:
                f.write("t,gt_x,gt_y,gt_z,slam_x,slam_y,slam_z\n")
                for r in rows:
                    f.write("%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n" % r)
            if not rows:
                lines.append("%s: NO SLAM ODOM" % b)
                continue
            e3 = [math.sqrt((r[1]-r[4])**2 + (r[2]-r[5])**2 + (r[3]-r[6])**2) for r in rows]
            e2 = [math.hypot(r[1]-r[4], r[2]-r[5]) for r in rows]
            rms = lambda v: math.sqrt(sum(x*x for x in v) / len(v))
            lines.append("%s: ATE3D=%.3f m  ATE2D=%.3f m  max=%.3f m  n=%d"
                         % (b, rms(e3), rms(e2), max(e3), len(rows)))
        with open(os.path.join(OUT_DIR, "ate_report.txt"), "w") as f:
            f.write("\n".join(lines) + "\n")
        print("\n".join(lines), flush=True)

    def save_extrinsics(self):
        """The metric that actually decides fusion. Compare the TF
        map_origin -> bot{k}/world that Swarm-LIO2 estimated against truth.
        map_origin is bot1's SLAM start frame == bot1's spawn pose."""
        lines = ["Inter-drone extrinsics (map_origin -> botK/world) vs truth",
                 "map_origin == bot1 spawn. This is what fusion stands or falls on.",
                 "=" * 62]
        ox, oy, oyaw = SPAWN["bot1"]
        ok = 0
        for b in BOTS[1:]:
            sx, sy, syaw = SPAWN[b]
            # truth: bot1_spawn^-1 * botK_spawn
            dx, dy = sx - ox, sy - oy
            c, s = math.cos(-oyaw), math.sin(-oyaw)
            tx, ty = c * dx - s * dy, s * dx + c * dy
            tyaw = wrap(syaw - oyaw)
            try:
                tf = self.tfbuf.lookup_transform(
                    "map_origin", "%s/world" % b, rclpy.time.Time())
            except Exception as e:
                lines.append("%s: NO TF -> not estimated, its points are dropped "
                             "from the fused map  (%s)" % (b, type(e).__name__))
                continue
            t, q = tf.transform.translation, tf.transform.rotation
            eyaw = yaw_from_quat(q.x, q.y, q.z, q.w)
            dyaw = math.degrees(abs(wrap(eyaw - tyaw)))
            dpos = math.hypot(t.x - tx, t.y - ty)
            good = dyaw < 5.0 and dpos < 0.3
            ok += good
            lines.append("%s: yaw est %7.2f deg vs true %7.2f -> err %5.2f deg | "
                         "xy err %.3f m  %s"
                         % (b, math.degrees(eyaw), math.degrees(tyaw), dyaw, dpos,
                            "OK" if good else "WRONG"))
        lines.append("-" * 62)
        lines.append("%d/%d teammate extrinsics correct" % (ok, len(BOTS) - 1))
        with open(os.path.join(OUT_DIR, "extrinsics_report.txt"), "w") as f:
            f.write("\n".join(lines) + "\n")
        print("\n".join(lines), flush=True)

    def save_dist(self):
        hdr = ["t"] + ["%s-%s" % (BOTS[i], BOTS[j])
                       for i in range(N_BOTS) for j in range(i + 1, N_BOTS)]
        with open(os.path.join(OUT_DIR, "dist_pairs.csv"), "w") as f:
            f.write(",".join(hdr) + "\n")
            for r in self.dist_rows:
                f.write(",".join("%.4f" % v for v in r) + "\n")
        mn = min(min(r[1:]) for r in self.dist_rows) if self.dist_rows else float("nan")
        print("min inter-drone distance %.3f m (contact would be %.2f m)  %s   "
              "| close encounters: %d"
              % (mn, COLLISION, "NO COLLISION" if mn > COLLISION else "COLLISION",
                 self.encounters), flush=True)


def main():
    rclpy.init()
    n = Swarm()
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
