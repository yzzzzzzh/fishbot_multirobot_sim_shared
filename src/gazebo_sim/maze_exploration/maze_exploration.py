#!/usr/bin/env python3
"""
5-robot coordinated exploration of the generated maze world.

Each robot follows a collision-free A* tour that repeatedly returns to the
central chamber and shares corners with its neighbours, so the swarm meets
several times during the run. A two-tier avoidance rule keeps them apart:
  * YIELD:      a robot stops when a *higher-priority* (lower-id) neighbour is
                close and ahead;
  * HARD_STOP:  ANY robot stops when *any* neighbour is within a hard radius
                ahead, regardless of priority.
This produces close encounters (good for Swarm-LIO2 mutual localisation) that
stay bounded away from collision.

Feedback = Gazebo ground truth (pose/info -> TF). Outputs to ./output/:
  global_map.ply, traj_botN.csv, dist_pairs.csv, ate_report.txt, encounters.txt
"""
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage

from gridmap import GridMap

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "output")
DEF = json.load(open(os.path.join(HERE, "maze_def.json")))

PITCH, X0, Y0 = DEF["pitch"], DEF["x0"], DEF["y0"]
BOTS = [f"bot{i}" for i in range(1, DEF["count"] + 1)]
SPAWN = {b: tuple(DEF["spawns"][b]) for b in BOTS}     # (x, y, yaw)


def cell(i, j):
    return (X0 + i * PITCH, Y0 + j * PITCH)


def warmup_solo(bot, spawn_x, spawn_y, n_spread=8, n_loop=60):
    """Fan out along this robot's spoke, then a closed loop of its own radius.

    Returns (points, n_spread) so the caller knows where the spread ends and the
    solo loop begins -- the two are gated on different times.
    """
    a0 = math.atan2(spawn_y, spawn_x)
    ca, sa = math.cos(a0), math.sin(a0)
    r0 = math.hypot(spawn_x, spawn_y)
    pts = []
    for k in range(1, n_spread + 1):            # fan out r0 -> SPREAD_R
        rr = r0 + (SPREAD_R - r0) * k / n_spread
        pts.append((rr * ca, rr * sa))
    # local loop: circle of radius rho whose centre sits rho inboard of the
    # parked spot, so the robot starts on the loop and never leaves the chamber.
    rho = LOOP_RADII[bot]
    cx, cy = (SPREAD_R - rho) * ca, (SPREAD_R - rho) * sa
    for k in range(1, n_loop + 1):
        phi = a0 + 2 * math.pi * k / n_loop
        pts.append((cx + rho * math.cos(phi), cy + rho * math.sin(phi)))
    return pts, n_spread


# Tours as cell waypoints. "S" = return to own spawn (spreads the chamber
# rendezvous). Corners/edges are shared between robots to force meetings.
C = DEF["corner_cells"]
bl, br, tl, tr = C["bl"], C["br"], C["tl"], C["tr"]
TOUR_CELLS = {
    "bot1": [tr, "S", bl, "S"],
    "bot2": [tl, "S", br, "S"],
    "bot3": [[5, 3], [3, 5], [1, 3], "S"],
    "bot4": [[3, 1], br, tr, "S"],
    "bot5": [[1, 3], bl, [3, 1], "S"],
}

# --- controller gains ---------------------------------------------------------
V_MAX = 0.32
W_MAX = 1.2
LOOKAHEAD = 0.45
GOAL_TOL = 0.15          # tight: every robot must come back to its spawn spot

# --- warm-up: STAGGERED solo loops --------------------------------------------
# Swarm-LIO2 locks a teammate id the FIRST time a tracker matches, and never
# revisits it (MultiUAV.cpp: `iter_tracker != teammate_tracker.end() -> continue`).
# So an early ambiguous match is permanent. Earlier attempts:
#   * carousel (same circle for all)  -> congruent paths  -> ids swapped;
#   * radial spokes                   -> motion along the line of sight, the
#     sigma2 excitation gate (traj_matching_start_thresh = 8.0) never fires;
#   * concentric circles, distinct radii -> all pairs clear sigma2 by 6-100x,
#     2/4 ids correct -- the rest still locked in wrong during the first seconds,
#     when short arcs still look alike.
# Fix: make the FIRST seconds unambiguous. Each robot loops ALONE in its slot
# while the others sit still, so there is exactly one moving cluster to match.
# Loop radii differ per robot as a second, independent disambiguator.
SPREAD_R = 1.7           # robots first fan out to this radius (spacing 2.0 m)
SPREAD_T = 5.0
LOOP_RADII = {"bot1": 0.55, "bot2": 0.60, "bot3": 0.65, "bot4": 0.70, "bot5": 0.75}
SOLO_SLOT = 22.0         # s per robot (covers the slowest loop under pursuit)
# everyone stays in the chamber watching until all five slots are done
DISPERSE_T = 12.0 + SPREAD_T + 5 * SOLO_SLOT   # START_DELAY + spread + 5 slots
SLOW_RADIUS = 0.55
ROTATE_THRESH = 1.1
CTRL_HZ = 20.0
START_DELAY = 12.0          # let IMU / SLAM initialise (robots sit + see each other)
GLOBAL_TIMEOUT = 420.0

# --- two-tier inter-robot avoidance ------------------------------------------
YIELD_DIST = 0.70           # yield to higher-priority neighbour ahead
HARD_STOP = 0.50            # anyone stops if a neighbour is this close ahead
AHEAD_DOT = 0.0
ENCOUNTER_TH = 1.20         # report a "close encounter" below this pair distance
INFLATION = 0.25            # robot radius (0.15) + margin

# Deadlock breaker: a head-on meeting in a corridor freezes both robots (the
# low-priority one yields, the high-priority one hard-stops because the other is
# ahead). After DEADLOCK_T of being blocked, the LOWER-priority robot reverses
# briefly to open a gap so the higher-priority one can pass.
DEADLOCK_T = 5.0
BACKOFF_T = 2.5
BACKOFF_V = -0.12


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class MazeExplorer(Node):
    def __init__(self):
        super().__init__("maze_exploration")
        os.makedirs(OUT_DIR, exist_ok=True)

        self.grid = GridMap(DEF["boxes"], DEF["bounds"], inflation=INFLATION)
        self.get_logger().info("Planning: 10 s warm-up circles (distinct radii) + A* tour ...")
        self.paths = {}
        self.spread_end = {}
        self.loop_end = {}
        self.slot_start = {}
        for b in BOTS:
            sx, sy, _ = SPAWN[b]
            # 1) warm-up (explicit points -- must NOT be smoothed away)
            arc, n_spread = warmup_solo(b, sx, sy)
            bad = [p for p in arc if not self.grid.is_free(*self.grid.world_to_grid(*p))]
            if bad:
                self.get_logger().warn(f"  {b}: {len(bad)} warm-up pts not free!")
            # 2) tour from the end of the arc, always finishing back at the spawn spot
            wps = [(sx, sy) if w == "S" else cell(*w) for w in TOUR_CELLS[b]]
            if wps[-1] != (sx, sy):
                wps.append((sx, sy))
            tail = self.grid.plan_tour(arc[-1], wps)
            self.paths[b] = [(sx, sy)] + arc + tail[1:]
            self.spread_end[b] = 1 + n_spread      # end of the fan-out segment
            self.loop_end[b] = 1 + len(arc)        # end of the solo loop
            k = BOTS.index(b)
            self.slot_start[b] = START_DELAY + SPREAD_T + k * SOLO_SLOT
            self.get_logger().info(
                f"  {b}: {len(self.paths[b])} pts | solo loop r={LOOP_RADII[b]} m "
                f"in slot t={self.slot_start[b]:.0f}-{self.slot_start[b]+SOLO_SLOT:.0f}s "
                f"| tour {len(tail)}")

        self.gt, self.slam = {}, {}
        self.idx = {b: 0 for b in BOTS}
        self.done = {b: (not self.paths[b]) for b in BOTS}
        self.traj = {b: [] for b in BOTS}
        self.dist_log = []          # (t, {pair: d})
        self.cloud = None
        self.t0 = time.time()
        self.blocked_since = {b: None for b in BOTS}
        self.backoff_until = {b: 0.0 for b in BOTS}

        self.cmd = {b: self.create_publisher(Twist, f"/{b}/cmd_vel", 10) for b in BOTS}
        best = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(TFMessage, "/world/default/pose/info", self.on_gt, best)
        for b in BOTS:
            self.create_subscription(Odometry, f"/{b}/lidar_slam/odom",
                                     lambda m, bb=b: self.on_slam(m, bb), best)
        self.create_subscription(PointCloud2, "/global_downsampled_map", self.on_cloud, best)
        self.timer = self.create_timer(1.0 / CTRL_HZ, self.step)
        self.get_logger().info(f"Running. Motion starts after {START_DELAY:.0f}s SLAM init.")

    def on_gt(self, msg):
        for tr in msg.transforms:
            if tr.child_frame_id in SPAWN:
                t, q = tr.transform.translation, tr.transform.rotation
                self.gt[tr.child_frame_id] = (t.x, t.y, yaw_from_quat(q.x, q.y, q.z, q.w))

    def on_slam(self, msg, bot):
        sx, sy, syaw = SPAWN[bot]
        px, py = msg.pose.pose.position.x, msg.pose.pose.position.y
        c, s = math.cos(syaw), math.sin(syaw)
        self.slam[bot] = (sx + c * px - s * py, sy + s * px + c * py)

    def on_cloud(self, msg):
        self.cloud = msg

    def step(self):
        now = time.time() - self.t0

        # log pairwise distances (ground truth)
        if all(b in self.gt for b in BOTS):
            d = {}
            for i in range(len(BOTS)):
                for j in range(i + 1, len(BOTS)):
                    a, b = BOTS[i], BOTS[j]
                    d[f"{a}-{b}"] = math.hypot(self.gt[a][0] - self.gt[b][0],
                                               self.gt[a][1] - self.gt[b][1])
            self.dist_log.append((now, d))

        for b in BOTS:
            if b not in self.gt:
                continue
            x, y, th = self.gt[b]
            if b in self.slam:
                self.traj[b].append((now, x, y, self.slam[b][0], self.slam[b][1]))

            if self.done[b] or now < START_DELAY:
                self.cmd[b].publish(Twist())
                continue

            # --- staggering gates -------------------------------------------
            # Hold BEFORE advancing the carrot, otherwise a parked robot would
            # silently consume the loop points lying within LOOKAHEAD of it.
            i = self.idx[b]
            if self.spread_end[b] <= i < self.loop_end[b] and now < self.slot_start[b]:
                self.cmd[b].publish(Twist())        # fanned out; waiting my solo slot
                continue
            if i >= self.loop_end[b] and now < DISPERSE_T:
                self.cmd[b].publish(Twist())        # my loop is done; watching the others
                continue

            path = self.paths[b]
            # monotonic carrot: consume points within LOOKAHEAD, index never
            # decreases and cannot jump across a loop-back to the final point.
            while i < len(path) - 1 and math.hypot(path[i][0] - x, path[i][1] - y) < LOOKAHEAD:
                i += 1
            self.idx[b] = i
            tx, ty = path[i]

            dgoal = math.hypot(path[-1][0] - x, path[-1][1] - y)
            if i >= len(path) - 1 and dgoal < GOAL_TOL:
                self.done[b] = True
                self.cmd[b].publish(Twist())
                self.get_logger().info(f"{b} finished tour.")
                continue
            alpha = wrap(math.atan2(ty - y, tx - x) - th)

            if abs(alpha) > ROTATE_THRESH:
                v, w = 0.0, max(-W_MAX, min(W_MAX, 2.0 * alpha))
            else:
                v = V_MAX * max(0.25, 1.0 - abs(alpha) / ROTATE_THRESH)
                if dgoal < SLOW_RADIUS:
                    v *= max(0.2, dgoal / SLOW_RADIUS)
                w = max(-W_MAX, min(W_MAX, v * 2.0 * math.sin(alpha) / LOOKAHEAD))

            # --- inter-robot avoidance with deadlock breaking ---
            if now < self.backoff_until[b]:
                v, w = BACKOFF_V, 0.0          # reversing to clear a head-on jam
            else:
                blocker = self._blocker(b, x, y, th)
                if blocker is not None:
                    v, w = 0.0, 0.0
                    if self.blocked_since[b] is None:
                        self.blocked_since[b] = now
                    elif (now - self.blocked_since[b] > DEADLOCK_T
                          and (BOTS.index(blocker) < BOTS.index(b) or self.done[blocker])):
                        # Back off if we are the lower-priority one, OR if the
                        # blocker has finished and parked -- it will never move,
                        # so waiting for it would freeze us forever.
                        self.backoff_until[b] = now + BACKOFF_T
                        self.blocked_since[b] = None
                        self.get_logger().info(
                            f"{b}: deadlock with {blocker} -> backing off")
                else:
                    self.blocked_since[b] = None

            m = Twist(); m.linear.x = v; m.angular.z = w
            self.cmd[b].publish(m)

        if all(self.done[b] for b in BOTS) or now > GLOBAL_TIMEOUT:
            self.finish()

    def _blocker(self, b, x, y, th):
        """Return the neighbour we must stop for, or None."""
        prio = BOTS.index(b)
        for k, other in enumerate(BOTS):
            if other == b or other not in self.gt:
                continue
            ox, oy, _ = self.gt[other]
            dx, dy = ox - x, oy - y
            dist = math.hypot(dx, dy)
            ahead = (math.cos(th) * dx + math.sin(th) * dy) / (dist + 1e-6) > AHEAD_DOT
            if dist < HARD_STOP and ahead:      # anyone stops for anyone very close ahead
                return other
            if k < prio and dist < YIELD_DIST and ahead:   # yield to higher priority
                return other
        return None

    def finish(self):
        self.timer.cancel()
        for b in BOTS:
            for _ in range(5):
                self.cmd[b].publish(Twist())
        self.get_logger().info("Saving results ...")
        self._save_cloud(); self._save_traj_ate(); self._save_distances()
        self.get_logger().info(f"Done. Outputs in {OUT_DIR}")
        rclpy.shutdown()

    def _save_cloud(self):
        if self.cloud is None:
            self.get_logger().warn("No fused map received.")
            return
        pts = list(point_cloud2.read_points(self.cloud, field_names=("x", "y", "z"),
                                            skip_nans=True))
        p = os.path.join(OUT_DIR, "global_map.ply")
        with open(p, "w") as f:
            f.write(f"ply\nformat ascii 1.0\nelement vertex {len(pts)}\n")
            f.write("property float x\nproperty float y\nproperty float z\nend_header\n")
            for q in pts:
                f.write(f"{float(q[0])} {float(q[1])} {float(q[2])}\n")
        self.get_logger().info(f"Saved fused map: {len(pts)} points.")

    def _save_traj_ate(self):
        lines = ["ATE (SLAM vs ground truth)  +  return-to-spawn accuracy", "=" * 58]
        for b in BOTS:
            rows = self.traj[b]
            sx, sy, _ = SPAWN[b]
            back = math.hypot(rows[-1][1] - sx, rows[-1][2] - sy) if rows else float("nan")
            with open(os.path.join(OUT_DIR, f"traj_{b}.csv"), "w") as f:
                f.write("t,gt_x,gt_y,slam_x,slam_y\n")
                for t, gx, gy, sx, sy in rows:
                    f.write(f"{t:.3f},{gx:.4f},{gy:.4f},{sx:.4f},{sy:.4f}\n")
            errs = [math.hypot(gx - sx, gy - sy) for _, gx, gy, sx, sy in rows]
            if errs:
                ate = math.sqrt(sum(e * e for e in errs) / len(errs))
                plen = sum(math.hypot(rows[i][1] - rows[i-1][1], rows[i][2] - rows[i-1][2])
                           for i in range(1, len(rows)))
                lines.append(f"{b}: ATE={ate:.3f} m  max={max(errs):.3f} m  "
                             f"path={plen:.1f} m  back-to-spawn={back:.3f} m "
                             f"{'OK' if back < 0.25 else 'NOT RETURNED'}")
        with open(os.path.join(OUT_DIR, "ate_report.txt"), "w") as f:
            f.write("\n".join(lines) + "\n")
        self.get_logger().info("\n" + "\n".join(lines))

    def _save_distances(self):
        if not self.dist_log:
            return
        pairs = list(self.dist_log[0][1].keys())
        with open(os.path.join(OUT_DIR, "dist_pairs.csv"), "w") as f:
            f.write("t," + ",".join(pairs) + "\n")
            for t, d in self.dist_log:
                f.write(f"{t:.2f}," + ",".join(f"{d[p]:.3f}" for p in pairs) + "\n")

        # encounter events: contiguous intervals where a pair stays < threshold
        report = ["Close-encounter events (pair distance < "
                  f"{ENCOUNTER_TH:.2f} m)", "=" * 46]
        global_min = float("inf")
        n_events = 0
        for p in pairs:
            series = [(t, d[p]) for t, d in self.dist_log]
            gmin = min(v for _, v in series)
            global_min = min(global_min, gmin)
            active, tmin, dmin = False, 0, 0
            for t, v in series:
                if v < ENCOUNTER_TH and not active:
                    active, tmin, dmin = True, t, v
                elif v < ENCOUNTER_TH:
                    dmin = min(dmin, v)
                elif active:
                    report.append(f"  {p}: t={tmin:5.1f}-{t:5.1f}s  min={dmin:.2f} m")
                    n_events += 1
                    active = False
            if active:
                report.append(f"  {p}: t={tmin:5.1f}s..end  min={dmin:.2f} m")
                n_events += 1
        report.append("-" * 46)
        report.append(f"total encounters: {n_events}   "
                      f"global min pair distance: {global_min:.2f} m "
                      f"({'NO collision' if global_min > 0.30 else 'COLLISION!'})")
        with open(os.path.join(OUT_DIR, "encounters.txt"), "w") as f:
            f.write("\n".join(report) + "\n")
        self.get_logger().info("\n" + "\n".join(report))


def main():
    rclpy.init()
    node = MazeExplorer()
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
