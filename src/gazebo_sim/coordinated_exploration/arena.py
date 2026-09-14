#!/usr/bin/env python3
"""
Arena model + occupancy grid + A* planner for fishbot2.world.

All geometry is transcribed from src/gazebo_sim/worlds/fishbot2.world and lives in
the world (arena) frame — the same frame the ground-truth pose/info reports in.

The grid is *inflated* by the robot radius plus a safety margin, so a cell being
"free" means the robot centre may legally sit there. A* therefore returns paths
that already keep clearance from every wall, pillar and block.
"""
import heapq
import math

import numpy as np

# --- Robot footprint (from robots/fishbot_v2_3d/config/fishbot_v2_3d.yaml) ------
BASE_RADIUS = 0.10          # chassis cylinder radius
WHEEL_HALF = 0.10           # wheels sit at y = +/-0.10 -> effective half-width
ROBOT_RADIUS = 0.15         # conservative disc that encloses base + wheels
SAFETY_MARGIN = 0.08        # extra clearance to walls
INFLATION = ROBOT_RADIUS + SAFETY_MARGIN   # 0.23 m

# --- Grid extents (a little past the outer wall) -------------------------------
X_MIN, X_MAX = -6.6, 6.6
Y_MIN, Y_MAX = -6.6, 6.6
RES = 0.05                  # metres per cell


def _box(cx, cy, sx, sy, yaw):
    return dict(kind="box", cx=cx, cy=cy, sx=sx, sy=sy, yaw=yaw)


def _cyl(cx, cy, r):
    return dict(kind="cyl", cx=cx, cy=cy, r=r)


# Obstacles, transcribed verbatim from fishbot2.world (centre, size, yaw).
OBSTACLES = [
    # Outer rounded-diamond wall (10 segments)
    _box(2, 5.5, 4.123105626, 0.25, -0.244978663),
    _box(5, 3.75, 3.20156212, 0.25, -0.896055385),
    _box(6, 0, 5, 0.25, -1.570796327),
    _box(5, -3.75, 3.20156212, 0.25, -2.245537269),
    _box(2, -5.5, 4.123105626, 0.25, -2.89661399),
    _box(-2, -5.5, 4.123105626, 0.25, 2.89661399),
    _box(-5, -3.75, 3.20156212, 0.25, 2.245537269),
    _box(-6, 0, 5, 0.25, 1.570796327),
    _box(-5, 3.75, 3.20156212, 0.25, 0.896055385),
    _box(-2, 5.5, 4.123105626, 0.25, 0.244978663),
    # Central divider broken into 5 segments -> 4 tunnels at y ~ +/-1.1, +/-3.3
    _box(0, 4.4, 1.4, 0.25, 1.570796327),
    _box(0, 2.2, 1.4, 0.25, 1.570796327),
    _box(0, 0.0, 1.4, 0.25, 1.570796327),
    _box(0, -2.2, 1.4, 0.25, 1.570796327),
    _box(0, -4.4, 1.4, 0.25, 1.570796327),
    # Side inset walls
    _box(-2.7, 2.7, 3, 0.2, 0.45),
    _box(-2.7, -2.7, 3, 0.2, -0.45),
    _box(2.7, 2.7, 3, 0.2, -0.45),
    _box(2.7, -2.7, 3, 0.2, 0.45),
    # Cylindrical pillars (left room, x = -1)
    _cyl(-1, 3.6, 0.25),
    _cyl(-1, 1.2, 0.25),
    _cyl(-1, -1.2, 0.25),
    _cyl(-1, -3.6, 0.25),
    # Square blocks (right room)
    _box(1.6, 3, 0.8, 0.8, 0.0),
    _box(1.6, -3, 0.8, 0.8, 0.0),
]

# Arena boundary polygon (wall centre-lines); anything outside is not drivable.
ARENA_POLYGON = [
    (6, 2.5), (4, 5), (0, 6), (-4, 5), (-6, 2.5),
    (-6, -2.5), (-4, -5), (0, -6), (4, -5), (6, -2.5),
]


class Arena:
    """Inflated occupancy grid + A* planner in the world frame."""

    def __init__(self, inflation=INFLATION, res=RES):
        self.res = res
        self.inflation = inflation
        self.nx = int(round((X_MAX - X_MIN) / res))
        self.ny = int(round((Y_MAX - Y_MIN) / res))
        self.occ = np.zeros((self.ny, self.nx), dtype=bool)
        self._build()

    # --- coordinate helpers ---------------------------------------------------
    def world_to_grid(self, x, y):
        gx = int((x - X_MIN) / self.res)
        gy = int((y - Y_MIN) / self.res)
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = X_MIN + (gx + 0.5) * self.res
        y = Y_MIN + (gy + 0.5) * self.res
        return x, y

    def in_bounds(self, gx, gy):
        return 0 <= gx < self.nx and 0 <= gy < self.ny

    def is_free(self, gx, gy):
        return self.in_bounds(gx, gy) and not self.occ[gy, gx]

    # --- grid construction ----------------------------------------------------
    def _build(self):
        xs = X_MIN + (np.arange(self.nx) + 0.5) * self.res
        ys = Y_MIN + (np.arange(self.ny) + 0.5) * self.res
        gx, gy = np.meshgrid(xs, ys)            # (ny, nx)
        inf = self.inflation

        # Mark cells inside any inflated obstacle.
        for ob in OBSTACLES:
            if ob["kind"] == "box":
                c, s = math.cos(-ob["yaw"]), math.sin(-ob["yaw"])
                dx = gx - ob["cx"]
                dy = gy - ob["cy"]
                lx = c * dx - s * dy
                ly = s * dx + c * dy
                hit = (np.abs(lx) <= ob["sx"] / 2 + inf) & (np.abs(ly) <= ob["sy"] / 2 + inf)
            else:  # cylinder
                d = np.hypot(gx - ob["cx"], gy - ob["cy"])
                hit = d <= ob["r"] + inf
            self.occ |= hit

        # Mark everything outside the (shrunk) arena polygon as occupied.
        inside = self._points_in_polygon(gx.ravel(), gy.ravel(),
                                         self._shrink_polygon(ARENA_POLYGON, inf + 0.125))
        outside = ~inside.reshape(self.occ.shape)
        self.occ |= outside

    @staticmethod
    def _shrink_polygon(poly, amount):
        """Move each vertex toward the centroid by `amount` metres (approx inset)."""
        cx = sum(p[0] for p in poly) / len(poly)
        cy = sum(p[1] for p in poly) / len(poly)
        out = []
        for x, y in poly:
            dx, dy = x - cx, y - cy
            d = math.hypot(dx, dy)
            k = max(0.0, (d - amount) / d) if d > 1e-6 else 0.0
            out.append((cx + dx * k, cy + dy * k))
        return out

    @staticmethod
    def _points_in_polygon(px, py, poly):
        """Vectorised ray-casting point-in-polygon test."""
        n = len(poly)
        inside = np.zeros(px.shape, dtype=bool)
        j = n - 1
        for i in range(n):
            xi, yi = poly[i]
            xj, yj = poly[j]
            cond = ((yi > py) != (yj > py)) & (
                px < (xj - xi) * (py - yi) / (yj - yi + 1e-12) + xi)
            inside ^= cond
            j = i
        return inside

    # --- nearest free cell (snap goals/starts out of inflation) ---------------
    def nearest_free(self, x, y, max_r=1.0):
        gx, gy = self.world_to_grid(x, y)
        if self.is_free(gx, gy):
            return gx, gy
        max_cells = int(max_r / self.res)
        for r in range(1, max_cells + 1):
            for dx in range(-r, r + 1):
                for dy in (-r, r):
                    if self.is_free(gx + dx, gy + dy):
                        return gx + dx, gy + dy
            for dy in range(-r, r + 1):
                for dx in (-r, r):
                    if self.is_free(gx + dx, gy + dy):
                        return gx + dx, gy + dy
        return None

    # --- A* -------------------------------------------------------------------
    _NEIGH = [(-1, 0), (1, 0), (0, -1), (0, 1),
              (-1, -1), (-1, 1), (1, -1), (1, 1)]

    def plan(self, start_xy, goal_xy):
        """A* from start to goal (world coords). Returns list[(x,y)] or None."""
        s = self.nearest_free(*start_xy)
        g = self.nearest_free(*goal_xy)
        if s is None or g is None:
            return None
        openq = [(0.0, s)]
        came = {s: None}
        gcost = {s: 0.0}
        while openq:
            _, cur = heapq.heappop(openq)
            if cur == g:
                return self._reconstruct(came, cur)
            cx, cy = cur
            for dx, dy in self._NEIGH:
                nx, ny = cx + dx, cy + dy
                if not self.is_free(nx, ny):
                    continue
                step = math.hypot(dx, dy)
                ng = gcost[cur] + step
                nb = (nx, ny)
                if ng < gcost.get(nb, math.inf):
                    gcost[nb] = ng
                    h = math.hypot(nx - g[0], ny - g[1])
                    heapq.heappush(openq, (ng + h, nb))
                    came[nb] = cur
        return None

    def _reconstruct(self, came, cur):
        cells = []
        while cur is not None:
            cells.append(cur)
            cur = came[cur]
        cells.reverse()
        return [self.grid_to_world(gx, gy) for gx, gy in cells]

    # --- line of sight on the inflated grid -----------------------------------
    def line_free(self, a, b):
        (x0, y0), (x1, y1) = a, b
        n = int(math.hypot(x1 - x0, y1 - y0) / (self.res * 0.5)) + 1
        for i in range(n + 1):
            t = i / n
            gx, gy = self.world_to_grid(x0 + t * (x1 - x0), y0 + t * (y1 - y0))
            if not self.is_free(gx, gy):
                return False
        return True

    # --- smoothing: string-pull shortcut, then resample -----------------------
    def smooth(self, path, resample=0.10):
        if not path or len(path) < 3:
            return path
        out = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not self.line_free(path[i], path[j]):
                j -= 1
            out.append(path[j])
            i = j
        return self._resample(out, resample)

    @staticmethod
    def _resample(poly, step):
        if len(poly) < 2:
            return poly
        dense = [poly[0]]
        for (x0, y0), (x1, y1) in zip(poly, poly[1:]):
            seg = math.hypot(x1 - x0, y1 - y0)
            k = max(1, int(seg / step))
            for m in range(1, k + 1):
                t = m / k
                dense.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
        return dense

    def plan_tour(self, start_xy, waypoints):
        """Plan through a list of waypoints; concatenate + smooth each leg."""
        full = []
        cur = start_xy
        for wp in waypoints:
            leg = self.plan(cur, wp)
            if leg is None:
                # Skip an unreachable waypoint but keep going.
                continue
            leg = self.smooth(leg)
            if full:
                leg = leg[1:]
            full.extend(leg)
            cur = full[-1] if full else cur
        return full


if __name__ == "__main__":
    # Quick self-test: build grid, report free-space fraction, plan one tour.
    a = Arena()
    free = 1.0 - a.occ.mean()
    print(f"grid {a.nx}x{a.ny}  free-space fraction = {free:.2%}")
    tour = a.plan_tour((-3.0, 1.0),
                       [(0.0, 1.1), (3.5, 2.5), (4.5, 3.8), (2.2, 4.5)])
    print(f"bot2 tour length = {len(tour)} pts, "
          f"end = {tour[-1] if tour else None}")
