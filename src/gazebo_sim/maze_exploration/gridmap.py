#!/usr/bin/env python3
"""Generic inflated occupancy grid + A* planner for axis-aligned box obstacles."""
import heapq
import math

import numpy as np


class GridMap:
    def __init__(self, boxes, bounds, inflation=0.25, res=0.05):
        """boxes: list of (cx, cy, sx, sy) axis-aligned. bounds: (xmin,xmax,ymin,ymax)."""
        self.xmin, self.xmax, self.ymin, self.ymax = bounds
        self.res = res
        self.inflation = inflation
        self.nx = int(round((self.xmax - self.xmin) / res))
        self.ny = int(round((self.ymax - self.ymin) / res))
        self.occ = np.zeros((self.ny, self.nx), dtype=bool)
        self._build(boxes)

    def _build(self, boxes):
        xs = self.xmin + (np.arange(self.nx) + 0.5) * self.res
        ys = self.ymin + (np.arange(self.ny) + 0.5) * self.res
        gx, gy = np.meshgrid(xs, ys)
        inf = self.inflation
        for cx, cy, sx, sy in boxes:
            hit = (np.abs(gx - cx) <= sx / 2 + inf) & (np.abs(gy - cy) <= sy / 2 + inf)
            self.occ |= hit

    # coordinate helpers
    def world_to_grid(self, x, y):
        return int((x - self.xmin) / self.res), int((y - self.ymin) / self.res)

    def grid_to_world(self, gx, gy):
        return self.xmin + (gx + 0.5) * self.res, self.ymin + (gy + 0.5) * self.res

    def in_bounds(self, gx, gy):
        return 0 <= gx < self.nx and 0 <= gy < self.ny

    def is_free(self, gx, gy):
        return self.in_bounds(gx, gy) and not self.occ[gy, gx]

    def nearest_free(self, x, y, max_r=1.2):
        gx, gy = self.world_to_grid(x, y)
        if self.is_free(gx, gy):
            return gx, gy
        for r in range(1, int(max_r / self.res) + 1):
            for dx in range(-r, r + 1):
                for dy in (-r, r):
                    if self.is_free(gx + dx, gy + dy):
                        return gx + dx, gy + dy
            for dy in range(-r, r + 1):
                for dx in (-r, r):
                    if self.is_free(gx + dx, gy + dy):
                        return gx + dx, gy + dy
        return None

    _NEIGH = [(-1, 0), (1, 0), (0, -1), (0, 1),
              (-1, -1), (-1, 1), (1, -1), (1, 1)]

    def plan(self, start_xy, goal_xy):
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
                cells = []
                while cur is not None:
                    cells.append(cur)
                    cur = came[cur]
                cells.reverse()
                return [self.grid_to_world(a, b) for a, b in cells]
            cx, cy = cur
            for dx, dy in self._NEIGH:
                nx, ny = cx + dx, cy + dy
                if not self.is_free(nx, ny):
                    continue
                ng = gcost[cur] + math.hypot(dx, dy)
                nb = (nx, ny)
                if ng < gcost.get(nb, math.inf):
                    gcost[nb] = ng
                    heapq.heappush(openq, (ng + math.hypot(nx - g[0], ny - g[1]), nb))
                    came[nb] = cur
        return None

    def line_free(self, a, b):
        (x0, y0), (x1, y1) = a, b
        n = int(math.hypot(x1 - x0, y1 - y0) / (self.res * 0.5)) + 1
        for i in range(n + 1):
            t = i / n
            if not self.is_free(*self.world_to_grid(x0 + t * (x1 - x0), y0 + t * (y1 - y0))):
                return False
        return True

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
        # resample
        dense = [out[0]]
        for (x0, y0), (x1, y1) in zip(out, out[1:]):
            seg = math.hypot(x1 - x0, y1 - y0)
            k = max(1, int(seg / resample))
            for m in range(1, k + 1):
                t = m / k
                dense.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
        return dense

    def plan_tour(self, start_xy, waypoints):
        full, cur = [], start_xy
        for wp in waypoints:
            leg = self.plan(cur, wp)
            if leg is None:
                continue
            leg = self.smooth(leg)
            if full:
                leg = leg[1:]
            full.extend(leg)
            cur = full[-1] if full else cur
        return full
