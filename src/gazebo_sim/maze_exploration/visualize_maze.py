#!/usr/bin/env python3
"""Render the maze run: top-down map+trajectories, and pair-distance vs time."""
import json
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "output")
DEF = json.load(open(os.path.join(HERE, "maze_def.json")))
BOTS = [f"bot{i}" for i in range(1, DEF["count"] + 1)]
COLORS = {"bot1": "#e6194B", "bot2": "#3cb44b", "bot3": "#4363d8",
          "bot4": "#f58231", "bot5": "#911eb4"}
ENCOUNTER_TH = 1.20


def fig_map():
    fig, ax = plt.subplots(figsize=(10, 10))
    # fused cloud is published in map_origin == root robot (bot1) SLAM start
    ply = os.path.join(OUT, "global_map.ply")
    if os.path.exists(ply):
        pts = np.loadtxt(ply, skiprows=7)
        ox, oy, oyaw = DEF["spawns"]["bot1"]
        c, s = math.cos(oyaw), math.sin(oyaw)
        wx = ox + c * pts[:, 0] - s * pts[:, 1]
        wy = oy + s * pts[:, 0] + c * pts[:, 1]
        ax.scatter(wx, wy, s=0.3, c="#9ec7e8", alpha=0.45, linewidths=0,
                   label="fused map (SLAM)")
    for cx, cy, sx, sy in DEF["boxes"]:
        ax.add_patch(Rectangle((cx - sx / 2, cy - sy / 2), sx, sy, color="0.35"))
    for b in BOTS:
        f = os.path.join(OUT, f"traj_{b}.csv")
        if not os.path.exists(f):
            continue
        d = np.loadtxt(f, delimiter=",", skiprows=1)
        if d.ndim < 2 or len(d) == 0:
            continue
        ax.plot(d[:, 1], d[:, 2], color=COLORS[b], lw=2.0, label=f"{b} truth")
        ax.plot(d[:, 3], d[:, 4], color=COLORS[b], lw=0.9, ls="--", alpha=0.7)
        ax.plot(d[0, 1], d[0, 2], "o", color=COLORS[b], ms=9, mec="k", zorder=5)
        ax.plot(d[-1, 1], d[-1, 2], "*", color=COLORS[b], ms=15, mec="k", zorder=5)
    h = DEF["half"]
    ax.set_xlim(-h - 0.5, h + 0.5); ax.set_ylim(-h - 0.5, h + 0.5)
    ax.set_aspect("equal"); ax.grid(alpha=0.3)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("5-robot coordinated SLAM in maze\n"
                 "solid = ground truth, dashed = SLAM estimate, ● start, ★ end")
    ax.legend(loc="upper left", fontsize=7, framealpha=0.9, ncol=2)
    p = os.path.join(OUT, "maze_result.png")
    fig.savefig(p, dpi=130, bbox_inches="tight")
    print("saved", p)


def fig_dist():
    f = os.path.join(OUT, "dist_pairs.csv")
    if not os.path.exists(f):
        return
    import csv
    with open(f) as fh:
        r = csv.reader(fh)
        hdr = next(r)
        rows = [[float(v) for v in row] for row in r]
    t = [row[0] for row in rows]
    fig, ax = plt.subplots(figsize=(12, 5.5))
    for k, name in enumerate(hdr[1:], start=1):
        ax.plot(t, [row[k] for row in rows], lw=0.9, alpha=0.75, label=name)
    ax.axhline(ENCOUNTER_TH, color="orange", ls="--", lw=1.6,
               label=f"encounter threshold {ENCOUNTER_TH} m")
    ax.axhline(0.30, color="red", ls="--", lw=1.6, label="collision (chassis contact) 0.30 m")
    ax.set_xlabel("t [s]"); ax.set_ylabel("pair distance [m]")
    ax.set_title("Inter-robot distances: dips below the orange line are close "
                 "encounters; the red line is never crossed")
    ax.set_ylim(0, 8); ax.grid(alpha=0.3)
    ax.legend(fontsize=7, ncol=4, loc="upper right")
    p = os.path.join(OUT, "maze_distances.png")
    fig.savefig(p, dpi=130, bbox_inches="tight")
    print("saved", p)


if __name__ == "__main__":
    fig_map()
    fig_dist()
