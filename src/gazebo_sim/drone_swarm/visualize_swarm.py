#!/usr/bin/env python3
"""Render the 5-drone run. The fused cloud is coloured BY SOURCE DRONE using the
intensity channel (map_fusion writes base_int = 100*(uid-1)), which is the whole
point of keeping it: a drone whose extrinsic is wrong shows up as its own
rotated ghost copy, and you can see exactly whose."""
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
DEF = json.load(open(os.path.join(HERE, "..", "maze_exploration", "maze_def.json")))
BOTS = ["bot%d" % i for i in range(1, 6)]
COLORS = {"bot1": "#e6194B", "bot2": "#3cb44b", "bot3": "#4363d8",
          "bot4": "#f58231", "bot5": "#911eb4"}
SPACING = 1.2
SPAWN = {b: (SPACING * math.cos(2 * math.pi * i / 5),
             SPACING * math.sin(2 * math.pi * i / 5),
             2 * math.pi * i / 5) for i, b in enumerate(BOTS)}
SPAWN_Z = 0.055
ENCOUNTER_TH = 1.60


def walls(ax):
    for cx, cy, sx, sy in DEF["boxes"]:
        ax.add_patch(Rectangle((cx - sx / 2, cy - sy / 2), sx, sy, color="0.35", zorder=2))


def load_cloud():
    p = os.path.join(OUT, "global_map.ply")
    d = np.loadtxt(p, skiprows=8)
    ox, oy, oyaw = SPAWN["bot1"]          # map_origin == bot1's spawn
    c, s = math.cos(oyaw), math.sin(oyaw)
    wx = ox + c * d[:, 0] - s * d[:, 1]
    wy = oy + s * d[:, 0] + c * d[:, 1]
    wz = SPAWN_Z + d[:, 2]
    uid = np.round(d[:, 3] / 100.0).astype(int) + 1 if d.shape[1] > 3 else np.ones(len(d), int)
    return wx, wy, wz, uid


def fig_map():
    wx, wy, wz, uid = load_cloud()
    keep = wz > 0.08                      # drop the floor
    fig, axes = plt.subplots(1, 2, figsize=(19, 9.2))

    ax = axes[0]
    walls(ax)
    ax.scatter(wx[keep], wy[keep], s=0.25, c="#9ec7e8", alpha=0.45, linewidths=0)
    for b in BOTS:
        f = os.path.join(OUT, "traj_%s.csv" % b)
        if not os.path.exists(f):
            continue
        d = np.loadtxt(f, delimiter=",", skiprows=1)
        ax.plot(d[:, 1], d[:, 2], color=COLORS[b], lw=1.6, label="%s truth" % b, zorder=6)
        ax.plot(d[:, 4], d[:, 5], color=COLORS[b], lw=0.8, ls="--", alpha=0.8, zorder=6)
    ax.set_title("Fused map + trajectories\nsolid = ground truth, dashed = SLAM")
    ax.legend(loc="upper left", fontsize=7, ncol=2)

    ax = axes[1]
    walls(ax)
    for i, b in enumerate(BOTS):
        k = keep & (uid == i + 1)
        if k.sum() == 0:
            continue
        ax.scatter(wx[k], wy[k], s=0.25, c=COLORS[b], alpha=0.5, linewidths=0,
                   label="%s (%d pts)" % (b, k.sum()))
    ax.set_title("Same cloud, coloured BY SOURCE DRONE (intensity channel)\n"
                 "a wrong extrinsic = that colour lands as a rotated ghost")
    ax.legend(loc="upper left", fontsize=8, markerscale=14)

    h = DEF["half"]
    for ax in axes:
        ax.set_xlim(-h - 0.5, h + 0.5); ax.set_ylim(-h - 0.5, h + 0.5)
        ax.set_aspect("equal"); ax.grid(alpha=0.3)
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    p = os.path.join(OUT, "swarm_result.png")
    fig.savefig(p, dpi=120, bbox_inches="tight")
    print("saved", p)


def fig_dist():
    f = os.path.join(OUT, "dist_pairs.csv")
    d = np.loadtxt(f, delimiter=",", skiprows=1)
    hdr = open(f).readline().strip().split(",")[1:]
    fig, ax = plt.subplots(figsize=(13, 5.5))
    for k, name in enumerate(hdr, start=1):
        ax.plot(d[:, 0], d[:, k], lw=0.8, alpha=0.75, label=name)
    ax.axhline(ENCOUNTER_TH, color="orange", ls="--", lw=1.5,
               label="encounter %.1f m" % ENCOUNTER_TH)
    ax.axhline(0.79, color="red", ls="--", lw=1.5, label="rotor-tip contact 0.79 m")
    ax.axvline(150, color="0.4", ls=":", lw=1.5, label="warm-up ends, tours begin")
    ax.set_xlabel("t [s]"); ax.set_ylabel("3D pair distance [m]")
    ax.set_ylim(0, 8); ax.grid(alpha=0.3)
    ax.set_title("Inter-drone distances. NOTE: during the warm-up (t<150) pairs dip "
                 "low in 3D\nbecause the mover is 0.35 m ABOVE the parked ones -- "
                 "vertical separation, not a near miss.")
    ax.legend(fontsize=6, ncol=6, loc="upper right")
    p = os.path.join(OUT, "swarm_distances.png")
    fig.savefig(p, dpi=120, bbox_inches="tight")
    print("saved", p)


if __name__ == "__main__":
    fig_map()
    fig_dist()
