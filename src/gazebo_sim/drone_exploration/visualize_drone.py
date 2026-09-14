#!/usr/bin/env python3
"""Render the single-drone SLAM run: top-down map + trajectory, altitude, error."""
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
SPAWN = DEF["spawns"]["bot1"]        # (x, y, yaw) -- the SLAM frame origin
SPAWN_Z = 0.055                      # base_link centre at rest


def _cloud_world():
    """The fused cloud is published in map_origin == bot1's SLAM start frame.
    Plotting it raw against world-frame walls makes it look translated."""
    ply = os.path.join(OUT, "global_map.ply")
    if not os.path.exists(ply):
        return None
    pts = np.loadtxt(ply, skiprows=7)
    ox, oy, oyaw = SPAWN
    c, s = math.cos(oyaw), math.sin(oyaw)
    wx = ox + c * pts[:, 0] - s * pts[:, 1]
    wy = oy + s * pts[:, 0] + c * pts[:, 1]
    wz = SPAWN_Z + pts[:, 2]
    return wx, wy, wz


def fig_map(d):
    fig, ax = plt.subplots(figsize=(10, 10))
    cw = _cloud_world()
    if cw is not None:
        wx, wy, wz = cw
        keep = wz > 0.08          # drop the floor so the walls read clearly
        ax.scatter(wx[keep], wy[keep], s=0.3, c="#9ec7e8", alpha=0.45,
                   linewidths=0, label="fused map (SLAM), floor removed")
    for cx, cy, sx, sy in DEF["boxes"]:
        ax.add_patch(Rectangle((cx - sx / 2, cy - sy / 2), sx, sy, color="0.35"))
    ax.plot(d[:, 1], d[:, 2], color="#e6194B", lw=2.0, label="ground truth")
    ax.plot(d[:, 4], d[:, 5], color="#3cb44b", lw=1.0, ls="--", label="SLAM estimate")
    ax.plot(d[0, 1], d[0, 2], "o", color="k", ms=9, zorder=5, label="spawn / land")
    ax.plot(d[-1, 1], d[-1, 2], "*", color="k", ms=16, zorder=5)
    h = DEF["half"]
    ax.set_xlim(-h - 0.5, h + 0.5)
    ax.set_ylim(-h - 0.5, h + 0.5)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Single drone (quad_mid360) flying SLAM through the maze\n"
                 "38.3 m A* tour, cruise 0.50 m, walls 1.0 m")
    ax.legend(loc="upper left", fontsize=8, framealpha=0.9)
    p = os.path.join(OUT, "drone_result.png")
    fig.savefig(p, dpi=130, bbox_inches="tight")
    print("saved", p)


def fig_time(d):
    t = d[:, 0]
    err = np.sqrt((d[:, 1] - d[:, 4]) ** 2 + (d[:, 2] - d[:, 5]) ** 2
                  + (d[:, 3] - d[:, 6]) ** 2)
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    a1.plot(t, d[:, 3], color="#e6194B", lw=1.6, label="altitude, ground truth")
    a1.plot(t, d[:, 6], color="#3cb44b", lw=1.0, ls="--", label="altitude, SLAM")
    a1.axhline(1.0, color="0.4", ls=":", lw=1.4, label="maze wall top (1.0 m)")
    a1.set_ylabel("z [m]")
    a1.grid(alpha=0.3)
    a1.legend(fontsize=8, ncol=3)
    a1.set_title("The trajectory is genuinely 3D, but stays below the wall tops "
                 "so the lidar sees wall, not sky")
    a2.plot(t, err, color="#4363d8", lw=1.0)
    a2.axhline(err.mean(), color="orange", ls="--", lw=1.4,
               label="mean %.3f m" % err.mean())
    a2.set_xlabel("t [s]")
    a2.set_ylabel("3D error [m]")
    a2.grid(alpha=0.3)
    a2.legend(fontsize=8)
    a2.set_title("SLAM position error vs ground truth")
    p = os.path.join(OUT, "drone_altitude_error.png")
    fig.savefig(p, dpi=130, bbox_inches="tight")
    print("saved", p)


if __name__ == "__main__":
    d = np.loadtxt(os.path.join(OUT, "traj_bot1.csv"), delimiter=",", skiprows=1)
    fig_map(d)
    fig_time(d)
