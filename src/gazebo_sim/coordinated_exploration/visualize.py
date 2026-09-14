#!/usr/bin/env python3
"""Render a top-down summary of the coordinated exploration run to output/."""
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle, Circle

from arena import OBSTACLES
from coordinated_exploration import SPAWN, BOTS

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
COLORS = {"bot1": "#e6194B", "bot2": "#3cb44b", "bot3": "#4363d8", "bot4": "#f58231"}


def draw_obstacles(ax):
    for ob in OBSTACLES:
        if ob["kind"] == "box":
            rect = Rectangle((-ob["sx"] / 2, -ob["sy"] / 2), ob["sx"], ob["sy"],
                             color="0.35")
            t = (matplotlib.transforms.Affine2D()
                 .rotate(ob["yaw"]).translate(ob["cx"], ob["cy"]) + ax.transData)
            rect.set_transform(t)
            ax.add_patch(rect)
        else:
            ax.add_patch(Circle((ob["cx"], ob["cy"]), ob["r"], color="0.35"))


def main():
    fig, ax = plt.subplots(figsize=(10, 10))

    # fused map point cloud (light background).
    # map_fusion publishes it in the `map_origin` frame, which is anchored at the
    # root robot's (bot1) SLAM start = its real-world spawn pose. Transform the
    # cloud by that spawn pose so it lines up with the world-frame obstacles.
    ply = os.path.join(OUT, "global_map.ply")
    if os.path.exists(ply):
        pts = np.loadtxt(ply, skiprows=7)
        ox, oy, oyaw = SPAWN["bot1"]
        c, s = math.cos(oyaw), math.sin(oyaw)
        wx = ox + c * pts[:, 0] - s * pts[:, 1]
        wy = oy + s * pts[:, 0] + c * pts[:, 1]
        ax.scatter(wx, wy, s=0.4, c="#9ec7e8", alpha=0.5,
                   linewidths=0, label="fused map (SLAM point cloud)")

    draw_obstacles(ax)

    # trajectories
    for b in BOTS:
        csv = os.path.join(OUT, f"traj_{b}.csv")
        if not os.path.exists(csv):
            continue
        d = np.loadtxt(csv, delimiter=",", skiprows=1)
        if d.ndim == 1 or len(d) == 0:
            continue
        ax.plot(d[:, 1], d[:, 2], color=COLORS[b], lw=2.2, label=f"{b} ground truth")
        ax.plot(d[:, 3], d[:, 4], color=COLORS[b], lw=1.0, ls="--", alpha=0.7)
        sx, sy, _ = SPAWN[b]
        ax.plot(sx, sy, "o", color=COLORS[b], ms=10, mec="k", zorder=5)
        ax.plot(d[-1, 1], d[-1, 2], "*", color=COLORS[b], ms=16, mec="k", zorder=5)

    ax.set_title("Coordinated 4-robot exploration of fishbot2 arena\n"
                 "solid = ground truth, dashed = SLAM estimate, "
                 "● start, ★ goal", fontsize=12)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
    ax.set_xlim(-7, 8.5); ax.set_ylim(-7, 7)
    ax.legend(loc="upper left", fontsize=8, framealpha=0.9)

    out = os.path.join(OUT, "exploration_result.png")
    fig.savefig(out, dpi=130, bbox_inches="tight")
    print(f"saved {out}")


if __name__ == "__main__":
    main()
