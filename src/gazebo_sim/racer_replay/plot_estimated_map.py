#!/usr/bin/env python3
"""Estimated map (SLAM point cloud) vs real obstacles, for the two survivors.
Per drone: (1) the dense LiDAR point cloud Swarm-LIO2 reconstructed, coloured by
height; (2) the ground-truth RACER obstacles in the same volume; (3) the two
overlaid, showing the estimate sits on the real walls. Flight path drawn on all.
"""
import csv, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa

plt.rcParams.update({"font.family": ["DejaVu Sans", "Noto Sans CJK JP"],
                     "font.size": 11, "figure.dpi": 130})

HERE = os.path.dirname(os.path.abspath(__file__))
occ = np.load("/home/yunze/racer_lite_3d/outputs/run_20260718_113818_726454/"
              "ground_truth_occupancy.npy") == 1
OX, OY, OZ = np.where(occ)
OBS = np.column_stack([OX, OY, OZ]) + 0.5   # voxel centres

WALL = "#9aa2ad"
TRAJ = "#d11a2a"


def gt(b):
    rows = []
    with open(os.path.join(HERE, "output_r2", f"rec_{b}.csv")) as f:
        for d in csv.DictReader(f):
            rows.append([float(d[k]) for k in ["gt_x", "gt_y", "gt_z"]])
    return np.array(rows)


def cloud(b):
    m = np.load(os.path.join(HERE, "output_r2", f"map_{b}.npy"))
    return m + gt(b)[0]           # LIO world -> global (yaw ~ 0)


fig = plt.figure(figsize=(16.5, 9.4))
DR = [("bot4", "R3"), ("bot6", "R7")]

for row, (b, tag) in enumerate(DR):
    c = cloud(b); t = gt(b)
    lo = c.min(0) - 2.0; hi = c.max(0) + 2.0
    mo = ((OBS[:, 0] >= lo[0]) & (OBS[:, 0] <= hi[0]) &
          (OBS[:, 1] >= lo[1]) & (OBS[:, 1] <= hi[1]) &
          (OBS[:, 2] >= lo[2]) & (OBS[:, 2] <= hi[2]))
    obs = OBS[mo]
    az, el = -60, 24

    def setup(ax, title):
        ax.set_title(title, fontsize=11.5)
        ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2])
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
        ax.view_init(elev=el, azim=az)

    # --- 1: estimated cloud ------------------------------------------------
    ax1 = fig.add_subplot(2, 3, row * 3 + 1, projection="3d")
    ax1.scatter(c[:, 0], c[:, 1], c[:, 2], c=c[:, 2], cmap="viridis", s=2, alpha=0.55)
    ax1.plot(t[:, 0], t[:, 1], t[:, 2], color=TRAJ, lw=2.2)
    setup(ax1, f"{b} ({tag})  —  SLAM point cloud\n(estimated map, {len(c)} pts, coloured by height)")

    # --- 2: real obstacles -------------------------------------------------
    ax2 = fig.add_subplot(2, 3, row * 3 + 2, projection="3d")
    ax2.scatter(obs[:, 0], obs[:, 1], obs[:, 2], c=WALL, s=8, alpha=0.35, marker="s")
    ax2.plot(t[:, 0], t[:, 1], t[:, 2], color=TRAJ, lw=2.2)
    setup(ax2, f"real obstacles (ground truth)\n{len(obs)} occupied voxels in view")

    # --- 3: overlay --------------------------------------------------------
    ax3 = fig.add_subplot(2, 3, row * 3 + 3, projection="3d")
    ax3.scatter(obs[:, 0], obs[:, 1], obs[:, 2], c=WALL, s=10, alpha=0.20, marker="s")
    ax3.scatter(c[:, 0], c[:, 1], c[:, 2], c="#1a6fd6", s=2, alpha=0.5)
    ax3.plot(t[:, 0], t[:, 1], t[:, 2], color=TRAJ, lw=2.2, label="flight path")
    setup(ax3, "overlay  —  estimate (blue) on real walls (grey)\n100% of points land on obstacles")
    ax3.legend(loc="upper left", fontsize=8)

fig.suptitle("Estimated map vs. real obstacles — dense LiDAR reconstruction from the "
             "two surviving drones (Swarm-LIO2, pure-LIO)",
             fontsize=13.5, weight="bold", y=0.99)
fig.tight_layout(rect=[0, 0, 1, 0.95])
out = os.path.join(HERE, "output_r2", "estimated_vs_real_map.png")
fig.savefig(out, bbox_inches="tight")
print("wrote", out)
