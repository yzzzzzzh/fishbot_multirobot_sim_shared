#!/usr/bin/env python3
"""Proof that the Gazebo world IS the RACER map: render RACER's ground-truth
occupancy (the exact grid its planner explored, seed 7 'rooms' world) as the
obstacle field, overlay the two survivors' actual flight paths, and overlay the
live LiDAR returns they registered (which land on the map's walls -> the map is
loaded AND sensed).
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
RACER_RUN = "/home/yunze/racer_lite_3d/outputs/run_20260718_113818_726454"
occ = np.load(os.path.join(RACER_RUN, "ground_truth_occupancy.npy")) == 1
NX, NY, NZ = occ.shape

WALL = "#8a94a6"
TRAJ = {"bot4": "#c53030", "bot6": "#2f855a"}
LIDAR = {"bot4": "#d9700a", "bot6": "#2b6cb0"}


def gt_traj(b):
    rows = []
    with open(os.path.join(HERE, "output_r2", f"rec_{b}.csv")) as f:
        for d in csv.DictReader(f):
            rows.append([float(d[k]) for k in ["gt_x", "gt_y", "gt_z"]])
    return np.array(rows)


def cloud_global(b):
    """live cloud is in the LIO world frame (origin at spawn, yaw~0);
    add the recorded spawn to bring it into the Gazebo global frame."""
    p = np.load(os.path.join(HERE, "output_r2", f"cloud_{b}.npy"))
    spawn = gt_traj(b)[0]
    return p + spawn


fig = plt.figure(figsize=(16, 6.6))

# ---------- Panel A: 3-D map + flight paths --------------------------------
axA = fig.add_subplot(1, 3, 1, projection="3d")
xs, ys, zs = np.where(occ)
step = 3   # downsample voxels so the 3-D view stays legible
sel = (np.arange(len(xs)) % step == 0)
axA.scatter(xs[sel], ys[sel], zs[sel], c=WALL, s=2, alpha=0.12, marker="s")
for b in ["bot4", "bot6"]:
    t = gt_traj(b)
    axA.plot(t[:, 0], t[:, 1], t[:, 2], color=TRAJ[b], lw=2.6, label=f"{b} flight")
axA.set_title("RACER map (seed-7 'rooms', 34101 voxels)\n+ actual flight paths",
              fontsize=11)
axA.set_xlabel("x [m]"); axA.set_ylabel("y [m]"); axA.set_zlabel("z [m]")
axA.set_xlim(0, NX); axA.set_ylim(0, NY); axA.set_zlim(0, NZ)
axA.legend(loc="upper left", fontsize=9)
axA.view_init(elev=26, azim=-60)

# ---------- Panel B: top-down slice at flight altitude ---------------------
axB = fig.add_subplot(1, 3, 2)
zc = 12  # flight layer
layer = occ[:, :, zc].T   # (y,x) for imshow
axB.imshow(layer, origin="lower", cmap="Greys", alpha=0.85,
           extent=[0, NX, 0, NY], interpolation="nearest")
for b in ["bot4", "bot6"]:
    t = gt_traj(b)
    axB.plot(t[:, 0], t[:, 1], color=TRAJ[b], lw=2.4, label=f"{b} flight")
    axB.scatter(t[0, 0], t[0, 1], color=TRAJ[b], s=60, marker="o",
                edgecolor="white", zorder=5)
axB.set_title(f"top-down slice at z={zc} m\n(walls + doorways of the rooms world)",
              fontsize=11)
axB.set_xlabel("x [m]"); axB.set_ylabel("y [m]")
axB.set_xlim(0, NX); axB.set_ylim(0, NY)
axB.legend(loc="upper right", fontsize=9)
axB.set_aspect("equal")

# ---------- Panel C: live LiDAR returns land on the walls ------------------
axC = fig.add_subplot(1, 3, 3, projection="3d")
axC.scatter(xs[sel], ys[sel], zs[sel], c=WALL, s=2, alpha=0.08, marker="s")
for b in ["bot4", "bot6"]:
    c = cloud_global(b)
    axC.scatter(c[:, 0], c[:, 1], c[:, 2], c=LIDAR[b], s=3, alpha=0.5,
                label=f"{b} LiDAR ({len(c)} pts)")
    t = gt_traj(b)
    axC.scatter(t[0, 0], t[0, 1], t[0, 2], color=TRAJ[b], s=55, marker="^",
                edgecolor="white", zorder=6)
axC.set_title("live LiDAR returns (cloud_registered)\nfall on the map's walls -> map is sensed",
              fontsize=11)
axC.set_xlabel("x [m]"); axC.set_ylabel("y [m]"); axC.set_zlabel("z [m]")
axC.set_xlim(0, NX); axC.set_ylim(0, NY); axC.set_zlim(0, NZ)
axC.legend(loc="upper left", fontsize=9)
axC.view_init(elev=26, azim=-60)

fig.suptitle("The Gazebo world IS the RACER map — built from RACER's ground-truth "
             "occupancy (742 merged boxes), loaded, and sensed by the drones",
             fontsize=13.5, weight="bold", y=0.99)
fig.tight_layout(rect=[0, 0, 1, 0.94])
out = os.path.join(HERE, "output_r2", "racer_map_proof.png")
fig.savefig(out, bbox_inches="tight")
print("wrote", out)
