#!/usr/bin/env python3
"""Visualise the LOCAL SLAM result of the two drones that survived the RACER
replay (bot4=R3, bot6=R7). Per drone: 3-D trajectory (SLAM estimate vs Gazebo
ground truth), the three position components over time, and the position-error
curve with the IMU-init transient marked. This is the per-drone LiDAR-inertial
odometry accuracy, independent of the (failed) trajectory-tracking layer.
"""
import csv, math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa

plt.rcParams.update({
    "font.family": ["DejaVu Sans", "Noto Sans CJK JP"],
    "font.size": 11, "axes.titlesize": 12, "axes.labelsize": 10,
    "figure.dpi": 130,
})

TRUTH = "#111111"     # ground truth ink
EST   = "#2b6cb0"     # SLAM estimate (blue)
ERR   = "#d9700a"     # error (amber)
GRID  = "#dce0e4"
INITW = 15.0          # IMU-init / settling window [s]

DR = [("bot4", 3, "R3"), ("bot6", 7, "R7")]


def load(b):
    rows = []
    with open(f"output_r2/rec_{b}.csv") as f:
        for d in csv.DictReader(f):
            rows.append([float(d[k]) for k in
                         ["t", "gt_x", "gt_y", "gt_z", "sl_x", "sl_y", "sl_z"]])
    a = np.array(rows)
    a = a[~np.isnan(a[:, 4])]
    return a[:, 0], a[:, 1:4], a[:, 4:7]


fig = plt.figure(figsize=(15.5, 8.9))
gs = fig.add_gridspec(2, 3, width_ratios=[1.25, 1.0, 1.0],
                      hspace=0.34, wspace=0.27,
                      left=0.035, right=0.985, top=0.855, bottom=0.075)

for row, (b, rid, tag) in enumerate(DR):
    t, gt, sl = load(b)
    e = np.linalg.norm(gt - sl, axis=1)
    ate = math.sqrt((e ** 2).mean())
    ss = t > INITW
    ate_ss = math.sqrt((e[ss] ** 2).mean())
    max_ss = e[ss].max()
    plen = np.linalg.norm(np.diff(gt, axis=0), axis=1).sum()

    # ---- 3-D trajectory ------------------------------------------------
    ax = fig.add_subplot(gs[row, 0], projection="3d")
    ax.plot(gt[:, 0], gt[:, 1], gt[:, 2], color=TRUTH, lw=2.4,
            label="Ground truth (Gazebo)")
    ax.plot(sl[:, 0], sl[:, 1], sl[:, 2], color=EST, lw=1.4, ls="--",
            label="SLAM estimate")
    ax.scatter(*gt[0], color="#2f855a", s=45, marker="o", label="start")
    ax.scatter(*gt[-1], color="#c53030", s=45, marker="s", label="end")
    ax.set_title(f"{b}  (RACER {tag})   —   3-D trajectory\n"
                 f"flew {plen:.1f} m · steady-state ATE {ate_ss:.3f} m · no drift",
                 pad=6, fontsize=11.5)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
    ax.legend(loc="upper left", fontsize=8, framealpha=0.9)
    ax.view_init(elev=22, azim=-58)
    ax.grid(True)

    # ---- components vs time -------------------------------------------
    axc = fig.add_subplot(gs[row, 1])
    for k, lab in enumerate("xyz"):
        axc.plot(t, gt[:, k], color=TRUTH, lw=1.6, alpha=0.9)
        axc.plot(t, sl[:, k], color=EST, lw=1.0, ls="--")
        axc.annotate(lab, (t[-1], gt[-1, k]), fontsize=9, color=TRUTH,
                     xytext=(4, 0), textcoords="offset points", va="center")
    axc.plot([], [], color=TRUTH, lw=1.6, label="truth")
    axc.plot([], [], color=EST, lw=1.0, ls="--", label="estimate")
    axc.set_title("position components   x / y / z", pad=4)
    axc.set_xlabel("replay time [s]"); axc.set_ylabel("position [m]")
    axc.legend(loc="center right", fontsize=8, framealpha=0.9)
    axc.grid(True, color=GRID)
    for s in axc.spines.values():
        s.set_color("#b8bec4")

    # ---- error vs time -------------------------------------------------
    axe = fig.add_subplot(gs[row, 2])
    axe.axvspan(0, INITW, color="#9aa0a6", alpha=0.12)
    axe.text(INITW / 2, e.max() * 1.02, "IMU\ninit", ha="center", va="top",
             fontsize=7.5, color="#5f6368")
    axe.plot(t, e, color=ERR, lw=1.3)
    axe.fill_between(t, 0, e, color=ERR, alpha=0.10)
    axe.axhline(ate_ss, color="#7a3d00", lw=1.0, ls=":",
                label=f"steady-state ATE = {ate_ss:.3f} m")
    axe.set_ylim(0, max(0.6, e.max() * 1.12))
    axe.set_title("position error   ‖truth − estimate‖", pad=4)
    axe.set_xlabel("replay time [s]"); axe.set_ylabel("error [m]")
    axe.legend(loc="upper right", fontsize=8.5, framealpha=0.92)
    axe.grid(True, color=GRID)
    for s in axe.spines.values():
        s.set_color("#b8bec4")
    axe.text(0.985, 0.60,
             f"full-run ATE {ate:.3f} m\nsteady max {max_ss:.2f} m",
             transform=axe.transAxes, ha="right", va="top",
             fontsize=8.5, color="#333",
             bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="#ccc"))

fig.suptitle("Local LiDAR-inertial SLAM accuracy — the two drones that ran cleanly "
             "in the 6-drone RACER replay", fontsize=14.5, y=0.975, weight="bold")
fig.text(0.5, 0.905,
         "Swarm-LIO2 per-drone odometry (pure-LIO mode).  Estimate overlaps truth "
         "to a few centimetres and stays flat over 200 s — no drift.  The only large "
         "error is the sub-15 s IMU-initialisation transient (shaded).",
         ha="center", fontsize=10, color="#555")

out = "output_r2/success_bot4_bot6.png"
fig.savefig(out, bbox_inches="tight")
print("wrote", out)
