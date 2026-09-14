#!/usr/bin/env python3
"""6-drone RACER-replay result after the C3 re-timing + crash fixes.
Per drone: RACER reference (re-timed, red dashed), actual flight / ground truth
(black), and the Swarm-LIO2 SLAM estimate (blue). Axes are scaled to GT+ref;
drones whose SLAM diverged are annotated (their estimate is off-scale)."""
import csv, math, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa

plt.rcParams.update({"font.family": ["DejaVu Sans", "Noto Sans CJK JP"],
                     "font.size": 10, "figure.dpi": 130})

HERE = os.path.dirname(os.path.abspath(__file__))
REF = "#d11a2a"; GT = "#111111"; SL = "#2b6cb0"
DR = [("bot1", "R0"), ("bot2", "R1"), ("bot3", "R2"),
      ("bot4", "R3"), ("bot5", "R6"), ("bot6", "R7")]


def load(b):
    rows = []
    with open(os.path.join(HERE, "output_c3", f"rec_{b}.csv")) as f:
        for d in csv.DictReader(f):
            rows.append([float(d[k]) for k in
                         ["gt_x", "gt_y", "gt_z", "sl_x", "sl_y", "sl_z",
                          "ref_x", "ref_y", "ref_z"]])
    a = np.array(rows)
    return a[:, 0:3], a[:, 3:6], a[:, 6:9]


fig = plt.figure(figsize=(16, 9))
for k, (b, tag) in enumerate(DR):
    gt, sl, ref = load(b)
    ate = math.sqrt((np.linalg.norm(gt - sl, axis=1) ** 2).mean())
    ok = ate < 1.0
    ax = fig.add_subplot(2, 3, k + 1, projection="3d")
    ax.plot(ref[:, 0], ref[:, 1], ref[:, 2], color=REF, lw=1.8, ls="--",
            label="RACER ref (re-timed)")
    ax.plot(gt[:, 0], gt[:, 1], gt[:, 2], color=GT, lw=2.4, label="actual flight")
    if ok:
        ax.plot(sl[:, 0], sl[:, 1], sl[:, 2], color=SL, lw=1.3, ls=":",
                label="SLAM estimate")
    ax.scatter(*gt[0], color="#2f855a", s=35, marker="o")
    # scale to GT+ref only (diverged SLAM would blow up the view)
    P = np.vstack([gt, ref])
    lo, hi = P.min(0) - 0.5, P.max(0) + 0.5
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2])
    ax.set_xlabel("x", labelpad=-4); ax.set_ylabel("y", labelpad=-4)
    ax.tick_params(labelsize=7)
    ax.view_init(elev=24, azim=-60)
    col = "#1a7f37" if ok else "#c0392b"
    title = (f"{b} ({tag})   ATE {ate:.2f} m  ✓" if ok
             else f"{b} ({tag})   SLAM DIVERGED ({ate:.0f} m, off-scale)")
    ax.set_title(title, fontsize=10.5, color=col)
    if k == 0:
        ax.legend(loc="upper left", fontsize=7.5)

fig.suptitle("6-drone RACER-replay + Swarm-LIO2 — flyable re-timed refs, all crashes fixed "
             "(6/6 survive), teammate detection ON",
             fontsize=13, weight="bold", y=0.98)
fig.text(0.5, 0.945,
         "4/6 drones: accurate SLAM (0.14–0.66 m).  bot1/bot2 SLAM diverged (mutual-observation "
         "corruption at the map corner).  Red=RACER ref, black=actual flight, blue=SLAM estimate.",
         ha="center", fontsize=9.5, color="#555")
fig.tight_layout(rect=[0, 0, 1, 0.93])
out = os.path.join(HERE, "output_c3", "c3_result.png")
fig.savefig(out, bbox_inches="tight")
print("wrote", out)
