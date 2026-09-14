#!/usr/bin/env python3
"""Diagnose bot3's off-obstacle cluster: it is UNFILTERED TEAMMATE points.
Top-down zoom of bot3's cloud with the real walls, the off-obstacle points
highlighted, and the teammate ground-truth paths that pass through the cluster
during the early fan-out (before teammate filtering locks in)."""
import csv, json, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "output_final")
occ = np.load("/home/yunze/racer_lite_3d/outputs/run_20260718_113818_726454/"
              "ground_truth_occupancy.npy") == 1
edt = ndimage.distance_transform_edt(~occ)
org = json.load(open(os.path.join(OUT, "origins.json")))
CLR = {"bot1": "#4e79a7", "bot2": "#f28e2b", "bot4": "#e15759",
       "bot5": "#b07aa1", "bot6": "#76b7b2"}


def load_gt(b):
    rows = []
    with open(os.path.join(OUT, f"rec_{b}.csv")) as f:
        for d in csv.DictReader(f):
            rows.append([float(d[k]) for k in ["gt_x", "gt_y", "gt_z", "t"]])
    return np.array(rows)


g = np.load(os.path.join(OUT, "map_bot3.npy")) + np.array(org["bot3"][:3])
idx = np.clip(np.floor(g).astype(int), 0, 49)
d2 = edt[idx[:, 0], idx[:, 1], idx[:, 2]]
onwall = g[d2 <= 1.0]
ghost = g[d2 > 1.5]

ZB = (11, 15)
X0, X1, Y0, Y1 = 1, 17, 4, 20      # zoom to the early-flight corner

fig, ax = plt.subplots(figsize=(9.5, 9))
band = occ[:, :, ZB[0]:ZB[1]].any(axis=2).T
ax.imshow(band, origin="lower", cmap="Greys", vmin=0, vmax=2.2, alpha=0.5,
          extent=[0, 50, 0, 50], interpolation="nearest")

mo = (onwall[:, 2] >= ZB[0]) & (onwall[:, 2] < ZB[1])
ax.scatter(onwall[mo, 0], onwall[mo, 1], c="#2b6cb0", s=1.4, alpha=0.35,
           label="bot3 点云:落在真实墙上 (99%)")
mg = (ghost[:, 2] >= ZB[0]) & (ghost[:, 2] < ZB[1])
ax.scatter(ghost[mg, 0], ghost[mg, 1], c="#d11a2a", s=7, alpha=0.8,
           label="bot3 点云:悬空/离墙>1.5m (1%)")

# teammate paths that pass through the ghost during t<35 s (early fan-out)
for o in ["bot1", "bot2", "bot5", "bot4"]:
    gt = load_gt(o)
    e = gt[gt[:, 3] < 35]
    ax.plot(e[:, 0], e[:, 1], color=CLR[o], lw=2.4, alpha=0.9,
            label=f"{o} 早期航迹 (t<35s)")

ax.add_patch(plt.Circle((7.5, 12.5), 1.6, fill=False, ec="#d11a2a", lw=2.2, ls="--"))
ax.annotate("悬空点簇中心 (7.5,12.5)\n此处 occ=空, 但 bot1@t18s / bot2@t19s / bot5@t30s\n"
            "的机体正好飞过 → 未过滤的队友点",
            (7.5, 12.5), (9.5, 6.0), fontsize=9.5, color="#a00",
            arrowprops=dict(arrowstyle="->", color="#a00"))

ax.set_xlim(X0, X1); ax.set_ylim(Y0, Y1); ax.set_aspect("equal")
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
ax.set_title("bot3 左下角悬空点簇诊断 = 未过滤的队友机体点\n"
             "(灰=真实墙, 蓝=贴墙点, 红=悬空点, 彩线=队友早期航迹)", fontsize=12)
ax.legend(loc="upper right", fontsize=8.5)
fig.tight_layout()
p = os.path.join(OUT, "bot3_ghost_diagnosis.png")
fig.savefig(p, bbox_inches="tight", dpi=130)
print("wrote", p)
