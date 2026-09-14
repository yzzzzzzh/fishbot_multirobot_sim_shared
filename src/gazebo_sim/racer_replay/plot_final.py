#!/usr/bin/env python3
"""Final 6-drone RACER-replay result — per-drone panels WITH the environment
obstacles and the SLAM-accumulated point cloud (user requirement), plus a
global swarm view combining all six clouds on the shared obstacle map."""
import csv, json, math, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa

plt.rcParams.update({"font.family": ["DejaVu Sans", "Noto Sans CJK JP"],
                     "font.size": 10, "figure.dpi": 130})

HERE = os.path.dirname(os.path.abspath(__file__))
# output dir selectable so new runs never overwrite older results
OUT = os.path.join(HERE, sys.argv[1] if len(sys.argv) > 1 else "output_final")
occ = np.load("/home/yunze/racer_lite_3d/outputs/run_20260718_113818_726454/"
              "ground_truth_occupancy.npy") == 1
OX, OY, OZ = np.where(occ)
OBS = np.column_stack([OX, OY, OZ]) + 0.5          # obstacle voxel centres
ORG = json.load(open(os.path.join(OUT, "origins.json")))

BOTS = ["bot1", "bot2", "bot3", "bot4", "bot5", "bot6"]
TAG = {"bot1": "R0", "bot2": "R1", "bot3": "R2", "bot4": "R3", "bot5": "R6", "bot6": "R7"}
CLR = {"bot1": "#4e79a7", "bot2": "#f28e2b", "bot3": "#59a14f",
       "bot4": "#e15759", "bot5": "#b07aa1", "bot6": "#76b7b2"}
WALL = "#8a94a6"


def load_rec(b):
    rows = []
    with open(os.path.join(OUT, f"rec_{b}.csv")) as f:
        for d in csv.DictReader(f):
            rows.append([float(d[k]) for k in
                         ["gt_x", "gt_y", "gt_z", "sl_x", "sl_y", "sl_z",
                          "ref_x", "ref_y", "ref_z"]])
    a = np.array(rows)
    return a[:, 0:3], a[:, 3:6], a[:, 6:9]


def load_map(b):
    m = np.load(os.path.join(OUT, f"map_{b}.npy"))
    g = m + np.array(ORG[b][:3])                   # LIO frame -> global
    # clip to the map region: a diverged run can spray points kilometres out,
    # which would blow up every axis extent; keep the honest in-map portion
    return g[((g >= -2.0) & (g <= 52.0)).all(axis=1)]


def run_duration():
    with open(os.path.join(OUT, "rec_bot1.csv")) as f:
        return max(float(d["t"]) for d in csv.DictReader(f))


# ---------------- Fig 1: per-drone panels (obstacles + cloud + trajs) -------
fig = plt.figure(figsize=(16.5, 10))
ates, npts = [], []
for k, b in enumerate(BOTS):
    gt, sl, ref = load_rec(b)
    cloud = load_map(b)
    ate = math.sqrt((np.linalg.norm(gt - sl, axis=1) ** 2).mean())
    ates.append(ate); npts.append(len(cloud))

    ax = fig.add_subplot(2, 3, k + 1, projection="3d")
    lo = np.minimum(cloud.min(0), gt.min(0)) - 1.0
    hi = np.maximum(cloud.max(0), gt.max(0)) + 1.0
    mo = ((OBS >= lo) & (OBS <= hi)).all(1)
    obs = OBS[mo]
    if len(obs) > 6000:
        obs = obs[:: len(obs) // 6000 + 1]
    cs = cloud[:: max(1, len(cloud) // 18000)]

    ax.scatter(obs[:, 0], obs[:, 1], obs[:, 2], c=WALL, s=4, alpha=0.10, marker="s")
    ax.scatter(cs[:, 0], cs[:, 1], cs[:, 2], c=cs[:, 2], cmap="viridis", s=1.0, alpha=0.30)
    ax.plot(ref[:, 0], ref[:, 1], ref[:, 2], color="#ff2d55", lw=2.4, ls="--",
            label="RACER 参考", zorder=9)
    ax.plot(gt[:, 0], gt[:, 1], gt[:, 2], color="#000000", lw=3.4,
            label="真实飞行", zorder=10)
    ax.plot(sl[:, 0], sl[:, 1], sl[:, 2], color="#00a2ff", lw=2.0, ls=":",
            label="SLAM 估计", zorder=11)
    ax.scatter(*gt[0], color="#2f855a", s=55, marker="o", zorder=12)
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2])
    ax.set_title(f"{b} ({TAG[b]})   ATE {ate:.2f} m   地图 {len(cloud)//1000}k 点",
                 fontsize=11, color="#1a7f37")
    ax.tick_params(labelsize=7)
    ax.view_init(elev=24, azim=-62)
    if k == 0:
        ax.legend(loc="upper left", fontsize=8)

fig.suptitle("6 机 RACER 回放 + Swarm-LIO2 — 每机:真实障碍(灰) + SLAM 点云(按高度着色) + 三条轨迹",
             fontsize=14, weight="bold", y=0.985)
fig.text(0.5, 0.945,
         "%.0fs 飞行:ATE %.2f–%.2f m;单机点云 %d–%dk 点(队友机体点已按高反射剔除)"
         % (run_duration(), min(ates), max(ates),
            min(npts) // 1000, max(npts) // 1000),
         ha="center", fontsize=9.5, color="#444")
fig.tight_layout(rect=[0, 0, 1, 0.94])
p1 = os.path.join(OUT, "final_result.png")
fig.savefig(p1, bbox_inches="tight")
print("wrote", p1)

# ---------------- Fig 2: real map (ref) + PER-DRONE top-down clouds ----------
import matplotlib.gridspec as gridspec
ZB = (11, 14)      # top-down projection band (metres): clean wall floorplan at
                   # flight height (z10-17 unions a z~15-17 slab -> 78% filled)
allpts = np.vstack([load_map(b) for b in BOTS])
lo = allpts.min(0) - 1.5; hi = allpts.max(0) + 1.5
band = occ[:, :, ZB[0]:ZB[1]].any(axis=2).T        # (y, x) ground-truth walls

fig2 = plt.figure(figsize=(18.5, 8.4))
gs = gridspec.GridSpec(2, 4, width_ratios=[1.35, 1, 1, 1],
                       hspace=0.28, wspace=0.28)

# --- left (spans both rows): REAL map reference ---
axr = fig2.add_subplot(gs[:, 0])
axr.imshow(band, origin="lower", cmap="Greys", vmin=0, vmax=1,
           extent=[0, 50, 0, 50], interpolation="nearest")
for b in BOTS:
    gt, _, _ = load_rec(b)
    axr.plot(gt[:, 0], gt[:, 1], color=CLR[b], lw=1.5)
    axr.scatter(gt[0, 0], gt[0, 1], color=CLR[b], s=42, marker="o",
                edgecolor="white", zorder=5)
axr.set_xlim(lo[0], hi[0]); axr.set_ylim(lo[1], hi[1]); axr.set_aspect("equal")
axr.set_xlabel("x [m]"); axr.set_ylabel("y [m]")
axr.set_title("真实环境地图(参考)\n黑=墙, 彩线=各机真实轨迹\n俯视 z=%d–%d m" % ZB,
              fontsize=11)

# --- right 2x3: one panel PER DRONE, its own SLAM cloud on faint truth walls ---
cells = [gs[0, 1], gs[0, 2], gs[0, 3], gs[1, 1], gs[1, 2], gs[1, 3]]
for cell, b in zip(cells, BOTS):
    ax = fig2.add_subplot(cell)
    ax.imshow(band, origin="lower", cmap="Greys", vmin=0, vmax=2.2,   # faint walls
              extent=[0, 50, 0, 50], interpolation="nearest", alpha=0.55)
    c = load_map(b)
    m = (c[:, 2] >= ZB[0]) & (c[:, 2] < ZB[1])
    cm = c[m]
    ax.scatter(cm[:, 0], cm[:, 1], c=cm[:, 2], cmap="viridis", s=1.4, alpha=0.6)
    gt, _, _ = load_rec(b)
    ax.plot(gt[:, 0], gt[:, 1], color="#d11a2a", lw=1.6)
    ax.scatter(gt[0, 0], gt[0, 1], color="#2f855a", s=36, marker="o",
               edgecolor="white", zorder=5)
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_aspect("equal")
    ax.tick_params(labelsize=7)
    ate = math.sqrt((np.linalg.norm(gt - load_rec(b)[1], axis=1) ** 2).mean())
    ax.set_title("%s (%s)  ATE %.2f m  云 %dk 点"
                 % (b, TAG[b], ate, len(c) // 1000), fontsize=10, color=CLR[b])

fig2.suptitle("俯视对比 — 每架无人机各自的 SLAM 点云地图(灰=真实墙, 点云按高度着色, 红=真实轨迹)",
              fontsize=13.5, weight="bold", y=0.98)
fig2.tight_layout(rect=[0, 0, 1, 0.94])
p2 = os.path.join(OUT, "final_global.png")
fig2.savefig(p2, bbox_inches="tight")
print("wrote", p2)
