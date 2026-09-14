#!/usr/bin/env python3
"""Multi-view 3D result visualisation for the 600s RACER-replay + Swarm-LIO2 run.
These RACER exploration paths span the FULL z=2-48 m volume, so a thin top-down
slab (final_global.png) hides most of each drone's map. Here everything is FULL
3D, from optimised angles.

Outputs (into output_600s_v2/):
  drone_bot1..bot6.png : ONE figure per drone, 4 viewpoints (iso / top / front /
                         side), each with real obstacles + SLAM cloud + 3 trajs.
  swarm_3d.png         : one summary — a 3D SLAM point cloud per drone (6 subplots)
                         + a 3D ground-truth environment map + the merged swarm map.
"""
import csv, json, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa

plt.rcParams.update({"font.family": ["DejaVu Sans", "Noto Sans CJK JP"],
                     "font.size": 9, "figure.dpi": 120})

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, sys.argv[1] if len(sys.argv) > 1 else "output_600s_v2")
occ = np.load("/home/yunze/racer_lite_3d/outputs/run_20260718_113818_726454/"
              "ground_truth_occupancy.npy") == 1
OX, OY, OZ = np.where(occ)
OBS = np.column_stack([OX, OY, OZ]) + 0.5
ORG = json.load(open(os.path.join(OUT, "origins.json")))

BOTS = ["bot1", "bot2", "bot3", "bot4", "bot5", "bot6"]
TAG = {"bot1": "R0", "bot2": "R1", "bot3": "R2", "bot4": "R3", "bot5": "R6", "bot6": "R7"}
# iso view is perspective; the three orthographic elevations use ortho projection.
# last field = the depth axis to blank (its ticks collapse to an unreadable pile).
VIEWS = [("立体等距视角", 26, -55, "persp", None),
         ("俯视 (x-y)", 89, -90, "ortho", "z"),
         ("正视 (x-z)", 6, -90, "ortho", "y"),
         ("侧视 (y-z)", 6, 0, "ortho", "x")]
WALL = "#5c6675"      # dark slate — real obstacles must read as solid structure


def load_map(b):
    m = np.load(os.path.join(OUT, "map_%s.npy" % b)) + np.array(ORG[b][:3])
    return m[((m >= -2) & (m <= 52)).all(1)]


def load_rec(b):
    rows = []
    with open(os.path.join(OUT, "rec_%s.csv" % b)) as f:
        for d in csv.DictReader(f):
            rows.append([float(d[k]) for k in
                         ["gt_x", "gt_y", "gt_z", "sl_x", "sl_y", "sl_z",
                          "ref_x", "ref_y", "ref_z"]])
    a = np.array(rows)
    return a[:, 0:3], a[:, 3:6], a[:, 6:9]


def drone_limits(cloud, gt, ref):
    """tight, equal-aspect cube around the drone's real workspace."""
    lo = np.minimum(np.percentile(cloud, 1, 0), np.nanmin(gt, 0))
    hi = np.maximum(np.percentile(cloud, 99, 0), np.nanmax(gt, 0))
    lo = np.minimum(lo, np.nanmin(ref, 0)); hi = np.maximum(hi, np.nanmax(ref, 0))
    c = (lo + hi) / 2; r = (hi - lo).max() / 2 + 1.5
    return c - r, c + r


def scene(ax, obs, cloud, gt, sl, ref, elev, azim, proj, lim, hide=None, legend=False):
    ax.set_proj_type(proj)
    if len(obs):
        ax.scatter(obs[:, 0], obs[:, 1], obs[:, 2], c=WALL, s=11, alpha=0.20,
                   marker="s", edgecolors="none", depthshade=False)
    cc = cloud[:: max(1, len(cloud) // 16000)]
    ax.scatter(cc[:, 0], cc[:, 1], cc[:, 2], c=cc[:, 2], cmap="viridis", s=1.7,
               alpha=0.75, edgecolors="none", depthshade=False)
    ax.plot(ref[:, 0], ref[:, 1], ref[:, 2], color="#ff2d55", lw=1.7, ls="--",
            label="RACER 参考", zorder=9)
    ax.plot(gt[:, 0], gt[:, 1], gt[:, 2], color="#111111", lw=2.6,
            label="真实飞行", zorder=10)
    ax.plot(sl[:, 0], sl[:, 1], sl[:, 2], color="#00a2ff", lw=1.5, ls=":",
            label="SLAM 估计", zorder=11)
    ax.scatter(*gt[0], color="#2f855a", s=45, marker="o", zorder=12)
    (lo, hi) = lim
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2])
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev, azim)
    ax.tick_params(labelsize=6.5)
    ax.set_xlabel("x [m]", fontsize=7, labelpad=-3)
    ax.set_ylabel("y [m]", fontsize=7, labelpad=-3)
    ax.set_zlabel("z [m]", fontsize=7, labelpad=-3)
    # blank the depth axis whose ticks collapse to an unreadable pile in ortho views
    if hide == "z":
        ax.set_zticks([]); ax.set_zlabel("")
    elif hide == "y":
        ax.set_yticks([]); ax.set_ylabel("")
    elif hide == "x":
        ax.set_xticks([]); ax.set_xlabel("")
    if legend:
        ax.legend(loc="upper left", fontsize=8)


# ============ per-drone multi-view figures ============
for b in BOTS:
    gt, sl, ref = load_rec(b)
    cloud = load_map(b)
    lim = drone_limits(cloud, gt, ref)
    # obstacles inside this drone's cube (subsampled)
    mo = ((OBS >= lim[0]) & (OBS <= lim[1])).all(1)
    obs = OBS[mo]
    if len(obs) > 16000:
        obs = obs[:: len(obs) // 16000 + 1]
    ate = np.sqrt((np.linalg.norm(gt - sl, axis=1) ** 2).mean())

    fig = plt.figure(figsize=(14.5, 12.5))
    for i, (name, elev, azim, proj, hide) in enumerate(VIEWS):
        ax = fig.add_subplot(2, 2, i + 1, projection="3d")
        scene(ax, obs, cloud, gt, sl, ref, elev, azim, proj, lim, hide=hide, legend=(i == 0))
        ax.set_title(name, fontsize=11, color="#1a4e8a")
    fig.suptitle("%s (%s) — 四视角:真实障碍(灰) + SLAM 点云(按高度着色) + 轨迹   "
                 "ATE %.2f m · 地图 %dk 点 · 飞行高度 %.0f–%.0f m"
                 % (b, TAG[b], ate, len(cloud) // 1000, gt[:, 2].min(), gt[:, 2].max()),
                 fontsize=13, weight="bold", y=0.98)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    p = os.path.join(OUT, "drone_%s.png" % b)
    fig.savefig(p, bbox_inches="tight"); plt.close(fig)
    print("wrote", p)


# ============ summary: 3D SLAM cloud per drone + real map + merged ============
fig = plt.figure(figsize=(21, 10.5))
clouds = {b: load_map(b) for b in BOTS}
for k, b in enumerate(BOTS):
    ax = fig.add_subplot(2, 4, k + 1, projection="3d")
    c = clouds[b][:: max(1, len(clouds[b]) // 20000)]
    ax.scatter(c[:, 0], c[:, 1], c[:, 2], c=c[:, 2], cmap="viridis", s=1.1,
               alpha=0.45, edgecolors="none")
    gt, _, _ = load_rec(b)
    ax.plot(gt[:, 0], gt[:, 1], gt[:, 2], color="#111", lw=1.4, alpha=0.8)
    ax.set_box_aspect((1, 1, 1)); ax.view_init(26, -55)
    ax.tick_params(labelsize=6); ax.set_title("%s (%s)  SLAM 点云 %dk 点"
                                              % (b, TAG[b], len(clouds[b]) // 1000),
                                              fontsize=10, color="#1a7f37")

# real 3D environment map (ground-truth occupancy voxels, height-coloured).
# drop the solid z=0 floor slab so the maze interior isn't hidden behind it.
axr = fig.add_subplot(2, 4, 7, projection="3d")
obm = OBS[OBS[:, 2] > 1.0]
ob = obm[:: max(1, len(obm) // 34000)]
axr.scatter(ob[:, 0], ob[:, 1], ob[:, 2], c=ob[:, 2], cmap="turbo", s=3.5,
            alpha=0.4, marker="s", edgecolors="none", depthshade=False)
axr.set_box_aspect((1, 1, 1)); axr.view_init(26, -55); axr.tick_params(labelsize=6)
axr.set_xlabel("x", fontsize=7, labelpad=-4); axr.set_ylabel("y", fontsize=7, labelpad=-4)
axr.set_title("真实环境地图(真值占据体素, 去 z=0 地板)",
              fontsize=10, weight="bold", color="#b02a2a")

# merged swarm map (all six clouds together)
axm = fig.add_subplot(2, 4, 8, projection="3d")
CLR = {"bot1": "#4e79a7", "bot2": "#f28e2b", "bot3": "#59a14f",
       "bot4": "#e15759", "bot5": "#b07aa1", "bot6": "#76b7b2"}
for b in BOTS:
    c = clouds[b][:: max(1, len(clouds[b]) // 9000)]
    axm.scatter(c[:, 0], c[:, 1], c[:, 2], c=CLR[b], s=1.0, alpha=0.35,
                edgecolors="none", label=TAG[b])
axm.set_box_aspect((1, 1, 1)); axm.view_init(26, -55); axm.tick_params(labelsize=6)
axm.legend(loc="upper left", fontsize=7, markerscale=4, ncol=2)
axm.set_title("6 机合并 SLAM 地图(按机着色)", fontsize=10, weight="bold", color="#333")

fig.suptitle("6 机 Swarm-LIO2 立体建图汇总 — 每机 SLAM 点云 + 真实环境地图 + 合并群图",
             fontsize=15, weight="bold", y=0.99)
fig.tight_layout(rect=[0, 0, 1, 0.96])
p = os.path.join(OUT, "swarm_3d.png")
fig.savefig(p, bbox_inches="tight"); plt.close(fig)
print("wrote", p)
