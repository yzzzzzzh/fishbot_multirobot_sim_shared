#!/usr/bin/env python3
"""Diagnostic for two questions about final_global.png:
 Q1  purple blobs in 'empty' areas (bot1/bot3)  -> top face of the z=10 slab
 Q2  trajectory with no cloud (bot2/bot4)        -> cloud mapped ABOVE the z=11-14 band
Both are display-slab artifacts, not SLAM defects. Proven from the real data."""
import csv, json, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy import ndimage

plt.rcParams.update({"font.family": ["DejaVu Sans", "Noto Sans CJK JP"], "font.size": 9})
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "output_600s_v2")
occ = np.load("/home/yunze/racer_lite_3d/outputs/run_20260718_113818_726454/"
              "ground_truth_occupancy.npy") == 1
ORG = json.load(open(os.path.join(OUT, "origins.json")))
BOTS = ["bot1", "bot2", "bot3", "bot4", "bot5", "bot6"]
TAG = {"bot1": "R0", "bot2": "R1", "bot3": "R2", "bot4": "R3", "bot5": "R6", "bot6": "R7"}


def load_map(b):
    m = np.load(os.path.join(OUT, "map_%s.npy" % b)) + np.array(ORG[b][:3])
    return m[((m >= -2) & (m <= 52)).all(1)]


def load_gt(b):
    r = [[float(d["gt_x"]), float(d["gt_y"]), float(d["gt_z"])]
         for d in csv.DictReader(open(os.path.join(OUT, "rec_%s.csv" % b)))]
    return np.array(r)


band_wall = occ[:, :, 11:14].any(2).T          # grey walls the subplot draws
fig = plt.figure(figsize=(17, 9))
gs = gridspec.GridSpec(2, 6, height_ratios=[1, 1.25], hspace=0.33, wspace=0.42)

# ---- Row 1: cloud height histogram per drone, with the z=11-14 display band ----
for k, b in enumerate(BOTS):
    ax = fig.add_subplot(gs[0, k])
    c = load_map(b); gt = load_gt(b)
    ax.hist(c[:, 2], bins=np.linspace(0, 50, 101), color="#4e79a7", alpha=0.85)
    ax.axvspan(11, 14, color="#e15759", alpha=0.30)             # display slab
    ax.axvline(gt[:, 2].mean(), color="k", lw=1.4, ls="--")     # mean flight height
    inband = 100 * ((c[:, 2] >= 11) & (c[:, 2] < 14)).mean()
    ax.set_title("%s (%s)\n飞行均高 %.0fm · 带内云 %.0f%%" % (b, TAG[b], gt[:, 2].mean(), inband),
                 fontsize=8.5, color="#c00" if inband < 8 else "#333")
    ax.set_xlabel("点云高度 z [m]", fontsize=7.5); ax.tick_params(labelsize=6.5)
    ax.set_yticks([])
fig.text(0.5, 0.955, "诊断:俯视图只切 z=11–14m(红带)→ 空白轨迹 与 空地紫云 的真正来源",
         ha="center", fontsize=14, weight="bold")
fig.text(0.5, 0.925, "上排=各机点云的高度分布:红带=俯视图显示的薄层,黑虚线=该机平均飞行高度。"
         "bot2/bot4 云量几乎全在红带之上 → 俯视图当然空", ha="center", fontsize=9, color="#444")

# ---- Row 2 left+mid: Q2 — bot4 in the band (empty) vs at its own height (full) ----
def topdown(ax, b, z0, z1, title, color_by_slab=False):
    ax.imshow(band_wall, origin="lower", cmap="Greys", vmin=0, vmax=2.2,
              extent=[0, 50, 0, 50], alpha=0.55, interpolation="nearest")
    c = load_map(b); m = (c[:, 2] >= z0) & (c[:, 2] < z1); cm = c[m]
    if color_by_slab and len(cm):
        xi = np.clip(np.floor(cm[:, 0]).astype(int), 0, 49)
        yi = np.clip(np.floor(cm[:, 1]).astype(int), 0, 49)
        onslab = occ[xi, yi, 10] & ~occ[xi, yi, 11:14].any(1)   # on z10 slab, not under a wall
        ax.scatter(cm[~onslab, 0], cm[~onslab, 1], s=1.4, c="#1f77b4", alpha=0.5, label="打在竖直墙上")
        ax.scatter(cm[onslab, 0], cm[onslab, 1], s=1.4, c="#d62728", alpha=0.6, label="打在z=10水平平板顶面(=紫色)")
        ax.legend(loc="upper right", fontsize=6.5, markerscale=3, framealpha=.9)
    else:
        ax.scatter(cm[:, 0], cm[:, 1], s=1.4, c=cm[:, 2], cmap="viridis", alpha=0.6)
    gt = load_gt(b); ax.plot(gt[:, 0], gt[:, 1], color="#d11a2a", lw=1.2, alpha=.8)
    ax.set_xlim(0, 45); ax.set_ylim(0, 45); ax.set_aspect("equal")
    ax.set_title(title, fontsize=9); ax.tick_params(labelsize=6.5)

ax1 = fig.add_subplot(gs[1, 0:2])
topdown(ax1, "bot4", 11, 14, "Q2  bot4(R3) 切 z=11–14m(俯视图现状)\n云仅3% → 轨迹下方几乎空白")
ax2 = fig.add_subplot(gs[1, 2:4])
topdown(ax2, "bot4", 30, 36, "Q2  同一bot4 改切 z=30–36m(它的真实飞行层)\n墙体点云密集 → 图早就建好了,只是没画在薄层里")
ax3 = fig.add_subplot(gs[1, 4:6])
topdown(ax3, "bot1", 11, 14, "Q1  bot1(R0) 切 z=11–14m,按落点归类\n红=z10平板顶面(渲染成紫),蓝=竖直墙", color_by_slab=True)

fig.savefig(os.path.join(OUT, "viz_diagnosis.png"), dpi=130, bbox_inches="tight")
print("wrote", os.path.join(OUT, "viz_diagnosis.png"))
