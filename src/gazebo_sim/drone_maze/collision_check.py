#!/usr/bin/env python3
"""Disc-based collision check. A quad is a 0.79 x 0.11 m DISC, not a sphere:
plain 3D pair distance flags the vertically-separated warm-up as a collision
when it is 0.30 m of deliberate altitude gap. Contact requires BOTH
horiz < 0.79 AND vert < 0.13 at the same instant (memory: drone_swarm run)."""
import os
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "output")
BOTS = ["bot%d" % i for i in range(1, 6)]
HORIZ, VERT = 0.79, 0.13

tr = {}
for b in BOTS:
    d = np.loadtxt(os.path.join(OUT, "traj_%s.csv" % b), delimiter=",", skiprows=1)
    tr[b] = d  # t, gt_x, gt_y, gt_z, slam_x, slam_y, slam_z

n = min(len(tr[b]) for b in BOTS)
lines = ["Disc-metric collision check (contact = horiz<%.2f AND vert<%.2f)" % (HORIZ, VERT),
         "=" * 66]
worst = None
collided = False
for i in range(5):
    for j in range(i + 1, 5):
        a, c = tr[BOTS[i]][:n], tr[BOTS[j]][:n]
        dh = np.hypot(a[:, 1] - c[:, 1], a[:, 2] - c[:, 2])
        dv = np.abs(a[:, 3] - c[:, 3])
        contact = (dh < HORIZ) & (dv < VERT)
        # closest true approach: min horiz among instants where vert overlaps
        overl = dv < VERT
        min_h = dh[overl].min() if overl.any() else float("inf")
        k = int(np.argmin(np.where(overl, dh, np.inf))) if overl.any() else -1
        t_at = a[k, 0] if k >= 0 else float("nan")
        stat = "CONTACT" if contact.any() else "ok"
        collided |= bool(contact.any())
        if min_h != float("inf") and (worst is None or min_h < worst[0]):
            worst = (min_h, BOTS[i], BOTS[j], t_at)
        lines.append("%s-%s: min horiz (while vert<%.2f) = %6s m at t=%5.1f s   %s"
                     % (BOTS[i], BOTS[j], VERT,
                        ("%.3f" % min_h) if min_h != float("inf") else "n/a", t_at, stat))
lines.append("-" * 66)
if worst:
    lines.append("closest true approach: %.3f m (%s-%s, t=%.1f s) -> %s"
                 % (worst[0], worst[1], worst[2], worst[3],
                    "COLLISION" if collided else "NO COLLISION"))
else:
    lines.append("no instant with vertical overlap -> NO COLLISION")
txt = "\n".join(lines)
open(os.path.join(OUT, "collision_report.txt"), "w").write(txt + "\n")
print(txt)
