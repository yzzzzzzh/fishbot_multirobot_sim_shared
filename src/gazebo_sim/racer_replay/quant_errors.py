#!/usr/bin/env python3
"""Quantitative SLAM-error report for a RACER-replay run directory.

Usage: python3 quant_errors.py <output_dir>            (e.g. output_1200s_10m20c)
Writes <output_dir>/quant_error_report.txt and prints it. Metrics per drone:
  ATE / max error / track-vs-ref RMSE
  first-half vs second-half ATE (drift over mission)
  error slope (linear fit, m per 100 s)
  corr(e, horizontal dist from start)  -- lever-arm / no-loop-closure signature
  corr(e, altitude z)                  -- high-open-zone signature
  revisit%% of 2nd half within 5 m of own 1st-half path (implicit loop closure)
  teammates within 20 m + nearest-teammate dist (1st/2nd half, from GT)
  odom coverage (%% of samples with SLAM output), divergence flag
"""
import csv, math, os, sys
import numpy as np

OUT = sys.argv[1] if len(sys.argv) > 1 else "output"
RR = os.path.dirname(os.path.abspath(__file__))
D = os.path.join(RR, OUT)
BOTS = ["bot1", "bot2", "bot3", "bot4", "bot5", "bot6"]
TAG = {"bot1": "R0", "bot2": "R1", "bot3": "R2", "bot4": "R3", "bot5": "R6", "bot6": "R7"}

def load(b):
    rows = []
    with open(os.path.join(D, "rec_%s.csv" % b)) as f:
        for d in csv.DictReader(f):
            rows.append([float(d[k]) for k in
                         ["t", "gt_x", "gt_y", "gt_z", "sl_x", "sl_y", "sl_z",
                          "ref_x", "ref_y", "ref_z"]])
    a = np.array(rows)
    return a

L = ["QUANTITATIVE ERROR REPORT — %s" % OUT, "=" * 78]
G = {}
for b in BOTS:
    a = load(b)
    t = a[:, 0]; gt = a[:, 1:4]; sl = a[:, 4:7]; ref = a[:, 7:10]
    G[b] = (t, gt)
    ok = ~np.isnan(sl[:, 0])
    cov = 100.0 * ok.sum() / len(a)
    tt, gg, ss = t[ok], gt[ok], sl[ok]
    e = np.linalg.norm(gg - ss, axis=1)
    trk = np.linalg.norm(gt - ref, axis=1)
    ate = math.sqrt((e ** 2).mean())
    half = tt.max() / 2.0
    a1 = math.sqrt((e[tt <= half] ** 2).mean()) if (tt <= half).any() else float("nan")
    a2 = math.sqrt((e[tt > half] ** 2).mean()) if (tt > half).any() else float("nan")
    slope = np.polyfit(tt, e, 1)[0] * 100 if len(tt) > 10 else float("nan")
    r = np.linalg.norm(gg[:, :2] - gg[0, :2], axis=1)
    cr = np.corrcoef(e, r)[0, 1] if len(e) > 10 else float("nan")
    cz = np.corrcoef(e, gg[:, 2])[0, 1] if len(e) > 10 else float("nan")
    # revisit: 2nd half within 5 m of own 1st-half GT path
    A = gt[t <= half][::10]; B = gt[t > half][::5]
    rv = float("nan")
    if len(A) and len(B):
        dmin = np.sqrt(((B[:, None, :] - A[None, :, :]) ** 2).sum(-1)).min(1)
        rv = 100.0 * (dmin < 5.0).mean()
    div = "DIVERGED" if (np.abs(ss).max() > 200) else "ok"
    imax = e.argmax()
    L.append("%s (%s)  [%s]  odom coverage %.1f%%" % (b, TAG[b], div, cov))
    L.append("  ATE %.3f m   max %.3f m (t=%.0fs z=%.1fm)   track-vs-ref %.3f m"
             % (ate, e.max(), tt[imax], gg[imax, 2], math.sqrt((trk ** 2).mean())))
    L.append("  1st-half ATE %.3f   2nd-half ATE %.3f   slope %+.3f m/100s"
             % (a1, a2, slope))
    L.append("  corr(e, dist-from-start) %+.2f   corr(e, z) %+.2f   revisit(2nd half) %.1f%%"
             % (cr, cz, rv))

# swarm geometry (GT): teammates within 20 m, nearest dist, per half
n = min(len(G[b][0]) for b in BOTS)
T = G["bot1"][0][:n]
P = np.stack([G[b][1][:n] for b in BOTS])
L.append("-" * 78)
L.append("swarm geometry from GT (comm-relevant):")
L.append("%-10s | nbrs<20m 1st | nbrs<20m 2nd | nearest 1st | nearest 2nd" % "bot")
half = T.max() / 2.0
for i, b in enumerate(BOTS):
    d = np.linalg.norm(P - P[i], axis=2); d[i] = 9e9
    nb = (d < 20).sum(axis=0); near = d.min(axis=0)
    m1 = T <= half; m2 = T > half
    L.append("%s (%s) |     %.2f     |     %.2f     |   %.1f m   |   %.1f m"
             % (b, TAG[b], nb[m1].mean(), nb[m2].mean(), near[m1].mean(), near[m2].mean()))

txt = "\n".join(L)
open(os.path.join(D, "quant_error_report.txt"), "w").write(txt + "\n")
print(txt)
print("\nwrote %s/quant_error_report.txt" % OUT)
