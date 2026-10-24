#!/usr/bin/env python3
"""Overlay mbcompare runs: plot_compare.py out.png label1=dir1 label2=dir2 ...  (one panel row per manoeuvre)"""
import sys, csv, os
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
out = sys.argv[1]; runs = [a.split("=", 1) for a in sys.argv[2:]]
PANELS = [("bounce", ["travel0", "travel2"], "travel [m]"), ("skidpad", ["sw", "column_torque"], ""),
          ("step", ["yaw_rate", "ay"], ""), ("liftoff", ["yaw_rate", "beta"], ""),
          ("handsoff", ["beta", "steer0"], "rad"), ("bump", ["column_torque", "sw"], ""), ("slalom", ["y", "column_torque"], "")]
def load(d, m):
    f = os.path.join(d, m + ".csv")
    if not os.path.exists(f): return None
    r = list(csv.DictReader(open(f)))
    return {k: [float(x[k]) for x in r] for k in r[0]} if r else None
fig, ax = plt.subplots(len(PANELS), 2, figsize=(13, 3.0 * len(PANELS)))
for i, (m, cols, _) in enumerate(PANELS):
    for j, c in enumerate(cols):
        a = ax[i][j]
        for lab, d in runs:
            D = load(d, m)
            if D and c in D: a.plot(D["t"], D[c], label=lab, lw=1)
        a.set_title(f"{m}: {c}"); a.grid(alpha=.3); a.legend(fontsize=7)
fig.tight_layout(); fig.savefig(out, dpi=90); print("wrote", out)
