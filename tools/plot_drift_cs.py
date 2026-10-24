#!/usr/bin/env python3
"""Countersteer before/after tracking plot and sustained-drift plot (results/countersteer, results/drift)."""
import csv, os, sys
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
R = sys.argv[1] if len(sys.argv) > 1 else "results"
def load(p):
    with open(p) as f: r = list(csv.DictReader(f))
    return {k: [float(x[k]) for x in r] for k in r[0]}
# countersteer: delta vs beta_front, before vs after
sc = ["lift_60kmh_R32", "lift_85kmh_R65", "power_40kmh_R20", "power_60kmh_R40"]
fig, ax = plt.subplots(len(sc), 2, figsize=(14, 3.0 * len(sc)), sharex=True)
for i, s in enumerate(sc):
    for j, tag in enumerate(["before", "after"]):
        a = ax[i][j]
        for drv, ls in [("hands_off", "-"), ("gp_zero", "--")]:
            p = f"{R}/countersteer/cs_{tag}_{s}_{drv}.csv"
            if not os.path.exists(p): continue
            d = load(p)
            a.plot(d["t"], d["beta_front_deg"], "k" + ls, lw=1, label=f"beta_f {drv}")
            a.plot(d["t"], d["delta_deg"], ("C3" if drv == "hands_off" else "C0") + ls, lw=1.2, label=f"delta {drv}")
        a.set_title(f"{s} [{tag}]"); a.grid(alpha=.3); a.set_ylabel("deg"); a.set_ylim(-45, 45)
        if i == 0: a.legend(fontsize=7, loc="lower left")
for a in ax[-1]: a.set_xlabel("t [s]")
fig.suptitle("Countersteer tracking, zero input: road-wheel angle delta vs front-axle slip beta_f (default car)\n"
             "before: range/expo gamepad + column_inertia 0.01 | after: deadzone = hands off, column_inertia 0.002")
fig.tight_layout(); fig.savefig(f"{R}/countersteer/countersteer_before_after.png", dpi=110); plt.close(fig)
# drift
runs = [("before_falloff1", "arms_b27", "falloff_speed 1 (before)"), ("after", "arms_b27", "falloff_speed 4 (after)"),
        ("after", "arms_b32", "after, beta* 32"), ("after", "arms_b27_transition", "after, transition")]
fig, ax = plt.subplots(5, 1, figsize=(13, 15), sharex=True)
for k, (tag, nm, lab) in enumerate(runs):
    p = f"{R}/drift/drift_{tag}_{nm}.csv"
    if not os.path.exists(p): continue
    d = load(p); c = f"C{k}"
    ax[0].plot(d["t"], d["beta_deg"], c, lw=1, label=lab)
    ax[1].plot(d["t"], d["delta_deg"], c, lw=1); ax[1].plot(d["t"], d["delta_req_deg"], c + ":", lw=.8)
    ax[2].plot(d["t"], d["front_slip_deg"], c, lw=1); ax[2].plot(d["t"], [100 * x for x in d["front_fy_use"]], c + ":", lw=.8)
    ax[3].plot(d["t"], d["rear_slip_ratio"], c, lw=1)
    ax[4].plot(d["t"], d["throttle"], c, lw=1)
ax[0].axhspan(20, 35, color="g", alpha=.08); ax[0].axhspan(-35, -20, color="g", alpha=.08)
ax[0].set_ylabel("beta [deg]"); ax[0].legend(fontsize=8); ax[0].set_ylim(-60, 60)
ax[1].set_ylabel("delta (solid), required (dotted) [deg]"); ax[1].axhline(33.6, color="k", lw=.6); ax[1].axhline(-33.6, color="k", lw=.6)
ax[2].set_ylabel("front slip [deg] / Fy use x100 (dotted)"); ax[2].set_ylim(-20, 105)
ax[3].set_ylabel("rear slip ratio"); ax[3].set_ylim(-0.2, 1.0); ax[4].set_ylabel("throttle"); ax[4].set_xlabel("t [s]")
for a in ax: a.grid(alpha=.3); a.axvline(8, color="k", lw=.5)
fig.suptitle("NA Miata (cars/mx5_na.ini) sustained power drift, R 40 m, human driver (0.15 s delay, arms k 40 N m/rad); green = 20-35 deg hold window")
fig.tight_layout(); fig.savefig(f"{R}/drift/drift_mx5.png", dpi=110); plt.close(fig)
print("wrote", f"{R}/countersteer/countersteer_before_after.png", f"{R}/drift/drift_mx5.png")
