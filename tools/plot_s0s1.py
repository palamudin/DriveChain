#!/usr/bin/env python3
"""Before/after plots, legacy reduced core (solver 0) vs coupled multibody core (solver 1).
plot_s0s1.py [drivetest_root=results/drivetest_mb] [dcdrive_root=results/driver] [out=results/plots]"""
import csv, os, sys
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
DT = sys.argv[1] if len(sys.argv) > 1 else "results/drivetest_mb"
DD = sys.argv[2] if len(sys.argv) > 2 else "results/driver"
OUT = sys.argv[3] if len(sys.argv) > 3 else "results/plots"
os.makedirs(OUT, exist_ok=True)
CORES = [("s0", "legacy", "tab:blue"), ("s1", "multibody", "tab:red")]
def load(p):
    if not os.path.exists(p): return None
    with open(p) as f: r = list(csv.DictReader(f))
    if not r: return None
    D = {}
    for k in r[0]:
        try: D[k] = [float(x[k]) for x in r]
        except (ValueError, TypeError): pass
    return D
# 1. hands-off countersteer: front-axle slip angle beta_f vs road-wheel angle delta (wheels follow the axle up to lock)
sc = ["lift_60kmh_R32", "lift_85kmh_R65", "power_40kmh_R20", "power_60kmh_R40"]
fig, ax = plt.subplots(len(sc), 2, figsize=(14, 3.0 * len(sc)))
for i, s in enumerate(sc):
    for s_, lab, col in CORES:
        d = load(f"{DT}/{s_}/counter/cs_run_{s}_hands_off.csv")
        if not d: continue
        ax[i][0].plot(d["t"], d["beta_front_deg"], color=col, ls="--", lw=1, label=f"beta_f {lab}")
        ax[i][0].plot(d["t"], d["delta_deg"], color=col, lw=1.3, label=f"delta {lab}")
        ax[i][1].plot(d["t"], d["sw_deg"], color=col, lw=1.3, label=f"rim {lab}")
        ax[i][1].plot(d["t"], d["yaw_rate_dps"], color=col, ls=":", lw=1, label=f"yaw rate {lab}")
    ax[i][0].set_title(f"hands-off {s}: front-axle slip vs road-wheel angle [deg]"); ax[i][1].set_title(f"hands-off {s}: rim [deg], yaw rate [deg/s]")
    for a in ax[i]: a.grid(alpha=.3); a.legend(fontsize=7)
fig.tight_layout(); fig.savefig(f"{OUT}/countersteer_s0_vs_s1.png", dpi=85); plt.close(fig)
# 2. sustained drift (arms driver), body slip and rim
runs = ["arms_b22", "arms_b27", "arms_b32", "gp_b27"]
fig, ax = plt.subplots(len(runs), 2, figsize=(14, 2.8 * len(runs)))
for i, r in enumerate(runs):
    for s_, lab, col in CORES:
        d = load(f"{DT}/{s_}/drift/drift_run_{r}.csv")
        if not d or "t" not in d: continue
        for j, k in enumerate(["beta_deg", "sw_deg"]):
            if k in d: ax[i][j].plot(d["t"], d[k], color=col, lw=1.2, label=lab)
    ax[i][0].set_title(f"drift {r}: body slip beta [deg]"); ax[i][1].set_title(f"drift {r}: rim [deg]")
    for a in ax[i]: a.grid(alpha=.3); a.legend(fontsize=7)
fig.tight_layout(); fig.savefig(f"{OUT}/drift_s0_vs_s1.png", dpi=85); plt.close(fig)
# 3. dcdrive manoeuvres (plug-in driver with finite hands)
man = [("skidpad", "lat_err", "beta_deg"), ("slalom", "y", "sw_deg"), ("lane_change", "y", "yaw_rate_dps"),
       ("lift_off", "beta_deg", "yaw_rate_dps"), ("bump_hold", "sw_deg", "column_nm")]
fig, ax = plt.subplots(len(man), 2, figsize=(14, 2.8 * len(man)))
for i, (m, k0, k1) in enumerate(man):
    for s_, lab, col in CORES:
        d = load(f"{DD}/{s_}/drv_{s_}_{m}.csv")
        if not d: continue
        for j, k in enumerate([k0, k1]):
            if k in d: ax[i][j].plot(d["t"], d[k], color=col, lw=1.2, label=lab)
    ax[i][0].set_title(f"{m}: {k0}"); ax[i][1].set_title(f"{m}: {k1}")
    for a in ax[i]: a.grid(alpha=.3); a.legend(fontsize=7)
fig.tight_layout(); fig.savefig(f"{OUT}/dcdrive_s0_vs_s1.png", dpi=85); plt.close(fig)
print("wrote", OUT)
