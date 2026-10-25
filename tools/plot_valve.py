#!/usr/bin/env python3
"""Plots for results/valve (make valvetest): parking effort at idle and the 25 m/s rim-torque step, models 0/1/2."""
import sys, glob, os, csv
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
d = sys.argv[1] if len(sys.argv) > 1 else "results/valve"
def rd(fn):
    with open(fn) as f:
        r = list(csv.DictReader(f))
    return {k: [float(x[k]) for x in r] for k in r[0]} if r else {}
cars = sorted({os.path.basename(f).rsplit("_v", 1)[0] for f in glob.glob(os.path.join(d, "*_park.csv"))})
for car in cars:
    fig, ax = plt.subplots(2, 2, figsize=(12, 8))
    for m in (0, 1, 2):
        p = os.path.join(d, f"{car}_v{m}_park.csv"); s = os.path.join(d, f"{car}_v{m}_step.csv")
        if os.path.exists(p):
            a = rd(p); ax[0][0].plot(a["tq"], a["sw_deg"], label=f"model {m}")
        if os.path.exists(s):
            a = rd(s); t = a["t"]
            ax[0][1].plot(t, a["sw_deg"], label=f"model {m}")
            ax[1][0].plot(t, a["rack_force"], lw=0.8, label=f"model {m}")
            if m: ax[1][1].plot(t, [(x - y) / 1e6 for x, y in zip(a["pA"], a["pB"])], lw=0.8, label=f"model {m} dP")
            if m: ax[1][1].plot(t, [x / 1e6 for x in a["pS"]], lw=0.6, ls="--", label=f"model {m} supply")
    ax[0][0].set(xlabel="rim torque N m (idle, braked, ramp)", ylabel="steering wheel deg", title=f"{car}: parking effort")
    ax[0][1].set(xlabel="s", ylabel="steering wheel deg", title="25 m/s, 2.5 N m rim-torque step 1-2 s")
    ax[1][0].set(xlabel="s", ylabel="N", title="rack force")
    ax[1][1].set(xlabel="s", ylabel="MPa", title="valve pressures (model 0 has none)")
    for r in ax:
        for x in r: x.grid(True); x.legend(fontsize=7)
    plt.tight_layout(); plt.savefig(os.path.join(d, f"{car}_valve.png"), dpi=80); plt.close(fig)
    print("wrote", os.path.join(d, f"{car}_valve.png"))
