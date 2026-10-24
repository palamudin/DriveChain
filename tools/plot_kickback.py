#!/usr/bin/env python3
"""Kickback plots from a kickback harness run dir (build/drivetest kickback ...):
   kick_traces_<event>_<kmh>.png  rim angle / column torque / heading / lateral for every driver model
   kick_attrib_<event>_<kmh>.png  kingpin-moment source breakdown (Fy*trail, Fx*scrub, Mz, Fz jacking, gyro)
   kick_overview.png              per-event peaks (column torque, rim excursion, heading change) for arms20/hands_off/locked
usage: plot_kickback.py RUNDIR [event_kmh ...]   (default: bump_left_60 bump_left_100 pothole_left_60 bumpy_corner_70)"""
import csv, sys, os
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt

DRIVERS = ["hands_off", "locked", "arms15_slow", "arms20", "arms25_quick", "gamepad_arms"]
def load(p):
    with open(p) as f:
        r = csv.DictReader(f); rows = list(r)
    return {k: [float(x[k]) for x in rows] for k in rows[0]} if rows else None

def traces(d, ev):
    fig, ax = plt.subplots(4, 1, figsize=(9, 10), sharex=True)
    for drv in DRIVERS:
        p = os.path.join(d, f"kick_{ev}_{drv}.csv")
        if not os.path.exists(p): continue
        c = load(p); x = c["ds_front"]
        ax[0].plot(x, c["sw_deg"], label=drv); ax[1].plot(x, c["column_nm"], label=drv)
        ax[2].plot(x, c["head_err_deg"], label=drv); ax[3].plot(x, c["lat_m"], label=drv)
    for a, l in zip(ax, ["rim angle [deg]", "column torque [N m]", "heading change [deg]", "lateral drift [m]"]): a.set_ylabel(l); a.grid(alpha=.3)
    ax[3].set_xlabel("distance past the feature (front axle) [m]"); ax[0].legend(fontsize=8, ncol=3)
    ax[0].set_title(f"kickback {ev} ({os.path.basename(d.rstrip('/'))})"); fig.tight_layout()
    fig.savefig(os.path.join(d, f"kick_traces_{ev}.png"), dpi=110); plt.close(fig)

def attrib(d, ev):
    p = os.path.join(d, f"kick_{ev}_arms20.csv")
    if not os.path.exists(p): return
    c = load(p); x = c["ds_front"]
    fig, ax = plt.subplots(figsize=(9, 4.5))
    for k, l in [("M_Fy", "Fy x trail"), ("M_Fx", "Fx x scrub"), ("M_Mz", "tyre Mz"), ("M_Fz", "Fz jacking (KPI/caster)"), ("M_gyro", "wheel gyro")]:
        ax.plot(x, c[k], label=l)
    ax.set_xlabel("distance past the feature [m]"); ax.set_ylabel("kingpin moment, struck side [N m]"); ax.grid(alpha=.3); ax.legend(fontsize=8)
    ax.set_title(f"kickback source breakdown {ev} (arms20)"); fig.tight_layout(); fig.savefig(os.path.join(d, f"kick_attrib_{ev}.png"), dpi=110); plt.close(fig)

def overview(d):
    with open(os.path.join(d, "kickback_summary.csv")) as f: rows = list(csv.DictReader(f))
    evs = []
    for r in rows:
        k = f"{r['event']}_{r['kmh']}"
        if k not in evs: evs.append(k)
    fig, ax = plt.subplots(3, 1, figsize=(12, 10), sharex=True); w = 0.27
    for j, drv in enumerate(["hands_off", "locked", "arms20"]):
        v = {f"{r['event']}_{r['kmh']}": r for r in rows if r["driver"] == drv}
        xs = [i + (j - 1) * w for i in range(len(evs))]
        ax[0].bar(xs, [float(v[e]["col_pk_nm"]) for e in evs], w, label=drv)
        ax[1].bar(xs, [float(v[e]["rim_pk_deg"]) for e in evs], w, label=drv)
        ax[2].bar(xs, [abs(float(v[e]["dpsi_pk_deg"])) for e in evs], w, label=drv)
    for a, l in zip(ax, ["peak column torque [N m]", "peak rim excursion [deg]", "|peak heading change| [deg]"]): a.set_ylabel(l); a.grid(alpha=.3, axis="y"); a.set_yscale("symlog", linthresh=1)
    ax[2].set_xticks(range(len(evs))); ax[2].set_xticklabels(evs, rotation=40, ha="right", fontsize=8); ax[0].legend()
    ax[0].set_title(f"kickback overview ({os.path.basename(d.rstrip('/'))})"); fig.tight_layout(); fig.savefig(os.path.join(d, "kick_overview.png"), dpi=110); plt.close(fig)

if __name__ == "__main__":
    d = sys.argv[1]; evs = sys.argv[2:] or ["bump_left_60", "bump_left_100", "pothole_left_60", "bumpy_corner_70"]
    for e in evs: traces(d, e); attrib(d, e)
    overview(d); print("plots written to", d)
