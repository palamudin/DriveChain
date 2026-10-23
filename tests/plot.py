#!/usr/bin/env python3
"""plot.py - renders results/*.csv (written by build/dc_tests) to results/*.png."""
import os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

R = sys.argv[1] if len(sys.argv) > 1 else "results"
def L(n):
    p = os.path.join(R, n)
    if not os.path.exists(p): return None
    d = np.genfromtxt(p, delimiter=",", names=True, dtype=None, encoding=None)
    return d
def save(fig, n):
    fig.tight_layout(); p = os.path.join(R, n); fig.savefig(p, dpi=110); plt.close(fig); print("wrote", p)

# 1 static settle + ride-mode FFT
s, nd, an = L("static_settle.csv"), L("static_nodamp.csv"), L("static_analytic.csv")
if s is not None:
    fig, ax = plt.subplots(2, 2, figsize=(12, 7))
    ax[0,0].plot(s["t"], s["z"]*1000 - s["z"][-1]*1000, label="damped"); 
    if nd is not None: ax[0,0].plot(nd["t"], nd["z"]*1000 - nd["z"][-1]*1000, alpha=.6, label="undamped")
    ax[0,0].set(title="1 static settle: CG height vs final (mm)", xlabel="t s"); ax[0,0].legend()
    for k in ["fz0","fz1","fz2","fz3"]: ax[0,1].plot(s["t"], s[k], label=k)
    if an is not None:
        ax[0,1].axhline(float(an["Wf"]), ls="--", c="k", lw=.8, label="analytic front corner"); ax[0,1].axhline(float(an["Wr"]), ls=":", c="k", lw=.8, label="analytic rear corner")
    ax[0,1].set(title="corner loads N", xlabel="t s"); ax[0,1].legend(fontsize=7)
    ax[1,0].plot(s["t"], s["pitch_deg"]); ax[1,0].set(title="pitch deg", xlabel="t s")
    if nd is not None:
        t = nd["t"]; m = t > 0.5
        for key, lab in [("z", "heave z"), ("pitch_deg", "pitch")]:
            y = nd[key][m] - nd[key][m].mean(); dt = np.mean(np.diff(t))
            f = np.fft.rfftfreq(len(y), dt); A = np.abs(np.fft.rfft(y * np.hanning(len(y))))
            ax[1,1].semilogy(f, A / A.max(), label=lab)
        if an is not None:
            for k in ["f_mode1_hz", "f_mode2_hz"]: ax[1,1].axvline(float(an[k]), c="r", ls="--", lw=.8)
        ax[1,1].set(xlim=(0, 4), title="undamped ride-mode spectrum (red: analytic 2-DOF)", xlabel="Hz"); ax[1,1].legend()
    save(fig, "01_static_settle.png")

# 2 conservation
f, r = L("cons_flat.csv"), L("cons_rough.csv")
if f is not None:
    fig, ax = plt.subplots(2, 2, figsize=(12, 7))
    for d, lab in [(f, "flat"), (r, "rough")]:
        if d is None: continue
        ax[0,0].plot(d["t"], (d["E"] - d["E"][0]) / d["E"][0], label=lab)
        ax[0,1].plot(d["t"], np.hypot(d["px"] - d["px"][0], d["py"] - d["py"][0]), label=lab)
        ax[1,0].plot(d["t"], d["Lz"] - d["Lz"][0], label=lab)
        ax[1,1].semilogy(d["t"], np.abs(d["resF"]) + 1e-18, label=lab + " |F resid|")
    ax[0,0].set(title="2 energy drift (E-E0)/E0, undamped+frictionless", xlabel="t s"); ax[0,0].legend()
    ax[0,1].set(title="|horizontal momentum change| kg m/s", xlabel="t s"); ax[0,1].legend()
    ax[1,0].set(title="yaw angular momentum change", xlabel="t s"); ax[1,0].legend()
    ax[1,1].set(title="Newton-3 audit: |sum m a - F_ext| N", xlabel="t s"); ax[1,1].legend()
    save(fig, "02_conservation.png")

# tyre curves (calibration)
tl, tlo, tc, trx = L("tyre_lateral.csv"), L("tyre_long.csv"), L("tyre_combined.csv"), L("tyre_relax.csv")
if tl is not None:
    fig, ax = plt.subplots(2, 2, figsize=(12, 7))
    for fz in np.unique(tl["Fz"]):
        m = tl["Fz"] == fz
        ax[0,0].plot(tl["alpha_deg"][m], tl["Fy"][m], label=f"Fz {fz:.0f}"); ax[0,1].plot(tl["alpha_deg"][m], tl["Mz"][m], label=f"Fz {fz:.0f}")
    ax[0,0].set(title="brush lateral force Fy N", xlabel="slip angle deg"); ax[0,0].legend(fontsize=7)
    ax[0,1].set(title="self-aligning moment Mz N m", xlabel="slip angle deg")
    if tlo is not None:
        for fz in np.unique(tlo["Fz"]):
            m = tlo["Fz"] == fz; ax[1,0].plot(tlo["kappa"][m], tlo["Fx"][m], label=f"Fz {fz:.0f}")
        ax[1,0].set(title="longitudinal Fx N", xlabel="slip ratio"); ax[1,0].legend(fontsize=7)
    if tc is not None:
        for a in np.unique(tc["alpha_deg"]):
            m = tc["alpha_deg"] == a; ax[1,1].plot(tc["Fx"][m], tc["Fy"][m], label=f"alpha {a:.0f}")
        ax[1,1].set(title="combined slip friction ellipse (Fx vs Fy)", xlabel="Fx N", ylabel="Fy N"); ax[1,1].legend(fontsize=7); ax[1,1].axis("equal")
    save(fig, "00_tyre_curves.png")
    if trx is not None:
        fig, ax = plt.subplots(figsize=(7, 4)); ax.plot(trx["dist_m"], trx["Fy"], label="Fy transient"); ax.plot(trx["dist_m"], trx["Fy_ss"], "--", label="steady")
        ax.set(title="tyre relaxation (carcass lateral compliance)", xlabel="distance rolled m", ylabel="N"); ax.legend(); save(fig, "00_tyre_relaxation.png")

kt, kr = L("kin_travel.csv"), L("kin_rack.csv")
if kt is not None:
    fig, ax = plt.subplots(2, 3, figsize=(14, 7))
    for a, nm in [(0, "front"), (1, "rear")]:
        m = kt["axle"] == a
        for i, (k, t) in enumerate([("camber_deg", "camber deg"), ("toe_deg", "toe deg"), ("caster_deg", "caster deg"), ("kpi_deg", "KPI deg"), ("scrub_mm", "scrub mm"), ("trail_mm", "mech. trail mm")]):
            ax.flat[i].plot(kt["travel_mm"][m], kt[k][m], label=nm); ax.flat[i].set(title=t, xlabel="wheel travel mm (+bump)")
    ax.flat[0].legend(); save(fig, "00_kinematics_travel.png")
if kr is not None:
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    ax[0].plot(kr["sw_deg"], kr["delta_L_deg"], label="left"); ax[0].plot(kr["sw_deg"], kr["delta_R_deg"], label="right")
    ax[0].plot(kr["sw_deg"], kr["ideal_outer_deg"], "--", label="ideal Ackermann outer (from left)"); ax[0].set(title="road-wheel steer vs steering wheel", xlabel="sw deg"); ax[0].legend()
    ax[1].plot(kr["sw_deg"], kr["ackermann_pct"]); ax[1].set(title="emergent Ackermann %", xlabel="sw deg", ylim=(-10, 110))
    save(fig, "00_kinematics_steer.png")

# 3 skidpad
d = L("skidpad.csv")
if d is not None:
    fig, ax = plt.subplots(2, 2, figsize=(12, 7))
    ax[0,0].plot(d["ay_g"], d["sw_deg"], ".", ms=1); ax[0,0].set(title="3 skidpad: steering-wheel angle vs ay", xlabel="ay g", ylabel="sw deg")
    ax[0,1].plot(d["ay_g"], d["alpha_f_deg"], ".", ms=1, label="front"); ax[0,1].plot(d["ay_g"], d["alpha_r_deg"], ".", ms=1, label="rear"); ax[0,1].set(title="axle slip angles", xlabel="ay g"); ax[0,1].legend()
    for k in ["fz0","fz1","fz2","fz3"]: ax[1,0].plot(d["ay_g"], d[k], ".", ms=1, label=k)
    ax[1,0].set(title="corner loads (load transfer)", xlabel="ay g"); ax[1,0].legend()
    ax[1,1].plot(d["ay_g"], d["col_torque"], ".", ms=1); ax[1,1].set(title="FFB column torque N m", xlabel="ay g")
    save(fig, "03_skidpad.png")

# 4 step steer
d = L("step_steer.csv")
if d is not None:
    fig, ax = plt.subplots(2, 2, figsize=(12, 7))
    ax[0,0].plot(d["t"], d["sw_deg"]); ax[0,0].set(title="4 step steer: sw deg")
    ax[0,1].plot(d["t"], d["r_degs"]); ax[0,1].set(title="yaw rate deg/s")
    ax[1,0].plot(d["t"], d["ay_g"], label="ay g"); ax[1,0].plot(d["t"], d["roll_deg"]/10, label="roll deg/10"); ax[1,0].legend()
    ax[1,1].plot(d["t"], d["col_torque"], label="column (FFB)"); ax[1,1].plot(d["t"], d["driver_torque"], label="driver"); ax[1,1].legend(); ax[1,1].set(title="N m")
    for a in ax.flat: a.set_xlabel("t s")
    save(fig, "04_step_steer.png")

# 5 hands-off
ho = [(L(n), lab) for n, lab in [("handsoff.csv", "hands off (hydraulic)"), ("handsoff_manual.csv", "hands off (manual rack)"), ("handsoff_held.csv", "hands clamped")]]
if ho[0][0] is not None:
    fig, ax = plt.subplots(2, 2, figsize=(12, 7))
    for d, lab in ho:
        if d is None: continue
        ax[0,0].plot(d["t"], d["sw_deg"], label=lab); ax[0,1].plot(d["t"], d["r_degs"], label=lab)
        ax[1,0].plot(d["t"], d["beta_deg"], label=lab); ax[1,1].plot(d["t"], d["col_torque"], label=lab)
    for a, t in zip(ax.flat, ["5 hands-off oversteer: steering wheel deg", "yaw rate deg/s", "body slip beta deg", "column torque N m"]):
        a.set(title=t, xlabel="t s"); a.legend(fontsize=7)
    save(fig, "05_handsoff.png")

# 6 kerb
a, b = L("kerb_fan.csv"), L("kerb_single.csv")
if a is not None:
    fig, ax = plt.subplots(2, 2, figsize=(12, 7))
    for d, lab in [(b, "single ray"), (a, "envelope")]:
        if d is None: continue
        ax[0,0].plot(d["x"], d["fz0"], label=lab); ax[0,1].plot(d["x"], d["col_torque"], label=lab)
        ax[1,0].plot(d["x"], d["travel0_mm"], label=lab); ax[1,1].plot(d["x"], d["az_g"], label=lab)
    for x, t in zip(ax.flat, ["6 kerb strike: FL Fz N", "column torque (FFB) N m", "FL wheel travel mm", "body vertical accel g"]):
        x.set(title=t, xlabel="x m"); x.legend()
    save(fig, "06_kerb.png")

# 7 puncture
d = L("puncture.csv")
if d is not None:
    fig, ax = plt.subplots(2, 2, figsize=(12, 7))
    ax[0,0].plot(d["t"], d["pressure_kpa"]); ax[0,0].set(title="7 puncture FR: pressure kPa")
    ax[0,1].plot(d["t"], d["defl1_mm"], label="deflection mm"); ax[0,1].plot(d["t"], d["patch1_mm"], label="patch half-length mm"); ax[0,1].legend()
    ax[1,0].plot(d["t"], d["fz1"], label="Fz"); ax[1,0].plot(d["t"], d["rim1"], label="rim force"); ax[1,0].legend(); ax[1,0].set(title="N")
    ax[1,1].plot(d["t"], d["sw_deg"], label="sw deg"); ax[1,1].plot(d["t"], d["col_torque"], label="column N m"); ax[1,1].plot(d["t"], d["r_degs"], label="yaw deg/s"); ax[1,1].legend()
    for x in ax.flat: x.set_xlabel("t s")
    save(fig, "07_puncture.png")

# 8 assist
a, b = L("assist_on.csv"), L("assist_off.csv")
if a is not None:
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.5))
    for d, lab in [(b, "manual"), (a, "hydraulic")]:
        for i, ph in enumerate(np.unique(d["phase"])[:2]):
            m = d["phase"] == ph
            ax[i].plot(d["sw_deg"][m], d["driver_torque"][m], label=lab)
            ax[i].set(title=f"8 assist, phase {ph}: driver torque vs sw", xlabel="sw deg", ylabel="N m")
        ax[2].plot(d["t"], d["assist_force"], label=lab)
    ax[2].set(title="assist rack force N", xlabel="t s")
    for x in ax: x.legend()
    save(fig, "08_assist.png")

# 9 perf
d = L("perf.csv")
if d is not None:
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.barh([str(c) for c in d["config"]], d["us_per_substep"]); ax.axvline(500, c="r", ls="--", label="real-time at 2 kHz (500 us)"); ax.axvline(100, c="orange", ls="--", label="5x budget (100 us)")
    ax.set(title="9 perf: us per 2 kHz substep (one car, one core)", xlabel="us"); ax.legend()
    save(fig, "09_perf.png")
