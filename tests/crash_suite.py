#!/usr/bin/env python3
"""Abuse-load regression: kerb strikes, wall strikes and ledge drops on both cores (build/kerbstrike).
Checks (each must hold on solver 0 and 1):
  kerb   135 mm square kerb, head-on, 25 and 40 km/h: no blow-up, peak single-tyre load in the band measured for a
         rim-clash on a square step (Ford "durability of kerb" style events: ~5-15x static); 50 mm at 60 km/h stays tyre-borne
  wall   0.3-1.3 m faces at 10/30/90 deg, 30/60 km/h: no blow-up, no kinetic-energy gain (no drive torque)
  drop   0.5 / 1.0 / 1.5 m ledges at 20/50/80 km/h: no blow-up, kinetic energy gain <= m g h (+5 %), travel stays at the solid stop
usage: crash_suite.py <car.ini> [outdir]"""
import subprocess, sys, csv, os
car = sys.argv[1] if len(sys.argv) > 1 else "cars/mx5_na.ini"
out = sys.argv[2] if len(sys.argv) > 2 else "results/crash"
os.makedirs(out, exist_ok=True)
mass = None
for ln in open(car):
    k = ln.split("=")
    if len(k) == 2 and k[0].strip() == "mass": mass = float(k[1].split()[0].rstrip(","))
mass = mass or 1050.0
tmp = os.path.join(out, "_run.csv"); rows = []; fails = 0
def run(s, args):
    r = subprocess.run(["./build/kerbstrike", car, str(s)] + args + [tmp], capture_output=True, text=True)
    ok = "RESULT OK" in r.stdout
    D = list(csv.DictReader(open(tmp))) if os.path.exists(tmp) else []
    return ok, D, r.stdout
def check(name, cond, msg):
    global fails
    print(("[PASS] " if cond else "[FAIL] ") + f"{name:44s} {msg}"); fails += 0 if cond else 1
    rows.append((name, "PASS" if cond else "FAIL", msg))
for s in (0, 1):
    for kmh, lo, hi in ((25, 8e3, 30e3), (40, 15e3, 45e3)):
        ok, D, _ = run(s, ["step", "0.135", "0", "90", str(kmh)])
        pk = max(max(float(d[f"fz{i}"]) for i in range(4)) for d in D) if D else 0
        check(f"kerb s{s} 135mm head-on {kmh} km/h", ok and lo <= pk <= hi, f"peak tyre load {pk/1e3:.1f} kN (band {lo/1e3:.0f}-{hi/1e3:.0f} kN)")
    ok, D, _ = run(s, ["step", "0.05", "0", "5", "60"])
    pk = max(max(float(d[f"fz{i}"]) for i in range(4)) for d in D) if D else 0
    check(f"kerb s{s} 50mm 5deg 60 km/h", ok and pk < 8e3, f"peak tyre load {pk/1e3:.1f} kN (< 8 kN: tyre-borne)")
    worst = (0, ""); allok = True
    for h in ("0.3", "0.5", "1.0", "1.3"):
        for a in ("90", "30", "10"):
            for v in ("30", "60"):
                ok, D, _ = run(s, ["step", h, "0", a, v]); allok &= ok
                if D:
                    # mechanical energy E = KE + PE (gravity, springs, tyre carcass, rim, body elastic). The car coasts
                    # in neutral with no energy source, so no instant may hold more than it started with: any excess is
                    # energy created by a contact. (Dissipation bookkeeping is not used: it only shifts where losses go.)
                    k0 = float(D[0]["ke"]); E0 = k0 + float(D[0]["pe"])
                    g = max(float(d["ke"]) + float(d["pe"]) for d in D) - E0
                    if g / k0 > worst[0]: worst = (g / k0, f"h{h} a{a} v{v} ({g:.0f} J)")
    check(f"wall s{s} 24 cases", allok and worst[0] < 0.01, f"all finite; worst KE+PE rise above start {100*worst[0]:.2f}% of KE0 {worst[1]}")
    for h in ("0.5", "1.0", "1.5"):
        for v in ("20", "50", "80"):
            ok, D, _ = run(s, ["drop", h, v])
            if not D: check(f"drop s{s} {h} m {v} km/h", False, "no output"); continue
            k0 = float(D[0]["ke"]); km = max(float(d["ke"]) for d in D)
            tr = max(max(float(d[f"trav{i}"]) for i in range(4)) for d in D)
            fz = max(max(float(d[f"fz{i}"]) for i in range(4)) for d in D)
            lim = mass * 9.81 * float(h) * 1.05
            check(f"drop s{s} {h} m {v} km/h", ok and km - k0 <= lim and tr < 0.2,
                  f"KE gain {(km-k0)/1e3:.1f} kJ <= mgh {lim/1e3:.1f} kJ, max travel {tr*1000:.0f} mm, peak tyre {fz/1e3:.1f} kN")
os.remove(tmp)
with open(os.path.join(out, "crash_summary.csv"), "w") as f:
    f.write("check,result,detail\n"); [f.write(f'"{a}",{b},"{c}"\n') for a, b, c in rows]
print(f"crash suite: {len(rows)-fails}/{len(rows)} passed"); sys.exit(1 if fails else 0)
