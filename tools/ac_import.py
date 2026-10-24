#!/usr/bin/env python3
"""ac_import.py - map an Assetto Corsa car data folder (car.ini, suspensions.ini, tyres.ini, drivetrain.ini, engine.ini,
brakes.ini, power.lut) onto a DriveChain car file.

    python3 tools/ac_import.py <ac_data_dir> <template.ini> <out.ini> [--ui ui_car.json]

The template supplies everything AC does not describe (body contact points, steering-chain masses, assist, carcass
compliance...). Output goes wherever you point it; for third-party mod data keep it OUT of version control
(the repo ignores private/).

Frame mapping (AC hardpoints are per corner, relative to the wheel centre; x = inboard, y = up, z = forward):
    ours.x = x_axle + ac.z      ours.y = track/2 - ac.x      ours.z = hub_z + ac.y        (left corner; right is mirrored)
with x_axle from WHEELBASE and CG_LOCATION, hub_z = BASEY (wheel centre relative to CG height).

Units/semantics notes (also written into the output as comments):
  * SPRING_RATE, DAMP_*, BUMP_STOP_RATE, ARB are AC wheel rates -> wheel_rate / damp_* / bumpstop_rate / arb directly.
  * MAX_TORQUE x FRONT_SHARE is used as the per-wheel front brake torque, x (1-FRONT_SHARE) per rear wheel (AC convention).
  * Tyre: RATE at the 26 psi reference + PRESSURE_SPRING_GAIN per psi -> kz_ref/kz_per_pa; DY_REF/DX_REF -> mu_y/mu_x at FZ0;
    LS_EXP*, FALLOFF_*, CX_MULT, CAMBER_GAIN, DCAMBER_* map 1:1; FRICTION_LIMIT_ANGLE -> peak_slip_deg (brush tread
    stiffness is calibrated so the brush peaks there). RELAXATION_LENGTH is NOT mapped: AC's value is an empirical
    filter constant; our relaxation emerges from carcass lateral stiffness (sigma = C_alpha / k_y, ~0.2 m for a 185 tyre).
  * INERTIA in car.ini is AC's box dimensions (w, h, l) -> box inertia m/12*(a^2+b^2).
"""
import sys, re, json, os, math, configparser

def ini(path):
    cp = configparser.ConfigParser(inline_comment_prefixes=(";", "#"), strict=False, interpolation=None)
    cp.optionxform = str.upper
    with open(path, encoding="utf-8", errors="replace") as f: cp.read_string(f.read())
    return cp
def vec(s): return [float(x) for x in re.split(r"[,\s]+", s.strip()) if x]
def f(cp, s, k, d=None):
    try:
        v = cp.get(s, k).strip()
        return float(v) if v else d
    except Exception: return d

def main():
    a = sys.argv[1:]
    if len(a) < 3: print(__doc__); sys.exit(1)
    D, tmpl, out = a[0], a[1], a[2]
    ui = a[a.index("--ui") + 1] if "--ui" in a else None
    car, sus, ty, dt, en, br = (ini(os.path.join(D, n)) for n in ["car.ini", "suspensions.ini", "tyres.ini", "drivetrain.ini", "engine.ini", "brakes.ini"])

    # --- template -> ordered dict of sections
    sects, order = {}, []
    cur = "car"; sects[cur] = []; order.append(cur)
    for line in open(tmpl):
        s = line.split(";")[0].strip()
        if s.startswith("["): cur = s[1:s.index("]")]; sects.setdefault(cur, []); order.append(cur); continue
        if "=" in s: k, v = s.split("=", 1); sects[cur].append([k.strip(), v.strip()])
    def put(sec, k, v, note=None):
        val = ", ".join(f"{x:.6g}" for x in v) if isinstance(v, (list, tuple)) else (f"{v:.6g}" if isinstance(v, float) else str(v))
        for kv in sects[sec]:
            if kv[0] == k: kv[1] = val; kv.append(note); return
        sects[sec].append([k, val, note])
    log = []

    m = f(car, "BASIC", "TOTALMASS"); put("car", "mass", m, "AC TOTALMASS")
    w, h, l = vec(car.get("BASIC", "INERTIA"))
    ms = m - 4 * 29                                    # sprung part only, AC HUB_MASS ~29 per corner
    I = [ms / 12 * (w * w + h * h), ms / 12 * (h * h + l * l), ms / 12 * (w * w + l * l)]
    put("car", "sprung_inertia", I, f"AC INERTIA box {w},{h},{l} m -> m_s/12(a^2+b^2), m_s = mass - 4*hub")
    put("car", "steer_lock_deg", f(car, "CONTROLS", "STEER_LOCK"), "AC STEER_LOCK"); put("car", "steer_ratio", f(car, "CONTROLS", "STEER_RATIO"), "AC STEER_RATIO")

    wb = f(sus, "BASIC", "WHEELBASE"); cgf = f(sus, "BASIC", "CG_LOCATION")
    xax = {"FRONT": wb * (1 - cgf), "REAR": -wb * cgf}
    for S, sec in [("FRONT", "front"), ("REAR", "rear")]:
        tr = f(sus, S, "TRACK"); hz = f(sus, S, "BASEY"); x0 = xax[S]
        if sus.get(S, "TYPE").strip().upper() != "DWB": log.append(f"{S}: TYPE {sus.get(S, 'TYPE')} not mapped (template kept)"); continue
        def P(k):
            ax, ay, az = vec(sus.get(S, k)); return [x0 + az, tr / 2 - ax, hz + ay]
        put(sec, "type", 0); put(sec, "nlinks", 5); put(sec, "rack_link", 4); put(sec, "lower_arm_a", 2); put(sec, "lower_arm_b", 3)
        put(sec, "hub", [x0, tr / 2, hz], f"AC TRACK {tr}, BASEY {hz}, axle x from WB {wb} & CG {cgf}")
        for i, (c, u) in enumerate([("WBCAR_TOP_FRONT", "WBTYRE_TOP"), ("WBCAR_TOP_REAR", "WBTYRE_TOP"), ("WBCAR_BOTTOM_FRONT", "WBTYRE_BOTTOM"),
                                    ("WBCAR_BOTTOM_REAR", "WBTYRE_BOTTOM"), ("WBCAR_STEER", "WBTYRE_STEER")]):
            put(sec, f"link{i}_c", P(c), "AC " + c); put(sec, f"link{i}_u", P(u), "AC " + u)
        put(sec, "toe_rod_delta", f(sus, S, "TOE_OUT"), "AC TOE_OUT"); put(sec, "static_camber_deg", f(sus, S, "STATIC_CAMBER"), "AC STATIC_CAMBER")
        put(sec, "wheel_rate", f(sus, S, "SPRING_RATE"), "AC SPRING_RATE (wheel rate)"); put(sec, "spring_rate", 0.0)
        put(sec, "bumpstop_rate", f(sus, S, "BUMP_STOP_RATE")); put(sec, "bumpstop_up", f(sus, S, "BUMPSTOP_UP")); put(sec, "bumpstop_dn", f(sus, S, "BUMPSTOP_DN"))
        for k, ak in [("damp_bump", "DAMP_BUMP"), ("damp_fast_bump", "DAMP_FAST_BUMP"), ("damp_bump_thr", "DAMP_FAST_BUMPTHRESHOLD"),
                      ("damp_reb", "DAMP_REBOUND"), ("damp_fast_reb", "DAMP_FAST_REBOUND"), ("damp_reb_thr", "DAMP_FAST_REBOUNDTHRESHOLD")]:
            put(sec, k, f(sus, S, ak), "AC " + ak)
        put(sec, "arb", f(sus, "ARB", S), "AC ARB " + S)
        put(sec, "unsprung_mass", f(sus, S, "HUB_MASS"), "AC HUB_MASS")
        log.append(f"{S}: hub ({x0:.4f}, {tr/2:.4f}, {hz:.3f})")
        # tyre
        T = S
        psi = 6894.757
        put(sec, "tyre_radius", f(ty, T, "RADIUS")); put(sec, "tyre_rim_radius", f(ty, T, "RIM_RADIUS")); put(sec, "tyre_width", f(ty, T, "WIDTH"))
        put(sec, "tyre_spin_inertia", f(ty, T, "ANGULAR_INERTIA")); put(sec, "tyre_cz", f(ty, T, "DAMP"))
        # AC: rate = RATE + PRESSURE_SPRING_GAIN*(P - PRESSURE_STATIC). Our reference point is moved to PRESSURE_IDEAL
        # (same linear law) because the brush is calibrated (peak slip, patch length) at pressure_ref, and AC's
        # grip numbers describe the tyre at its hot operating pressure.
        p_st = f(ty, T, "PRESSURE_STATIC", 26.0); p_id = f(ty, T, "PRESSURE_IDEAL", p_st); gain = f(ty, T, "PRESSURE_SPRING_GAIN")
        rate = f(ty, T, "RATE")
        put(sec, "tyre_kz_ref", rate + gain * (p_id - p_st), f"AC RATE {rate:.0f} @ {p_st:g} psi + GAIN*(IDEAL-STATIC)")
        put(sec, "tyre_pressure_ref", p_id * psi, "AC PRESSURE_IDEAL psi -> Pa")
        put(sec, "tyre_kz_per_pa", gain / psi, "AC PRESSURE_SPRING_GAIN per psi -> per Pa")
        put(sec, "tyre_pressure", p_id * psi, "AC PRESSURE_IDEAL (hot) psi")
        put(sec, "tyre_kz_flat", 0.1 * rate, "not in AC: carcass-only rate at 0 gauge (puncture); AC's linear law is unphysical below ~10 psi")
        put(sec, "tyre_mu_y", f(ty, T, "DY_REF", f(ty, T, "DY0")), "AC DY_REF"); put(sec, "tyre_mu_x", f(ty, T, "DX_REF", f(ty, T, "DX0")), "AC DX_REF")
        put(sec, "tyre_fz0", f(ty, T, "FZ0")); put(sec, "tyre_ls_exp_y", f(ty, T, "LS_EXPY")); put(sec, "tyre_ls_exp_x", f(ty, T, "LS_EXPX"))
        put(sec, "tyre_falloff_level", f(ty, T, "FALLOFF_LEVEL")); put(sec, "tyre_falloff_speed", f(ty, T, "FALLOFF_SPEED"))
        put(sec, "tyre_cx_mult", f(ty, T, "CX_MULT")); put(sec, "tyre_camber_gain", f(ty, T, "CAMBER_GAIN"))
        put(sec, "tyre_dcamber0", f(ty, T, "DCAMBER_0")); put(sec, "tyre_dcamber1", f(ty, T, "DCAMBER_1"))
        put(sec, "tyre_peak_slip_deg", f(ty, T, "FRICTION_LIMIT_ANGLE"), "AC FRICTION_LIMIT_ANGLE (brush calibration target)")
        # brakes
        mt, fs = f(br, "DATA", "MAX_TORQUE"), f(br, "DATA", "FRONT_SHARE")
        put(sec, "brake_torque", mt * (fs if S == "FRONT" else 1 - fs), "AC MAX_TORQUE x share (per wheel)")
        put(sec, "handbrake_torque", 0.0 if S == "FRONT" else f(br, "DATA", "HANDBRAKE_TORQUE"))
        put(sec, "driven", 1 if (dt.get("TRACTION", "TYPE").strip().upper() == "RWD") == (S == "REAR") else 0)

    put("car", "drive", {"RWD": 0, "FWD": 1, "AWD": 2}[dt.get("TRACTION", "TYPE").strip().upper()])
    n = int(f(dt, "GEARS", "COUNT")); put("car", "ngears", n)
    put("car", "gears", [f(dt, "GEARS", f"GEAR_{i}") for i in range(1, n + 1)], "AC GEARS")
    put("car", "gear_reverse", f(dt, "GEARS", "GEAR_R")); put("car", "final_drive", f(dt, "GEARS", "FINAL"))
    put("car", "diff_power", f(dt, "DIFFERENTIAL", "POWER")); put("car", "diff_coast", f(dt, "DIFFERENTIAL", "COAST")); put("car", "diff_preload", f(dt, "DIFFERENTIAL", "PRELOAD"))
    put("car", "gearbox_inertia", f(dt, "GEARBOX", "INERTIA", 0.02)); put("car", "clutch_torque", f(dt, "CLUTCH", "MAX_TORQUE"))
    put("car", "engine_inertia", f(en, "ENGINE_DATA", "INERTIA")); put("car", "limiter_rpm", f(en, "ENGINE_DATA", "LIMITER")); put("car", "idle_rpm", f(en, "ENGINE_DATA", "MINIMUM"))
    put("car", "coast_rpm", f(en, "COAST_REF", "RPM")); put("car", "coast_torque", f(en, "COAST_REF", "TORQUE"))
    lut = [vec(l.replace("|", " ")) for l in open(os.path.join(D, en.get("HEADER", "POWER_CURVE").strip())) if "|" in l]
    sects["engine_lut"] = [[f"{r:g}", f"{t:g}"] for r, t in lut]
    log.append(f"engine: {len(lut)} lut points, peak {max(t for _, t in lut):.0f} N m @ {max(lut, key=lambda x: x[1])[0]:.0f} rpm")
    if ui:
        u = json.loads(open(ui, encoding="utf-8", errors="replace").read(), strict=False)
        tc = {float(r): float(t) for r, t in u.get("torqueCurve", [])}
        ratios = [tc[r] / t for r, t in lut if r in tc and t > 0 and r >= 1000]
        log.append(f"ui_car.json: {u['specs']['torque']} {u['specs']['bhp']} {u['specs']['weight']}; ui/power.lut torque ratio "
                   f"{min(ratios):.3f}..{max(ratios):.3f} (mean {sum(ratios)/len(ratios):.3f}) -> power.lut is ~wheel torque (drivetrain loss), used as-is")

    with open(out, "w") as o:
        o.write(f"; generated by tools/ac_import.py from {os.path.abspath(D)} - PRIVATE third-party data, do not publish\n")
        o.write("; AC RELAXATION_LENGTH not mapped (different meaning); relaxation emerges from tyre_carcass_ky.\n")
        for s in dict.fromkeys(order):
            o.write(f"\n[{s}]\n")
            for kv in sects[s]:
                o.write(f"{kv[0]} = {kv[1]}" + (f"   ; {kv[2]}" if len(kv) > 2 and kv[2] else "") + "\n")
    for x in log: print(x)
    print("wrote", out)

main()
