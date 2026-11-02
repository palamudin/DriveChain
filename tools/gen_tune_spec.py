#!/usr/bin/env python3
"""gen_tune_spec.py - writes demo/ui/tune_spec.json, the ONE tuning spec shared by the native demo menu (demo/tune.c)
and the Unity GUI package (DcTuneSpec.cs): pages, rows (keys, ranges, units, steps, detents) and the live info
panels (derived vars + readout expressions + chart kind).

It also carries a reference evaluator for the expression language and, with --check <car.ini>, prints every var and
readout so the C and C# evaluators can be compared against it (tools/tune_check.sh).

Expression language (identical in C and C#): numbers, + - * / ^ (right assoc), unary -, parens, comparisons < > <= >=
(1/0), identifiers = a var (earlier in "vars") or a car-file key "section.key" with optional "#n" list index,
functions sqrt abs min max pow sin cos tan atan atan2 deg rad clamp sel(c,a,b) axis(w) torque(rpm) gear(i) vgear(i) vtop()
pmax() and constants pi g rho. Unknown / missing -> NaN, shown as "-".
"""
import json, math, re, sys, os

G = 9.81
RHO = 1.225
CRR = 0.012   # rolling-resistance coefficient for the top-speed estimate (sim: tyre_rr_factor model; estimate)
ETA = 0.90    # driveline efficiency for the top-speed estimate (estimate)

# ---------------------------------------------------------------------------------------------- vars (in order)
VARS = [
    ("m", "car.mass"),
    ("a", "front.hub#0"),                       # CG -> front axle (car frame origin = CG)
    ("b", "-rear.hub#0"),
    ("L", "a + b"),
    ("tf", "2*front.hub#1"),
    ("tr", "2*rear.hub#1"),
    ("rf", "front.tyre_radius"),
    ("rr", "rear.tyre_radius"),
    ("h", "rf - front.hub#2"),                  # CG height at design ride height (est.)
    ("wf", "b/L"),                              # static front weight fraction
    ("fzf", "m*g*wf/2"), ("fzr", "m*g*(1-wf)/2"),
    ("msf", "m*wf/2 - front.unsprung_mass"), ("msr", "m*(1-wf)/2 - rear.unsprung_mass"),
    ("kf", "front.wheel_rate"), ("kr", "rear.wheel_rate"),
    ("fnf", "sqrt(kf/msf)/(2*pi)"), ("fnr", "sqrt(kr/msr)/(2*pi)"),
    ("ccf", "2*sqrt(kf*msf)"), ("ccr", "2*sqrt(kr*msr)"),
    ("zbf", "front.damp_bump/ccf"), ("zrf", "front.damp_reb/ccf"),
    ("zbr", "rear.damp_bump/ccr"), ("zrr", "rear.damp_reb/ccr"),
    ("kpf", "(kf + front.arb)*tf^2/2"), ("kpr", "(kr + rear.arb)*tr^2/2"),
    ("rsf", "100*kpf/(kpf + kpr)"),
    ("rollg", "deg(m*g*h/(kpf + kpr))"),        # roll gradient, roll axis at ground (est.)
    ("sagf", "1000*fzf/kf"), ("sagr", "1000*fzr/kr"),
    # steering
    ("rwl", "car.steer_lock_deg/car.steer_ratio"),
    ("turns", "2*car.steer_lock_deg/360"),
    ("tcirc", "2*(L/sin(rad(rwl)) + tf/2)"),
    ("caster", "deg(axis(0))"),
    ("kpi", "deg(axis(1))"),
    ("toef", "2*1000*2*rf*sin(rad(front.static_toe_deg))"), ("toer", "2*1000*2*rr*sin(rad(rear.static_toe_deg))"),
    # brakes
    ("tbf", "front.brake_torque"), ("tbr", "rear.brake_torque"),
    ("bias", "100*tbf/(tbf + tbr)"),
    ("dmax", "2*(tbf/rf + tbr/rr)/(m*g)"),      # full-pedal decel [g] if nothing locks
    ("mxf", "front.tyre_mu_x"), ("mxr", "rear.tyre_mu_x"),
    ("fzbf", "m*g*(b + h*dmax)/L/2"), ("fzbr", "max(m*g*(a - h*dmax)/L/2, 0)"),
    ("limf", "mxf*front.tyre_fz0*(fzbf/front.tyre_fz0)^front.tyre_ls_exp_x*rf"),
    ("limr", "mxr*rear.tyre_fz0*(fzbr/rear.tyre_fz0)^rear.tyre_ls_exp_x*rr"),
    ("dlf", "mxf*m*g*b/(2*L)*rf/(tbf/dmax - mxf*m*g*h/(2*L)*rf)"),   # decel at which the front locks (linear tyre, est.)
    ("dlr", "mxr*m*g*a/(2*L)*rr/(tbr/dmax + mxr*m*g*h/(2*L)*rr)"),
    ("dlock", "min(sel(dlf > 0, dlf, 99), dlr)"),
    # tyres
    ("kzf", "front.tyre_kz_ref + front.tyre_kz_per_pa*(front.tyre_pressure - front.tyre_pressure_ref)"),
    ("kzr", "rear.tyre_kz_ref + rear.tyre_kz_per_pa*(rear.tyre_pressure - rear.tyre_pressure_ref)"),
    ("dzf", "1000*fzf/kzf"), ("dzr", "1000*fzr/kzr"),
    ("plf", "1000*front.tyre_patch_factor*sqrt(2*rf*dzf/1000)*2"), ("plr", "1000*rear.tyre_patch_factor*sqrt(2*rr*dzr/1000)*2"),
    ("csf", "100*(0.35 + 0.65*min(front.tyre_pressure/front.tyre_pressure_ref, 1.5))"),
    ("csr", "100*(0.35 + 0.65*min(rear.tyre_pressure/rear.tyre_pressure_ref, 1.5))"),
    ("hotf", "(front.tyre_pressure/1000 + 101.3)*(273 + 50)/(273 + 20) - 101.3"),   # ideal gas, 20 -> 50 C (est.)
    ("hotr", "(rear.tyre_pressure/1000 + 101.3)*(273 + 50)/(273 + 20) - 101.3"),
    ("winf", "100*clamp(1 - ((hotf - 225)/45)^2, 0, 1)"), ("winr", "100*clamp(1 - ((hotr - 225)/45)^2, 0, 1)"),
    # drivetrain
    ("tpk", "tmax()"), ("ppk", "pmax()"),
    ("axt1", "tpk*gear(1)*car.final_drive"),
    ("lockp", "car.diff_preload + car.diff_power*axt1"),
    ("lockc", "car.diff_preload + car.diff_coast*car.coast_torque*gear(1)*car.final_drive"),
    ("visc100", "car.diff_viscous*100*2*pi/60"),
    # gearing
    ("v1", "vgear(1)"), ("v2", "vgear(2)"), ("v3", "vgear(3)"), ("v4", "vgear(4)"), ("v5", "vgear(5)"), ("v6", "vgear(6)"),
    ("d12", "car.limiter_rpm*(1 - gear(2)/gear(1))"), ("d23", "car.limiter_rpm*(1 - gear(3)/gear(2))"),
    ("d34", "car.limiter_rpm*(1 - gear(4)/gear(3))"), ("d45", "car.limiter_rpm*(1 - gear(5)/gear(4))"),
    ("vt", "vtop()"),
    # aero
    ("q100", "0.5*rho*(100/3.6)^2"), ("q200", "0.5*rho*(200/3.6)^2"),
    ("dr200", "car.cda*q200"),
    ("df200f", "car.cla_front*q200"), ("df200r", "car.cla_rear*q200"),
    ("abal", "sel(abs(car.cla_front + car.cla_rear) > 0.0001, 100*car.cla_front/(car.cla_front + car.cla_rear), 50)"),
    ("pdrag", "car.cda*0.5*rho*(vt/3.6)^3/1000"),
    # weight
    ("izz", "car.sprung_inertia#2 + 2*front.unsprung_mass*(a^2 + (tf/2)^2) + 2*rear.unsprung_mass*(b^2 + (tr/2)^2)"),
    ("pw", "ppk/(m/1000)"),
]

# --------------------------------------------------------------------------------------------- rows + panels
def R(k, l, u, mn, mx, st, d, k2=None, s=1, lim=None, i="", fine=None, opts=None, vals=None, t="car", ro=0, id=None, rs=0):
    r = {"k": k, "l": l, "u": u, "min": mn, "max": mx, "st": st, "d": d}
    if k2: r["k2"] = k2
    if s != 1: r["s"] = s
    if lim: r["lim"] = lim
    if fine is not None: r["fine"] = fine
    if opts: r["opts"] = opts; r["vals"] = vals
    if t != "car": r["t"] = t
    if ro: r["ro"] = 1
    if id: r["id"] = id
    if rs: r["rs"] = rs          # demo keys: 0 = live, 1 = applied on Drive / Restart (sim re-init), 2 = needs a program restart
    r["i"] = i
    return r

def O(l, e, u="", d=1, est=0, good=None, tone=None):
    o = {"l": l, "e": e, "u": u, "d": d}
    if est: o["est"] = 1
    if good: o["good"] = good
    if tone: o["tone"] = tone
    return o

FR = lambda key: ("front." + key, "rear." + key)
PAGES = [
  {"id": "align", "title": "ALIGNMENT & STEERING", "unity": "08", "panel": "align", "rows": [
    R(*FR("static_camber_deg")[:1], "Camber", "deg", -5, 5, 0.1, 1, k2="rear.static_camber_deg", lim=[-3, 0], i="Static camber at design ride height. Negative = top of the wheel leans in; more negative widens the cornering contact patch but costs braking and traction in a straight line."),
    R("front.static_toe_deg", "Toe", "deg / side", -1, 1, 0.01, 2, k2="rear.static_toe_deg", lim=[-0.2, 0.3], i="Static toe per side, + = toe-in. Front toe-out sharpens turn-in; rear toe-in adds straight-line stability."),
    R("car.steer_ratio", "Steering ratio", ":1", 10, 20, 0.1, 1, lim=[12, 17], i="Steering-wheel angle per road-wheel angle. Lower = quicker steering."),
    R("car.steer_lock_deg", "Steering lock", "deg at wheel", 360, 900, 1, 0, lim=[450, 540], i="Steering-wheel angle at full lock (each way)."),
    R("car.assist", "Power assist", "", 0, 1, 1, 0, opts=["OFF", "ON"], vals=["0", "1"], i="Hydraulic rack assist on / off (car.assist)."),
    R("car.assist_tref", "Assist threshold", "N·m", 0, 8, 0.1, 1, i="Torsion-bar torque at which the assist valve opens."),
    R("car.rack_friction", "Rack friction", "N", 0, 200, 1, 0, lim=[30, 120], i="Coulomb friction in the rack; a little gives the wheel weight on centre, too much hides the tyre."),
   ],
   "out": [O("Caster", "caster", "deg", 1, 0), O("King-pin incl.", "kpi", "deg", 1, 0),
           O("Road-wheel lock", "rwl", "deg", 1), O("Turns lock-to-lock", "turns", "", 2),
           O("Turning circle", "tcirc", "m", 1, 1), O("Total toe F", "toef", "mm", 1), O("Total toe R", "toer", "mm", 1)],
   "note": "Caster and KPI from the front ball joints (car file). Turning circle: kerb-to-kerb, simple bicycle model."},
  {"id": "springs", "title": "SPRINGS & DAMPERS", "unity": "09", "panel": "springs", "rows": [
    R("front.wheel_rate", "Wheel rate", "kN/m", 0, 30, 0.1, 1, k2="rear.wheel_rate", s=0.001, lim=[8, 25], i="Spring rate measured at the wheel (spring x motion ratio squared). Sets ride frequency and body control."),
    R("front.arb", "Anti-roll bar", "kN/m", 0, 20, 0.1, 1, k2="rear.arb", s=0.001, lim=[0, 15], i="Bar rate at the wheel. More front bar = more front roll stiffness share = more understeer."),
    R("front.preload_travel", "Ride height", "mm", -30, 30, 1, 0, k2="rear.preload_travel", s=1000, lim=[-15, 15], i="Spring preload travel: + raises the corner, - lowers it."),
    R("front.damp_bump", "Bump, slow", "N·s/m", 0, 2000, 10, 0, k2="rear.damp_bump", lim=[400, 1600], i="Low-speed bump damping: body motions under roll, pitch and heave."),
    R("front.damp_fast_bump", "Bump, fast", "N·s/m", 0, 1400, 10, 0, k2="rear.damp_fast_bump", i="High-speed bump damping past the knee: kerbs and sharp bumps."),
    R("front.damp_reb", "Rebound, slow", "N·s/m", 0, 2900, 10, 0, k2="rear.damp_reb", lim=[800, 2600], i="Low-speed rebound damping: how fast the body recovers."),
    R("front.damp_fast_reb", "Rebound, fast", "N·s/m", 0, 2500, 10, 0, k2="rear.damp_fast_reb", i="High-speed rebound past the knee."),
    R("front.bumpstop_up", "Bump-stop gap", "mm", 0, 120, 1, 0, k2="rear.bumpstop_up", s=1000, lim=[30, 90], i="Free bump travel before the bump rubber engages."),
   ],
   "out": [O("Ride freq F", "fnf", "Hz", 2, 0, [1.0, 2.2]), O("Ride freq R", "fnr", "Hz", 2, 0, [1.0, 2.2]),
           O("Damping ζ bump F/R", "zbf", "", 2, 0, [0.15, 0.7]), O("", "zbr", "", 2, 0, [0.15, 0.7]),
           O("Damping ζ reb F/R", "zrf", "", 2, 0, [0.25, 0.9]), O("", "zrr", "", 2, 0, [0.25, 0.9]),
           O("Roll stiffness F", "rsf", "%", 1, 0, [45, 70]), O("Roll gradient", "rollg", "deg/g", 2, 1)],
   "note": "Sprung corner mass = static axle load / 2 - unsprung. Roll gradient assumes a ground-level roll axis (estimate)."},
  {"id": "tyres", "title": "TYRES", "unity": "10", "panel": "tyres", "rows": [
    R("front.tyre_pressure", "Cold pressure", "kPa", 120, 280, 1, 0, k2="rear.tyre_pressure", s=0.001, lim=[170, 240], i="Cold inflation pressure. In the sim it sets vertical stiffness, sidewall progression and carcass stiffness."),
    R("car.tyre_slide_dir", "Sliding direction", "", 0, 1, 1, 0, opts=["LEGACY", "ISOTROPIC"], vals=["0", "1"], t="demo", i="TARMAC sliding direction: isotropic Coulomb (default) or the legacy radial return."),
    R("front.tyre_mu_x", "Peak μ long.", "", 0, 2, 0.01, 2, k2="rear.tyre_mu_x", ro=1, i="Peak longitudinal friction (setup default overlay applied)."),
    R("front.tyre_mu_y", "Peak μ lat.", "", 0, 2, 0.01, 2, k2="rear.tyre_mu_y", ro=1, i="Peak lateral friction (setup default overlay applied)."),
   ],
   "out": [O("Vertical rate F/R", "kzf/1000", "kN/m", 0), O("", "kzr/1000", "kN/m", 0),
           O("Static deflection F/R", "dzf", "mm", 1), O("", "dzr", "mm", 1),
           O("Contact length F/R", "plf", "mm", 0), O("", "plr", "mm", 0),
           O("Carcass stiffness F/R", "csf", "%", 0), O("", "csr", "%", 0),
           O("Hot pressure F/R", "hotf", "kPa", 0, 1), O("", "hotr", "kPa", 0, 1),
           O("Grip window F/R", "winf", "%", 0, 1, [70, 101]), O("", "winr", "%", 0, 1, [70, 101])],
   "note": "The sim has no tyre temperature: hot pressure (20 → 50 °C) and the grip window (centred 225 kPa hot) are estimates."},
  {"id": "brakes", "title": "BRAKES", "unity": "11", "panel": "brakes", "rows": [
    R("front.brake_torque", "Brake torque", "N·m", 0, 1900, 5, 0, k2="rear.brake_torque", lim=[600, 1300], i="Maximum brake torque per wheel at full pedal."),
    R("ui.brake_bias", "Brake bias", "% front", 40, 85, 0.5, 1, t="bias", lim=[60, 75], i="Front share of total brake torque. Moves torque between the axles and keeps the total."),
    R("rear.handbrake_torque", "Handbrake", "N·m", 0, 1000, 5, 0, i="Handbrake torque per rear wheel."),
   ],
   "out": [O("Bias", "bias", "% F", 1), O("Full-pedal decel", "dmax", "g", 2, 1),
           O("Front @ full pedal", "tbf", "N·m", 0), O("Front lock limit", "limf", "N·m", 0, 1),
           O("Rear @ full pedal", "tbr", "N·m", 0), O("Rear lock limit", "limr", "N·m", 0, 1),
           O("First lock at", "dlock", "g", 2, 1), O("Locks first", "sel(dlr < dlf, 1, sel(dlf > 0, 0, 1))", "", 0, 1, tone="axle")],
   "note": "Lock limits use static load transfer (CG height estimate) and the tyre load sensitivity at full-pedal decel."},
  {"id": "drivetrain", "title": "DIFFERENTIAL & DRIVETRAIN", "unity": "12", "panel": "diff", "rows": [
    R("car.diff_viscous", "Viscous coupling", "N·m/(rad/s)", 0, 30, 0.1, 1, lim=[0, 15], i="Viscous LSD: locking torque per rad/s of axle-half speed difference."),
    R("car.diff_power", "Ramp lock, power", "", 0, 1, 0.01, 2, i="Clutch-pack lock fraction of input torque on power."),
    R("car.diff_coast", "Ramp lock, coast", "", 0, 1, 0.01, 2, i="Clutch-pack lock fraction of input torque on the overrun."),
    R("car.diff_preload", "Preload", "N·m", 0, 200, 1, 0, i="Locking torque with no input torque."),
    R("car.clutch_torque", "Clutch capacity", "N·m", 0, 500, 1, 0, lim=[180, 400], i="Clutch slip torque."),
    R("car.engine_inertia", "Engine inertia", "kg·m²", 0, 0.25, 0.001, 3, i="Flywheel + crank inertia: lower revs faster, lower stores less."),
    R("car.coast_torque", "Engine braking", "N·m", 0, 120, 1, 0, i="Overrun drag torque at the coast rpm."),
   ],
   "out": [O("Peak torque", "tpk", "N·m", 0), O("Peak power", "ppk", "kW", 0),
           O("Axle torque, 1st", "axt1", "N·m", 0), O("Lock on power, 1st", "lockp", "N·m", 0),
           O("Lock on coast, 1st", "lockc", "N·m", 0), O("Viscous @ 100 rpm", "visc100", "N·m", 0)],
   "note": "Ramp LSD: T_lock = preload + ramp × input torque (core model). Viscous: T = c × Δω."},
  {"id": "gearing", "title": "GEARING", "unity": "13", "panel": "gearing", "rows": [
    R("car.gears#0", "1st", ":1", 0.5, 4.5, 0.001, 3, lim=[2.8, 3.8], fine=0.001, i="1st gear ratio."),
    R("car.gears#1", "2nd", ":1", 0.5, 4.5, 0.001, 3, i="2nd gear ratio."),
    R("car.gears#2", "3rd", ":1", 0.5, 4.5, 0.001, 3, i="3rd gear ratio."),
    R("car.gears#3", "4th", ":1", 0.5, 4.5, 0.001, 3, i="4th gear ratio."),
    R("car.gears#4", "5th", ":1", 0.5, 4.5, 0.001, 3, i="5th gear ratio."),
    R("car.final_drive", "Final drive", ":1", 2.5, 6.5, 0.001, 3, lim=[3.6, 4.8], i="Final drive ratio: shorter (higher) = more acceleration, lower top speed."),
    R("car.limiter_rpm", "Rev limiter", "rpm", 5000, 9000, 50, 0, lim=[6500, 7600], i="Fuel cut rpm."),
   ],
   "out": [O("1st @ limiter", "v1", "km/h", 0), O("2nd", "v2", "km/h", 0), O("3rd", "v3", "km/h", 0),
           O("4th", "v4", "km/h", 0), O("5th", "v5", "km/h", 0),
           O("Drop 1→2", "d12", "rpm", 0), O("Drop 2→3", "d23", "rpm", 0), O("Drop 3→4", "d34", "rpm", 0), O("Drop 4→5", "d45", "rpm", 0),
           O("Top speed", "vt", "km/h", 0, 1)],
   "note": "Speeds at the limiter use the loaded tyre radius = car-file radius. Top speed: power vs drag + rolling (η 0.90, Crr 0.012, estimate)."},
  {"id": "aero", "title": "AERODYNAMICS", "unity": "14", "panel": "aero", "rows": [
    R("car.cda", "Drag area CdA", "m²", 0.3, 1.2, 0.005, 3, lim=[0.5, 0.8], i="Drag coefficient × frontal area."),
    R("car.cla_front", "Lift area front", "m²", -0.5, 0.8, 0.01, 2, k2="car.cla_rear", lim=[-0.1, 0.3], i="Downforce coefficient × area per axle; negative = lift."),
   ],
   "out": [O("Drag @ 200 km/h", "dr200", "N", 0), O("Downforce F @ 200", "df200f", "N", 0), O("Downforce R @ 200", "df200r", "N", 0),
           O("Aero balance F", "abal", "%", 1), O("Drag power @ vmax", "pdrag", "kW", 1, 1), O("Top speed", "vt", "km/h", 0, 1)],
   "note": "F = ½ ρ v² × area, ρ = 1.225 kg/m³. Negative downforce = lift."},
  {"id": "weight", "title": "WEIGHT", "unity": "07", "panel": "weight", "rows": [
    R("car.mass", "Mass incl. driver", "kg", 700, 1600, 5, 0, lim=[900, 1200], i="Total mass. Inertias stay as set in the car file."),
   ],
   "out": [O("Mass", "m", "kg", 0), O("Front weight", "100*wf", "%", 1), O("Axle load F/R", "2*fzf/g", "kg", 0), O("", "2*fzr/g", "kg", 0),
           O("CG height", "1000*h", "mm", 0, 1), O("Yaw inertia Izz", "izz", "kg·m²", 0), O("Power / tonne", "pw", "kW/t", 0)],
   "note": "Izz = sprung inertia + unsprung corners as point masses. CG height = tyre radius + hub offset (estimate)."},
  {"id": "engine", "title": "ENGINE", "unity": "04", "panel": "engine", "rows": [],
   "out": [O("Peak torque", "tpk", "N·m", 0), O("Peak power", "ppk", "kW", 0), O("Limiter", "car.limiter_rpm", "rpm", 0),
           O("Power / tonne", "pw", "kW/t", 0)],
   "note": "Full-load torque from the car file's engine table."},
]


# settings pages (group "settings"): demo.ini keys, written to drivechain_overrides.ini
D = lambda *a, **k: R(*a, t="demo", **k)
SETTINGS = [
  {"id": "camera", "title": "CAMERA", "group": "settings", "panel": "settings", "rows": [
    D("camera.mode", "Start camera", "", 0, 4, 1, 0, opts=["CHASE", "COCKPIT", "WHEEL", "ORBIT", "TV"], vals=["chase", "cockpit", "wheel", "orbit", "tv"], rs=1, i="Camera at the start of a drive (C cycles while driving)."),
    D("camera.follow", "Chase follow", "", 0, 2, 1, 0, opts=["BODY", "VELOCITY", "BLEND"], vals=["body", "velocity", "blend"], i="Body: the car always points into the screen. Velocity: the camera trails the direction of travel, so slides show as yaw. Blend: a mix."),
    D("camera.follow_blend", "Follow blend", "% velocity", 0, 100, 5, 0, s=100, i="Share of the velocity direction when follow = blend."),
    D("camera.chase_dist", "Chase distance", "m", 3, 12, 0.1, 1, lim=[5, 7], i="Distance of the chase camera behind the car."),
    D("camera.chase_height", "Chase height", "m", 0.8, 4, 0.1, 1, i="Height of the chase camera."),
    D("display.fov_v_deg", "Vertical FOV", "deg", 30, 90, 1, 0, lim=[45, 60], rs=1, i="Vertical field of view; ultrawide keeps it and widens horizontally."),
   ], "out": [], "note": "Follow, distance and height apply live; start camera and FOV on the next drive."},
  {"id": "physics", "title": "PHYSICS", "group": "settings", "panel": "settings", "rows": [
    D("physics.model", "Solver model", "", 0, 1, 1, 0, opts=["FAST", "REFERENCE"], vals=["fast", "reference"], rs=1, i="Fast = same physics with bounded solver work. Reference = full numerics (slower)."),
    D("physics.substeps", "Substeps per tick", "", 1, 8, 1, 0, rs=1, lim=[2, 4], i="Mechanics rate = tick rate x substeps. Lower on a slow PC."),
    D("physics.threads", "Worker threads", "", 1, 4, 1, 0, rs=1, i="Threads in the physics step; results are bit-identical for any count."),
    D("car.setup", "Car setup", "", 0, 1, 1, 0, opts=["DEFAULT", "LEGACY"], vals=["default", "legacy"], rs=1, i="Default = car file + proposal sets (bushings, TARMAC tyre set on matching cars). Legacy = car file as-is."),
    D("physics.tick_hz", "Host tick", "Hz", 240, 1000, 240, 0, opts=["240", "480", "960"], vals=["240", "480", "960"], rs=2, i="Input, force feedback and audio rate. Needs a program restart (audio runs at this rate)."),
   ], "out": [], "note": "Applied when you press Drive (the car is rebuilt at the start). Host tick needs a restart."},
  {"id": "video", "title": "VIDEO & AUDIO", "group": "settings", "panel": "settings", "rows": [
    D("display.fps_cap", "Frame cap", "fps", 0, 240, 10, 0, i="0 = uncapped. Physics runs on its own thread."),
    D("display.vsync", "V-sync", "", 0, 1, 1, 0, opts=["OFF", "ON"], vals=["0", "1"], rs=2, i="Rendering only. Needs a restart."),
    D("display.msaa", "Anti-aliasing", "", 0, 1, 1, 0, opts=["OFF", "4x MSAA"], vals=["0", "1"], rs=2, i="Needs a restart."),
    D("display.fullscreen", "Fullscreen", "", 0, 1, 1, 0, opts=["OFF", "ON"], vals=["0", "1"], rs=2, i="Needs a restart."),
    D("audio.master", "Master volume", "%", 0, 100, 5, 0, s=100, i="Overall volume."),
    D("audio.engine", "Engine", "%", 0, 200, 5, 0, s=100, i="Engine sound gain."),
    D("audio.tyres", "Tyres", "%", 0, 200, 5, 0, s=100, i="Tyre scrub / squeal gain."),
    D("audio.road", "Road", "%", 0, 200, 5, 0, s=100, i="Road rumble gain."),
    D("audio.wind", "Wind", "%", 0, 200, 5, 0, s=100, i="Wind noise gain."),
   ], "out": [], "note": "Frame cap and volumes apply live."},
  {"id": "devices", "title": "DEVICES", "group": "settings", "panel": "settings", "rows": [
    D("input.device", "Input device", "", 0, 3, 1, 0, opts=["AUTO", "WHEEL", "GAMEPAD", "KEYBOARD"], vals=["auto", "wheel", "gamepad", "keyboard"], rs=1, i="Auto picks a wheel, then a gamepad, then the keyboard."),
    D("input.rotation_deg", "Wheel rotation", "deg", 180, 1080, 10, 0, lim=[540, 900], rs=1, i="Lock-to-lock rotation of YOUR wheel as set in its driver (1:1 to the car's column)."),
    D("ffb.gain", "FFB gain", "%", 0, 200, 5, 0, s=100, rs=1, i="Force feedback output = gain x column torque / max torque."),
    D("ffb.max_torque_nm", "FFB full scale", "N·m", 2, 25, 0.5, 1, rs=1, i="Column torque that maps to 100 % of your wheel base."),
    D("input.auto_clutch", "Auto clutch", "", 0, 2, 1, 0, opts=["AUTO", "MANUAL", "ON"], vals=["-1", "0", "1"], rs=1, i="Auto: off when an H-shifter is configured."),
    D("gamepad.mode", "Gamepad steering", "", 0, 1, 1, 0, opts=["ARMS", "RAW"], vals=["arms", "raw"], rs=1, i="Arms: the stick is the target of a compliant grip, the tyres can still turn the rim. Raw: stiff legacy servo."),
    D("gamepad.deadzone", "Stick deadzone", "", 0, 0.3, 0.01, 2, rs=1, i="Stick jitter isolation."),
   ], "out": [], "note": "Applied when you press Drive. Menu: arrows / d-pad move, left-right adjust, Enter / A select, Esc / B back; wheel paddles move between rows."},
]

OVERLAY = {   # car.setup = default overlays the TARMAC propD tyre set on 185/60R14 cars (demo/sim.c SETUP_D); used for the read-outs
  "when": {"front.tyre_width": [0.182, 0.188], "front.tyre_radius": [0.285, 0.293]},
  "keys": {"tyre_mu_y": "0.955", "tyre_mu_x": "0.926", "tyre_ls_exp_y": "0.90", "tyre_ls_exp_x": "0.92"}}

SPEC = {"version": 1, "consts": {"g": G, "rho": RHO, "crr": CRR, "eta": ETA},
        "vars": [{"n": n, "e": e} for n, e in VARS], "overlay_default": OVERLAY, "pages": PAGES + SETTINGS}

# ------------------------------------------------------------------------------------- reference evaluator
def load_ini(path):
    kv, order = {}, []
    sec = ""
    for line in open(path, encoding="utf-8"):
        s = line.split(";")[0].strip()
        if not s: continue
        if s.startswith("["): sec = s[1:s.index("]")]; continue
        if "=" in s:
            k, v = s.split("=", 1); k = k.strip(); v = v.strip()
            kv[sec + "." + k] = v; order.append((sec, k, v))
    return kv, order

class Ev:
    def __init__(self, kv, order):
        self.kv, self.order, self.vars = kv, order, {}
    def key(self, name):
        idx = 0
        if "#" in name: name, i = name.split("#"); idx = int(i)
        v = self.kv.get(name)
        if v is None: return float("nan")
        parts = [p.strip() for p in v.split(",")]
        if idx >= len(parts): return float("nan")
        try: return float(parts[idx])
        except ValueError: return float("nan")
    def lut(self):
        pts = sorted((float(k), float(v)) for s, k, v in self.order if s == "engine_lut")
        return pts
    def torque(self, rpm):
        p = self.lut()
        if not p: return float("nan")
        if rpm <= p[0][0]: return p[0][1]
        for (x0, y0), (x1, y1) in zip(p, p[1:]):
            if rpm <= x1: return y0 + (y1 - y0) * (rpm - x0) / (x1 - x0)
        return p[-1][1]
    def gear(self, i):
        n = int(self.key("car.ngears")) if self.key("car.ngears") == self.key("car.ngears") else 5
        if i < 1 or i > n: return float("nan")
        return self.key("car.gears#%d" % (i - 1))
    def vgear(self, i):
        gr = self.gear(i); r = self.key("rear.tyre_radius") if self.key("car.drive") == 0 else self.key("front.tyre_radius")
        return self.key("car.limiter_rpm") * math.pi / 30 * r / (gr * self.key("car.final_drive")) * 3.6
    def axis(self, w):
        """front steering axis from the car-file hardpoints: lower ball joint = link[lower_arm_a].u, upper = strut_top
        (type 1) or the first link that is not a lower-arm or rack link; w 0 = caster, 1 = KPI angle [rad]"""
        la = int(self.key("front.lower_arm_a")); lb = int(self.key("front.lower_arm_b")); rk = int(self.key("front.rack_link"))
        lo = [self.key("front.link%d_u#%d" % (la, j)) for j in range(3)]
        if self.key("front.type") == 1: up = [self.key("front.strut_top#%d" % j) for j in range(3)]
        else:
            n = int(self.key("front.nlinks")); u = next((i for i in range(n) if i not in (la, lb, rk)), -1)
            if u < 0: return float("nan")
            up = [self.key("front.link%d_u#%d" % (u, j)) for j in range(3)]
        return math.atan2((lo[0] - up[0]) if w == 0 else (lo[1] - up[1]), up[2] - lo[2])
    def tmax(self): return max(t for _, t in self.lut())
    def pmax(self):
        lim = self.key("car.limiter_rpm"); best = 0
        rpm = 500.0
        while rpm <= lim + 1e-9: best = max(best, self.torque(rpm) * rpm * math.pi / 30 / 1000); rpm += 25
        return best
    def vtop(self):
        m = self.key("car.mass"); cda = self.key("car.cda"); fd = self.key("car.final_drive"); lim = self.key("car.limiter_rpm")
        r = self.key("rear.tyre_radius") if self.key("car.drive") == 0 else self.key("front.tyre_radius")
        n = int(self.key("car.ngears")); best = 0.0
        for gi in range(1, n + 1):
            gr = self.gear(gi); v = 1.0
            while v < 120:
                rpm = v / r * gr * fd * 30 / math.pi
                if rpm > lim: break
                f = self.torque(rpm) * gr * fd * ETA / r - 0.5 * RHO * cda * v * v - CRR * m * G
                if f < 0: break
                best = max(best, v); v += 0.05
        return best * 3.6
    # ---- parser
    def ev(self, s):
        self.t = re.findall(r"\d+\.\d*(?:[eE][-+]?\d+)?|\d*\.\d+(?:[eE][-+]?\d+)?|\d+(?:[eE][-+]?\d+)?|[A-Za-z_][A-Za-z0-9_.]*(?:#\d+)?|<=|>=|[-+*/^(),<>]", s)
        self.p = 0
        v = self.cmp()
        if self.p != len(self.t): raise ValueError("trailing in " + s)
        return v
    def peek(self): return self.t[self.p] if self.p < len(self.t) else None
    def nx(self): self.p += 1; return self.t[self.p - 1]
    def cmp(self):
        a = self.add()
        while self.peek() in ("<", ">", "<=", ">="):
            o = self.nx(); b = self.add()
            a = float({"<": a < b, ">": a > b, "<=": a <= b, ">=": a >= b}[o])
        return a
    def add(self):
        a = self.mul()
        while self.peek() in ("+", "-"):
            o = self.nx(); b = self.mul(); a = a + b if o == "+" else a - b
        return a
    def mul(self):
        a = self.un()
        while self.peek() in ("*", "/"):
            o = self.nx(); b = self.un()
            a = a * b if o == "*" else (a / b if b != 0 else float("nan"))
        return a
    def un(self):
        if self.peek() == "-": self.nx(); return -self.un()
        if self.peek() == "+": self.nx(); return self.un()
        return self.pw()
    def pw(self):
        a = self.atom()
        if self.peek() == "^":
            self.nx(); b = self.un()
            try: return a ** b if not (a < 0 and b != int(b)) else float("nan")
            except (OverflowError, ZeroDivisionError): return float("nan")
        return a
    def atom(self):
        t = self.nx()
        if t == "(":
            v = self.cmp(); assert self.nx() == ")"; return v
        if re.match(r"[\d.]", t): return float(t)
        if self.peek() == "(":
            self.nx(); args = []
            if self.peek() != ")":
                args.append(self.cmp())
                while self.peek() == ",": self.nx(); args.append(self.cmp())
            assert self.nx() == ")"
            return self.call(t, args)
        if t in ("pi",): return math.pi
        if t in ("g", "rho", "crr", "eta"): return SPEC["consts"][t]
        if t in self.vars: return self.vars[t]
        return self.key(t)
    def call(self, f, a):
        nan = float("nan")
        try:
            if f == "sqrt": return math.sqrt(a[0]) if a[0] >= 0 else nan
            if f == "abs": return abs(a[0])
            if f == "min": return min(a)
            if f == "max": return max(a)
            if f == "pow": return a[0] ** a[1]
            if f == "sin": return math.sin(a[0])
            if f == "cos": return math.cos(a[0])
            if f == "tan": return math.tan(a[0])
            if f == "atan": return math.atan(a[0])
            if f == "atan2": return math.atan2(a[0], a[1])
            if f == "deg": return a[0] * 180 / math.pi
            if f == "rad": return a[0] * math.pi / 180
            if f == "clamp": return min(max(a[0], a[1]), a[2])
            if f == "sel": return a[1] if a[0] > 0 else a[2]
            if f == "torque": return self.torque(a[0])
            if f == "gear": return self.gear(int(round(a[0])))
            if f == "vgear": return self.vgear(int(round(a[0])))
            if f == "vtop": return self.vtop()
            if f == "tmax": return self.tmax()
            if f == "pmax": return self.pmax()
            if f == "axis": return self.axis(int(round(a[0])))
        except (ValueError, ZeroDivisionError, OverflowError):
            return nan
        raise ValueError("unknown function " + f)

def apply_overlay(kv):
    ok = all(lo <= float(kv.get(k, "nan").split(",")[0]) <= hi for k, (lo, hi) in OVERLAY["when"].items())
    if ok:
        for s in ("front", "rear"):
            for k, v in OVERLAY["keys"].items(): kv[s + "." + k] = v
    return ok

def check(car):
    kv, order = load_ini(car)
    apply_overlay(kv)
    e = Ev(kv, order)
    for v in SPEC["vars"]:
        e.vars[v["n"]] = e.ev(v["e"])
        print("var %-8s %.6g" % (v["n"], e.vars[v["n"]]))
    for p in PAGES:
        for o in p["out"]:
            print("out %s.%-24s %.6g" % (p["id"], o["e"][:24], e.ev(o["e"])))

if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    if len(sys.argv) > 2 and sys.argv[1] == "--check":
        check(sys.argv[2]); sys.exit(0)
    out = os.path.join(here, "..", "demo", "ui", "tune_spec.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(SPEC, f, indent=1, ensure_ascii=False); f.write("\n")
    print("wrote", os.path.normpath(out))
