#!/usr/bin/env python3
"""foam_forces.py - integrate surface pressure forces on the car patches of a decomposed OpenFOAM case (ascii).

Needed because the Debian OpenFOAM 1912 build aborts in any function object (sha1 stream bug), so the
standard 'forces' FO cannot run.  Run `foamFormatConvert -parallel` first so every file is ascii.
Pressure only (zeroGradient on walls -> face p = owner-cell p); viscous drag is NOT included (for bluff road
cars skin friction is a small part of Cd, Hucho 1998 ch. 4; lift is pressure-dominated).
Prints Cd_p, CL, CLF, CLR (SAE J1594: CPM about the wheelbase centre at ground, incl. drag moment), plus a
breakdown of the lift by surface class (upper-facing vs lower-facing faces) and by x-station.
usage: foam_forces.py <case> --U 30 --A <frontal area m2> --xf <front axle x> --xr <rear axle x> --zg <ground z> [--time latest]
"""
import os, re, glob, argparse
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("case"); ap.add_argument("--U", type=float, default=30.0); ap.add_argument("--A", type=float, required=True)
ap.add_argument("--xf", type=float, required=True); ap.add_argument("--xr", type=float, required=True)
ap.add_argument("--zg", type=float, required=True); ap.add_argument("--rho", type=float, default=1.2)
ap.add_argument("--patches", default="body,wheels"); ap.add_argument("--time", default="latest")
a = ap.parse_args()

def body(path):
    s = open(path, errors="replace").read()
    s = re.sub(r"/\*.*?\*/", "", s, flags=re.S); s = re.sub(r"//[^\n]*", "", s)
    return s[s.index("}", s.index("FoamFile")) + 1:]

def read_list(s, start=0):
    """parse 'N ( ... )' at/after start; returns (text inside, end index)."""
    m = re.compile(r"(\d+)\s*\(").search(s, start)
    depth, i = 1, m.end()
    while depth:
        c = s[i]; depth += (c == "(") - (c == ")"); i += 1
    return int(m.group(1)), s[m.end():i - 1], i

def points(p):
    n, t, _ = read_list(body(p)); return np.array(t.replace("(", " ").replace(")", " ").split(), float).reshape(n, 3)

def faces(p):
    s = body(p); n, t, e = read_list(s)
    if "(" in t:                          # ascii faceList: k(a b c)
        return [np.array(x.split(), int) for x in re.findall(r"\d+\s*\(([^)]*)\)", t)]
    off = np.array(t.split(), int); n2, t2, _ = read_list(s, e); lab = np.array(t2.split(), int)   # faceCompactList
    return [lab[off[i]:off[i + 1]] for i in range(len(off) - 1)]

def labels(p):
    n, t, _ = read_list(body(p)); return np.array(t.split(), int)

def boundary(p):
    s = body(p); out = {}
    for m in re.finditer(r"(\w+)\s*\{([^}]*)\}", s):
        d = dict(re.findall(r"(\w+)\s+([^;]+);", m.group(2)))
        if "startFace" in d: out[m.group(1)] = (int(d["nFaces"]), int(d["startFace"]))
    return out

def internal_scalar(p):
    s = body(p); m = re.search(r"internalField\s+(uniform\s+[^;]+|nonuniform\s+List<scalar>)", s)
    if m.group(1).startswith("uniform"): return float(m.group(1).split()[1])
    n, t, _ = read_list(s, m.end()); return np.array(t.split(), float)

pats = a.patches.split(","); xm = 0.5 * (a.xf + a.xr); wb = a.xf - a.xr
F = np.zeros(3); M = np.zeros(3); rows = []
for proc in sorted(glob.glob(f"{a.case}/processor*")):
    times = [d for d in os.listdir(proc) if re.fullmatch(r"[0-9.eE+-]+", d) and d != "0"]
    t = max(times, key=float) if a.time == "latest" else a.time
    pm = f"{proc}/constant/polyMesh"
    P = points(f"{pm}/points"); Fs = faces(f"{pm}/faces"); own = labels(f"{pm}/owner"); bnd = boundary(f"{pm}/boundary")
    p = internal_scalar(f"{proc}/{t}/p")
    for name in pats:
        if name not in bnd: continue
        n, st = bnd[name]
        for fi in range(st, st + n):
            v = P[Fs[fi]]; c0 = v.mean(0)
            Sf = 0.5 * sum(np.cross(v[k] - c0, v[(k + 1) % len(v)] - c0) for k in range(len(v)))
            ctr = c0
            pf = (p[own[fi]] if np.ndim(p) else p) * a.rho
            dF = pf * Sf                     # Sf points out of the fluid, i.e. into the car
            F += dF; M += np.cross(ctr - np.array([xm, 0, a.zg]), dF)
            rows.append((ctr[0], ctr[2], Sf[2], dF[2], dF[0]))
    print(f"{os.path.basename(proc)}: time {t}, faces {sum(bnd.get(n, (0, 0))[0] for n in pats)}")
q = 0.5 * a.rho * a.U ** 2; qA = q * a.A
D = -F[0]; L = F[2]; S = F[1]
CD, CL, CS = D / qA, L / qA, S / qA; CPM = -M[1] / (qA * wb)
print(f"\npressure forces at U={a.U} m/s, rho={a.rho}: drag {D:.1f} N, lift {L:+.1f} N, side {S:+.1f} N")
print(f"Cd_p = {CD:.3f}  CL = {CL:+.3f}  CLF = {CL/2+CPM:+.3f}  CLR = {CL/2-CPM:+.3f}  CPM = {CPM:+.3f}  (A = {a.A} m^2)")
R = np.array(rows)
up = R[:, 2] < 0   # Sf into car: Sf_z < 0 means the wetted face looks upward (roof, bonnet, deck)
print(f"lift from upward-looking surfaces (bonnet/screen/deck/top): {R[up,3].sum()/qA:+.3f};  "
      f"downward-looking (floor, under-bumpers): {R[~up,3].sum()/qA:+.3f}")
for x0 in np.arange(-2.1, 2.0, 0.5):
    m = (R[:, 0] >= x0) & (R[:, 0] < x0 + 0.5)
    print(f"  x {x0:+.1f}..{x0+0.5:+.1f}: CL contribution {R[m,3].sum()/qA:+.3f} (upper {R[m&up,3].sum()/qA:+.3f}, lower {R[m&~up,3].sum()/qA:+.3f}), Cd_p {-R[m,4].sum()/qA:+.3f}")
