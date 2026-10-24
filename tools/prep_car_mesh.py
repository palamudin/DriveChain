#!/usr/bin/env python3
"""prep_car_mesh.py - split a game-style car OBJ (e.g. a LOD B export) into the parts the demo animates.

usage: prep_car_mesh.py <car.obj> <car.ini (physics, for hub positions)> <out_dir> [--paint r,g,b] [--max-tex 1024]

Writes into out_dir (keep it under private/ for third-party assets - never commit or publish them):
  body.obj/.mtl          chassis, cockpit, lights (no glass, damage, blur or light-beam meshes)
  wheel_<c>.obj/.mtl     tyre + rim + brake disc, centred on its wheel centre      (c = fl fr rl rr) - spins & steers
  caliper_<c>.obj/.mtl   caliper, relative to the same wheel centre                  - steers, does not spin
  steer.obj/.mtl         steering wheel, centred on its hub
  mesh.cfg               placement (physics frame: x fwd, y left, z up, origin = CG) + cross-check numbers
  tex/*.png              DDS -> PNG (Pillow), downscaled to --max-tex
Frame: such models are +X left, +Y up, +Z forward  ->  physics (x, y, z) = (ac.z, ac.x, ac.y) + offset,
with the offset chosen so the mesh wheel centres land on the physics hub points (least squares in x and z).
"""
import sys, os, re, math, argparse

ap = argparse.ArgumentParser()
ap.add_argument("obj"); ap.add_argument("ini"); ap.add_argument("out")
ap.add_argument("--paint", default="0.62,0.04,0.035"); ap.add_argument("--max-tex", type=int, default=1024)
a = ap.parse_args()
src_dir = os.path.dirname(os.path.abspath(a.obj))
os.makedirs(os.path.join(a.out, "tex"), exist_ok=True)

# ---------- physics hubs from the car ini
hubs = {}; sec = None
for ln in open(a.ini):
    ln = ln.split(";")[0].strip()
    m = re.match(r"\[(\w+)\]", ln)
    if m: sec = m.group(1); continue
    if "=" in ln and sec in ("front", "rear"):
        k, v = [s.strip() for s in ln.split("=", 1)]
        if k == "hub": hubs[sec] = [float(t) for t in v.split(",")]
assert "front" in hubs and "rear" in hubs, "car ini needs [front]/[rear] hub = x, y, z"

# ---------- parse OBJ
V, VT, VN = [], [], []
groups = {}   # name -> list of (mtl, face list)
order = []
mtllib = None; g = "default"; mtl = None
for ln in open(a.obj, errors="replace"):
    if ln.startswith("v "): V.append(tuple(map(float, ln.split()[1:4])))
    elif ln.startswith("vt "): t = ln.split(); VT.append((float(t[1]), float(t[2]) if len(t) > 2 else 0.0))
    elif ln.startswith("vn "): VN.append(tuple(map(float, ln.split()[1:4])))
    elif ln.startswith("g ") or ln.startswith("o "): g = ln.split(None, 1)[1].strip() if len(ln.split()) > 1 else "default"
    elif ln.startswith("usemtl"): mtl = ln.split(None, 1)[1].strip()
    elif ln.startswith("mtllib"): mtllib = ln.split(None, 1)[1].strip()
    elif ln.startswith("f "):
        if g not in groups: groups[g] = []; order.append(g)
        f = []
        for t in ln.split()[1:]:
            p = t.split("/")
            ix = [int(x) if x else 0 for x in p] + [0] * (3 - len(p))
            f.append(tuple((i - 1) if i > 0 else (len(L) + i) if i < 0 else -1 for i, L in zip(ix, (V, VT, VN))))
        groups[g].append((mtl, f))

def to_phys(p): return (p[2], p[0], p[1])
# ---------- materials
mats = {}; cur = None
for ln in open(os.path.join(src_dir, mtllib), errors="replace"):
    t = ln.strip()
    if t.startswith("newmtl"): cur = t.split(None, 1)[1].strip(); mats[cur] = {"Kd": (0.6, 0.6, 0.6), "map": None}
    elif cur and t.startswith("Kd "): mats[cur]["Kd"] = tuple(map(float, t.split()[1:4]))
    elif cur and t.startswith("map_Kd"): mats[cur]["map"] = t.split(None, 1)[1].strip().replace("\\", "/")

# ---------- classify
CORNER = {"lf": "fl", "rf": "fr", "lr": "rl", "rr": "rr"}
def part_of(name):
    n = name.lower()
    if "damage" in n or "rim_blur" in n or "light_beam" in n or "geo_susp" in n: return None
    if n in ("geo_ext_windshield_b", "windshield_hr_int_front_b"): return None      # glass: drawn without alpha it would blind the cockpit cam
    for k, c in CORNER.items():
        if re.match(rf"(tyre|rim)_{k}_", n) or n == f"geo_disc_{k}_b": return "wheel_" + c
        if n == f"brake_caliper_{k}_b": return "caliper_" + c
    if n.startswith("geo_steer"): return "steer"
    return "body"
parts = {}
for gname in order:
    p = part_of(gname)
    if p: parts.setdefault(p, []).append(gname)

def verts_of(gnames):
    s = set()
    for gn in gnames:
        for _, f in groups[gn]:
            for v in f: s.add(v[0])
    return [to_phys(V[i]) for i in s]
def bbox(pts):
    return [min(p[k] for p in pts) for k in range(3)], [max(p[k] for p in pts) for k in range(3)]

# wheel centres from the tyre bounding boxes (x, z centre; y = mid of tread width)
wc = {}; wr = {}
for c in ("fl", "fr", "rl", "rr"):
    tyre = [gn for gn in parts["wheel_" + c] if gn.lower().startswith("tyre")]
    mn, mx = bbox(verts_of(tyre))
    wc[c] = [(mn[k] + mx[k]) / 2 for k in range(3)]
    wr[c] = 0.25 * ((mx[0] - mn[0]) + (mx[2] - mn[2]))
# offset: mesh -> physics frame
phys = {"fl": hubs["front"], "fr": [hubs["front"][0], -hubs["front"][1], hubs["front"][2]],
        "rl": hubs["rear"], "rr": [hubs["rear"][0], -hubs["rear"][1], hubs["rear"][2]]}
off = [sum(phys[c][0] - wc[c][0] for c in wc) / 4, 0.0, sum(phys[c][2] - wc[c][2] for c in wc) / 4]
def P(p): return (p[0] + off[0], p[1] + off[1], p[2] + off[2])

# steering wheel centre + axis (PCA: least-variance direction of the rim vertices)
sw = verts_of(parts["steer"]); n = len(sw)
cen = [sum(p[k] for p in sw) / n for k in range(3)]
C = [[sum((p[i] - cen[i]) * (p[j] - cen[j]) for p in sw) / n for j in range(3)] for i in range(3)]
# smallest eigenvector by inverse power iteration on (C + eps I)
import random
def solve3(M, b):
    import copy
    A = [row[:] + [b[i]] for i, row in enumerate(M)]
    for i in range(3):
        pv = max(range(i, 3), key=lambda r: abs(A[r][i])); A[i], A[pv] = A[pv], A[i]
        for r in range(3):
            if r != i:
                f = A[r][i] / A[i][i]
                for k in range(4): A[r][k] -= f * A[i][k]
    return [A[i][3] / A[i][i] for i in range(3)]
Ce = [[C[i][j] + (1e-9 if i == j else 0) for j in range(3)] for i in range(3)]
ax = [0.3, 0.2, 0.9]
for _ in range(60):
    ax = solve3(Ce, ax); L = math.sqrt(sum(t * t for t in ax)); ax = [t / L for t in ax]
if ax[0] > 0: ax = [-t for t in ax]          # column axis pointing from the wheel toward the dash is +x-ish; store driver-facing (-x)
sw_mn, sw_mx = bbox(sw)
sw_radius = 0.5 * max(sw_mx[1] - sw_mn[1], math.hypot(sw_mx[0] - sw_mn[0], sw_mx[2] - sw_mn[2]))
cenP = P(cen)

# ---------- textures
try:
    from PIL import Image
except ImportError:
    Image = None
tex_done = {}
def tex(name):
    if not name: return None
    if name in tex_done: return tex_done[name]
    path = os.path.join(src_dir, name)
    if not os.path.exists(path):
        alt = [f for f in os.listdir(os.path.dirname(path)) if f.lower() == os.path.basename(path).lower()] if os.path.isdir(os.path.dirname(path)) else []
        if not alt: tex_done[name] = None; return None
        path = os.path.join(os.path.dirname(path), alt[0])
    out = re.sub(r"[^A-Za-z0-9_]", "_", os.path.splitext(os.path.basename(path))[0]) + ".png"
    try:
        im = Image.open(path); im.load(); im = im.convert("RGBA")
        if max(im.size) > a.max_tex:
            s = a.max_tex / max(im.size); im = im.resize((max(1, int(im.size[0] * s)), max(1, int(im.size[1] * s))), Image.LANCZOS)
        im.save(os.path.join(a.out, "tex", out)); tex_done[name] = "tex/" + out
    except Exception as e:
        print("  texture skipped:", name, e); tex_done[name] = None
    return tex_done[name]
paint = tuple(float(t) for t in a.paint.split(","))

# ---------- write parts
def write_part(pname, gnames, origin):
    used = []
    with open(os.path.join(a.out, pname + ".obj"), "w") as fo:
        fo.write(f"# {pname}: physics frame (x fwd, y left, z up), relative to {origin}\nmtllib {pname}.mtl\n")
        vmap, tmap, nmap = {}, {}, {}
        body = []
        for gn in gnames:
            for m, f in groups[gn]:
                if m not in used: used.append(m)
                body.append((m, f))
        # emit vertices on demand
        lines_v, lines_t, lines_n, lines_f = [], [], [], []
        curm = None
        for m, f in body:
            if m != curm: lines_f.append(f"usemtl {m}"); curm = m
            idx = []
            for vi, ti, ni in f:
                if vi not in vmap:
                    p = P(to_phys(V[vi])); vmap[vi] = len(vmap) + 1
                    lines_v.append("v %.5f %.5f %.5f" % (p[0] - origin[0], p[1] - origin[1], p[2] - origin[2]))
                s = str(vmap[vi])
                if ti >= 0:
                    if ti not in tmap: tmap[ti] = len(tmap) + 1; lines_t.append("vt %.5f %.5f" % VT[ti])
                    s += "/" + str(tmap[ti])
                else: s += "/"
                if ni >= 0:
                    if ni not in nmap: nmap[ni] = len(nmap) + 1; n_ = to_phys(VN[ni]); lines_n.append("vn %.4f %.4f %.4f" % n_)
                    s += "/" + str(nmap[ni])
                idx.append(s)
            lines_f.append("f " + " ".join(idx))
        fo.write("\n".join(lines_v + lines_t + lines_n + lines_f) + "\n")
    with open(os.path.join(a.out, pname + ".mtl"), "w") as fm:
        for m in used:
            M = mats.get(m, {"Kd": (0.6, 0.6, 0.6), "map": None})
            fm.write(f"newmtl {m}\n")
            if m == "ext_chassis": fm.write("Kd %.3f %.3f %.3f\n\n" % paint); continue      # body paint (skins are not part of the model)
            fm.write("Kd %.3f %.3f %.3f\n" % tuple(M["Kd"]))
            t = tex(M["map"])
            if t: fm.write(f"map_Kd {t}\n")
            fm.write("\n")
    return len(vmap)

stats = {}
for pname, gnames in sorted(parts.items()):
    c = pname.split("_")[-1]
    if pname.startswith(("wheel_", "caliper_")): origin = P(wc[c])
    elif pname == "steer": origin = cenP
    else: origin = (0, 0, 0)
    stats[pname] = write_part(pname, gnames, origin)

wb = 0.5 * ((wc["fl"][0] + wc["fr"][0]) - (wc["rl"][0] + wc["rr"][0]))
trf = wc["fl"][1] - wc["fr"][1]; trr = wc["rl"][1] - wc["rr"][1]
with open(os.path.join(a.out, "mesh.cfg"), "w") as f:
    f.write("# generated by tools/prep_car_mesh.py from %s\n" % os.path.basename(a.obj))
    f.write("# physics frame: x fwd, y left, z up, origin = CG of the physics car %s\n" % os.path.basename(a.ini))
    f.write("offset = %.4f %.4f %.4f\n" % tuple(off))
    for c in ("fl", "fr", "rl", "rr"):
        p = P(wc[c]); f.write("wheel_%s = %.4f %.4f %.4f\n" % (c, *p))
    f.write("wheel_radius_mesh = %.4f\n" % (sum(wr.values()) / 4))
    f.write("steer_center = %.4f %.4f %.4f\n" % cenP)
    f.write("steer_axis = %.4f %.4f %.4f\n" % tuple(ax))
    f.write("steer_radius = %.4f\n" % sw_radius)
    f.write("# cross-check: mesh wheelbase %.4f, track F %.4f R %.4f (tyre bbox centres)\n" % (wb, trf, trr))
print("parts:", ", ".join(f"{k} ({v} v)" for k, v in sorted(stats.items())))
print("mesh WB %.4f  track F %.4f R %.4f  wheel radius %.4f (bbox)  offset %s" % (wb, trf, trr, sum(wr.values()) / 4, ", ".join("%.4f" % t for t in off)))
print("steering wheel centre %s axis %s radius %.3f" % (", ".join("%.3f" % t for t in cenP), ", ".join("%.3f" % t for t in ax), sw_radius))
print("textures:", sum(1 for v in tex_done.values() if v), "converted,", sum(1 for v in tex_done.values() if not v), "missing/skipped")
