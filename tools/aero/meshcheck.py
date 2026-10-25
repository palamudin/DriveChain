#!/usr/bin/env python3
"""meshcheck.py - aero suitability check of a car OBJ (prints aggregate numbers only; never copies geometry).

usage: meshcheck.py <car.obj> [--offset dx,dy,dz] [--frame yup|phys] [--cell 0.01]

Reports, for the exterior aero surface (drops interior and non-aero groups, see objload.DROP):
  - bounding box, ground plane (lowest tyre vertex), wheelbase-ish extents
  - frontal area A (projection on the y-z plane, rasterised at --cell; includes wheels, mirrors)
  - topology: open (boundary) edges, non-manifold edges, duplicate-vertex welding at 0.5 mm
    -> watertight or not (CFD needs a closed, or at least gap-tolerant, surface: snappyHexMesh tolerates gaps
       smaller than the local cell, panel methods do not)
  - underbody: fraction of the plan-view footprint whose lowest surface is a real underbody surface,
    and the clearance histogram (detail of the floor drives lift; a missing floor makes CFD lift meaningless)
  - cockpit: is there a roof/top over the cabin (open roadster vs closed)?
Frame 'yup': model +X left, +Y up, +Z forward -> physics x fwd, y left, z up (as tools/prep_car_mesh.py).
"""
import sys, argparse, re
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("obj"); ap.add_argument("--offset", default="0,0,0"); ap.add_argument("--frame", default="yup")
ap.add_argument("--cell", type=float, default=0.01)
a = ap.parse_args()
off = np.array([float(t) for t in a.offset.split(",")])

from objload import DROP
V = []; F = []; G = []; gname = "default"; groups = []
for ln in open(a.obj, errors="replace"):
    if ln.startswith("v "): V.append([float(t) for t in ln.split()[1:4]])
    elif ln.startswith("g ") or ln.startswith("o "):
        gname = ln.split(None, 1)[1].strip() if len(ln.split()) > 1 else "default"
        if gname not in groups: groups.append(gname)
    elif ln.startswith("f "):
        idx = [int(t.split("/")[0]) for t in ln.split()[1:]]
        idx = [i - 1 if i > 0 else len(V) + i for i in idx]
        for k in range(1, len(idx) - 1): F.append((idx[0], idx[k], idx[k + 1])); G.append(gname)
V = np.array(V); F = np.array(F); G = np.array(G)
if a.frame == "yup": V = V[:, [2, 0, 1]]
V = V + off
keep = np.array([not DROP.search(g) for g in G])
Fe, Ge = F[keep], G[keep]
used = sorted(set(Ge)); dropped = sorted(set(G[~keep]))
print(f"faces total {len(F)}, exterior kept {len(Fe)} in {len(used)} groups; dropped {len(dropped)} groups")
tyre = np.array([bool(re.match(r"tyre|rim", g, re.I)) for g in Ge])
P = V[np.unique(Fe.ravel())]
zg = V[np.unique(Fe[tyre].ravel()), 2].min() if tyre.any() else P[:, 2].min()
print(f"bbox x {P[:,0].min():+.3f}..{P[:,0].max():+.3f} (L {np.ptp(P[:,0]):.3f})  y {P[:,1].min():+.3f}..{P[:,1].max():+.3f} (W {np.ptp(P[:,1]):.3f})"
      f"  z {P[:,2].min():+.3f}..{P[:,2].max():+.3f};  ground (lowest tyre vertex) z {zg:+.3f}; height above ground {P[:,2].max()-zg:.3f}")

def raster(T, ax_u, ax_v, cell, lo, hi, key=None):
    """rasterise triangles T (n,3,3) onto the (ax_u, ax_v) plane; returns occupancy (and min of key coordinate)."""
    nu = int(np.ceil((hi[0] - lo[0]) / cell)) + 1; nv = int(np.ceil((hi[1] - lo[1]) / cell)) + 1
    occ = np.zeros((nu, nv), bool); kmin = np.full((nu, nv), np.inf)
    for t in T:
        u = (t[:, ax_u] - lo[0]) / cell; v = (t[:, ax_v] - lo[1]) / cell
        i0, i1 = int(np.floor(u.min())), int(np.ceil(u.max())); j0, j1 = int(np.floor(v.min())), int(np.ceil(v.max()))
        if i1 - i0 > 400 or j1 - j0 > 400: continue
        ii, jj = np.meshgrid(np.arange(i0, i1 + 1), np.arange(j0, j1 + 1), indexing="ij")
        pu, pv = ii + 0.5, jj + 0.5
        d = (v[1] - v[2]) * (u[0] - u[2]) + (u[2] - u[1]) * (v[0] - v[2])
        if abs(d) < 1e-12:   # edge-on triangle: mark the cells its edges cross (thin panels still block air)
            for s in np.linspace(0, 1, 8):
                for e0, e1 in ((0, 1), (1, 2), (2, 0)):
                    iu = int(u[e0] + s * (u[e1] - u[e0])); iv = int(v[e0] + s * (v[e1] - v[e0]))
                    if 0 <= iu < nu and 0 <= iv < nv: occ[iu, iv] = True
            continue
        l0 = ((v[1] - v[2]) * (pu - u[2]) + (u[2] - u[1]) * (pv - v[2])) / d
        l1 = ((v[2] - v[0]) * (pu - u[2]) + (u[0] - u[2]) * (pv - v[2])) / d
        l2 = 1 - l0 - l1
        m = (l0 >= -1e-9) & (l1 >= -1e-9) & (l2 >= -1e-9) & (ii >= 0) & (jj >= 0) & (ii < nu) & (jj < nv)
        occ[ii[m], jj[m]] = True
        if key is not None:
            kz = l0 * t[0, key] + l1 * t[1, key] + l2 * t[2, key]
            np.minimum.at(kmin, (ii[m], jj[m]), kz[m])
    return occ, kmin

T = V[Fe]
c = a.cell
occ, _ = raster(T, 1, 2, c, (P[:, 1].min(), zg), (P[:, 1].max(), P[:, 2].max()))
print(f"frontal area (y-z projection, cell {c*100:.0f} mm, incl. wheels/mirrors): A = {occ.sum()*c*c:.3f} m^2")
occ_nw, _ = raster(T[~tyre], 1, 2, c, (P[:, 1].min(), zg), (P[:, 1].max(), P[:, 2].max()))
print(f"   body only (no tyres/rims): {occ_nw.sum()*c*c:.3f} m^2")

# topology
q = np.round(V / 5e-4).astype(np.int64)
_, weld = np.unique(q, axis=0, return_inverse=True); weld = weld.ravel()
Fw = weld[Fe]
E = np.sort(np.concatenate([Fw[:, [0, 1]], Fw[:, [1, 2]], Fw[:, [2, 0]]]), axis=1)
E = E[E[:, 0] != E[:, 1]]
_, cnt = np.unique(E, axis=0, return_counts=True)
print(f"topology after 0.5 mm weld: edges {len(cnt)}, boundary (open) {np.sum(cnt==1)}, non-manifold (>2 faces) {np.sum(cnt>2)}"
      f"  -> watertight: {'YES' if np.sum(cnt==1)==0 and np.sum(cnt>2)==0 else 'NO'}")
for g in ("GEO_body_main", "undercarriage", "bumper_front", "bumper_rear", "GEO_Motorhood"):
    m = np.array([gg.startswith(g) for gg in Ge])
    if not m.any(): continue
    Eg = np.sort(np.concatenate([Fw[m][:, [0, 1]], Fw[m][:, [1, 2]], Fw[m][:, [2, 0]]]), axis=1)
    _, cg = np.unique(Eg, axis=0, return_counts=True)
    print(f"   group {g:22s}: faces {m.sum():6d}, open edges {np.sum(cg==1):6d}")

# underbody: plan view (x-y), lowest surface z per cell, body only
occp, zmin = raster(T[~tyre], 0, 1, 0.02, (P[:, 0].min(), P[:, 1].min()), (P[:, 0].max(), P[:, 1].max()), key=2)
occt, _ = raster(T, 0, 1, 0.02, (P[:, 0].min(), P[:, 1].min()), (P[:, 0].max(), P[:, 1].max()))
cl = zmin[occp] - zg
print(f"plan-view footprint {occp.sum()*4e-4:.2f} m^2 (body), lowest-surface clearance above ground: "
      f"p5 {np.percentile(cl,5)*1000:.0f} mm, p50 {np.percentile(cl,50)*1000:.0f} mm, p95 {np.percentile(cl,95)*1000:.0f} mm")
for lim in (0.15, 0.25, 0.40):
    print(f"   footprint fraction with a surface below {lim*1000:.0f} mm: {np.mean(cl < lim)*100:.0f}%")
ug = np.array([bool(re.search("undercarriage|susp|exhaust|spring|hub", g, re.I)) for g in Ge])
print(f"underbody-ish groups: {sorted(set(Ge[ug]))}, faces {ug.sum()}")
roof = np.array([bool(re.search("soft_top|roof|hardtop|top", g, re.I)) for g in Ge])
print(f"top/roof groups: {sorted(set(Ge[roof]))}, faces {roof.sum()}")
glass = np.array([bool(re.search("windshield|glass|window", g, re.I)) for g in Ge])
print(f"glass groups kept: {sorted(set(Ge[glass]))}")
