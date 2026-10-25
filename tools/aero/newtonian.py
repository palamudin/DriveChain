#!/usr/bin/env python3
"""newtonian.py - falsification test: Newtonian (impact) surface-normal aero on the real car mesh.

Method (the "surface-normal / air displacement" approximation, Newton 1687; modern hypersonic form
Cp = Cp_max sin^2(theta), Anderson, Hypersonic and High-Temperature Gas Dynamics, 2nd ed., AIAA 2006, ch. 3):
  - free stream U along -x (car driving +x), optionally yawed by beta;
  - every surface element that the oncoming air *sees first* (visibility by rasterising the projection
    perpendicular to U; the most-upstream triangle in each cell wins) gets Cp = 2 sin^2(theta), theta =
    angle between U and the surface tangent plane; all shadowed / leeward surfaces get Cp = 0;
  - load per cell: dF = -q Cp n dA, with dA = cell area / |n.u| (true area of the surface patch).
Outputs Cd, CL, CLF, CLR (SAE J1594 conventions: CLF = CL/2 + CPM, CPM about the wheelbase centre at
ground level, including the drag moment), frontal area A from the same raster.
Acceptance criterion under test (AERO_PLAN.md): net lift CL > 0 from geometry alone, CLR > CLF for the NA.
"""
import sys, argparse
import numpy as np
sys.path.insert(0, __import__("os").path.dirname(__file__))
from objload import load, wheel_x

ap = argparse.ArgumentParser()
ap.add_argument("obj"); ap.add_argument("--offset", default="0,0,0"); ap.add_argument("--cell", type=float, default=0.01)
ap.add_argument("--yaw", type=float, default=0.0, help="yaw angle beta in degrees (wind from the left = +)")
ap.add_argument("--cpmax", type=float, default=2.0, help="2 = classical Newton; 1 = stagnation-limited (incompressible)")
a = ap.parse_args()
V, F, G = load(a.obj, [float(t) for t in a.offset.split(",")])
xf, xr, zg = wheel_x(V, F, G); wb = xf - xr; xm = 0.5 * (xf + xr)

b = np.radians(a.yaw)
u = np.array([-np.cos(b), -np.sin(b), 0.0])          # air velocity direction relative to the car
e1 = np.array([-u[1], u[0], 0.0]); e2 = np.array([0, 0, 1.0])  # projection plane basis
T = V[F]
N = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]); An = np.linalg.norm(N, axis=1); ok = An > 1e-14
T, N = T[ok], N[ok] / An[ok, None]
s = -(T.mean(1) @ u)                                   # upstream distance (bigger = hit first)
pu = T @ e1; pv = T @ e2
c = a.cell; u0, v0 = pu.min(), pv.min()
nu = int((pu.max() - u0) / c) + 2; nv = int((pv.max() - v0) / c) + 2
best = np.full((nu, nv), -np.inf); who = np.full((nu, nv), -1, np.int64)
for k in range(len(T)):
    uu = (pu[k] - u0) / c; vv = (pv[k] - v0) / c
    i0, i1 = int(uu.min()), int(np.ceil(uu.max())); j0, j1 = int(vv.min()), int(np.ceil(vv.max()))
    ii, jj = np.meshgrid(np.arange(i0, i1 + 1), np.arange(j0, j1 + 1), indexing="ij"); x, y = ii + .5, jj + .5
    d = (vv[1] - vv[2]) * (uu[0] - uu[2]) + (uu[2] - uu[1]) * (vv[0] - vv[2])
    if abs(d) < 1e-12: continue
    l0 = ((vv[1] - vv[2]) * (x - uu[2]) + (uu[2] - uu[1]) * (y - vv[2])) / d
    l1 = ((vv[2] - vv[0]) * (x - uu[2]) + (uu[0] - uu[2]) * (y - vv[2])) / d
    m = (l0 >= 0) & (l1 >= 0) & (1 - l0 - l1 >= 0)
    if not m.any(): continue
    dk = -(T[k] @ u)
    sk = l0 * dk[0] + l1 * dk[1] + (1 - l0 - l1) * dk[2]   # exact hit depth per cell
    ii, jj, sk = ii[m], jj[m], sk[m]
    w = sk > best[ii, jj]; best[ii[w], jj[w]] = sk[w]; who[ii[w], jj[w]] = k

hit = who[who >= 0]
n = N[hit].copy(); n[(n @ u) > 0] *= -1                 # orient each seen face toward the oncoming air (n.u < 0)
cosn = -(n @ u)                                         # = sin(theta)
cp = a.cpmax * cosn ** 2
dA = c * c / np.maximum(cosn, 1e-3)
dF = -(cp * dA)[:, None] * n                            # per unit q
# application points: cell centres on the hit surface
iu, jv = np.nonzero(who >= 0)
P = (u0 + (iu + .5) * c)[:, None] * e1 + (v0 + (jv + .5) * c)[:, None] * e2 - best[iu, jv][:, None] * u
A = len(hit) * c * c
Fx, Fy, Fz = dF.sum(0)
# moments about wheelbase centre at ground (xm, 0, zg); pitch moment positive nose-up = -(My) in x-fwd,y-left,z-up
r = P - np.array([xm, 0, zg]); M = np.cross(r, dF).sum(0)
D = (dF @ u).sum()                                      # drag = force component along the air velocity
CD = D / A; CL = Fz / A; CS = Fy / A; CPM = -M[1] / (A * wb)
CLF = CL / 2 + CPM; CLR = CL / 2 - CPM
print(f"Newtonian Cp=({a.cpmax:g})sin^2, yaw {a.yaw:+.1f} deg, cell {c*1000:.0f} mm; axles x {xf:+.3f}/{xr:+.3f} (wb {wb:.3f}), ground z {zg:+.3f}")
print(f"  projected area A = {A:.3f} m^2   CdA = {D:.3f} m^2   ClA = {Fz:+.3f} m^2")
print(f"  Cd = {CD:.3f}   CL = {CL:+.3f}   CLF = {CLF:+.3f}   CLR = {CLR:+.3f}   CS = {CS:+.3f}   CPM = {CPM:+.3f}")
up = dF[:, 2] > 0
print(f"  vertical load split: upward-pushing cells {up.mean()*100:.0f}% (sum {dF[up,2].sum()/A:+.3f}), downward {(~up).mean()*100:.0f}% (sum {dF[~up,2].sum()/A:+.3f})")
print(f"  verdict vs net-lift criterion (CL > 0, CLR > CLF): {'PASS' if CL > 0 and CLR > CLF else 'FAIL'}")
