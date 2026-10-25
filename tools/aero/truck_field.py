#!/usr/bin/env python3
"""Prototype: truck "bow wave" pressure field acting on a car-sized box (option b of AERO_PLAN 12).

Truck is a slender body of equivalent cross-section S(x), on the ground. Together with its
ground image it is a body of area 2S whose axis lies ON the ground plane, so a line of sources
on z=0 with strength sigma(x) = U * d(2S)/dx satisfies the ground condition automatically
(slender-body theory: Katz & Plotkin, Low-Speed Aerodynamics, ch. 3/10; Newman, Marine
Hydrodynamics, ch. 7).  The separated wake is represented as a displacement body that closes
over L_wake behind the base [U: L_wake is a CFD-calibration parameter].

Car: a box (L 3.97, W 1.68, H 1.23 m, 0.15 m clearance), the default and no private data.
Force on the car = Froude-Krylov integral of the truck's (steady, truck-frame) pressure over
the car surface: F = -sum p n dA.  The car's own diffraction/added-mass response and the
car's own map loads in the induced crossflow are NOT included: this is an order-of-magnitude
check of the incident-pressure loads, and a timing check for the runtime source model.

Prints the side force, yaw moment (about the car centre, +z up, + = nose toward truck) and
lift vs longitudinal offset X (car centre minus truck centre, +ahead), for a given gap.
"""
import argparse, time
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--U", type=float, default=25.0, help="truck speed through still air, m/s")
ap.add_argument("--gap", type=float, default=1.0, help="lateral clearance truck side to car side, m")
ap.add_argument("--Lt", type=float, default=16.5); ap.add_argument("--Wt", type=float, default=2.55)
ap.add_argument("--Ht", type=float, default=4.0); ap.add_argument("--Lnose", type=float, default=1.5)
ap.add_argument("--Lwake", type=float, default=4.0)
ap.add_argument("--rho", type=float, default=1.2)
ap.add_argument("--nsrc", type=int, default=64)
a = ap.parse_args()

# ---- truck source line on z=0, truck centre at x=0, centreline y=0, moving +x ----
S2 = 2 * a.Wt * a.Ht                          # body+image area
xn, xb = a.Lt / 2, -a.Lt / 2                  # nose, base
def area(x):                                  # smooth nose ramp, constant, wake closure
    s = np.where(x > xn, 0.0, 1.0)
    nose = (x > xn - a.Lnose) & (x <= xn)
    s = np.where(nose, np.sin(0.5 * np.pi * (xn - x) / a.Lnose) ** 2, s)
    wake = x < xb
    s = np.where(wake, np.exp(-((xb - x) / a.Lwake) ** 2), s)
    return S2 * s
xs = np.linspace(xn + 0.1, xb - 3 * a.Lwake, 4 * a.nsrc)
Sx = area(xs)
# in the truck frame the air moves at -U x; source strength per length = -U dS/dx (dS/dx<0 at nose
# going +x means area grows going -x, i.e. downstream) -> positive source at the nose, sinks in wake
dSdx = np.gradient(Sx, xs)
sig = -a.U * dSdx * abs(xs[1] - xs[0])         # lumped point-source strengths (m^3/s)
# lump to nsrc sources
idx = np.array_split(np.arange(len(xs)), a.nsrc)
px = np.array([np.average(xs[i], weights=np.abs(sig[i]) + 1e-12) for i in idx])
pq = np.array([sig[i].sum() for i in idx])

def vel(P):                                   # P (n,3) -> induced velocity (n,3), truck frame
    d = P[:, None, :] - np.stack([px, np.zeros_like(px), np.zeros_like(px)], 1)[None]
    r3 = (np.einsum("ijk,ijk->ij", d, d) + 1e-6) ** 1.5
    return np.einsum("j,ijk->ik", pq / (4 * np.pi), d / r3[..., None])

# ---- car box panels ----
Lc, Wc, Hc, hc = 3.97, 1.68, 1.23, 0.15
n = 8
def face(c, e1, e2, nrm):
    u = (np.arange(n) + 0.5) / n - 0.5
    U1, U2 = np.meshgrid(u, u)
    P = c + U1.reshape(-1, 1) * e1 + U2.reshape(-1, 1) * e2
    A = np.linalg.norm(np.cross(e1, e2)) / n ** 2
    return P, np.tile(nrm, (len(P), 1)), np.full(len(P), A)
zc = hc + Hc / 2
faces = [face(np.array([ Lc/2, 0, zc]), np.array([0, Wc, 0]), np.array([0, 0, Hc]), np.array([ 1, 0, 0])),
         face(np.array([-Lc/2, 0, zc]), np.array([0, Wc, 0]), np.array([0, 0, Hc]), np.array([-1, 0, 0])),
         face(np.array([0,  Wc/2, zc]), np.array([Lc, 0, 0]), np.array([0, 0, Hc]), np.array([0,  1, 0])),
         face(np.array([0, -Wc/2, zc]), np.array([Lc, 0, 0]), np.array([0, 0, Hc]), np.array([0, -1, 0])),
         face(np.array([0, 0, hc + Hc]), np.array([Lc, 0, 0]), np.array([0, Wc, 0]), np.array([0, 0,  1])),
         face(np.array([0, 0, hc]),      np.array([Lc, 0, 0]), np.array([0, Wc, 0]), np.array([0, 0, -1]))]
Pc = np.vstack([f[0] for f in faces]); Nc = np.vstack([f[1] for f in faces]); Ac = np.concatenate([f[2] for f in faces])

yoff = -(a.Wt / 2 + a.gap + Wc / 2)           # car on the truck's -y side (truck to the car's +y, i.e. car's left)
q = 0.5 * a.rho * a.U ** 2
print(f"truck {a.Lt}x{a.Wt}x{a.Ht} m at {a.U} m/s through still air; gap {a.gap} m; q_truck = {q:.0f} Pa")
print(" X/Lt   Fy[N] (+ toward truck)  Mz[N m] (+ nose toward truck)  Fz[N]   Fx[N]")
rows = []
for X in np.arange(1.6, -1.61, -0.1) * a.Lt:
    P = Pc + np.array([X, yoff, 0])
    V = vel(P) + np.array([-a.U, 0, 0])
    p = 0.5 * a.rho * (a.U ** 2 - np.einsum("ij,ij->i", V, V))
    dF = -(p * Ac)[:, None] * Nc
    F = dF.sum(0); M = np.cross(Pc, dF).sum(0)
    rows.append((X / a.Lt, F[1], M[2], F[2], F[0]))
    print(f"{X/a.Lt:+5.2f}  {F[1]:+9.1f}               {M[2]:+9.1f}                    {F[2]:+7.1f}  {F[0]:+7.1f}")
r = np.array(rows)
print(f"peak |Fy| {np.abs(r[:,1]).max():.0f} N, peak |Mz| {np.abs(r[:,2]).max():.0f} N m; "
      f"as car coefficients (q_car=q, A=1.6 m^2, l=2.265 m): dCS {np.abs(r[:,1]).max()/(q*1.6):.3f}, "
      f"dCN {np.abs(r[:,2]).max()/(q*1.6*2.265):.3f}")
# timing: 1024 panels x nsrc sources
P = np.random.rand(1024, 3)
t = time.perf_counter(); [vel(P) for _ in range(20)]; dt = (time.perf_counter() - t) / 20
print(f"numpy timing: 1024 panels x {a.nsrc} sources: {dt*1e6:.0f} us/eval (C estimate ~{1024*a.nsrc*20/3e9*1e6:.0f} us at ~20 flop/pair, 3 Gflop/s)")
