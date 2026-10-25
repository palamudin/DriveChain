#!/usr/bin/env python3
"""wheel_inertia.py - component estimate of wheel spin inertia (tyre + rim + brake disc + hub) about the axle.

Tyre and rim terms are scaled from Ubysz, "Problems of rotational mass in passenger vehicles" (Transport Problems,
Silesian Univ. of Technology), Table 3: superposition/CAD values checked against a measured chart. Reference rows
(tyre mass kg, dynamic radius m, tyre I kg m^2 / rim mass kg, rim I kg m^2):
  175/70R14: tyre 7.2 kg, rd 0.291, I_tyre 0.518; steel rim 7.0 kg, I_rim 0.191; wheel total 0.71 (chart 0.69)
  195/60R15: tyre 7.8 kg, rd 0.298, I_tyre 0.583; steel rim 7.1 kg, I_rim 0.191; wheel total 0.774 (chart 0.77)
Scaling: I_tyre ~ m r^2 (70-75 % of tyre inertia sits in the tread band, Ubysz), I_rim ~ m (r_rim)^2.
The brake disc is an annulus, I = m (ro^2 + ri^2) / 2. Masses are documented estimates (see docs/WHEEL_INERTIA.md).
"""
REF_T = (7.8, 0.298, 0.583)      # 195/60R15 tyre: mass, rd, I
REF_R = (7.1, 0.1905, 0.191)     # 15" steel rim: mass, bead-seat radius, I

def tyre(m, rd): return REF_T[2] * (m / REF_T[0]) * (rd / REF_T[1]) ** 2
def rim(m, r, alloy): return REF_R[2] * (m / REF_R[0]) * (r / REF_R[1]) ** 2 * (0.85 if alloy else 1.0)  # alloy: more mass in spokes/hub
def disc(m, ro, ri): return m * (ro * ro + ri * ri) / 2

CASES = [
    # name, tyre kg, rd m, rim kg, bead-seat r m, alloy, disc kg, disc ro, disc ri, hub/rotating-bearing I
    ("mx5_na front: 185/60R14, 14x5.5 alloy, 235 mm vented disc", 7.5, 0.285, 6.0, 0.1778, True, 4.0, 0.1175, 0.065, 0.005),
    ("mx5_na rear:  185/60R14, 14x5.5 alloy, 231 mm solid disc",  7.5, 0.285, 6.0, 0.1778, True, 2.8, 0.1155, 0.065, 0.005),
    ("hatch_fwd front: 205/55R16, 16x6.5 alloy, 280 mm vented",   8.5, 0.310, 8.0, 0.2032, True, 6.0, 0.140, 0.075, 0.006),
    ("hatch_fwd rear:  205/55R16, 16x6.5 alloy, 264 mm solid",    8.5, 0.310, 8.0, 0.2032, True, 4.0, 0.132, 0.075, 0.006),
    ("check: 13in steel 165/70R13 (Ubysz 0.39 calc / 0.435 chart)", 5.5, 0.273, 5.2, 0.165, False, 0, 0.1, 0.05, 0.0),
]
if __name__ == "__main__":
    for n, mt, rd, mr, rr, al, md, ro, ri, hub in CASES:
        it, ir, idc = tyre(mt, rd), rim(mr, rr, al), disc(md, ro, ri)
        print(f"{n:62s} tyre {it:.3f} rim {ir:.3f} disc {idc:.3f} hub {hub:.3f}  total {it+ir+idc+hub:.3f} kg m^2")
