#!/usr/bin/env python3
"""tyre_reference.py - measured-fit passenger-tyre reference vs our tyres.

Reference: Magic Formula 6.1 pure-lateral fit of a 205/60R15 passenger tyre (Fz0 4 kN, 200 kPa,
16.7 m/s), the public MF-Tyre example set (redistributed e.g. as ExternData sample.tir, BSD-3;
coefficients as in Pacejka, Tyre and Vehicle Dynamics, 3rd ed. 2012, App. 3).  Only the
coefficients needed for Fy0 and Mz0 at zero camber and nominal pressure are used.

Usage: tyre_reference.py tyre_compare_outdir [png] [csv]
Reads lat_<model>.csv written by build/tyre_compare and prints, per load:
C_alpha (N/rad), peak mu, slip angle at peak, Fy(15deg)/peak, Fy(20deg)/peak, trail at 0.5 deg."""
import sys, math, csv, os

MF = dict(FNOMIN=4000.0, R0=0.3135,
          PCY1=1.337, PDY1=0.8785, PDY2=-0.06452, PEY1=-0.8057, PEY2=-0.6046, PEY3=0.09854,
          PKY1=-15.324, PKY2=1.715, PKY4=2.0005, PHY1=-0.001806, PHY2=0.00352,
          PVY1=-0.00661, PVY2=0.03592,
          QBZ1=12.035, QBZ2=-1.33, QBZ3=0.0, QCZ1=1.2923, QDZ1=0.09068, QDZ2=-0.00565,
          QEZ1=-1.7924, QEZ2=0.8975, QEZ3=0.0, QEZ4=0.2895, QHZ1=0.0014333, QHZ2=0.0024087,
          QBZ9=34.5, QDZ6=0.0017015, QDZ7=-0.002091,
          PCX1=1.579, PDX1=1.0422, PDX2=-0.08285, PEX1=0.11113, PEX2=0.3143, PEX3=0.0, PEX4=0.001719,
          PKX1=21.687, PKX2=13.728, PKX3=-0.4098)

# Burckhardt (1993) dry-asphalt mu(slip) fit, as tabulated in Kiencke & Nielsen, Automotive Control Systems:
# mu(s) = c1 (1 - exp(-c2 s)) - c3 s.  Locked-wheel reference for large sliding speeds (s = 1 -> slide = V).
BURCKHARDT_DRY = (1.2801, 23.99, 0.52)
def burckhardt(s, c=BURCKHARDT_DRY):
    return c[0] * (1 - math.exp(-c[1] * s)) - c[2] * s

def mf_fx(Fz, k, p=MF):
    """Pure longitudinal Fx magnitude for slip ratio k (braking side, sym, no shifts)."""
    Fz0 = p['FNOMIN']; dfz = (Fz - Fz0) / Fz0
    Cx = p['PCX1']; Dx = (p['PDX1'] + p['PDX2'] * dfz) * Fz
    Ex = min(1.0, (p['PEX1'] + p['PEX2'] * dfz + p['PEX3'] * dfz * dfz) * (1 + p['PEX4']))
    Kx = Fz * (p['PKX1'] + p['PKX2'] * dfz) * math.exp(p['PKX3'] * dfz)
    Bx = Kx / (Cx * Dx); x = Bx * k
    return Dx * math.sin(Cx * math.atan(x - Ex * (x - math.atan(x))))

def mf(Fz, a, p=MF, sym=True):
    """Pure lateral Fy (N, magnitude convention: positive for positive slip), trail t (m), Mz (N m).
    sym=True drops the small ply-steer/conicity shifts (SHy, SVy, SHt) so curves are odd in alpha."""
    Fz0 = p['FNOMIN']; dfz = (Fz - Fz0) / Fz0
    SHy = 0 if sym else p['PHY1'] + p['PHY2'] * dfz
    SVy = 0 if sym else Fz * (p['PVY1'] + p['PVY2'] * dfz)
    ay = a + SHy
    Cy = p['PCY1']; Dy = (p['PDY1'] + p['PDY2'] * dfz) * Fz
    Ey = (p['PEY1'] + p['PEY2'] * dfz) * (1 - p['PEY3'] * math.copysign(1, ay) if ay else 1)
    Ey = min(Ey, 1.0)
    Ky = abs(p['PKY1']) * Fz0 * math.sin(p['PKY4'] * math.atan(Fz / (p['PKY2'] * Fz0)))
    By = Ky / (Cy * Dy)
    x = By * ay
    Fy = Dy * math.sin(Cy * math.atan(x - Ey * (x - math.atan(x)))) + SVy
    SHt = 0 if sym else p['QHZ1'] + p['QHZ2'] * dfz
    at = a + SHt
    Bt = p['QBZ1'] + p['QBZ2'] * dfz + p['QBZ3'] * dfz * dfz
    Ct = p['QCZ1']
    Dt = Fz * (p['R0'] / Fz0) * (p['QDZ1'] + p['QDZ2'] * dfz)
    Et = (p['QEZ1'] + p['QEZ2'] * dfz + p['QEZ3'] * dfz * dfz) * (1 + p['QEZ4'] * (2 / math.pi) * math.atan(Bt * Ct * at))
    Et = min(Et, 1.0)
    t = Dt * math.cos(Ct * math.atan(Bt * at - Et * (Bt * at - math.atan(Bt * at)))) * math.cos(a)
    Br = p['QBZ9']; Dr = Fz * p['R0'] * (p['QDZ6'] + p['QDZ7'] * dfz)
    Mzr = Dr * math.cos(math.atan(Br * a)) * math.cos(a) if not sym else 0.0
    return Fy, t, -t * Fy + Mzr

def metrics(curve):
    """curve: list of (alpha_deg, Fy, trail_mm) for alpha >= 0, one load."""
    curve = sorted(curve)
    a1 = [c for c in curve if 0 < c[0] <= 0.5 + 1e-9]
    C = a1[-1][1] / math.radians(a1[-1][0])
    pk = max(curve, key=lambda c: c[1])
    def at(deg):
        best = min(curve, key=lambda c: abs(c[0] - deg)); return best[1]
    tr = a1[-1][2]
    return C, pk[1], pk[0], at(15) / pk[1], at(20) / pk[1], tr

def ref_curve(Fz):
    out = []
    for i in range(0, 401):
        ad = i * 0.05
        Fy, t, _ = mf(Fz, math.radians(ad)); out.append((ad, Fy, t * 1000))
    return out

def read_ours(path):
    d = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            if abs(float(r['gamma_deg'])) > 1e-6: continue
            a = float(r['alpha_deg'])
            if a < 0: continue
            d.setdefault(float(r['Fz']), []).append((a, abs(float(r['Fy'])), abs(float(r['trail_mm']))))
    return d

def main():
    od = sys.argv[1]; png = sys.argv[2] if len(sys.argv) > 2 else None; csvo = sys.argv[3] if len(sys.argv) > 3 else None
    models = [m for m in ('envelope', 'tarmac', 'tarmac_brush') if os.path.exists(os.path.join(od, 'lat_%s.csv' % m))]
    ours = {m: read_ours(os.path.join(od, 'lat_%s.csv' % m)) for m in models}
    loads = sorted(next(iter(ours.values())).keys()) if ours else [2000, 3000, 4000, 5000]
    rows = []
    hdr = '%-13s %6s %9s %6s %7s %7s %7s %8s' % ('model', 'Fz', 'Ca N/rad', 'mu', 'a_pk', 'F15/pk', 'F20/pk', 'tr.5 mm')
    print(hdr)
    for Fz in loads:
        for name, cur in [('MF205/60R15', ref_curve(Fz))] + [(m, ours[m][Fz]) for m in models]:
            C, pk, apk, r15, r20, tr = metrics(cur)
            rows.append((name, Fz, C, pk / Fz, apk, r15, r20, tr))
            print('%-13s %6.0f %9.0f %6.3f %7.2f %7.3f %7.3f %8.1f' % rows[-1])
    print()
    print('longitudinal (braking side, 15 m/s rig): C_kappa N, mu_x peak, kappa at peak, Fx(0.3)/pk, Fx(0.6)/pk')
    lfz = None
    for m in models:
        fn = os.path.join(od, 'long_%s.csv' % m)
        if not os.path.exists(fn): continue
        cur = []
        for r in csv.DictReader(open(fn)):
            k = float(r['kappa'])
            if k <= 0: cur.append((-k, abs(float(r['Fx'])))); lfz = float(r['Fz'])
        cur.sort()
        C = cur[1][1] / cur[1][0]; pk = max(cur, key=lambda c: c[1])
        at = lambda kk: min(cur, key=lambda c: abs(c[0] - kk))[1]
        lrow = (m, lfz, C, pk[1] / lfz, pk[0], at(0.3) / pk[1], at(0.6) / pk[1]); rows.append(lrow + (float('nan'),))
        print('%-13s %6.0f %9.0f %6.3f %7.2f %7.3f %7.3f' % lrow)
    if lfz:
        ks = [i * 0.001 for i in range(0, 1001)]
        f = [mf_fx(lfz, k) for k in ks]; i = max(range(len(f)), key=lambda j: f[j])
        lrow = ('MFx205/60R15', lfz, mf_fx(lfz, 0.01) / 0.01, f[i] / lfz, ks[i], mf_fx(lfz, 0.3) / f[i], mf_fx(lfz, 0.6) / f[i])
        rows.append(lrow + (float('nan'),)); print('%-13s %6.0f %9.0f %6.3f %7.2f %7.3f %7.3f' % lrow)
        b = [burckhardt(k) for k in ks]; i = max(range(len(b)), key=lambda j: b[j])
        print('Burckhardt dry asphalt: mu peak %.3f at s %.3f, mu(0.3)/pk %.3f, mu(0.6)/pk %.3f, mu(1.0)/pk %.3f (locked)'
              % (b[i], ks[i], burckhardt(0.3) / b[i], burckhardt(0.6) / b[i], burckhardt(1.0) / b[i]))
    if csvo:
        with open(csvo, 'w') as f:
            f.write('model,Fz,C_alpha,mu_peak,alpha_peak_deg,Fy15_over_peak,Fy20_over_peak,trail_0p5deg_mm\n')
            for r in rows: f.write('%s,%.0f,%.0f,%.4f,%.2f,%.4f,%.4f,%.2f\n' % r)
    if png:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(13, 5))
        cols = {'MF205/60R15': 'k', 'envelope': 'tab:blue', 'tarmac': 'tab:red', 'tarmac_brush': 'tab:orange'}
        for Fz, ls in zip(loads, ['-', '--', '-.', ':']):
            for name, cur in [('MF205/60R15', ref_curve(Fz))] + [(m, ours[m][Fz]) for m in models]:
                cur = sorted(cur)
                ax[0].plot([c[0] for c in cur], [c[1] / Fz for c in cur], ls, color=cols[name], label='%s %.0f N' % (name, Fz))
                ax[1].plot([c[0] for c in cur], [c[2] for c in cur], ls, color=cols[name])
        ax[0].set_xlabel('slip angle deg'); ax[0].set_ylabel('Fy/Fz'); ax[0].grid(alpha=.3); ax[0].legend(fontsize=6, ncol=2)
        ax[1].set_xlabel('slip angle deg'); ax[1].set_ylabel('pneumatic trail mm'); ax[1].set_xlim(0, 10); ax[1].grid(alpha=.3)
        fig.suptitle('Lateral: measured-fit MF 6.1 205/60R15 reference (black) vs DriveChain tyres (mx5_na front)')
        fig.tight_layout(); fig.savefig(png, dpi=110)

if __name__ == '__main__':
    main()
