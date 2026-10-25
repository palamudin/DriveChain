#!/usr/bin/env python3
"""plot_hyst.py - on-centre steering hysteresis from `mbcompare ... hyst` runs.

Usage: plot_hyst.py rootdir out.png out.csv
rootdir holds <car>_<jf>_a<assist>/hyst0g.csv and hyst06g.csv (jf = car.joint_friction, assist = steer.assist),
e.g. produced by
  PSET="car.joint_friction=1;steer.assist=0" build/mbcompare cars/mx5_na.ini root/mx5_na_1_a0 1 hyst
Metric: rim friction = half the loop width, (T_up - T_down)/2, of column torque at the mean rim angle
over the fitted cycles (t > 40 s). A cosine fit is not used: vehicle response lag adds a phase term that
masquerades as friction."""
import csv, math, sys, os

def width(fn):
    r = [x for x in csv.DictReader(open(fn)) if float(x['t']) > 40]
    th = [float(x['sw']) for x in r]; tq = [float(x['column_torque']) for x in r]
    th0 = sum(th) / len(th); up = []; dn = []
    for i in range(1, len(r)):
        if (th[i-1] - th0) * (th[i] - th0) <= 0 and th[i] != th[i-1]:
            f = (th0 - th[i-1]) / (th[i] - th[i-1]); T = tq[i-1] + f * (tq[i] - tq[i-1])
            (up if th[i] > th[i-1] else dn).append(T)
    ay = sum(float(x['ay']) for x in r) / len(r)
    rack = sum(abs(float(x['rack_force'])) for x in r) / len(r)
    return (sum(up) / len(up) - sum(dn) / len(dn)) / 2, th, tq, ay, rack

def main():
    root, png, out = sys.argv[1], sys.argv[2], sys.argv[3]
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    import re
    cars = sorted({m.group(1) for d in os.listdir(root) for m in [re.match(r'(.+)_[01]_a[01]$', d)] if m})
    fig, ax = plt.subplots(len(cars), 4, figsize=(18, 4 * len(cars)), squeeze=False)
    rows = []
    for ci, c in enumerate(cars):
        for k, (case, a) in enumerate([(x, y) for y in (1, 0) for x in ('hyst0g', 'hyst06g')]):
            for jf, col in [(0, 'tab:blue'), (1, 'tab:red')]:
                fn = os.path.join(root, '%s_%d_a%d' % (c, jf, a), case + '.csv')
                if not os.path.exists(fn): continue
                w, th, tq, ay, rack = width(fn); rows.append((c, a, case, jf, ay, w, rack))
                ax[ci][k].plot([t * 180 / math.pi for t in th], tq, col, lw=.8, label='jf=%d: Tf %.2f N m' % (jf, w))
            ax[ci][k].set_title('%s assist=%d %s' % (c, a, case)); ax[ci][k].legend(fontsize=8); ax[ci][k].grid(alpha=.3)
            ax[ci][k].set_xlabel('rim deg'); ax[ci][k].set_ylabel('column torque N m')
    plt.tight_layout(); plt.savefig(png, dpi=70)
    with open(out, 'w') as f:
        f.write('car,assist,case,joint_friction,ay_mean,rim_friction_Nm,rack_force_abs_mean_N\n')
        for r in rows:
            f.write('%s,%d,%s,%d,%.3f,%.3f,%.1f\n' % r)
            print('%-10s assist=%d %-8s jf=%d ay %.2f  rim friction %.3f N m' % r[:6])

if __name__ == '__main__':
    main()
