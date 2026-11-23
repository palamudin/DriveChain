#!/usr/bin/env python3
"""flicker_stats.py prefix [prefix...] [--mask mask.png] - temporal stability of a static-camera frame sequence (<prefix>_NNN.png).
Per pixel: temporal std-dev of luma (0..255) over the sequence. Foliage/grass pixels = green-dominant in the mean frame
(g > 1.08 r, g > 1.15 b, not sky); reference = the rest. Reports mean std, 99th percentile and '% flickering pixels'
(std > 4 = visible frame-to-frame shimmer), plus a heat-map PNG <prefix>_flicker.png."""
import sys, glob
import numpy as np
from PIL import Image
args = sys.argv[1:]; mask = None
if '--mask' in args: i = args.index('--mask'); mask = np.asarray(Image.open(args[i + 1]).convert('L')) > 127; del args[i:i + 2]
# --mask: pixels of one material group (tools/flicker_mask.sh renders it from the debug material mode) instead of the green heuristic
for pre in args:
    fs = sorted(glob.glob(pre + '_[0-9][0-9][0-9].png'))
    if len(fs) < 3: print(pre, 'not enough frames'); continue
    S = np.stack([np.asarray(Image.open(f).convert('RGB'), np.float32) for f in fs])
    Y = S @ np.array([0.2126, 0.7152, 0.0722], np.float32); sd = Y.std(0); M = S.mean(0)
    r, g, b = M[..., 0], M[..., 1], M[..., 2]; fol = (g > 1.08 * r) & (g > 1.15 * b) & (g < 235) & (g > 12)
    if mask is not None: fol = mask
    # temporal 2nd difference: smooth motion (slow dolly / wind sway) is ~linear frame to frame, aliasing pops are spikes
    D2 = np.abs(Y[2:] - 2 * Y[1:-1] + Y[:-2]).mean(0)
    def st(m): v = sd[m]; d = D2[m]; return (v.mean(), np.percentile(v, 99), 100 * (v > 4).mean(), d.mean(), 100 * (d > 8).mean()) if v.size else (0,) * 5
    a, o = st(fol), st(~fol)
    tag = "mask px" if mask is not None else "foliage/grass px"
    print(f"{pre}: {len(fs)} frames | {tag} {100*fol.mean():.1f}%: std {a[0]:.2f} (p99 {a[1]:.1f}, >4: {a[2]:.2f}%), 2nd-diff {a[3]:.2f} (pops >8: {a[4]:.2f}%) | other px: std {o[0]:.2f}, 2nd-diff {o[3]:.2f} (pops {o[4]:.2f}%)")
    h = np.clip(D2 * 16, 0, 255).astype(np.uint8); Image.fromarray(np.stack([h, h // 3, 255 - h], -1)).save(pre + '_flicker.png')
