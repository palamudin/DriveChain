#!/bin/bash
# Pixel mask of one or more material groups at the flicker-benchmark view: render the frame with every group in 'debug' (unlit
# solid colour), once more with the target group(s) off, and keep the pixels that differ (= where the group is visible, occlusion
# included). usage: tools/flicker_mask.sh <name> <group[,group...]>   -> scratch/flk/mask_<name>.png  (then MASK=... tools/flicker_native.sh)
cd "$(dirname "$0")/.."; N=$1; G=$2; mkdir -p scratch/flk
off=$(echo "$G" | sed 's/,/=off,/g; s/$/=off/')
run() { ( cd native && ./target/release/drivechain-native --still ../scratch/flk/$1.png --frames 200 --size ${SIZE:-960x540} --cam-rel ${CAMREL:-14,4.5,6} \
    --dt 0 --grain 0 --gi 0 --volumetric 0 --bloom 0 --taa off --cheap-sky --mat "all=debug${2:+,$2}" >/dev/null 2>&1 ); }
run _m_a "" && run _m_b "$off"
python3 - "$N" <<'PY'
import sys, numpy as np; from PIL import Image
a = np.asarray(Image.open('scratch/flk/_m_a.png').convert('RGB'), np.int16); b = np.asarray(Image.open('scratch/flk/_m_b.png').convert('RGB'), np.int16)
m = (np.abs(a - b).max(-1) > 6); Image.fromarray((m * 255).astype(np.uint8)).save(f'scratch/flk/mask_{sys.argv[1]}.png'); print(f"mask_{sys.argv[1]}: {100*m.mean():.1f}% of pixels")
PY
