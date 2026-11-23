#!/bin/bash
# Moving-camera flicker benchmark (native, lavapipe-friendly): 12-frame dolly from a fixed camera, then tools/flicker_stats.py.
# usage: tools/flicker_native.sh <out-prefix> [extra drivechain-native args...]   (run from the repo root; output scratch/flk/)
# Default view = the "trees" still (cam-rel 14,4.5,6), dolly 1.3 cm/frame (sub-pixel motion: filtered content changes ~linearly, aliasing pops) sideways+forward, 960x540, grain 0, frozen wind (--dt 0).
cd "$(dirname "$0")/.."; P=$1; shift; mkdir -p scratch/flk
( cd native && ./target/release/drivechain-native --still ../scratch/flk/$P.png --frames 200 --size ${SIZE:-960x540} --cam-rel ${CAMREL:-14,4.5,6} \
    --seq ${N:-12} --cam-step ${STEP:-0.012,0,0.006} --dt 0 --grain 0 "$@" 2>&1 | grep -E "ms|vegetation:|isolate|mat" ) > scratch/flk/$P.log
python3 tools/flicker_stats.py scratch/flk/$P ${MASK:+--mask $MASK} | tee -a scratch/flk/$P.log
