#!/bin/bash
# Package the Windows build: dist/drivechain_win64.zip (exe, DLLs, config, car inis, Miata assets, README).
# The zip includes PRIVATE assets when present (owner's own testing only; never publish). dist/ is gitignored.
set -euo pipefail
cd "$(dirname "$0")/.."
SDL2_DIR=${SDL2_DIR:-/workspace/deps/SDL2-2.30.9/x86_64-w64-mingw32}
[ -f build/win64/drivechain_demo.exe ] || tools/build_windows.sh
D=dist/drivechain_win64; rm -rf $D dist/drivechain_win64.zip; mkdir -p $D/cars $D/assets
cp build/win64/drivechain_demo.exe build/win64/drivechain.dll "$SDL2_DIR/bin/SDL2.dll" $D/
cp cars/*.ini $D/cars/
mkdir -p $D/tools $D/unity; cp build/win64/drvhost.exe $D/; cp tools/dcdrive.py $D/tools/; cp unity/DriveChain.cs unity/DriveChainDriverBridge.cs $D/unity/; mkdir -p $D/docs; cp docs/*.md $D/docs/
sed -e 's|^ini = cars/mx5_na.ini .*|ini = cars/miata_ac.ini        ; privately calibrated MX-5 (private, this build only). Reference cars: cars/mx5_na.ini, cars/hatch_fwd.ini|' \
    -e 's|^mesh_dir = private/miata_mesh .*|mesh_dir = assets/miata_mesh   ; missing -> placeholder components|' demo/demo.ini | sed 's/$/\r/' > $D/demo.ini
if [ -f private/miata_ac.ini ]; then cp private/miata_ac.ini $D/cars/; cp -r private/miata_mesh $D/assets/; else
  sed -i 's|cars/miata_ac.ini|cars/mx5_na.ini|' $D/demo.ini; fi
# TARMAC tyre variants of every car (tyre_model = 1; _tarmac_brush adds the brush-saturation option), see docs/TYRE_TARMAC.md
for f in $D/cars/*.ini; do b=${f%.ini}
  sed -e '/^\[front\]/a tyre_model = 1' -e '/^\[rear\]/a tyre_model = 1' "$f" > ${b}_tarmac.ini
  sed -e '/^\[front\]/a tyre_model = 1\ntyre_tm_shape = 1' -e '/^\[rear\]/a tyre_model = 1\ntyre_tm_shape = 1' "$f" > ${b}_tarmac_brush.ini; done
# private mesh maps (owner's testing only): every private/tracks/<name> with a track.cfg -> tracks/<name> (F6 in the demo)
for t in private/tracks/*/; do [ -f "$t/track.cfg" ] || continue; n=$(basename "$t"); mkdir -p $D/tracks/$n
  for f in track.cfg collider.bin scenery.bin line.csv pitline.csv CREDITS.md; do [ -f "$t/$f" ] && cp "$t/$f" $D/tracks/$n/; done; done
sed 's/$/\r/' tools/dist_README.md > $D/README.md
# private map notes (display name, credits) live with the map, never in tracked files: appended to the dist README only
for t in private/tracks/*/; do [ -f "$t/DIST_README.md" ] && { printf '\n'; cat "$t/DIST_README.md"; } | sed 's/$/\r/' >> $D/README.md; done
mkdir -p $D/fonts; cp demo/fonts/DejaVuSans.ttf demo/fonts/LICENSE_DejaVu.txt $D/fonts/
(cd dist && zip -qr9 drivechain_win64.zip drivechain_win64)
ls -la dist/drivechain_win64.zip
