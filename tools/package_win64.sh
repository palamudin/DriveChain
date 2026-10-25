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
# opt-in tyre-study proposals for the 185/60R14 cars (docs/TYRE_DATA.md; defaults unchanged): _propF2 (envelope), _propD (TARMAC).
# Keys are appended at the end of [front] and [rear], so they override the car's own values (last key wins).
for f in $D/cars/mx5_na.ini $D/cars/miata_ac.ini; do [ -f "$f" ] || continue; python3 - "$f" <<'PY'
import sys, re
f = sys.argv[1]; src = open(f, newline='').read()
COMMON = ["tyre_mu_y = 0.955", "tyre_mu_x = 0.926", "tyre_ls_exp_y = 0.90", "tyre_ls_exp_x = 0.92", "tyre_trail_factor = 1.3"]
SETS = {"propF2": COMMON + ["tyre_falloff_level = 0.75", "tyre_falloff_speed = 4"],
        "propD": COMMON + ["tyre_model = 1", "tyre_tm_shape = 1", "tyre_tm_damp_mode = 1", "tyre_tm_load_sens = 0.10",
                           "tyre_tm_slide_floor = 0.70", "tyre_tm_slide_exp = 2", "tyre_tm_slide_speed = 8"]}
for name, keys in SETS.items():
    out, sec = [], None
    lines = src.splitlines()
    for i, l in enumerate(lines):
        m = re.match(r"\s*\[(\w+)\]", l)
        if m and sec in ("front", "rear"): out += ["; tyre-study proposal " + name] + keys
        if m: sec = m.group(1)
        out.append(l)
    if sec in ("front", "rear"): out += ["; tyre-study proposal " + name] + keys
    open(f[:-4] + "_" + name + ".ini", "w", newline='').write("\r\n".join(x.rstrip("\r") for x in out) + "\r\n")
PY
done
# opt-in proposals for every base car (defaults unchanged); each writes <car>_<suffix>.ini with keys appended to [car]:
#  _propV = rotary-valve pressure states (mb_valve_model = 1, contoured edges), pump at full flow from idle (docs/VALVE.md)
#  _propB = compliant arm-pivot bushings at documented estimates (docs/BUSHINGS.md)
#  _propC = propshaft and diff-carrier rotors in the driveline (docs/CARRIER.md)
# MB-core opt-ins: only for cars that run the multibody core (solver = 1); on legacy-core cars the keys would be inert
mkprop() { suffix=$1; shift; for f in $D/cars/*.ini; do case "$f" in *_tarmac*|*_prop*) continue;; esac
grep -Eq '^solver *= *1' "$f" || continue
python3 - "$f" "$suffix" "$@" <<'PY'
import sys, re
f, suffix, KEYS = sys.argv[1], sys.argv[2], sys.argv[3:]
lines = open(f, newline='').read().splitlines(); out, sec = [], None
for l in lines:
    m = re.match(r"\s*\[(\w+)\]", l)
    if m and sec == "car": out += KEYS
    if m: sec = m.group(1)
    out.append(l)
if sec == "car": out += KEYS
open(f[:-4] + "_" + suffix + ".ini", "w", newline='').write("\r\n".join(x.rstrip("\r") for x in out) + "\r\n")
PY
done; }
mkprop propV "; hydraulic-valve proposal propV (docs/VALVE.md)" "mb_valve_model = 1" "assist_rpm_full = 800"
mkprop propB "; compliant-bushing proposal propB (docs/BUSHINGS.md)" "mb_bush_kr = 2e6, 2e6" "mb_bush_ka = 5e5, 5e5" "mb_bush_kt = 300, 300"
mkprop propC "; carrier-inertia proposal propC (docs/CARRIER.md)" "mb_diff_inertia = 0.02" "mb_gb_out_inertia = 0.01"
# private mesh maps (owner's testing only): every private/tracks/<name> with a track.cfg -> tracks/<name> (F6 in the demo)
for t in private/tracks/*/; do [ -f "$t/track.cfg" ] || continue; n=$(basename "$t"); mkdir -p $D/tracks/$n
  for f in track.cfg collider.bin scenery.bin line.csv pitline.csv CREDITS.md; do [ -f "$t/$f" ] && cp "$t/$f" $D/tracks/$n/; done; done
sed 's/$/\r/' tools/dist_README.md > $D/README.md
# private map notes (display name, credits) live with the map, never in tracked files: appended to the dist README only
for t in private/tracks/*/; do [ -f "$t/DIST_README.md" ] && { printf '\n'; cat "$t/DIST_README.md"; } | sed 's/$/\r/' >> $D/README.md; done
mkdir -p $D/fonts; cp demo/fonts/DejaVuSans.ttf demo/fonts/LICENSE_DejaVu.txt $D/fonts/
(cd dist && zip -qr9 drivechain_win64.zip drivechain_win64)
ls -la dist/drivechain_win64.zip
