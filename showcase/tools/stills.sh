#!/bin/bash
# Beauty stills + old-vs-new comparison (headless Chrome, SwiftShader WebGL2). Output: ../stills. Skips existing files.
# warm=90: drive 3 s on the cheap tier, then freeze the sim and switch tier (full tiers are ~0.3 fps under SwiftShader).
# godrays=0: god rays hang SwiftShader's WebGL2 main thread (unverified on real GPUs).
cd "$(dirname "$0")/.." ; S=tools/shoot.mjs; O=stills; mkdir -p $O; W=${W:-1280}; H=${H:-720}
COMMON="warm=90&auto&still=1&webgl&clean&sunel=17&sunaz=235"; SC="tier=showcase&godrays=0"
HERO="window.__dcs.setCam('orbit',{yaw:5.2,pitch:0.08,dist:8.5})"
run(){ [ -s "$2" ] && { echo "skip $2"; return; }; timeout 900 node $S "$1" "$2" "$3" $W $H "$4" | head -1 | cut -c1-200; }
job(){ case $1 in
 a) run "$SC&$COMMON&cam=orbit" $O/05_showcase_hero.png 110 "$HERO"
    run "$SC&$COMMON&cam=orbit&trees=0" $O/10_trees_before.png 110 "$HERO"
    run "$SC&$COMMON&wet=1&sunel=9&sunaz=200&cam=orbit" $O/07_showcase_wet_dusk.png 110 "window.__dcs.setCam('orbit',{yaw:5.6,pitch:0.1,dist:7})"
    run "tier=high&$COMMON&imp=0&cam=orbit" $O/08_high_no_impostors.png 110 "window.__dcs.setCam('orbit',{yaw:5.2,pitch:0.35,dist:16})" ;;
 b) run "tier=raw&$COMMON" $O/01_raw_chase.png 110
    run "$SC&$COMMON" $O/02_showcase_chase.png 110
    run "tier=raw&$COMMON&cam=orbit" $O/06_raw_hero.png 110 "$HERO"
    run "tier=base&$COMMON" $O/03_base_chase.png 110
    run "tier=high&$COMMON" $O/04_high_chase.png 110
    run "tier=high&$COMMON&cam=orbit" $O/09_high_impostors.png 110 "window.__dcs.setCam('orbit',{yaw:5.2,pitch:0.35,dist:16})" ;;
 esac; }
if [ -n "$1" ]; then job $1; else job a & job b & wait; fi
[ -s $O/01_raw_chase.png ] && [ -s $O/02_showcase_chase.png ] && python3 tools/compare.py $O/01_raw_chase.png $O/02_showcase_chase.png $O/compare_raw_vs_showcase.png
[ -s $O/06_raw_hero.png ] && [ -s $O/05_showcase_hero.png ] && python3 tools/compare.py $O/06_raw_hero.png $O/05_showcase_hero.png $O/compare_hero_raw_vs_showcase.png
[ -s $O/10_trees_before.png ] && [ -s $O/05_showcase_hero.png ] && python3 tools/compare.py $O/10_trees_before.png $O/05_showcase_hero.png $O/compare_trees_before_after.png 'BEFORE: stand-in foliage blocks' 'AFTER: ez-tree meshes + octahedral impostors'
