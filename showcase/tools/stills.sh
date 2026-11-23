#!/bin/bash
# Beauty stills + old-vs-new comparison (headless Chrome, SwiftShader WebGL2). Output: ../stills
cd "$(dirname "$0")/.." ; S=tools/shoot.mjs; O=stills; mkdir -p $O
COMMON="warm=90&auto&still=1&webgl&clean&sunel=17&sunaz=235"
run(){ [ -s "$2" ] && { echo "skip $2"; return; }; timeout 900 node $S "$1" "$2" "$3" 1600 900 "$4" | head -1 | cut -c1-300; }
run "tier=raw&$COMMON"      $O/01_raw_chase.png 110
run "tier=showcase&godrays=0&$COMMON" $O/02_showcase_chase.png 110
run "tier=base&$COMMON"     $O/03_base_chase.png 110
run "tier=high&$COMMON"     $O/04_high_chase.png 110
run "tier=showcase&godrays=0&$COMMON&cam=orbit" $O/05_showcase_hero.png 110 "window.__dcs.setCam('orbit',{yaw:2.3,pitch:0.12,dist:6.5})"
run "tier=raw&$COMMON&cam=orbit"      $O/06_raw_hero.png 110 "window.__dcs.setCam('orbit',{yaw:2.3,pitch:0.12,dist:6.5})"
run "tier=showcase&godrays=0&$COMMON&wet=1&sunel=9&sunaz=200&cam=orbit" $O/07_showcase_wet_dusk.png 110 "window.__dcs.setCam('orbit',{yaw:3.4,pitch:0.08,dist:7.5})"
run "tier=high&$COMMON&imp=0&cam=orbit" $O/08_high_no_impostors.png 110 "window.__dcs.setCam('orbit',{yaw:0.4,pitch:0.35,dist:14})"
run "tier=high&$COMMON&cam=orbit"       $O/09_high_impostors.png 110 "window.__dcs.setCam('orbit',{yaw:0.4,pitch:0.35,dist:14})"
