#!/bin/bash
# Build the Tarmac core + tms C API to WebAssembly (ES module) for the three.js variant.
set -e
D=$(cd "$(dirname "$0")" && pwd); U=$D/upstream
source /workspace/sc-tools/emsdk/emsdk_env.sh >/dev/null 2>&1
OUT=${1:-$D/../web/wasm}; mkdir -p "$OUT" "$D/obj-wasm"
FL="-std=c++17 -O3 -ffp-contract=off -fno-fast-math -fwasm-exceptions -sMEMORY64=1 -DTARMAC_FRONT_TYRE176 -DTARMAC_TRACTION188 -DTARMAC_TYRE_RESPONSE192 \
 -include $D/shim/port_prelude.h -I$U/WheelPreparation92/native -I$U/FrontTyreObserver176 -I$U/JoypadSpan186 -I$D/shim -I$D/shim/rel/a/b -I$U/TyreResponse192 -I$D/api"
pids=()
for n in city_surface vehicle envelope141 wheel_workers92 abi; do em++ $FL -c $U/WheelPreparation92/native/$n.cpp -o $D/obj-wasm/$n.o & pids+=($!); done
em++ $FL -c $D/api/tms.cpp -o $D/obj-wasm/tms.o & pids+=($!)
for p in "${pids[@]}"; do wait $p; done
em++ -O3 -fwasm-exceptions -sMEMORY64=1 $D/obj-wasm/*.o -o "$OUT/tarmac.mjs" \
 -sMODULARIZE=1 -sEXPORT_ES6=1 -sEXPORT_NAME=TarmacCore -sENVIRONMENT=web,worker,node \
 -sALLOW_MEMORY_GROWTH=1 -sINITIAL_MEMORY=268435456 -sMAXIMUM_MEMORY=4294967296 -sSTACK_SIZE=4194304 \
 -sEXPORTED_FUNCTIONS=_malloc,_free,_tms_version,_tms_last_error,_tms_world_create,_tms_world_destroy,_tms_world_height,_tms_car_create,_tms_car_destroy,_tms_car_reset,_tms_car_step6,_tms_car_step7,_tms_car_state,_tms_tick_hz,_tms_car_set_assists,_tms_car_set_steer_owner \
 -sEXPORTED_RUNTIME_METHODS=ccall,cwrap,UTF8ToString,HEAPF32,HEAPU32,HEAPU8
ls -la "$OUT"
