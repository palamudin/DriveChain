#!/bin/bash
# Cross-build the DriveChain raylib test drive for Windows x64 with mingw-w64 and package dist/drivechain_win64.zip.
# Needs: x86_64-w64-mingw32-gcc, raylib 5.5 source (RAYLIB_SRC), SDL2 mingw dev package (SDL2_DIR).
# The dist zip includes private car assets when present (for the owner's own testing only; never publish it).
set -euo pipefail
cd "$(dirname "$0")/.."
CC=x86_64-w64-mingw32-gcc
RAYLIB_SRC=${RAYLIB_SRC:-/workspace/deps/raylib-5.5/src}
SDL2_DIR=${SDL2_DIR:-/workspace/deps/SDL2-2.30.9/x86_64-w64-mingw32}
OUT=build/win64; mkdir -p $OUT
[ -f build-deps/win64/libraylib.a ] || tools/build_raylib.sh "$RAYLIB_SRC" build-deps/win64 $CC x86_64-w64-mingw32-ar windows
CORE="core/src/dc_kin.c core/src/dc_params.c core/src/dc_tyre.c core/src/dc_car.c core/src/dc_road.c core/src/dc_out.c core/src/dc_driver.c"
DEMO="demo/main.c demo/sim.c demo/track.c demo/audio.c demo/input.c demo/cfg.c demo/objload.c"
$CC -O2 -std=gnu11 -Wall -Wno-misleading-indentation -Wno-unused-parameter -DDC_BUILD \
  -Icore/include -Icore/src -Ibuild-deps/win64 -I"$SDL2_DIR/include/SDL2" -Dmain=SDL_main \
  -o $OUT/drivechain_demo.exe $CORE $DEMO \
  build-deps/win64/libraylib.a -L"$SDL2_DIR/lib" -lmingw32 -lSDL2main -lSDL2 \
  -lopengl32 -lgdi32 -lwinmm -lsetupapi -limm32 -lversion -lole32 -loleaut32 -luuid -lshell32 \
  -static-libgcc -Wl,-Bstatic -lpthread -Wl,-Bdynamic -lm
# also the core as a DLL for Unity (x64 plugin)
$CC -O2 -std=c11 -DDC_BUILD -Icore/include -Icore/src -shared -o $OUT/drivechain.dll $CORE -static-libgcc -Wl,--out-implib,$OUT/libdrivechain.dll.a
echo "built $OUT/drivechain_demo.exe and $OUT/drivechain.dll"
