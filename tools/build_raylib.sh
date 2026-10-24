#!/bin/sh
# build_raylib.sh <raylib-src-dir> <out-dir> <cc> <ar> [linux|windows]
# Compiles raylib (desktop / GLFW / OpenGL 3.3) straight from its sources into <out-dir>/libraylib.a.
set -e
SRC=$1; OUT=$2; CC=$3; AR=$4; OSN=${5:-linux}
mkdir -p "$OUT"
DEF="-DPLATFORM_DESKTOP -DGRAPHICS_API_OPENGL_33 -DSUPPORT_FILEFORMAT_OBJ"
[ "$OSN" = linux ] && DEF="$DEF -D_GLFW_X11 -D_GNU_SOURCE"
for f in rcore rshapes rtextures rtext rmodels raudio utils rglfw; do
  $CC -O2 -std=gnu99 $DEF -I"$SRC" -I"$SRC/external/glfw/include" -Wno-everything -w -c "$SRC/$f.c" -o "$OUT/$f.o" &
done
wait
$AR rcs "$OUT/libraylib.a" "$OUT"/r*.o "$OUT"/utils.o
cp "$SRC/raylib.h" "$SRC/raymath.h" "$SRC/rlgl.h" "$OUT/"
echo "built $OUT/libraylib.a"
