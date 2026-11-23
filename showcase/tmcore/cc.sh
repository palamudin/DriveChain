#!/bin/bash
# compile one TU of the Tarmac native core against shim headers
D=$(cd "$(dirname "$0")" && pwd); U=${TARMAC_SRC:-$D/upstream}
CXX=${CXX:-g++}
exec $CXX -std=c++17 ${OPT:--O1} -ffp-contract=off -fno-fast-math -pthread -DTARMAC_FRONT_TYRE176 -DTARMAC_TRACTION188 -DTARMAC_TYRE_RESPONSE192 \
 -I$U/WheelPreparation92/native -I$U/FrontTyreObserver176 -I$U/JoypadSpan186 -include $D/shim/port_prelude.h -I$D/shim -I$D/shim/rel/a/b -I$U/TyreResponse192 "$@"
