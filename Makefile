# Linux build (core lib, tests, demo). Windows: see build_windows.sh
CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -Wno-unused-parameter -Wno-misleading-indentation -fPIC -Icore/include -Icore/src
LDLIBS = -lm -lpthread -ldl
CORE_SRC = core/src/dc_kin.c core/src/dc_params.c core/src/dc_tyre.c core/src/dc_tyre_tm.c core/src/dc_car.c core/src/dc_road.c core/src/dc_out.c core/src/dc_driver.c core/src/dc_mb.c core/src/dc_mbcar.c core/src/dc_pool.c core/src/dc_lane.c
CORE_OBJ = $(CORE_SRC:core/src/%.c=build/%.o)

all: build/libdrivechain.a build/libdrivechain.so build/dc_tests

build/%.o: core/src/%.c core/src/*.h core/include/drivechain.h | build
	$(CC) $(CFLAGS) -DDC_BUILD -c $< -o $@
build:
	mkdir -p build
build/libdrivechain.a: $(CORE_OBJ)
	ar rcs $@ $^
build/libdrivechain.so: $(CORE_OBJ)
	$(CC) -shared -o $@ $^ $(LDLIBS)
build/dc_tests: tests/dc_tests.c build/libdrivechain.a
	$(CC) $(CFLAGS) -o $@ $< build/libdrivechain.a $(LDLIBS)
build/drive: tests/drive.c build/libdrivechain.a
	$(CC) $(CFLAGS) -o $@ $< build/libdrivechain.a $(LDLIBS)
build/kerbstrike: tests/kerbstrike.c build/libdrivechain.a
	$(CC) $(CFLAGS) -o $@ $< build/libdrivechain.a $(LDLIBS)
# abuse loads: kerb/wall strikes and 0.5-1.5 m drops on both cores (no blow-up, no energy gain, load bands)
crashtest: build/kerbstrike
	python3 tests/crash_suite.py "$(or $(DC_CAR),cars/mx5_na.ini)" results/crash | tee results/crash/crash_summary.txt
.PHONY: crashtest
build/smoke: tests/smoke.c build/libdrivechain.a
	$(CC) $(CFLAGS) -o $@ $< build/libdrivechain.a $(LDLIBS)
build/mb_tests: tests/mb_tests.c core/src/dc_mb.c core/src/dc_pool.c core/src/dc_mb.h core/src/dc_math.h
	$(CC) $(CFLAGS) -std=gnu11 -o $@ tests/mb_tests.c core/src/dc_mb.c core/src/dc_pool.c $(LDLIBS)
build/mbcar_tests: tests/mbcar_tests.c build/libdrivechain.a
	$(CC) -O2 -std=gnu11 -Icore/include -o $@ $< build/libdrivechain.a -lm -lpthread
build/mbcar_smoke: tests/mbcar_smoke.c build/libdrivechain.a
	$(CC) -O2 -std=gnu11 -Icore/include -o $@ $< build/libdrivechain.a -lm -lpthread
build/contact_point_tests: tests/contact_point_tests.c build/libdrivechain.a
	$(CC) -O2 -std=gnu11 -Icore/include -o $@ $< build/libdrivechain.a -lm -lpthread
contacttest: build/contact_point_tests
	./build/contact_point_tests cars/mx5_na.ini | tee results/contact_point_summary.txt
.PHONY: contacttest
build/reset_tests: tests/reset_tests.c build/libdrivechain.a
	$(CC) -O2 -std=gnu11 -Icore/include -o $@ $< build/libdrivechain.a -lm -lpthread
build/valve_tests: tests/valve_tests.c build/libdrivechain.a
	$(CC) -O2 -std=gnu11 -Icore/include -o $@ $< build/libdrivechain.a -lm -lpthread
valvetest: build/valve_tests
	mkdir -p results/valve && ./build/valve_tests cars/mx5_na.ini cars/hatch_fwd.ini | tee results/valve/valve_summary.txt
	-python3 tools/plot_valve.py results/valve
.PHONY: valvetest
build/rate_conv: tools/rate_conv.c build/libdrivechain.a
	$(CC) -O2 -std=gnu11 -Icore/include -o $@ $< build/libdrivechain.a -lm -lpthread
rateconv: build/rate_conv
	mkdir -p results/rate && ./build/rate_conv cars/mx5_na.ini | tee results/rate/rate_conv_mx5_na.txt && ./build/rate_conv cars/hatch_fwd.ini | tee results/rate/rate_conv_hatch_fwd.txt
.PHONY: rateconv
build/lane_tests: tests/lane_tests.c build/libdrivechain.a
	$(CC) -O2 -std=gnu11 -Icore/include -Icore/src -o $@ $< build/libdrivechain.a -lm -lpthread -ldl
build/thread_tests: tests/thread_tests.c build/libdrivechain.a
	$(CC) -O2 -std=gnu11 -Icore/include -o $@ $< build/libdrivechain.a -lm -lpthread
threadtest: build/thread_tests
	mkdir -p results && ./build/thread_tests cars/mx5_na.ini | tee results/thread_summary.txt
.PHONY: threadtest
build/road_cache_tests: tests/road_cache_tests.c build/libdrivechain.a
	$(CC) -O2 -std=gnu11 -Icore/include -o $@ $< build/libdrivechain.a -lm -lpthread
roadcachetest: build/road_cache_tests
	./build/road_cache_tests cars/mx5_na.ini | tee results/road_cache_summary.txt
.PHONY: roadcachetest
resettest: build/reset_tests
	./build/reset_tests cars/mx5_na.ini cars/hatch_fwd.ini | tee results/reset_summary.txt
.PHONY: resettest
mbcartest: build/mbcar_tests build/mbcar_smoke
	./build/mbcar_tests cars/mx5_na.ini
.PHONY: mbcartest
mbtest: build/mb_tests
	./build/mb_tests | tee results/mb/phase0_summary.txt
.PHONY: mbtest

test: build/dc_tests build/mb_tests build/mbcar_tests build/reset_tests build/contact_point_tests build/valve_tests build/road_cache_tests build/thread_tests
	./build/dc_tests | tee results/summary.txt
	mkdir -p results/mb && ./build/mb_tests | tee results/mb/phase0_summary.txt
	./build/mbcar_tests cars/mx5_na.ini | tee results/mb/phase12_summary.txt
	./build/mbcar_tests cars/hatch_fwd.ini energy | tee results/mb/energy_audit_hatch_fwd.txt
	./build/reset_tests cars/mx5_na.ini cars/hatch_fwd.ini | tee results/reset_summary.txt
	./build/road_cache_tests cars/mx5_na.ini | tee results/road_cache_summary.txt
	./build/thread_tests cars/mx5_na.ini | tee results/thread_summary.txt
	./build/contact_point_tests cars/mx5_na.ini | tee results/contact_point_summary.txt
	$(MAKE) valvetest
	python3 tests/plot.py results
	$(MAKE) crashtest
	$(MAKE) fishtest
	$(MAKE) drvtest
	@if command -v mcs >/dev/null && command -v mono >/dev/null; then $(MAKE) unitytest; else echo 'mono/mcs not found: skipping unitytest'; fi

cars: build/libdrivechain.a
	$(CC) $(CFLAGS) -o build/gen_cars tools/gen_cars.c build/libdrivechain.a $(LDLIBS) && ./build/gen_cars

clean:
	rm -rf build
.PHONY: all clean test cars

# ---- raylib test drive (Linux). raylib is compiled from source by tools/build_raylib.sh into build-deps/linux
RAYLIB_SRC ?= /workspace/deps/raylib-5.5/src
DEMO_SRC = demo/main.c demo/sim.c demo/roadbake.c demo/track.c demo/meshtrack.c demo/audio.c demo/input.c demo/cfg.c demo/objload.c demo/drvproto.c demo/net.c
build-deps/linux/libraylib.a:
	tools/build_raylib.sh $(RAYLIB_SRC) build-deps/linux gcc ar linux
# Built with DC_PROF (core compiled in): the F7 perf overlay reads the per-stage counters.
build/drivechain_demo: $(DEMO_SRC) demo/*.h $(CORE_SRC) core/src/*.h core/include/*.h build-deps/linux/libraylib.a
	$(CC) -O2 -std=gnu11 -Wall -Wno-misleading-indentation -Wno-unused-parameter -DDC_PROF -DDC_BUILD -Icore/include -Icore/src -Ibuild-deps/linux $$(sdl2-config --cflags) -o $@ $(DEMO_SRC) \
	  $(CORE_SRC) build-deps/linux/libraylib.a $$(sdl2-config --libs) -lGL -lm -lpthread -ldl -lrt -lX11
demo: build/drivechain_demo
.PHONY: demo

# ---- headless closed-loop driver harness (exact demo tick, no SDL / raylib)
DT_SRC = tools/drivetest.c demo/sim.c demo/roadbake.c demo/track.c demo/meshtrack.c demo/audio.c demo/input.c demo/cfg.c
build/drivetest: $(DT_SRC) demo/*.h build/libdrivechain.a
	$(CC) -O2 -std=gnu11 -Wall -Wno-misleading-indentation -Wno-unused-parameter -DDEMO_NO_SDL -Icore/include -o $@ $(DT_SRC) build/libdrivechain.a -lm -lpthread
drivetest: build/drivetest
# fishtail-settle regression (gamepad arms layer + orifice back-drive), part of `make test`
fishtest: build/drivetest
	mkdir -p results/fishtail/check && ./build/drivetest fishcheck "$(or $(DC_CAR),cars/mx5_na.ini)" results/fishtail/check | tee results/fishtail/check/fishcheck.txt
.PHONY: drivetest fishtest

# headless lockstep host for the JSON-lines driver protocol (agent / Python / CI driving)
DH_SRC = tools/drvhost.c demo/drvproto.c demo/net.c demo/sim.c demo/roadbake.c demo/track.c demo/meshtrack.c demo/audio.c demo/input.c demo/cfg.c
build/meshdiag: tools/meshdiag.c demo/track.c demo/meshtrack.c demo/cfg.c demo/*.h
	$(CC) -O2 -std=gnu11 -Wall -Wno-misleading-indentation -Wno-unused-parameter -Icore/include -o $@ tools/meshdiag.c demo/track.c demo/meshtrack.c demo/cfg.c -lm

build/drvhost: $(DH_SRC) demo/*.h build/libdrivechain.a
	$(CC) -O2 -std=gnu11 -Wall -Wno-misleading-indentation -Wno-unused-parameter -DDEMO_NO_SDL -Icore/include -o $@ $(DH_SRC) build/libdrivechain.a -lm -lpthread
drvhost: build/drvhost
# driver-interface regression: scripted manoeuvres through the Python client against drvhost
drvtest: build/drvhost
	python3 tools/dcdrive.py selftest --out results/driver
.PHONY: drvhost drvtest

# fast-model fidelity + timing harness (docs/PERF.md). Built with DC_PROF: the solver-floor ensemble and stage costs need it.
FC_SRC = tools/fastcheck.c demo/sim.c demo/roadbake.c demo/track.c demo/meshtrack.c demo/audio.c demo/input.c demo/cfg.c
build/fastcheck: $(FC_SRC) demo/*.h core/src/*.c core/src/*.h core/include/*.h
	@mkdir -p build
	$(CC) -O2 -std=gnu11 -Wall -Wno-misleading-indentation -Wno-unused-parameter -DDEMO_NO_SDL -DDC_PROF -DDC_BUILD -Icore/include -Icore/src -o $@ $(FC_SRC) core/src/*.c -lm -lpthread
fastcheck: build/fastcheck
	mkdir -p results/fastcheck && ./build/fastcheck "$(or $(DC_CAR),cars/mx5_na.ini)" results/fastcheck | tee results/fastcheck/fastcheck.txt
.PHONY: fastcheck

# input record / replay determinism (demo/sim.c F9 recordings): record once, replay on every lane backend
RT_SRC = tools/replaytest.c demo/sim.c demo/roadbake.c demo/track.c demo/meshtrack.c demo/audio.c demo/input.c demo/cfg.c
build/replaytest: $(RT_SRC) demo/*.h core/src/*.c core/src/*.h core/include/*.h
	@mkdir -p build
	$(CC) -O2 -std=gnu11 -Wall -Wno-misleading-indentation -Wno-unused-parameter -DDEMO_NO_SDL -DDC_BUILD -Icore/include -Icore/src -o $@ $(RT_SRC) core/src/*.c -lm -lpthread -ldl
replaytest: build/replaytest
	./build/replaytest "$(or $(DC_CAR),cars/mx5_na.ini)"
.PHONY: replaytest

# closed-loop response harnesses (docs/BRICKS.md): max steady skidpad speed, closed-loop slalom across model versions
CL_SRC = demo/sim.c demo/roadbake.c demo/track.c demo/meshtrack.c demo/audio.c demo/input.c demo/cfg.c
build/skidmax build/slalomcl: build/%: tools/%.c $(CL_SRC) demo/*.h core/src/*.c core/src/*.h core/include/*.h
	@mkdir -p build
	$(CC) -O2 -std=gnu11 -Wall -Wno-misleading-indentation -Wno-unused-parameter -DDEMO_NO_SDL -DDC_BUILD -Icore/include -Icore/src -o $@ $< $(CL_SRC) core/src/*.c -lm -lpthread

# step-cost regression under drive torque (sweep-cap count is machine independent; the wall budget is per box)
build/perf_tests: tests/perf_tests.c demo/drvproto.c demo/net.c demo/sim.c demo/roadbake.c demo/track.c demo/meshtrack.c demo/audio.c demo/input.c demo/cfg.c demo/*.h build/libdrivechain.a
	$(CC) -O2 -std=gnu11 -Wall -Wno-misleading-indentation -Wno-unused-parameter -DDEMO_NO_SDL -Icore/include -o $@ tests/perf_tests.c demo/drvproto.c demo/net.c demo/sim.c demo/roadbake.c demo/track.c demo/meshtrack.c demo/audio.c demo/input.c demo/cfg.c build/libdrivechain.a -lm -lpthread
perftest: build/perf_tests
	mkdir -p results/perf && (./build/perf_tests cars/mx5_na.ini proving && ./build/perf_tests cars/mx5_na.ini circuit) | tee results/perf/perf_summary.txt
.PHONY: perftest

# Unity binding: P/Invoke ABI + driver + TCP bridge test (mono), and stub-UnityEngine compile check of the MonoBehaviours
UNITY_CS = unity/DriveChain.cs unity/DriveChainDriverBridge.cs
unitytest: build/libdrivechain.so
	mcs -nowarn:414 -out:build/abitest.exe $(UNITY_CS) unity/AbiTest.cs
	LD_LIBRARY_PATH=build mono build/abitest.exe
	mcs -nowarn:414 -define:UNITY_5_3_OR_NEWER -t:library -out:build/unitycheck.dll tools/unity_stub/UnityEngine.cs $(UNITY_CS)
	@echo "UNITY COMPILE CHECK PASS"
.PHONY: unitytest

# solver / parameter-set comparison suite through the driver interface (CSV per manoeuvre + summary)
build/mbcompare: tools/mbcompare.c build/libdrivechain.a
	$(CC) $(CFLAGS) -o $@ $< build/libdrivechain.a $(LDLIBS)
mbcompare: build/mbcompare
.PHONY: mbcompare
build/tyre_compare: tests/tyre_compare.c build/libdrivechain.a
	$(CC) $(CFLAGS) -o $@ $< build/libdrivechain.a $(LDLIBS)
# head-to-head tyre rig: envelope (tyre_model 0) vs TARMAC (tyre_model 1)
tyrecompare: build/tyre_compare
	mkdir -p results/tyre_compare && ./build/tyre_compare cars/mx5_na.ini results/tyre_compare
