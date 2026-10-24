# Linux build (core lib, tests, demo). Windows: see build_windows.sh
CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -Wno-unused-parameter -Wno-misleading-indentation -fPIC -Icore/include -Icore/src
LDLIBS = -lm
CORE_SRC = core/src/dc_kin.c core/src/dc_params.c core/src/dc_tyre.c core/src/dc_car.c core/src/dc_road.c core/src/dc_out.c core/src/dc_driver.c
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
build/smoke: tests/smoke.c build/libdrivechain.a
	$(CC) $(CFLAGS) -o $@ $< build/libdrivechain.a $(LDLIBS)
test: build/dc_tests
	./build/dc_tests | tee results/summary.txt
	python3 tests/plot.py results
	$(MAKE) fishtest

cars: build/libdrivechain.a
	$(CC) $(CFLAGS) -o build/gen_cars tools/gen_cars.c build/libdrivechain.a $(LDLIBS) && ./build/gen_cars

clean:
	rm -rf build
.PHONY: all clean test cars

# ---- raylib test drive (Linux). raylib is compiled from source by tools/build_raylib.sh into build-deps/linux
RAYLIB_SRC ?= /workspace/deps/raylib-5.5/src
DEMO_SRC = demo/main.c demo/sim.c demo/track.c demo/audio.c demo/input.c demo/cfg.c demo/objload.c demo/drvproto.c demo/net.c
build-deps/linux/libraylib.a:
	tools/build_raylib.sh $(RAYLIB_SRC) build-deps/linux gcc ar linux
build/drivechain_demo: $(DEMO_SRC) demo/*.h build/libdrivechain.a build-deps/linux/libraylib.a
	$(CC) -O2 -std=gnu11 -Wall -Wno-misleading-indentation -Wno-unused-parameter -Icore/include -Ibuild-deps/linux $$(sdl2-config --cflags) -o $@ $(DEMO_SRC) \
	  build/libdrivechain.a build-deps/linux/libraylib.a $$(sdl2-config --libs) -lGL -lm -lpthread -ldl -lrt -lX11
demo: build/drivechain_demo
.PHONY: demo

# ---- headless closed-loop driver harness (exact demo tick, no SDL / raylib)
DT_SRC = tools/drivetest.c demo/sim.c demo/track.c demo/audio.c demo/input.c demo/cfg.c
build/drivetest: $(DT_SRC) demo/*.h build/libdrivechain.a
	$(CC) -O2 -std=gnu11 -Wall -Wno-misleading-indentation -Wno-unused-parameter -DDEMO_NO_SDL -Icore/include -o $@ $(DT_SRC) build/libdrivechain.a -lm -lpthread
drivetest: build/drivetest
# fishtail-settle regression (gamepad arms layer + orifice back-drive), part of `make test`
fishtest: build/drivetest
	mkdir -p results/fishtail/check && ./build/drivetest fishcheck "$(or $(DC_CAR),cars/mx5_na.ini)" results/fishtail/check | tee results/fishtail/check/fishcheck.txt
.PHONY: drivetest fishtest

# headless lockstep host for the JSON-lines driver protocol (agent / Python / CI driving)
DH_SRC = tools/drvhost.c demo/drvproto.c demo/net.c demo/sim.c demo/track.c demo/audio.c demo/input.c demo/cfg.c
build/drvhost: $(DH_SRC) demo/*.h build/libdrivechain.a
	$(CC) -O2 -std=gnu11 -Wall -Wno-misleading-indentation -Wno-unused-parameter -DDEMO_NO_SDL -Icore/include -o $@ $(DH_SRC) build/libdrivechain.a -lm -lpthread
drvhost: build/drvhost
# driver-interface regression: scripted manoeuvres through the Python client against drvhost
drvtest: build/drvhost
	python3 tools/dcdrive.py selftest --out results/driver
.PHONY: drvhost drvtest
