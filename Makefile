# Linux build (core lib, tests, demo). Windows: see build_windows.sh
CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -Wno-unused-parameter -Wno-misleading-indentation -fPIC -Icore/include -Icore/src
LDLIBS = -lm
CORE_SRC = core/src/dc_kin.c core/src/dc_params.c core/src/dc_tyre.c core/src/dc_car.c core/src/dc_road.c core/src/dc_out.c
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
clean:
	rm -rf build
.PHONY: all clean
