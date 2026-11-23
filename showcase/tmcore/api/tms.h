/* tms.h - renderer-agnostic C API over Jakov's Tarmac Lab native vehicle core (NativeCurrent287).
   Same API is used by the wasm/three.js build and the native Vulkan build.
   Coordinates: Tarmac native frame, SI, right-handed, Y up, Z forward, X driver-left (= three.js frame). */
#ifndef TMS_H
#define TMS_H
#include <stdint.h>
#if defined(__EMSCRIPTEN__)
#include <emscripten/emscripten.h>
#endif
#ifdef __cplusplus
extern "C" {
#endif
#if defined(_WIN32)
#define TMS_API __declspec(dllexport)
#elif defined(__EMSCRIPTEN__)
#define TMS_API EMSCRIPTEN_KEEPALIVE
#else
#define TMS_API __attribute__((visibility("default")))
#endif

/* Surface classes (DriveChain SURF_* ids, also used as glTF material names surf_<name>). */
enum { TMS_SURF_TARMAC=0, TMS_SURF_KERB=1, TMS_SURF_GRASS=2, TMS_SURF_ROUGH=3, TMS_SURF_SAUSAGE=4, TMS_SURF_LINE=5,
       TMS_SURF_PAVE=6, TMS_SURF_PAD=7, TMS_SURF_GRAVEL=8, TMS_SURF_WALL=9, TMS_SURF_OTHER=10, TMS_SURF_CONCRETE=11 };

/* Cars: 0 = the core's own default vehicle, 3 = stock 1990 Mazda MX-5 NA6 (public-spec values, see tms.cpp). */
enum { TMS_CAR_BASELINE=0, TMS_CAR_MIATA_NA=3 };
/* Tune: reserved (only the stock setup is shipped). */
enum { TMS_TUNE_STOCK=0 };

typedef struct TMSInput { double throttle, brake, steer, handbrake, clutch; int32_t gear; int32_t starter; } TMSInput;

/* Packed state for renderers (floats; all in world frame).
   [0..2] pos, [3..6] quat xyzw, [7..9] vel, [10] speed m/s, [11] rpm, [12] gear, [13] running, [14] time s, [15] tick,
   per wheel w (base 16+w*16): [0..2] centre, [3..6] quat, [7] spin angle, [8] omega, [9] compression, [10] load N,
   [11] slip, [12] fx, [13] fy, [14] contact, [15] surface role. Total 80 floats. */
#define TMS_STATE_FLOATS 80

TMS_API const char* tms_version(void);
TMS_API const char* tms_last_error(void);
/* World: triangles in Tarmac frame. surf = per-triangle TMS_SURF_* (NULL -> tarmac). Returns handle or NULL. */
TMS_API void* tms_world_create(const float* positions_xyz, uint32_t vertex_count, const uint32_t* indices, uint32_t triangle_count, const uint8_t* surf);
TMS_API void tms_world_destroy(void* world);
/* Raycast down from (x, y_from, z); returns hit height or -1e9. */
TMS_API double tms_world_height(void* world, double x, double y_from, double z);

TMS_API void* tms_car_create(void* world, int car_id, int tune, double x, double z, double yaw, double y_hint);
TMS_API void tms_car_destroy(void* car);
TMS_API int32_t tms_car_reset(void* car, double x, double z, double yaw, double y_hint);
/* Advance by n fixed 240 Hz ticks with the same input. Returns ticks actually accepted, negative on error. */
TMS_API int32_t tms_car_step(void* car, const TMSInput* in, int32_t ticks);
/* Convenience for wasm (no struct marshalling). */
TMS_API int32_t tms_car_step6(void* car, double throttle, double brake, double steer, double handbrake, double clutch, int32_t gear, int32_t ticks);
TMS_API int32_t tms_car_step7(void* car, double throttle, double brake, double steer, double handbrake, double clutch, int32_t gear, int32_t starter, int32_t ticks);
TMS_API int32_t tms_car_state(void* car, float* out80);
TMS_API double tms_tick_hz(void);
/* Raw62 steering ownership: 1 = driver hands own the rack (keyboard/wheel), 7 = neutral gamepad releases hands. */
TMS_API int32_t tms_car_set_steer_owner(void* car, uint32_t flags);
TMS_API int32_t tms_car_set_assists(void* car, int abs_on, int esp_on);
#ifdef __cplusplus
}
#endif
#endif
