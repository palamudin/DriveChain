/* thread_tests.c - dc_set_threads(n) must not change results: on a static rough road (30 mm step strips) with steer
   and throttle sweeps, cars stepped with 2, 3 and 4 threads give trajectories bit-identical (memcmp) to the serial
   car, for both cores x both tyre models, with and without DC_ROAD_STATIC, with profiling on (DC_DBG_PROFILE), and
   when the thread count is changed mid-run. Usage: thread_tests car.ini   Build: make threadtest */
#include "drivechain.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
static int fails = 0;
#include <stdint.h>
#define CHECK(cond, ...) do { int ok_ = (cond); printf("  %-4s ", ok_ ? "ok" : "FAIL"); printf(__VA_ARGS__); printf("\n"); if (!ok_) fails++; } while (0)
static double g_lift = 0;   /* host-edited road offset (raised plateau after the edit) */
static double hgt(double x, double y) {
    double h = 0.012 * sin(1.7 * x) * cos(2.3 * y) + 0.004 * sin(9.1 * x + 3.7 * y);
    if (fmod(fabs(x), 6.0) < 0.4) h += 0.03;                  /* 30 mm step strips every 6 m */
    if (x > 12) h += g_lift;
    return h;
}
static double road(void* u, double x, double y, double* nx, double* ny, double* nz, double* grip) {
    (void)u; *grip = 1.0; double h = hgt(x, y);
    if (!nx) return h;
    const double e = 1e-3; double hx = (hgt(x + e, y) - hgt(x - e, y)) / (2 * e), hy = (hgt(x, y + e) - hgt(x, y - e)) / (2 * e);
    double l = sqrt(hx * hx + hy * hy + 1); *nx = -hx / l; *ny = -hy / l; *nz = 1 / l; return h;
}
static void inp(dc_car* c, double steer, double thr) {
    dc_input in; memset(&in, 0, sizeof in);
    in.steer_mode = DC_STEER_ANGLE; in.steer_target = steer; in.hands_k = 25; in.hands_d = 1.5; in.hands_tmax = 15;
    in.throttle = thr; in.gear = 2; dc_set_input(c, &in);
}
enum { NS = 3 + 4 * 4 };
static void take(dc_car* c, double* s) {
    dc_telemetry t; dc_get_telemetry(c, &t); dc_xform xf[DC_PART_COUNT]; dc_get_parts(c, xf, DC_PART_COUNT);
    for (int k = 0; k < 3; k++) s[k] = xf[0].pos[k];
    for (int i = 0; i < 4; i++) { s[3 + 4 * i] = t.w[i].fz; s[4 + 4 * i] = t.w[i].fy; s[5 + 4 * i] = t.w[i].fx; s[6 + 4 * i] = t.w[i].omega; }
}
/* returns the number of frames whose snapshot is not bit-identical to the serial car */
static int run_cmp(const dc_car_params* p, int nthr, int flags, int switch_at) {
    dc_car* a = dc_create(p); dc_car* b = dc_create(p);
    dc_set_road(a, road, NULL); dc_set_road(b, road, NULL); dc_set_road_flags(a, flags); dc_set_road_flags(b, flags);
    if (switch_at < 0) dc_set_threads(b, nthr);
    dc_debug_flags(b, DC_DBG_PROFILE);
    const double P0[3] = {0, 0, 0}; g_lift = 0;
    dc_reset(a, P0, 0.05, 8); dc_reset(b, P0, 0.05, 8);
    int bad = 0; double sa[NS], sb[NS];
    for (int f = 0; f < 180; f++) {
        if (f == switch_at) dc_set_threads(b, nthr);
        double st = f < 40 ? 0 : 0.6 * sin(f * 0.07), th = f < 130 ? 0.6 : 0.0;
        inp(a, st, th); inp(b, st, th); dc_step(a, 1.0 / 60); dc_step(b, 1.0 / 60);
        take(a, sa); take(b, sb);
        if (memcmp(sa, sb, sizeof sa) != 0) bad++;
    }
    dc_destroy(a); dc_destroy(b);
    return bad;
}
int main(int argc, char** argv) {
    const char* ini = argc > 1 ? argv[1] : "cars/mx5_na.ini";
    for (int solver = 0; solver < 2; solver++) for (int tm = 0; tm < 2; tm++) {
        dc_car_params p; char err[256]; dc_default_params(&p);
        if (!dc_load_params(ini, &p, err, sizeof err)) { printf("load %s: %s\n", ini, err); return 2; }
        p.solver = solver; char v[8]; snprintf(v, sizeof v, "%d", tm);
        dc_set_param(&p, "front", "tyre_model", v); dc_set_param(&p, "rear", "tyre_model", v);
        printf("%s, %s core, %s tyre\n", ini, solver ? "multibody" : "reduced", tm ? "TARMAC" : "envelope");
        for (int n = 2; n <= 4; n++) for (int fl = 0; fl < 2; fl++) {
            int bad = run_cmp(&p, n, fl ? DC_ROAD_STATIC : 0, -1);
            CHECK(bad == 0, "%d threads vs 1%s: bit-identical over 180 frames (%d differ)", n, fl ? ", DC_ROAD_STATIC" : "", bad);
        }
        { int bad = run_cmp(&p, 4, DC_ROAD_STATIC, 90); CHECK(bad == 0, "1 -> 4 threads at frame 90: bit-identical (%d differ)", bad); }
    }
    { dc_car_params p; dc_default_params(&p); dc_car* c = dc_create(&p);
      int r5 = dc_set_threads(c, 9), r0 = dc_set_threads(c, 0), r3 = dc_set_threads(c, 3);
      CHECK(r5 == 4 && r0 == 1 && r3 == 3, "dc_set_threads clamps to 1..4 (9->%d, 0->%d, 3->%d)", r5, r0, r3); dc_destroy(c); }
    { dc_car_params p; char err[256]; dc_default_params(&p); dc_load_params(ini, &p, err, sizeof err); p.solver = 1; int ok = 1;
      for (int it = 0; it < 300 && ok; it++) {   /* pool start-up race: dispatch right after creating the helpers */
          dc_car* c = dc_create(&p); dc_set_road(c, road, NULL); dc_set_threads(c, 2 + it % 3);
          dc_step(c, 0.001); if (it % 7 == 0) { dc_set_threads(c, 4); dc_step(c, 0.001); dc_set_threads(c, 2); dc_step(c, 0.001); }
          dc_telemetry t; dc_get_telemetry(c, &t); if (!(t.speed == t.speed)) ok = 0; dc_destroy(c);
      }
      CHECK(ok, "pool stress: 300 create / set_threads / step / destroy cycles with thread-count switches complete"); }
    printf("\nTHREAD TESTS: %s (%d failures)\n", fails ? "FAIL" : "PASS", fails);
    return fails ? 1 : 0;
}
