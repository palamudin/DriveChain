/* road_cache_tests.c - DC_ROAD_STATIC (persistent per-wheel road-node cache) must not change results:
   on a static rough road, a car with the flag gives a trajectory bit-identical to one without it, for every tyre
   model on both cores; and after the host edits the road and calls dc_road_changed(), it still matches a car that
   never cached (control: without the call the stale cache shows, proving the cache is live). Usage: road_cache_tests car.ini   Build: make roadcachetest */
#include "drivechain.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
static int fails = 0;
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
/* edit_at >= 0: frame at which the host raises the plateau (and calls dc_road_changed on cached cars) */
static double run_pair(const dc_car_params* p, int edit_at, int notify) {
    dc_car* a = dc_create(p); dc_car* b = dc_create(p);
    dc_set_road(a, road, NULL); dc_set_road(b, road, NULL); dc_set_road_flags(b, DC_ROAD_STATIC);
    const double P0[3] = {0, 0, 0}; g_lift = 0;
    dc_reset(a, P0, 0.05, 8); dc_reset(b, P0, 0.05, 8);
    double d = 0, sa[NS], sb[NS];
    for (int f = 0; f < 180; f++) {
        if (f == edit_at) { g_lift = 0.02; if (notify) dc_road_changed(b); }
        double st = f < 40 ? 0 : 0.6 * sin(f * 0.07);
        inp(a, st, 0.4); inp(b, st, 0.4); dc_step(a, 1.0 / 60); dc_step(b, 1.0 / 60);
        take(a, sa); take(b, sb);
        for (int k = 0; k < NS; k++) { double e = fabs(sa[k] - sb[k]); if (!(e <= d)) d = isnan(e) ? INFINITY : e; }
    }
    dc_destroy(a); dc_destroy(b); g_lift = 0;
    return d;
}
int main(int argc, char** argv) {
    const char* ini = argc > 1 ? argv[1] : "cars/mx5_na.ini";
    for (int solver = 0; solver < 2; solver++) for (int tm = 0; tm < 2; tm++) {
        dc_car_params p; char err[256]; dc_default_params(&p);
        if (!dc_load_params(ini, &p, err, sizeof err)) { printf("load %s: %s\n", ini, err); return 2; }
        p.solver = solver; char v[8]; snprintf(v, sizeof v, "%d", tm);
        dc_set_param(&p, "front", "tyre_model", v); dc_set_param(&p, "rear", "tyre_model", v);
        const char* nm = tm ? "TARMAC" : "envelope";
        printf("%s, %s core, %s tyre\n", ini, solver ? "multibody" : "reduced", nm);
        CHECK(run_pair(&p, -1, 1) == 0, "static rough road: DC_ROAD_STATIC trajectory bit-identical");
        CHECK(run_pair(&p, 90, 1) == 0, "road edited mid-run + dc_road_changed(): still bit-identical");
        { double d = run_pair(&p, 90, 0); CHECK(d > 0, "control: edit without dc_road_changed() is visible (cache active), max diff %.3g", d); }
    }
    printf("\nROAD CACHE TESTS: %s (%d failures)\n", fails ? "FAIL" : "PASS", fails);
    return fails ? 1 : 0;
}
