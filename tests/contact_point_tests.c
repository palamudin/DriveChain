/* contact_point_tests.c - where the tyre forces act. The friction forces must act at the tread/road interface
   (on the road surface), not inside the ground: a lateral force applied below the road adds a roll moment and
   over-states lateral load transfer. Checks, for every tyre model on both cores:
     - static: contact point height == road height (flat road at z = 0) within 1 mm;
     - steady skidpad: |Fy|-weighted application height within 2 mm of the road, and the load-transfer height
       sum(Fz y_contact) / sum(Fy) (actual contact track) of the envelope within 1.5 cm of TARMAC on the same chassis.
   Usage: contact_point_tests car.ini   Build: make contacttest */
#include "drivechain.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
static int fails = 0;
#define CHECK(cond, ...) do { int ok_ = (cond); printf("  %-4s ", ok_ ? "ok" : "FAIL"); printf(__VA_ARGS__); printf("\n"); if (!ok_) fails++; } while (0)
static void inp(dc_car* c, double steer, double thr) {
    dc_input in; memset(&in, 0, sizeof in); in.steer_mode = DC_STEER_ANGLE; in.steer_target = steer;
    in.hands_k = 25; in.hands_d = 1.5; in.hands_tmax = 15; in.throttle = thr; in.gear = 2; dc_set_input(c, &in);
}
static void rotT(const double q[4], const double v[3], double o[3]) {   /* q^-1 v, q = (w, x, y, z) */
    double w = q[0], x = -q[1], y = -q[2], z = -q[3];
    double tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
    o[0] = v[0] + w * tx + (y * tz - z * ty); o[1] = v[1] + w * ty + (z * tx - x * tz); o[2] = v[2] + w * tz + (x * ty - y * tx);
}
static double one(const char* ini, int solver, int model) {
    dc_car_params p; char err[256]; dc_default_params(&p);
    if (!dc_load_params(ini, &p, err, sizeof err)) { printf("load %s: %s\n", ini, err); fails++; return 0; }
    p.solver = solver; char v[4]; snprintf(v, sizeof v, "%d", model);
    dc_set_param(&p, "front", "tyre_model", v); dc_set_param(&p, "rear", "tyre_model", v);
    printf("%s, %s core, %s\n", ini, solver ? "multibody" : "reduced", model ? "TARMAC" : "envelope");
    dc_car* c = dc_create(&p); double P0[3] = {0, 0, 0}; dc_telemetry t; dc_xform xf[DC_PART_COUNT];
    dc_reset(c, P0, 0, 0); for (int f = 0; f < 180; f++) { inp(c, 0, 0); dc_step(c, 1.0 / 60); }
    dc_get_telemetry(c, &t); double zmax = 0;
    for (int i = 0; i < 4; i++) zmax = fmax(zmax, fabs(t.w[i].contact[2]));
    CHECK(zmax < 1e-3, "static: contact point on the road (max |z| %.4f m)", zmax);
    dc_reset(c, P0, 0, 16.6);
    double lt = 0, za = 0; int n = 0;
    for (int f = 0; f < 900; f++) {
        inp(c, 0.9, 0.35); dc_step(c, 1.0 / 60); if (f <= 600) continue;
        dc_get_telemetry(c, &t); dc_get_parts(c, xf, DC_PART_COUNT);
        double fy = 0, mfz = 0, zc = 0, sF = 0;
        for (int i = 0; i < 4; i++) {
            double d[3] = {t.w[i].contact[0] - xf[0].pos[0], t.w[i].contact[1] - xf[0].pos[1], t.w[i].contact[2] - xf[0].pos[2]}, o[3], F[3];
            rotT(xf[0].quat, d, o); rotT(xf[0].quat, t.w[i].force_world, F);
            fy += F[1]; mfz += F[2] * o[1]; zc += t.w[i].contact[2] * fabs(F[1]); sF += fabs(F[1]);
        }
        lt += -mfz / fy; za += zc / sF; n++;
    }
    lt /= n; za /= n;
    CHECK(fabs(za) < 2e-3, "skidpad: lateral force applied at road height (%.4f m)", za);
    printf("       load-transfer height (actual contact track) %.3f m\n", lt);
    dc_destroy(c); return lt;
}
int main(int argc, char** argv) {
    if (argc < 2) { fprintf(stderr, "usage: contact_point_tests car.ini\n"); return 2; }
    for (int s = 0; s <= 1; s++) {
        double le = one(argv[1], s, 0), lm = one(argv[1], s, 1);
        CHECK(fabs(le - lm) < 0.015, "%s core: envelope LT height %.3f vs TARMAC %.3f m (same chassis)", s ? "multibody" : "reduced", le, lm);
    }
    printf("\nCONTACT POINT TESTS: %s (%d failures)\n", fails ? "FAIL" : "PASS", fails);
    return fails ? 1 : 0;
}
