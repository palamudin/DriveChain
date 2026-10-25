/* reset_tests.c - dc_reset must fully re-initialise the car: after any history (sliding, locked brakes, full lock,
   power-on slides), a reset to pose P must give a trajectory bit-identical to a freshly created car reset to P.
   Covers every tyre model (envelope, TARMAC, TARMAC brush shape, TARMAC LuGre damping) on both cores.
   Also checks that right after a moving reset the tyre's reported transient slip agrees with the rigid-body
   (kinematic) slip of the contact point once the relaxation length has been rolled out.
   Usage: reset_tests car.ini [more.ini ...]   Build: make resettest */
#include "drivechain.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
static int fails = 0;
#define CHECK(cond, ...) do { int ok_ = (cond); printf("  %-4s ", ok_ ? "ok" : "FAIL"); printf(__VA_ARGS__); printf("\n"); if (!ok_) fails++; } while (0)

typedef struct { double x[3], q[4], v[3], sw, fy[4], fz[4], sa[4], omega[4]; } snap;
static void take(dc_car* c, snap* s) {
    dc_telemetry t; dc_get_telemetry(c, &t); dc_xform xf[DC_PART_COUNT]; dc_get_parts(c, xf, DC_PART_COUNT);
    memset(s, 0, sizeof *s);
    for (int k = 0; k < 3; k++) s->x[k] = xf[0].pos[k];
    for (int k = 0; k < 4; k++) s->q[k] = xf[0].quat[k];
    s->v[0] = t.speed; s->v[1] = t.yaw_rate; s->v[2] = t.ay; s->sw = t.sw_angle;
    for (int i = 0; i < 4; i++) { s->fy[i] = t.w[i].fy; s->fz[i] = t.w[i].fz; s->sa[i] = t.w[i].slip_angle; s->omega[i] = t.w[i].omega; }
}
static void inp(dc_car* c, double steer, double thr, double brk, double hb, int gear) {
    dc_input in; memset(&in, 0, sizeof in);
    in.steer_mode = DC_STEER_ANGLE; in.steer_target = steer; in.hands_k = 25; in.hands_d = 1.5; in.hands_tmax = 15;
    in.throttle = thr; in.brake = brk; in.handbrake = hb; in.gear = gear; dc_set_input(c, &in);
}
static void run(dc_car* c, double T, double dt) { for (double t = 0; t < T - 1e-9; t += dt) dc_step(c, dt); }
/* the scripted test after the reset: 1 s straight, then a steer step with part throttle in 2nd */
static int script(dc_car* c, snap* out, int n) {
    const double dt = 1.0 / 60; int k = 0;
    for (int f = 0; f < 240 && k < n; f++) {
        double t = f * dt;
        if (t < 1) inp(c, 0, 0.2, 0, 0, 2); else inp(c, 1.2, 0.6, 0, 0, 2);
        dc_step(c, dt);
        if (f % 4 == 3) take(c, &out[k++]);
    }
    return k;
}
static double maxdiff(const snap* a, const snap* b, int n) {
    double d = 0; const double* pa = (const double*)a; const double* pb = (const double*)b; int m = n * (int)(sizeof(snap) / sizeof(double));
    for (int i = 0; i < m; i++) { double e = fabs(pa[i] - pb[i]); if (!(e <= d)) d = isnan(e) ? INFINITY : e; }
    return d;
}

static void one(const char* ini, int solver, int tmodel, int shape, int damp) {
    dc_car_params p; char err[256]; dc_default_params(&p);
    if (!dc_load_params(ini, &p, err, sizeof err)) { printf("load %s: %s\n", ini, err); fails++; return; }
    p.solver = solver;
    char v[16];
    snprintf(v, sizeof v, "%d", tmodel); dc_set_param(&p, "front", "tyre_model", v); dc_set_param(&p, "rear", "tyre_model", v);
    snprintf(v, sizeof v, "%d", shape); dc_set_param(&p, "front", "tyre_tm_shape", v); dc_set_param(&p, "rear", "tyre_tm_shape", v);
    snprintf(v, sizeof v, "%d", damp); dc_set_param(&p, "front", "tyre_tm_damp_mode", v); dc_set_param(&p, "rear", "tyre_tm_damp_mode", v);
    const char* name = tmodel == 0 ? "envelope" : shape ? (damp ? "TARMAC brush+LuGre" : "TARMAC brush") : "TARMAC";
    printf("%s, %s core, %s\n", ini, solver ? "multibody" : "reduced", name);
    enum { N = 64 };
    static snap A[N], B[N];
    const double P0[3] = {0, 0, 0}, v0 = 20 / 3.6;
    /* A: fresh car */
    dc_car* a = dc_create(&p); dc_reset(a, P0, 0.3, v0); int na = script(a, A, N);
    /* B: abuse first (power slide at full lock, handbrake, locked brakes, pressure change), then reset to the same pose */
    dc_car* b = dc_create(&p); dc_reset(b, P0, 1.0, 15);
    inp(b, 7.0, 1.0, 0, 0, 2); run(b, 2.0, 1.0 / 60);
    inp(b, -7.0, 1.0, 0, 1, 2); run(b, 1.5, 1.0 / 60);
    inp(b, 0, 0, 1, 0, 2); run(b, 1.0, 1.0 / 60);
    double far[3] = {150, -80, 0}; dc_reset(b, far, -2.0, 30);    /* a mid-air-ish teleport with speed, then drive */
    inp(b, 3.0, 1.0, 0, 0, 3); run(b, 1.5, 1.0 / 60);
    dc_reset(b, P0, 0.3, v0); int nb = script(b, B, N);
    double d = na == nb ? maxdiff(A, B, na) : INFINITY;
    CHECK(d == 0, "reset after abuse is bit-identical to a fresh car (%d samples, max |diff| %.3g)", na, d);
    /* reported transient slip vs kinematic slip after the reset, rear axle, during the steer step */
    dc_telemetry t; dc_get_telemetry(a, &t);
    double sa = 0.5 * (t.w[2].slip_angle + t.w[3].slip_angle), ss = 0.5 * (t.w[2].slip_angle_ss + t.w[3].slip_angle_ss);
    CHECK(fabs(sa - ss) < 0.15 * fabs(ss) + 0.002, "rear reported slip %.2f deg vs kinematic %.2f deg (steady cornering after reset)", sa * 57.2958, ss * 57.2958);
    /* in a slide the reported transient slip must still follow the kinematic slip (it must not saturate at the
       carcass-deflection limit mu Fz / C_alpha): reset at 60 km/h, handbrake + lock, sample every wheel with slip > 8 deg */
    dc_reset(a, P0, 0.3, 60 / 3.6); inp(a, 0, 0, 0, 0, 3); run(a, 0.5, 1.0 / 60);
    double num = 0, den = 0; int ns = 0;
    for (int f = 0; f < 90; f++) {
        inp(a, 4.0, 0, 0, 1, 3); dc_step(a, 1.0 / 60); dc_get_telemetry(a, &t);
        for (int w = 0; w < 4; w++) if (fabs(t.w[w].slip_angle_ss) > 8 / 57.2958 && t.w[w].fz > 300) {
            num += t.w[w].slip_angle * t.w[w].slip_angle_ss; den += t.w[w].slip_angle_ss * t.w[w].slip_angle_ss; ns++; }
    }
    double gain = den > 0 ? num / den : 0;
    CHECK(ns >= 10 && gain > 0.8 && gain < 1.1, "sliding: reported/kinematic slip gain %.2f over %d samples > 8 deg", gain, ns);
    dc_destroy(a); dc_destroy(b);
}
int main(int argc, char** argv) {
    if (argc < 2) { fprintf(stderr, "usage: reset_tests car.ini [...]\n"); return 2; }
    for (int k = 1; k < argc; k++)
        for (int s = 0; s <= 1; s++) { one(argv[k], s, 0, 0, 0); one(argv[k], s, 1, 0, 0); one(argv[k], s, 1, 1, 0); one(argv[k], s, 1, 1, 1); }
    printf("\nRESET TESTS: %s (%d failures)\n", fails ? "FAIL" : "PASS", fails);
    return fails ? 1 : 0;
}
