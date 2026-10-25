/* valve_tests.c - rotary-valve pressure-state model (mb.valve_model 1/2), docs/VALVE.md.
   1. bench (dc_valve_solve, rated flow, rack held): model 1 (contoured edges) reproduces the boost law
      dP = pmax x^4/(1+x^4), x = T/assist_tref, to 3 % of pmax; model 2 (linear chamfer) is monotone;
      the relief valve caps supply pressure; backward-Euler Newton converges everywhere;
   2. in-car parking at idle, braked (assist_rpm_full = 800): rim torque needed for 30/60 deg of steering wheel,
      model 1 within 15 % of model 0 (model 2, the emergent chamfer, is reported only);
   3. in-car 25 m/s rim-torque step (2.5 N m, 1-2 s, then hands off to 3.5 s) sampled at 0.5 ms: no Newton failures,
      per-step dissipation >= 0; the rack/pressure ringing (~35-40 Hz, set off by rack-friction break-away and the
      release) stays bounded: worst 100 ms high-pass rack-force rms < 120 N (default hose 2e-12: MX-5 56 N, hatch 99 N; model 0 ~31 N;
      hose 1e-12: 146-166 N, supply oil only: 300-350 N, both fail) and
      dies out (last 250 ms < 5 N);
   4. determinism: the step repeated after dc_reset is bit-identical.
   CSV: results/valve/<car>_v<model>_{park,step}.csv ; plots: tools/plot_valve.py. Build: make valvetest */
#include "drivechain.h"
#include "../core/src/dc_mbcar.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
static int fails = 0;
#define CHECK(cond, ...) do { int ok_ = (cond); printf("  %-4s ", ok_ ? "ok" : "FAIL"); printf(__VA_ARGS__); printf("\n"); if (!ok_) fails++; } while (0)
static double road(void* u, double x, double y, double* nx, double* ny, double* nz, double* g) { (void)u; (void)x; (void)y; *nx = 0; *ny = 0; *nz = 1; *g = 1; return 0; }

static void bench(void) {
    printf("--- bench (rack held, rated flow)\n");
    dc_valve v; memset(&v, 0, sizeof v);
    v.a0 = 4.4e-6; v.amin = 2e-8; v.thc = 0.05; v.cd = 0.6; v.rho = 860; v.beta = 3e8; v.Vc = 5e-5; v.Vs = 1e-4;
    v.Cs = 1e-4 / 3e8 + 2e-12; v.Ap = 8.8e-4; v.half = 0.075; v.prel = 7e6; v.Qr = 1e-4; v.grel = 1e-4 / (0.05 * 7e6);
    v.ktb = 115; v.tref = 3.5; v.pmax = 7e6; v.ptr = 1e4;
    for (int e = 0; e < 2; e++) {
        v.edge = e; double z[3] = {0, 0, 0}, P[3], d[3], err = 0, prev = -1e30, psmax = 0; int mono = 1, itmax = 0;
        for (double T = 0; T <= 8.001; T += 0.25) {
            int it = dc_valve_solve(&v, z, T / v.ktb, 0, 0, v.Qr, 0, P, d); itmax = it > itmax ? it : itmax;
            double x = T / v.tref, x4 = x * x * x * x, dp = P[0] - P[1];
            if (T <= 6.001) err = fmax(err, fabs(dp - v.pmax * x4 / (1 + x4)));
            if (dp < prev - 1) mono = 0;
            prev = dp; psmax = fmax(psmax, P[2]);
        }
        if (e == 0) CHECK(err < 0.03 * v.pmax, "model 1: steady dP = boost law to %.3f MPa (limit %.2f), T 0..6 N m", err / 1e6, 0.03 * v.pmax / 1e6);
        else CHECK(mono, "model 2: dP monotone in twist (dP at 8 N m %.2f MPa)", prev / 1e6);
        CHECK(psmax < 1.05 * v.prel, "model %d: relief caps supply at %.2f MPa (setting %.1f)", e + 1, psmax / 1e6, v.prel / 1e6);
        CHECK(itmax > 0 && itmax < 40, "model %d: Newton converged, max %d iterations", e + 1, itmax);
    }
}

static dc_car* mk(const char* ini, int model, double v0) {
    dc_car_params P; char err[256]; dc_default_params(&P);
    if (!dc_load_params(ini, &P, err, sizeof err)) { printf("load %s: %s\n", ini, err); exit(1); }
    P.solver = DC_SOLVER_MULTIBODY; P.mb.valve_model = model; P.assist_rpm_full = 800;
    if (getenv("VALVE_HC")) P.mb.hose_compliance = atof(getenv("VALVE_HC"));   /* sensitivity check: 1e-12 must FAIL the oscillation check on the hatch */
    dc_car* C = dc_create(&P); dc_set_road(C, road, NULL); double pos[3] = {0, 0, 0}; dc_reset(C, pos, 0, v0);
    return C;
}
static const char* base(const char* ini) { const char* b = strrchr(ini, '/'); return b ? b + 1 : ini; }

static double park(const char* ini, int model, double sw_at[3]) {
    sw_at[0] = sw_at[1] = sw_at[2] = -1; dc_car* C = mk(ini, model, 0); char fn[512]; snprintf(fn, sizeof fn, "results/valve/%.*s_v%d_park.csv", (int)(strlen(base(ini)) - 4), base(ini), model);
    FILE* f = fopen(fn, "w"); if (f) fprintf(f, "t,tq,sw_deg,rack_force,pA,pB,pS\n");
    dc_input in; memset(&in, 0, sizeof in); in.steer_mode = DC_STEER_TORQUE; in.brake = 1;
    dc_telemetry T; double at90 = -1, dt = 1e-3;
    for (int i = 0; i < 7000; i++) {
        double t = i * dt; in.steer_torque = t < 1 ? 0 : fmin(6, t - 1); dc_set_input(C, &in); dc_step(C, dt); dc_get_telemetry(C, &T);
        if (at90 < 0 && T.sw_angle > M_PI / 2) at90 = in.steer_torque;
        if (sw_at[0] < 0 && T.sw_angle > M_PI / 6) sw_at[0] = in.steer_torque;
        if (sw_at[1] < 0 && T.sw_angle > M_PI / 3) sw_at[1] = in.steer_torque;
        if (f && i % 10 == 0) fprintf(f, "%.3f,%.3f,%.3f,%.1f,%.0f,%.0f,%.0f\n", t, in.steer_torque, T.sw_angle * 180 / M_PI, T.rack_force, T.valve_p[0], T.valve_p[1], T.valve_p[2]);
    }
    if (f) fclose(f); dc_destroy(C); return at90;
}

typedef struct { double hf_peak, hf_tail, dmin, nfail, sw_end, fin[6]; } stepres;
#define NSTEP 7000
static double hf_rms(const double* F, int a, int b) {   /* rms of F minus its 20 ms moving average over [a, b) */
    double s = 0; int n = 0;
    for (int k = a; k < b; k++) { double m = 0; for (int j = -20; j <= 20; j++) m += F[k + j]; m /= 41; s += (F[k] - m) * (F[k] - m); n++; }
    return n ? sqrt(s / n) : 0;
}
static stepres step_run(dc_car* C, FILE* f) {
    stepres r; memset(&r, 0, sizeof r); r.dmin = 1e30;
    dc_input in; memset(&in, 0, sizeof in); in.steer_mode = DC_STEER_TORQUE; in.gear = 4; in.throttle = 0.25;
    dc_telemetry T; double dt = 5e-4, dprev = 0; static double F[NSTEP];
    for (int i = 0; i < NSTEP; i++) {
        double t = i * dt; in.steer_torque = (t > 1 && t < 2) ? 2.5 : 0; dc_set_input(C, &in); dc_step(C, dt); dc_get_telemetry(C, &T);
        if (i > 0) r.dmin = fmin(r.dmin, T.dissipated - dprev);
        dprev = T.dissipated;
        F[i] = T.rack_force;
        if (f && i % 2 == 0) fprintf(f, "%.4f,%.2f,%.4f,%.2f,%.0f,%.0f,%.0f,%.5f,%.4f\n", t, in.steer_torque, T.sw_angle * 180 / M_PI, T.rack_force, T.valve_p[0], T.valve_p[1], T.valve_p[2], T.rack_vel, T.column_torque);
    }
    r.hf_peak = 0;   /* 100 ms windows from 1 s, skipping the input steps themselves (t = 1.0, 2.0 +- 10 ms) */
    for (int a = 2020; a + 200 < NSTEP - 20; a += 100) {
        int b = a + 200; if ((a < 4000 && b > 3980)) continue;
        r.hf_peak = fmax(r.hf_peak, hf_rms(F, a, b));
    }
    r.hf_tail = hf_rms(F, NSTEP - 520, NSTEP - 20); r.nfail = T.valve_nfail; r.sw_end = T.sw_angle;
    r.fin[0] = T.sw_angle; r.fin[1] = T.rack_pos; r.fin[2] = T.yaw_rate; r.fin[3] = T.valve_p[0]; r.fin[4] = T.valve_p[2]; r.fin[5] = T.dissipated;
    return r;
}

int main(int argc, char** argv) {
    printf("=== valve pressure-state model ===\n");
    bench();
    for (int a = 1; a < argc; a++) {
        const char* ini = argv[a]; printf("--- %s (assist_rpm_full 800)\n", ini);
        double a0[3], a1[3], a2[3]; double p0 = park(ini, 0, a0), p1 = park(ini, 1, a1), p2 = park(ini, 2, a2); int ok = 1;
        for (int k = 0; k < 2; k++) if (a0[k] <= 0 || a1[k] <= 0 || fabs(a1[k] - a0[k]) > 0.15 * a0[k]) ok = 0;
        CHECK(ok, "parking at idle: rim torque for 30/60 deg  model0 %.2f/%.2f  model1 %.2f/%.2f  (model2 %.2f/%.2f) N m",
              a0[0], a0[1], a1[0], a1[1], a2[0], a2[1]);
        printf("  info rim torque for 90 deg: model0 %.2f model1 %.2f model2 %.2f N m (-1 = not reached by 6 N m)\n", p0, p1, p2);
        for (int m = 1; m <= 2; m++) {
            char fn[512]; snprintf(fn, sizeof fn, "results/valve/%.*s_v%d_step.csv", (int)(strlen(base(ini)) - 4), base(ini), m);
            FILE* f = fopen(fn, "w"); if (f) fprintf(f, "t,tq,sw_deg,rack_force,pA,pB,pS,rack_vel,col_tq\n");
            dc_car* C = mk(ini, m, 25); stepres r = step_run(C, f); if (f) fclose(f);
            CHECK(r.nfail == 0, "model %d step: valve Newton failures %.0f", m, r.nfail);
            CHECK(r.dmin > -1e-3, "model %d step: per-step dissipation >= 0 (min %.2e J)", m, r.dmin);
            CHECK(r.hf_peak < 120 && r.hf_tail < 5, "model %d step: rack/pressure ringing bounded and dying out (worst 100 ms HF rack-force rms %.1f N, last 250 ms %.2f N)", m, r.hf_peak, r.hf_tail);
            if (m == 1) {
                double pos[3] = {0, 0, 0}; dc_reset(C, pos, 0, 25); stepres r2 = step_run(C, NULL);
                CHECK(memcmp(r.fin, r2.fin, sizeof r.fin) == 0 && r.hf_peak == r2.hf_peak, "model 1: repeat after dc_reset is bit-identical");
            }
            dc_destroy(C);
        }
        { dc_car* C = mk(ini, 0, 25); FILE* f = NULL; char fn[512];
          snprintf(fn, sizeof fn, "results/valve/%.*s_v0_step.csv", (int)(strlen(base(ini)) - 4), base(ini)); f = fopen(fn, "w");
          if (f) fprintf(f, "t,tq,sw_deg,rack_force,pA,pB,pS,rack_vel,col_tq\n");
          stepres r = step_run(C, f); if (f) fclose(f); printf("  info model 0 step: worst 100 ms HF rack-force rms %.1f N, last 250 ms %.2f N\n", r.hf_peak, r.hf_tail); dc_destroy(C); }
    }
    printf("%s (%d failed)\n", fails ? "VALVE TESTS FAILED" : "all valve tests passed", fails);
    return fails ? 1 : 0;
}
