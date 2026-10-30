/* slalomcl.c - closed-loop slalom: the same path-following driver on every model version, response and cost compared.
   Path: the proving-ground slalom (12 cones, 18 m pitch, y = 150), weave y = 150 + A cos(pi (x - x0) / 18) blended in
   over the 30 m before the first cone. Driver: pure pursuit (lookahead 0.5 s, min 6 m), steering-wheel angle through the
   hands, speed-hold PI. Metrics over the cone section: path error rms / max, yaw-rate phase lag and delay behind the
   steering-wheel angle at the weave frequency f0 = v / 36 m (single-bin DFT), peak |ay|, rack-force band powers
   (Hann periodogram, bands 0-5 / 5-15 / 15-40 / 40-120 Hz, dB vs the first version), bit-identity with the first version,
   tick wall time mean / p99 / max.
   Run: ./build/slalomcl [car.ini] label:model:substeps:bricks[:tick_hz] ...   (first version = comparison base)
   env SL_V (km/h, 50), SL_A (m, 1.2), SL_REP (timing repeats, 1)                                              */
#define _GNU_SOURCE
#include "../demo/sim.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
static double clampd(double v, double a, double b) { return v < a ? a : (v > b ? b : v); }
static double yaw_of(const dc_xform* c) { double q0 = c->quat[0], q1 = c->quat[1], q2 = c->quat[2], q3 = c->quat[3]; return atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)); }
static double wrap(double a) { while (a > M_PI) a -= 2 * M_PI; while (a < -M_PI) a += 2 * M_PI; return a; }
static double now_s(void) { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return ts.tv_sec + 1e-9 * ts.tv_nsec; }
static int cmpd(const void* a, const void* b) { double x = *(const double*)a, y = *(const double*)b; return x < y ? -1 : x > y; }

static track_t TRK;
static double X0, XE, AMP, VT;
static double ypath(double x) {
    if (x < X0 - 30) return 150;
    double w = x < X0 ? (x - (X0 - 30)) / 30 : 1; w = w * w * (3 - 2 * w);
    return 150 + w * AMP * cos(M_PI * (x - X0) / 18);
}
enum { NMAX = 960 * 40 };
typedef struct { double x, y, r, sw, ay, rack, wall; } rec_t;
static rec_t REC[16][NMAX]; static int NREC[16];

static int run(const char* car, const char* model, const char* sub, const char* bricks, const char* thz, rec_t* out) {
    static sim_t S; static cfg_t cfg; memset(&S, 0, sizeof S); memset(&cfg, 0, sizeof cfg);
    cfg_set(&cfg, "autopilot.enabled", "0"); cfg_set(&cfg, "car.setup", "default"); cfg_set(&cfg, "physics.model", model);
    cfg_set(&cfg, "physics.tick_hz", thz); cfg_set(&cfg, "physics.substeps", sub); cfg_set(&cfg, "physics.bricks", bricks);
    cfg_set(&cfg, "car.ini", car);
    char err[256]; if (!sim_init(&S, &cfg, &TRK, NULL, err, sizeof err)) { fprintf(stderr, "sim_init: %s\n", err); exit(1); }
    const double WB = 2.265; double ratio = S.P.steer_ratio, lock = S.P.steer_lock_deg * M_PI / 180;
    sim_reset_at(&S, X0 - 70, 150, 0, VT); S.gear = 3; S.ext_steer = 1; S.ext_pedals = 1;
    double dt = S.tick_dt, I = 0.2; int n = 0;
    for (double t = 0; n < NMAX; t += dt) {
        dc_telemetry tl; dc_get_telemetry(S.car, &tl);
        dc_xform ch[DC_PART_COUNT]; dc_get_parts(S.car, ch, DC_PART_COUNT);
        double x = ch[0].pos[0], y = ch[0].pos[1], yaw = yaw_of(&ch[0]);
        if (x > XE + 20) break;
        double ld = fmax(6, 0.5 * tl.vx), gx = x + ld, gy = ypath(gx);
        double eta = wrap(atan2(gy - y, gx - x) - yaw), L = hypot(gx - x, gy - y);
        double sw = clampd(atan(2 * WB * sin(eta) / L) * ratio, -lock, lock);
        double e = VT - tl.vx; I = clampd(I + 0.08 * e * dt, 0, 1);
        double c = 0.25 * e + I;
        dc_input* in = &S.ext_in; memset(in, 0, sizeof *in);
        in->steer_mode = DC_STEER_ANGLE; in->steer_target = sw; in->hands_k = 80; in->hands_d = 1.2; in->hands_tmax = 30;
        in->throttle = clampd(c, 0, 1); in->brake = clampd(-c * 0.5, 0, 1);
        double w0 = now_s(); sim_tick(&S, dt); double w1 = now_s();
        dc_get_telemetry(S.car, &tl); dc_get_parts(S.car, ch, DC_PART_COUNT);
        rec_t* o = &out[n++]; o->x = ch[0].pos[0]; o->y = ch[0].pos[1]; o->r = tl.yaw_rate; o->sw = tl.sw_angle; o->ay = tl.ay; o->rack = tl.rack_force; o->wall = (w1 - w0) * 1e6;
    }
    dc_destroy(S.car);
    return n;
}
static void band_pow(const rec_t* q, int i0, int i1, double fs, double bp[4]) {
    int n = i1 - i0; double m = 0; for (int i = i0; i < i1; i++) m += q[i].rack; m /= n;
    static const double E[5] = {0, 5, 15, 40, 120}; for (int b = 0; b < 4; b++) bp[b] = 0;
    for (int k = 1; k < n / 2 && k * fs / n < E[4]; k++) {
        double f = k * fs / n, re = 0, im = 0;
        for (int i = 0; i < n; i++) { double w = 0.5 - 0.5 * cos(2 * M_PI * i / (n - 1)), v = (q[i0 + i].rack - m) * w; re += v * cos(2 * M_PI * k * i / n); im -= v * sin(2 * M_PI * k * i / n); }
        for (int b = 0; b < 4; b++) if (f >= E[b] && f < E[b + 1]) bp[b] += (re * re + im * im) / ((double)n * n);   /* per-sample normalised: comparable across tick rates */
    }
}
static double phase_at(const rec_t* q, int i0, int i1, double f, double fs, int which) {
    double re = 0, im = 0;
    for (int i = i0; i < i1; i++) { double v = which ? q[i].r : q[i].sw, a = 2 * M_PI * f * i / fs; re += v * cos(a); im -= v * sin(a); }
    return atan2(im, re);
}
int main(int argc, char** argv) {
    const char* car = argc > 1 ? argv[1] : "cars/mx5_na.ini";
    track_build(&TRK, 1.0, TRACK_PROVING);
    X0 = TRK.cone_x[0]; XE = TRK.cone_x[TRK.ncone - 1];
    VT = (getenv("SL_V") ? atof(getenv("SL_V")) : 50) / 3.6; AMP = getenv("SL_A") ? atof(getenv("SL_A")) : 1.2;
    int rep = getenv("SL_REP") ? atoi(getenv("SL_REP")) : 1;
    double f0 = VT / 36;
    printf("slalomcl: %s, v %.0f km/h, A %.2f m, f0 %.3f Hz, closed-loop pure pursuit (same driver every version)\n", car, VT * 3.6, AMP, f0);
    printf("%-22s %8s %8s %8s %8s %7s %26s %-24s %s\n", "version", "pe_rms_m", "pe_max_m", "lag_deg", "delay_ms", "ay_g", "rack dB 0-5/5-15/15-40/40-120", "bitwise_vs_first", "tick us mean/p99/max");
    double bp0[4] = {0};
    for (int v = 2; v < argc && v - 2 < 16; v++) {
        char spec[128]; snprintf(spec, sizeof spec, "%s", argv[v]);
        char* f[5] = {0}; int nf = 0; for (char* t = strtok(spec, ":"); t && nf < 5; t = strtok(NULL, ":")) f[nf++] = t;
        if (nf < 4) { fprintf(stderr, "spec label:model:substeps:bricks[:tick_hz]\n"); return 1; }
        const char* thz = nf > 4 ? f[4] : "240"; double fs = atof(thz); static double fs0; if (v == 2) fs0 = fs;
        rec_t* q = REC[v - 2]; double wsum[NMAX]; int n = 0;
        static double wall_all[NMAX * 8]; int nw = 0;
        for (int r = 0; r < rep; r++) { n = run(car, f[1], f[2], f[3], thz, q); for (int i = 0; i < n; i++) { if (r == 0) wsum[i] = q[i].wall; else wsum[i] = fmin(wsum[i], q[i].wall); } }
        for (int i = 0; i < n; i++) wall_all[nw++] = wsum[i];
        NREC[v - 2] = n;
        int i0 = -1, i1 = n; for (int i = 0; i < n; i++) { if (i0 < 0 && q[i].x >= X0) i0 = i; if (q[i].x <= XE) i1 = i + 1; }
        double se = 0, me = 0, ay = 0; for (int i = i0; i < i1; i++) { double e = q[i].y - ypath(q[i].x); se += e * e; me = fmax(me, fabs(e)); ay = fmax(ay, fabs(q[i].ay)); }
        /* whole number of weave periods for the single-bin DFT */
        int per = (int)lround(fs / f0); int i1p = i0 + ((i1 - i0) / per) * per;
        double lag = wrap(phase_at(q, i0, i1p, f0, fs, 0) - phase_at(q, i0, i1p, f0, fs, 1)) * 180 / M_PI;
        double bp[4]; band_pow(q, i0, i1, fs, bp); if (v == 2) memcpy(bp0, bp, sizeof bp);
        int bw = -1; double dmax = 0; if (fs != fs0) { bw = 0; } const rec_t* b = REC[0]; int nc = n < NREC[0] ? n : NREC[0];
        for (int i = 0; fs == fs0 && i < nc; i++) { if (bw < 0 && (q[i].x != b[i].x || q[i].y != b[i].y || q[i].r != b[i].r || q[i].rack != b[i].rack)) bw = i; dmax = fmax(dmax, hypot(q[i].x - b[i].x, q[i].y - b[i].y)); }
        char bws[64]; if (fs != fs0) snprintf(bws, sizeof bws, "n/a (other tick rate)"); else if (bw < 0 && n == NREC[0]) snprintf(bws, sizeof bws, "IDENTICAL"); else snprintf(bws, sizeof bws, "from tick %d, dpos %.3f m", bw < 0 ? nc : bw, dmax);
        qsort(wall_all, nw, sizeof(double), cmpd); double wm = 0; for (int i = 0; i < nw; i++) wm += wall_all[i]; wm /= nw;
        printf("%-22s %8.3f %8.3f %8.2f %8.1f %7.3f   %+6.2f %+6.2f %+6.2f %+6.2f   %-24s %5.0f/%5.0f/%5.0f  (%3.0f%% of a core)\n", f[0], sqrt(se / (i1 - i0)), me, lag, lag / 360 / f0 * 1000, ay / 9.81,
               10 * log10(bp[0] / bp0[0]), 10 * log10(bp[1] / bp0[1]), 10 * log10(bp[2] / bp0[2]), 10 * log10(bp[3] / bp0[3]), bws, wm, wall_all[(int)(0.99 * (nw - 1))], wall_all[nw - 1], wm * fs / 1e4);
        fflush(stdout);
    }
    return 0;
}
