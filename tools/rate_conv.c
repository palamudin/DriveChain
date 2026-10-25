/* rate_conv.c - mechanics-rate convergence: the host ticks at a fixed outer rate (game logic, input, FFB; default
   240 Hz) and the core integrates at substep_hz (240/480/960/1920 Hz = 1/2/4/8 substeps per tick). Each manoeuvre is
   run at every rate and compared with the finest rate. Inputs are host-side and identical at every rate.
   Usage: rate_conv car.ini [outer_hz]   Build: make rateconv */
#include "drivechain.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + 1e-9 * t.tv_nsec; }
static double OUTER = 240;
/* road: flat, plus (bump manoeuvre) a 40 mm x 0.6 m half-cosine bump across the road at x = 30 m */
static int g_bump = 0;
static double hgt(double x) { if (!g_bump || x < 30 || x > 30.6) return 0; return 0.02 * (1 - cos(2 * M_PI * (x - 30) / 0.6)); }
static double road(void* u, double x, double y, double* nx, double* ny, double* nz, double* grip) {
    (void)u; (void)y; *grip = 1; double h = hgt(x); if (!nx) return h;
    double e = 1e-4, hx = (hgt(x + e) - hgt(x - e)) / (2 * e), l = sqrt(hx * hx + 1); *nx = -hx / l; *ny = 0; *nz = 1 / l; return h;
}
typedef struct { dc_car* c; double vt, ii, L, ratio; int gear; double wall, simt; } run_t;
static void start(run_t* R, const dc_car_params* P0, double hz, double v, int gear) {
    dc_car_params p = *P0; p.substep_hz = hz;
    R->c = dc_create(&p); dc_set_road(R->c, road, NULL); dc_set_road_flags(R->c, DC_ROAD_HEIGHT_ONLY | DC_ROAD_STATIC);
    const double z[3] = {0, 0, 0}; dc_reset(R->c, z, 0, v); R->vt = v; R->ii = 0; R->gear = gear; R->ratio = p.steer_ratio;
    dc_xform xf[DC_PART_COUNT]; dc_get_parts(R->c, xf, DC_PART_COUNT);
    R->L = 0.5 * (xf[DC_PART_WHEEL0].pos[0] + xf[DC_PART_WHEEL0 + 1].pos[0]) - 0.5 * (xf[DC_PART_WHEEL0 + 2].pos[0] + xf[DC_PART_WHEEL0 + 3].pos[0]);
    R->wall = 0; R->simt = 0;
}
/* one outer tick: host speed PI on the throttle (lift: throttle forced to 0), angle-mode hands on the rim */
static void tick(run_t* R, double sw, int lift, dc_telemetry* t) {
    double dt = 1.0 / OUTER; dc_get_telemetry(R->c, t);
    double e = R->vt - t->speed; R->ii += e * dt; if (R->ii > 20) R->ii = 20; if (R->ii < -20) R->ii = -20;
    double th = 0.35 * e + 0.25 * R->ii + 0.12; th = th < 0 ? 0 : (th > 1 ? 1 : th);
    dc_input in; memset(&in, 0, sizeof in);
    in.steer_mode = DC_STEER_ANGLE; in.steer_target = sw; in.hands_k = 80; in.hands_d = 1.2; in.hands_tmax = 30;
    in.throttle = lift ? 0 : th; in.gear = R->gear; dc_set_input(R->c, &in);
    double a = now(); dc_step(R->c, dt); R->wall += now() - a; R->simt += dt;
    dc_get_telemetry(R->c, t);
}
#define NM 12
static const char* NAMES[NM] = {"skidpad ay [m/s2]", "skidpad yaw rate [deg/s]", "step: yaw-rate peak [deg/s]", "step: yaw-rate t90 [ms]",
    "step: yaw rate @3 s [deg/s]", "lift-off: d(yaw) 1 s [deg]", "lift-off: d(yaw rate) 1 s [deg/s]", "bump: peak Fz front [N]",
    "bump: peak Fz rear [N]", "bump: body z range [mm]", "understeer gradient [deg/g]", "WOT 0-8 s: v_end [km/h]"};
/* tolerance per metric: {absolute, relative} - a metric passes if |d| <= abs OR |d|/|ref| <= rel */
static const double TOL[NM][2] = {{0.03, 0.01}, {0.2, 0.01}, {0.3, 0.02}, {8, 0}, {0.2, 0.01}, {0.2, 0.03}, {0.3, 0.05},
    {0, 0.05}, {0, 0.05}, {0.5, 0.05}, {0.1, 0}, {0.3, 0.005}};
static void measure(const dc_car_params* P, double hz, double* m, double* cost) {
    const double D = M_PI / 180; dc_telemetry t; run_t R; double wall = 0, simt = 0;
    /* skidpad: 60 km/h, rim 60 deg, steady over 8-10 s */
    start(&R, P, hz, 60 / 3.6, 3);
    { double ay = 0, r = 0; int n = 0; for (int k = 0; k < 10 * OUTER; k++) { tick(&R, 60 * D, 0, &t); if (k >= 8 * OUTER) { ay += t.ay; r += t.yaw_rate; n++; } } m[0] = fabs(ay / n); m[1] = fabs(r / n) / D; }
    wall += R.wall; simt += R.simt; dc_destroy(R.c);
    /* step steer: 100 km/h, rim 0 -> 40 deg in 0.1 s at t = 1 s */
    start(&R, P, hz, 100 / 3.6, 4);
    { double pk = 0, r3 = 0, rp = 0, tp = 0, t90 = -1; static double rr[2000];
      int N = (int)(4 * OUTER);
      for (int k = 0; k < N; k++) { double tt = (k + 1) / OUTER, sw = tt < 1 ? 0 : (tt < 1.1 ? 40 * D * (tt - 1) / 0.1 : 40 * D); tick(&R, sw, 0, &t); rr[k] = fabs(t.yaw_rate) / D; if (rr[k] > pk) pk = rr[k]; if (fabs(tt - 3) < 0.5 / OUTER) r3 = rr[k]; }
      for (int k = 1; k < N; k++) if (rr[k] >= 0.9 * r3 && rr[k - 1] < 0.9 * r3) { double f = (0.9 * r3 - rr[k - 1]) / (rr[k] - rr[k - 1]); t90 = ((k + f) / OUTER - 1.0) * 1e3; break; }
      (void)rp; (void)tp; m[2] = pk; m[3] = t90; m[4] = r3; }
    wall += R.wall; simt += R.simt; dc_destroy(R.c);
    /* lift-off: 80 km/h, rim 55 deg, steady by 7 s, throttle released at 7 s; yaw change over the next 1 s */
    start(&R, P, hz, 80 / 3.6, 3);
    { double psi = 0, psi0 = 0, r0 = 0, r1 = 0;
      for (int k = 0; k < 8 * OUTER; k++) { int lift = k >= 7 * OUTER; tick(&R, 55 * D, lift, &t); psi += t.yaw_rate / OUTER; if (k == 7 * OUTER - 1) { psi0 = psi; r0 = t.yaw_rate; } r1 = t.yaw_rate; }
      double sg = r0 < 0 ? -1 : 1; m[5] = sg * (psi - psi0 - r0 * 1.0) / D; m[6] = sg * (r1 - r0) / D; }
    wall += R.wall; simt += R.simt; dc_destroy(R.c);
    /* bump: 60 km/h straight over the bump */
    g_bump = 1; start(&R, P, hz, 60 / 3.6, 3);
    { double ff = 0, fr = 0, z0 = 1e9, z1 = -1e9;
      for (int k = 0; k < 4 * OUTER; k++) { tick(&R, 0, 0, &t); dc_xform xf[DC_PART_COUNT]; dc_get_parts(R.c, xf, DC_PART_COUNT);
        ff = fmax(ff, t.w[0].fz); fr = fmax(fr, t.w[2].fz);
        if (xf[0].pos[0] > 25) { if (xf[0].pos[2] < z0) z0 = xf[0].pos[2]; if (xf[0].pos[2] > z1) z1 = xf[0].pos[2]; } }
      m[7] = ff; m[8] = fr; m[9] = (z1 - z0) * 1e3; }
    wall += R.wall; simt += R.simt; dc_destroy(R.c); g_bump = 0;
    /* understeer gradient: 80 km/h, rim ramps 0 -> 90 deg over 18 s; least-squares slope of (front steer - L r/v) vs ay, ay in 1..4 m/s2 */
    start(&R, P, hz, 80 / 3.6, 3);
    { double sx = 0, sy = 0, sxx = 0, sxy = 0; int n = 0;
      for (int k = 0; k < 20 * OUTER; k++) { double tt = (k + 1) / OUTER; tick(&R, 90 * D * fmin(tt / 18, 1), 0, &t);
        double ay = fabs(t.ay); if (ay < 1 || ay > 4 || t.speed < 5) continue;
        double df = 0.5 * fabs(t.w[0].steer + t.w[1].steer), y = df - R.L * fabs(t.yaw_rate) / t.speed;
        sx += ay; sy += y; sxx += ay * ay; sxy += ay * y; n++; }
      double sl = n > 2 ? (n * sxy - sx * sy) / (n * sxx - sx * sx) : 0; m[10] = sl / D * 9.81; }
    wall += R.wall; simt += R.simt; dc_destroy(R.c);
    /* WOT: 0 -> 8 s from 20 km/h in 2nd, then 3rd above 70 km/h (host shift) */
    start(&R, P, hz, 20 / 3.6, 2); R.vt = 1e3;
    for (int k = 0; k < 8 * OUTER; k++) { tick(&R, 0, 0, &t); if (t.speed > 70 / 3.6) R.gear = 3; }
    m[11] = t.speed * 3.6;
    wall += R.wall; simt += R.simt; dc_destroy(R.c);
    *cost = wall / simt * 1e3;   /* ms wall per s sim = us per ms */
}
int main(int argc, char** argv) {
    const char* ini = argc > 1 ? argv[1] : "cars/mx5_na.ini"; if (argc > 2) OUTER = atof(argv[2]);
    dc_car_params P; char err[256]; dc_default_params(&P);
    if (!dc_load_params(ini, &P, err, sizeof err)) { fprintf(stderr, "%s\n", err); return 2; }
    const double HZ[4] = {240, 480, 960, 1920}; double M[4][NM], C[4];
    for (int r = 0; r < 4; r++) { measure(&P, HZ[r], M[r], &C[r]); fprintf(stderr, "  %g Hz done\n", HZ[r]); }
    printf("mechanics-rate convergence, %s, outer tick %g Hz, reference = 1920 Hz\n", ini, OUTER);
    printf("%-34s %12s %12s %12s %12s   tolerance\n", "metric", "240 Hz", "480 Hz", "960 Hz", "1920 Hz");
    int pass[4] = {1, 1, 1, 1};
    for (int k = 0; k < NM; k++) {
        printf("%-34s", NAMES[k]);
        for (int r = 0; r < 4; r++) {
            double d = M[r][k] - M[3][k]; int ok = fabs(d) <= TOL[k][0] || (TOL[k][1] > 0 && fabs(d) <= TOL[k][1] * fabs(M[3][k]));
            if (!ok) pass[r] = 0;
            printf(" %10.3f%s", M[r][k], r == 3 ? "  " : (ok ? " ." : " X"));
        }
        { char tb[64] = ""; int o = 0;
          if (TOL[k][0] > 0) o += snprintf(tb + o, sizeof tb - o, "abs %g", TOL[k][0]);
          if (TOL[k][1] > 0) o += snprintf(tb + o, sizeof tb - o, "%srel %g%%", o ? " or " : "", TOL[k][1] * 100);
          printf("   %s\n", tb); }
    }
    printf("%-34s", "cost [us wall per ms sim, 1 core]"); for (int r = 0; r < 4; r++) printf(" %10.1f  ", C[r]); printf("\n");
    printf("%-34s", "converged (all within tolerance)"); for (int r = 0; r < 4; r++) printf(" %10s  ", r == 3 ? "ref" : (pass[r] ? "PASS" : "fail")); printf("\n");
    int mi = -1; for (int r = 0; r < 3; r++) if (pass[r]) { mi = r; break; }
    if (mi >= 0) printf("minimum converging rate: %g Hz (%g substeps per %g Hz tick), %.1f us per ms of sim\n", HZ[mi], HZ[mi] / OUTER, OUTER, C[mi]);
    else printf("minimum converging rate: none below 1920 Hz\n");
    return 0;
}
