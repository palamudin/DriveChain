/* skidmax.c - maximum steady speed on the proving-ground skidpad (R 40 m circle, lanes R 38 / 42), closed loop.
   Constant-radius method (ISO 4138 style): a path-following driver (pure pursuit + radius-error integral, steering-wheel
   angle through the hands) holds the circle while a speed-hold PI ramps the target speed slowly. Reports per 1 km/h bin
   the mean lateral acceleration, radius error, steering-wheel angle and body slip; the max steady speed is the highest
   bin whose 2 s window stays inside the lanes (|R - 40| < 2 m) with the target speed held (within 1 km/h).
   Build: make build/skidmax   Run: ./build/skidmax [car.ini] [setup default|legacy] [model fast|reference] [csv]
   env SK_V0 (km/h, 35), SK_RAMP (km/h per s, 0.25), SK_DIR (+1 = counter-clockwise, -1 = clockwise) */
#define _GNU_SOURCE
#include "../demo/sim.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
static double clampd(double v, double a, double b) { return v < a ? a : (v > b ? b : v); }
static double yaw_of(const dc_xform* c) { double q0 = c->quat[0], q1 = c->quat[1], q2 = c->quat[2], q3 = c->quat[3]; return atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)); }
static double wrap(double a) { while (a > M_PI) a -= 2 * M_PI; while (a < -M_PI) a += 2 * M_PI; return a; }

int main(int argc, char** argv) {
    const char* car = argc > 1 ? argv[1] : "cars/mx5_na.ini";
    const char* setup = argc > 2 ? argv[2] : "default";
    const char* model = argc > 3 ? argv[3] : "fast";
    FILE* csv = argc > 4 ? fopen(argv[4], "w") : NULL;
    double v0 = getenv("SK_V0") ? atof(getenv("SK_V0")) : 35, ramp = getenv("SK_RAMP") ? atof(getenv("SK_RAMP")) : 0.25;
    double dir = getenv("SK_DIR") ? atof(getenv("SK_DIR")) : 1;
    static track_t TRK; track_build(&TRK, 1.0, TRACK_PROVING);
    static sim_t S; static cfg_t cfg;
    cfg_set(&cfg, "autopilot.enabled", "0"); cfg_set(&cfg, "car.setup", setup); cfg_set(&cfg, "physics.model", model);
    cfg_set(&cfg, "physics.tick_hz", "240"); cfg_set(&cfg, "physics.substeps", "4"); cfg_set(&cfg, "car.ini", car);
    { const char* cs = getenv("CFGSET");
      if (cs && *cs) { char buf[1024]; snprintf(buf, sizeof buf, "%s", cs);
          for (char* tok = strtok(buf, ";"); tok; tok = strtok(NULL, ";")) { char* eq = strchr(tok, '='); if (eq) { *eq = 0; cfg_set(&cfg, tok, eq + 1); } } } }
    char err[256];
    if (!sim_init(&S, &cfg, &TRK, NULL, err, sizeof err)) { fprintf(stderr, "sim_init: %s\n", err); return 1; }
    const double cx = 170, cy = 150, R = 40, WB = 2.265, LD = 10;
    double ratio = S.P.steer_ratio, lock = S.P.steer_lock_deg * M_PI / 180;
    sim_reset_at(&S, cx, cy - dir * R, 0, v0 / 3.6); S.gear = 3;
    S.ext_steer = 1; S.ext_pedals = 1;
    double dt = S.tick_dt, I = 0.2, Ir = 0, t = 0;
    enum { NB = 200 }; double bs[NB][6]; int bn[NB]; memset(bs, 0, sizeof bs); memset(bn, 0, sizeof bn);
    /* 2 s window ring for the steadiness test */
    enum { W = 480 }; double wr[W], wv[W], wa[W]; int wi = 0, wn = 0; double rr_ok = R, vmax_ok = 0, ay_ok = 0, sw_ok = 0, beta_ok = 0;
    if (csv) fprintf(csv, "t,v_target,v,ay,yaw_rate,r_err,sw_deg,beta_deg,gear\n");
    double tend = (130 - v0) / ramp + 4;
    for (; t < tend; t += dt) {
        dc_telemetry tl; dc_get_telemetry(S.car, &tl);
        dc_xform ch[DC_PART_COUNT]; dc_get_parts(S.car, ch, DC_PART_COUNT);
        double x = ch[0].pos[0], y = ch[0].pos[1], yaw = yaw_of(&ch[0]);
        double rr = hypot(x - cx, y - cy), rerr = rr - R, th = atan2(y - cy, x - cx);
        /* pure pursuit: goal point LD ahead along the circle */
        double tg = th + dir * LD / R, gx = cx + R * cos(tg), gy = cy + R * sin(tg);
        double eta = wrap(atan2(gy - y, gx - x) - yaw), ld = hypot(gx - x, gy - y);
        Ir = clampd(Ir + dir * rerr * 0.01 * dt, -0.15, 0.15);   /* radius-error integral (rad road wheel; outside -> steer more into the turn) */
        double kap = 2 * sin(eta) / ld, dl = atan(WB * kap) + dir * 0.01 * rerr + Ir;
        double sw = clampd(dl * ratio, -lock, lock);
        double vt = (v0 + ramp * fmax(t - 2, 0)) / 3.6;
        double e = vt - tl.vx; I = clampd(I + 0.08 * e * dt, 0, 1);
        double c = 0.25 * e + I, thr = clampd(c, 0, 1), brk = clampd(-c * 0.5, 0, 1);
        int g = tl.gear; double rpm = tl.engine_rpm;
        if (rpm > 6500 && g < 5) S.gear = g + 1; else if (rpm < 2500 && g > 2) S.gear = g - 1;
        dc_input* in = &S.ext_in; memset(in, 0, sizeof *in);
        in->steer_mode = DC_STEER_ANGLE; in->steer_target = sw; in->hands_k = 80; in->hands_d = 1.2; in->hands_tmax = 30;
        in->throttle = thr; in->brake = brk;
        sim_tick(&S, dt);
        dc_get_telemetry(S.car, &tl);
        double beta = atan2(tl.vy, fabs(tl.vx) + 1e-6) * 180 / M_PI, v = tl.speed;
        if (csv && ((long)(t / dt) % 24 == 0)) fprintf(csv, "%.3f,%.2f,%.2f,%.3f,%.4f,%.3f,%.1f,%.2f,%d\n", t, vt * 3.6, v * 3.6, tl.ay, tl.yaw_rate, rerr, tl.sw_angle * 180 / M_PI, beta, tl.gear);
        int b = (int)(vt * 3.6); if (b >= 0 && b < NB && t > 4) { bs[b][0] += fabs(tl.ay); bs[b][1] += rerr; bs[b][2] += tl.sw_angle * 180 / M_PI; bs[b][3] += beta; bs[b][4] += v * 3.6; bn[b]++; }
        wa[wi] = fabs(tl.ay); wr[wi] = rerr; wv[wi] = (vt - v) * 3.6; wi = (wi + 1) % W; if (wn < W) wn++;
        if (wn == W && t > 6) {
            int ok = 1; for (int k = 0; k < W; k++) if (fabs(wr[k]) > 2 || fabs(wv[k]) > 1) { ok = 0; break; }
            if (ok) { double am = 0; for (int k = 0; k < W; k++) am += wa[k]; vmax_ok = v * 3.6; ay_ok = am / W; rr_ok = rr; sw_ok = tl.sw_angle * 180 / M_PI; beta_ok = beta; }
        }
        if (fabs(rerr) > 6 || fabs(beta) > 45) break;   /* left the pad or spun */
    }
    printf("skidmax %s setup=%s model=%s dir=%+.0f: v_max_steady %.1f km/h  ay(2 s mean) %.3f g (v^2/r %.3f g, r %.1f m)  sw %.0f deg  beta %.1f deg  (ended t=%.1f s)\n",
           car, setup, model, dir, vmax_ok, ay_ok / 9.81, pow(vmax_ok / 3.6, 2) / rr_ok / 9.81, rr_ok, sw_ok, beta_ok, t);
    printf("  bin km/h  |ay| g   r_err m  sw deg  beta deg\n");
    for (int b = 0; b < NB; b++) if (bn[b] > 0 && b % 5 == 0)
        printf("  %4d     %6.3f  %+6.2f  %6.1f  %+6.2f\n", b, bs[b][0] / bn[b] / 9.81, bs[b][1] / bn[b], bs[b][2] / bn[b], bs[b][3] / bn[b]);
    if (csv) fclose(csv);
    return 0;
}
