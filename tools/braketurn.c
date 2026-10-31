/* braketurn.c - braking from a turn / slide / yawing straight: per-tick log of the yaw and slip state, per-wheel forces,
   slip, wheel speed and brake torque, for the "car straightens under braking" investigation.
   Build: make braketurn   Run: ./build/braketurn <car.ini> <scenario> <brake 0..1> <out.csv>
   Scenarios: turn45 / turn70 (steady 60 km/h turn, steering-wheel 45 / 70 deg held, brake at t = 5 s),
   drift (power-oversteer entry, brake at t = 4 s with the countersteer held), yawst (100 km/h, 30 deg steer pulse
   0.3 s at t = 1 s, brake at t = 1.3 s, wheel held centred). Env CFGSET=k=v;.. as in fastcheck (model, setup). */
#define _GNU_SOURCE
#include "../demo/sim.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define DEG (180.0 / M_PI)
static double clampd(double v, double a, double b) { return v < a ? a : (v > b ? b : v); }
static track_t TRK;
int main(int argc, char** argv) {
    if (argc < 5) { fprintf(stderr, "usage: braketurn car.ini turn45|turn70|drift|yawst brake out.csv\n"); return 1; }
    const char* sc = argv[2]; double B = atof(argv[3]);
    track_build(&TRK, 1.0, TRACK_PROVING);
    static sim_t S; static cfg_t cfg;
    cfg_set(&cfg, "autopilot.enabled", "0"); cfg_set(&cfg, "car.setup", "default"); cfg_set(&cfg, "physics.tick_hz", "960"); cfg_set(&cfg, "physics.substeps", "4");
    cfg_set(&cfg, "physics.model", "fast"); cfg_set(&cfg, "car.ini", argv[1]);
    { const char* cs = getenv("CFGSET"); if (cs && *cs) { char buf[1024]; snprintf(buf, sizeof buf, "%s", cs);
        for (char* t = strtok(buf, ";"); t; t = strtok(NULL, ";")) { char* eq = strchr(t, '='); if (eq) { *eq = 0; cfg_set(&cfg, t, eq + 1); } } } }
    char err[256]; if (!sim_init(&S, &cfg, &TRK, NULL, err, sizeof err)) { fprintf(stderr, "%s\n", err); return 1; }
    { const char* fp = getenv("PARAM");   /* car parameters "section:key=value;..." (section car/front/rear), applied after the setup */
      if (fp && *fp) { char buf[1024]; snprintf(buf, sizeof buf, "%s", fp);
          for (char* t = strtok(buf, ";"); t; t = strtok(NULL, ";")) { char* c = strchr(t, ':'); char* eq = strchr(t, '='); if (!c || !eq) continue; *c = 0; *eq = 0;
              if (!dc_set_param(&S.P, t, c + 1, eq + 1)) { fprintf(stderr, "PARAM %s\n", c + 1); return 1; } }
          if (!sim_rebuild_car(&S)) return 1; } }
    int drift = !strcmp(sc, "drift"), yawst = !strcmp(sc, "yawst"); double swh = !strncmp(sc, "turn", 4) ? atof(sc + 4) / DEG : 0;
    double HK = 80, HD = 1.2, HT = 30; if (getenv("HANDS")) sscanf(getenv("HANDS"), "%lf,%lf,%lf", &HK, &HD, &HT);   /* default: demo wheel hands */
    double v0 = yawst ? 100 / 3.6 : drift ? 50 / 3.6 : 60 / 3.6, tb = drift ? 4.0 : yawst ? 1.3 : 5.0, T = tb + 3.0;
    sim_reset_at(&S, 100, yawst ? 150 : 60, 0, v0); S.gear = drift ? 2 : yawst ? 4 : 3;
    if (getenv("V0")) { v0 = atof(getenv("V0")) / 3.6; sim_reset_at(&S, 100, 60, 0, v0); if (getenv("GEAR")) S.gear = atoi(getenv("GEAR")); }
    S.ext_steer = 1; S.ext_pedals = 1; double I = 0.2, dt = S.tick_dt;
    FILE* f = fopen(argv[4], "w");
    fprintf(f, "t,sw_deg,rack_pos_mm,col_nm,drv_nm,st0,st1,brake,v,vx,vy,r_dps,beta_deg,kappa,ax,ay,rack,pgs,capped");
    for (int k = 0; k < 4; k++) fprintf(f, ",fz%d,fx%d,fy%d,sr%d,sa%d,srss%d,sass%d,om%d,bt%d,sl%d,mu%d,mz%d", k, k, k, k, k, k, k, k, k, k, k, k);
    fprintf(f, "\n");
    for (double t = 0; t < T - 1e-9; t += dt) {
        dc_telemetry tl; dc_get_telemetry(S.car, &tl);
        double sw = 0, thr = 0, brk = 0; int hold = 1;
        if (swh) { sw = clampd((t - 1) / 1.0, 0, 1) * swh; }
        if (drift) { double u = t - 2; if (u > 0) { hold = 0; sw = fmin(u / 0.2, 1) * 160 / DEG; thr = 1; } if (u > 2.0) sw = -60 / DEG; }
        if (drift && t > 2 && t < 2.5) { /* handbrake kick */ }
        if (yawst && t > 1 && t < 1.3) sw = 30 / DEG;
        if (t >= tb) { hold = 0; thr = 0; brk = clampd((t - tb) / 0.1, 0, 1) * B; }
        if (hold) { double e = v0 - tl.vx; I = clampd(I + 0.08 * e * dt, 0, 1); double c = 0.25 * e + I; thr = clampd(c, 0, 1); brk = clampd(-c * 0.5, 0, 1); }
        dc_input* in = &S.ext_in; memset(in, 0, sizeof *in);
        in->steer_mode = DC_STEER_ANGLE; in->steer_target = sw; in->hands_k = HK; in->hands_d = HD; in->hands_tmax = HT;
        in->throttle = thr; in->brake = brk; in->handbrake = drift && t > 2 && t < 2.5 ? 1 : 0;
        sim_tick(&S, dt);
        dc_get_telemetry(S.car, &tl);
        double v = hypot(tl.vx, tl.vy);
        fprintf(f, "%.5f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.6f,%.3f,%.3f,%.1f,%d,%d", t + dt, tl.sw_angle * DEG, tl.rack_pos * 1e3, tl.column_torque, tl.driver_torque, tl.w[0].steer * DEG, tl.w[1].steer * DEG, brk, v, tl.vx, tl.vy, tl.yaw_rate * DEG,
                atan2(tl.vy, fabs(tl.vx)) * DEG, v > 0.5 ? tl.yaw_rate / v : 0, tl.ax, tl.ay, tl.rack_force, tl.pgs_iters, tl.pgs_capped);
        for (int k = 0; k < 4; k++) { const dc_wheel_telem* w = &tl.w[k];
            fprintf(f, ",%.1f,%.1f,%.1f,%.4f,%.4f,%.4f,%.4f,%.3f,%.1f,%.3f,%.3f,%.2f", w->fz, w->fx, w->fy, w->slip_ratio, w->slip_angle * DEG, w->slip_ratio_ss, w->slip_angle_ss * DEG, w->omega, w->brake_torque, w->sliding, w->mu, w->mz); }
        fprintf(f, "\n");
    }
    fclose(f); return 0;
}
