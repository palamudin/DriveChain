/* gpturn.c - corner + brake / throttle through the GAMEPAD input path (demo/sim.c kind 2: stick -> rim target via the
   gamepad "arms", triggers through the [input] trigger curve), with the stick held all the way.
   Build: make gpturn   Run: ./build/gpturn <car.ini> <stick 0..1 (+ = left)> <phase> <trigger 0..1> <out.csv>
   Env CHECK=<deg>: exit 2 if the rim leaves its pre-trigger angle by more than <deg> (default 15) or the hands saturate
   (make gamepadtest). Env REL_T / REL_X / REL_DUR: move the stick linearly to REL_X from REL_T s over REL_DUR s (stick-release comparison).
   Phases (at t = 4 s, after 1 s stick ramp-in and a speed hold at V0 km/h, default 50):
     brake  - throttle off, brake trigger;  both - throttle trigger kept at its hold value + brake trigger;
     lift   - throttle off;                 gas  - throttle trigger.
   Env CFGSET=k=v;.. (e.g. gamepad.arms_tmax=20, input.brake_gamma=1.8), PARAM as in braketurn, V0, GEAR, T (end s). */
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
    if (argc < 6) { fprintf(stderr, "usage: gpturn car.ini stick brake|both|lift|gas trigger out.csv\n"); return 1; }
    double stick = atof(argv[2]), trig = atof(argv[4]); const char* ph = argv[3];
    track_build(&TRK, 1.0, TRACK_PROVING);
    static sim_t S; static cfg_t cfg;
    cfg_set(&cfg, "autopilot.enabled", "0"); cfg_set(&cfg, "car.setup", "default"); cfg_set(&cfg, "physics.tick_hz", "960"); cfg_set(&cfg, "physics.substeps", "4");
    cfg_set(&cfg, "physics.model", "fast"); cfg_set(&cfg, "car.ini", argv[1]);
    { const char* cs = getenv("CFGSET"); if (cs && *cs) { char buf[1024]; snprintf(buf, sizeof buf, "%s", cs);
        for (char* tok = strtok(buf, ";"); tok; tok = strtok(NULL, ";")) { char* e = strchr(tok, '='); if (e) { *e = 0; cfg_set(&cfg, tok, e + 1); } } } }
#ifdef DC_PROF
    if (getenv("PROF_DUMP")) prof_init(20000);   /* the F8 per-tick record, dumped as <PROF_DUMP>_ticks.csv at the end */
#endif
    char err[256]; if (!sim_init(&S, &cfg, &TRK, NULL, err, sizeof err)) { fprintf(stderr, "%s\n", err); return 1; }
    { const char* fp = getenv("PARAM");
      if (fp && *fp) { char buf[1024]; snprintf(buf, sizeof buf, "%s", fp);
          for (char* tok = strtok(buf, ";"); tok; tok = strtok(NULL, ";")) { char* c = strchr(tok, ':'); char* e = strchr(tok, '=');
              if (c && e && c < e) { *c = 0; *e = 0; if (dc_set_param(&S.P, tok, c + 1, e + 1) != 0) fprintf(stderr, "bad param %s\n", c + 1); } }
          if (!sim_rebuild_car(&S)) return 1; } }
    double sw_tb = 0, dev_max = 0; int nsat = 0;
    double rel_t = getenv("REL_T") ? atof(getenv("REL_T")) : 1e9, rel_x = getenv("REL_X") ? atof(getenv("REL_X")) : 0;   /* stick ramp (release test) */
    double rel_dur = getenv("REL_DUR") ? atof(getenv("REL_DUR")) : 0;
    double v0 = (getenv("V0") ? atof(getenv("V0")) : 50) / 3.6, tb = 4.0, T = getenv("T") ? atof(getenv("T")) : 7.0;
    sim_reset_at(&S, 100, 60, 0, v0); S.gear = getenv("GEAR") ? atoi(getenv("GEAR")) : 3;
    double dz, ga, mx; input_trigger_params(1, &dz, &ga, &mx);
    S.ext_steer = 0; S.ext_pedals = 0; S.dev.kind = 2;   /* the demo's gamepad branch: no SDL device, fields set here */
    double I = 0.2, dt = S.tick_dt, thr_hold = 0;
    FILE* f = fopen(argv[5], "w");
    fprintf(f, "t,stick,thr_trig,brk_trig,thr,brk,steer_mode,steer_target_deg,sw_deg,drv_nm,hands_sat,col_nm,rack_n,st0,st1,v,r_dps,beta_deg,kappa,ay");
    for (int k = 0; k < 4; k++) fprintf(f, ",fz%d,fx%d,fy%d,srss%d,sass%d,om%d,bt%d,mz%d", k, k, k, k, k, k, k, k);
    fprintf(f, "\n");
    for (double t = 0; t < T - 1e-9; t += dt) {
        dc_telemetry tl; dc_get_telemetry(S.car, &tl);
        double st = clampd((t - 1) / 0.5, 0, 1) * stick, tt = 0, bt = 0;
        if (t >= rel_t) { double a = rel_dur > 0 ? clampd((t - rel_t) / rel_dur, 0, 1) : 1; st = stick + a * (rel_x - stick); }
        if (t < tb) { double e = v0 - tl.vx; I = clampd(I + 0.08 * e * dt, 0, 1); tt = clampd(0.25 * e + I, 0, 1); thr_hold = tt; }
        else {
            double r = clampd((t - tb) / 0.1, 0, 1) * trig;
            if (!strcmp(ph, "brake")) bt = r;
            else if (!strcmp(ph, "both")) { tt = thr_hold; bt = r; }
            else if (!strcmp(ph, "gas")) tt = r;
        }
        S.dev.steer = st; S.dev.throttle_raw = tt; S.dev.brake_raw = bt;
        { double tdz, tga, tmx; input_trigger_params(0, &tdz, &tga, &tmx); S.dev.throttle = input_trigger_curve(tt, tdz, tga, tmx); }
        S.dev.brake = input_trigger_curve(bt, dz, ga, mx);
        sim_tick(&S, dt);
        dc_get_telemetry(S.car, &tl); const dc_input* in = &S.snap.in;
        double v = hypot(tl.vx, tl.vy), tmax = in->hands_tmax;
        fprintf(f, "%.5f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%.3f,%.3f,%.3f,%d,%.3f,%.1f,%.4f,%.4f,%.4f,%.4f,%.4f,%.6f,%.3f", t + dt, st, tt, bt, in->throttle, in->brake, in->steer_mode,
                in->steer_mode == DC_STEER_ANGLE ? in->steer_target * DEG : NAN, tl.sw_angle * DEG, tl.driver_torque, tmax > 0 && fabs(tl.driver_torque) >= 0.999 * tmax, tl.column_torque, tl.rack_force,
                tl.w[0].steer * DEG, tl.w[1].steer * DEG, v, tl.yaw_rate * DEG, atan2(tl.vy, fabs(tl.vx)) * DEG, v > 0.5 ? tl.yaw_rate / v : 0, tl.ay);
        for (int k = 0; k < 4; k++) { const dc_wheel_telem* w = &tl.w[k];
            fprintf(f, ",%.1f,%.1f,%.1f,%.4f,%.4f,%.3f,%.1f,%.2f", w->fz, w->fx, w->fy, w->slip_ratio_ss, w->slip_angle_ss * DEG, w->omega, w->brake_torque, w->mz); }
        fprintf(f, "\n");
        if (t + dt >= tb && t < tb) sw_tb = tl.sw_angle;   /* rim angle when the trigger phase starts */
        if (t + dt > tb && t + dt < T - 1.0 && v > 2) { double e = fabs(tl.sw_angle - sw_tb) * DEG; if (e > dev_max) dev_max = e;   /* stop the check 1 s before the end / below 2 m/s */
                                                       if (tmax > 0 && fabs(tl.driver_torque) >= 0.999 * tmax) nsat++; }
    }
    fclose(f);
    if (getenv("CHECK")) {   /* driver test: with the stick held, triggers must not move the rim off its target (no rim return) */
        double lim = atof(getenv("CHECK")) > 0 ? atof(getenv("CHECK")) : 15;
        int ok = dev_max <= lim && nsat == 0;
        printf("gpturn %s stick %.2f %s %.2f: rim moved %.1f deg after the trigger (limit %.0f), hands saturated %d ticks -> %s\n", argv[1], stick, ph, trig, dev_max, lim, nsat, ok ? "PASS" : "FAIL");
        if (!ok) return 2;
    }
#ifdef DC_PROF
    if (getenv("PROF_DUMP")) prof_dump(getenv("PROF_DUMP"));
#endif
    return 0;
}
