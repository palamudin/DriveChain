/* mbcompare.c - manoeuvre suite driven through the driver interface (dc_driver), for comparing solvers and
   parameter sets. Writes one CSV per manoeuvre and a summary line per metric.
   Build: make mbcompare   Run: ./build/mbcompare car.ini outdir [solver 0|1] [manoeuvre|all]
   Manoeuvres: settle, skidpad, step, liftoff, handsoff, bump, slalom, bounce                              */
#include "drivechain.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG (180.0 / M_PI)

typedef struct { double bx, len, h; int side; } bump_t;   /* cosine bump at x in [bx, bx+len], side +1 left (y>0), -1 right, 0 both */
static bump_t BUMP; static double GRIP = 1.0;   /* surface friction multiplier (wet-pad manoeuvres) */
static double road(void* u, double x, double y, double* nx, double* ny, double* nz, double* g) {
    *nx = 0; *ny = 0; *nz = 1; *g = GRIP;
    if (BUMP.h == 0 || x < BUMP.bx || x > BUMP.bx + BUMP.len) return 0;
    if (BUMP.side > 0 && y < 0) return 0;
    if (BUMP.side < 0 && y > 0) return 0;
    double s = (x - BUMP.bx) / BUMP.len;
    double z = 0.5 * BUMP.h * (1 - cos(2 * M_PI * s));
    double dz = 0.5 * BUMP.h * 2 * M_PI / BUMP.len * sin(2 * M_PI * s);
    double n = sqrt(1 + dz * dz); *nx = -dz / n; *nz = 1 / n;
    return z;
}

static dc_car_params P; static dc_car* C; static dc_driver* D; static dc_input IN; static dc_telemetry T;
static FILE* CSV; static double SIMT; static double CPU;
static double H_DPREV, H_DMIN = 1e30, H_PMAX, H_NFAIL, H_DMIN_T; static int H_FIRST; static char H_CUR[32], H_DMIN_M[32];   /* health: min per-step dissipation, peak valve pressure, valve fails */
static const double DT = 1.0 / 120.0;

static int auto_gear(int g, double rpm) {
    if (g < 1) return 1;
    if (rpm > 6500 && g < 5) return g + 1;
    if (rpm < 2500 && g > 1) return g - 1;
    return g;
}
static void start(double speed, int gear) {
    if (C) dc_destroy(C);
    C = dc_create(&P);
    dc_set_road(C, road, NULL);
    double pos[3] = {0, 0, 0};
    dc_reset(C, pos, 0, speed);
    if (getenv("DCDBG")) dc_debug_flags(C, atoi(getenv("DCDBG")));
    memset(&IN, 0, sizeof IN); IN.gear = gear; IN.steer_mode = DC_STEER_TORQUE;
    dc_driver_params dp; dc_driver_default_params(&dp);
    if (D) dc_driver_destroy(D);
    D = dc_driver_create(&dp);
    SIMT = 0; H_FIRST = 1;
}
static void tick(int autoshift) {
    dc_driver_update(D, C, DT, &IN);
    if (autoshift) IN.gear = auto_gear(IN.gear, T.engine_rpm);
    dc_set_input(C, &IN);
    clock_t c0 = clock();
    dc_step(C, DT);
    CPU += (double)(clock() - c0) / CLOCKS_PER_SEC;
    dc_get_telemetry(C, &T); SIMT += DT;
    if (!H_FIRST && T.dissipated - H_DPREV < H_DMIN) { H_DMIN = T.dissipated - H_DPREV; H_DMIN_T = SIMT; snprintf(H_DMIN_M, sizeof H_DMIN_M, "%s", H_CUR); }
    if (getenv("DTRACE") && !H_FIRST && T.dissipated - H_DPREV < atof(getenv("DTRACE"))) fprintf(stderr, "dtrace %s t %.3f dE %.4f\n", H_CUR, SIMT, T.dissipated - H_DPREV);
    H_FIRST = 0; H_DPREV = T.dissipated;
    for (int k = 0; k < 3; k++) H_PMAX = fmax(H_PMAX, fabs(T.valve_p[k]));
    H_NFAIL = fmax(H_NFAIL, T.valve_nfail);
}
static double yaw_of(void) { dc_xform x[DC_PART_COUNT]; dc_get_parts(C, x, DC_PART_COUNT);
    double q0 = x[0].quat[0], q1 = x[0].quat[1], q2 = x[0].quat[2], q3 = x[0].quat[3];
    return atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)); }
static void pos_of(double* x, double* y) { dc_xform p[DC_PART_COUNT]; dc_get_parts(C, p, DC_PART_COUNT); *x = p[0].pos[0]; *y = p[0].pos[1]; }
static void csv_open(const char* dir, const char* name) {
    char fn[512]; snprintf(fn, sizeof fn, "%s/%s.csv", dir, name); CSV = fopen(fn, "w"); snprintf(H_CUR, sizeof H_CUR, "%s", name);
    fprintf(CSV, "t,x,y,yaw,speed,ay,yaw_rate,beta,sw,column_torque,rack_force,fz0,fz1,fz2,fz3,alpha0,alpha2,travel0,travel1,travel2,travel3,steer0,steer1,fx0,fy0,fx1,fy1,fx2,fy2,fx3,fy3,aks0,aks1,aks2,aks3,kss2,kss3\n");
}
static void csv_row(void) {
    double x, y; pos_of(&x, &y);
    fprintf(CSV, "%.4f,%.4f,%.4f,%.5f,%.4f,%.4f,%.5f,%.5f,%.5f,%.4f,%.2f,%.1f,%.1f,%.1f,%.1f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
            SIMT, x, y, yaw_of(), T.speed, T.ay, T.yaw_rate, atan2(T.vy, fmax(fabs(T.vx), 0.5)), T.sw_angle, T.column_torque, T.rack_force,
            T.w[0].fz, T.w[1].fz, T.w[2].fz, T.w[3].fz, T.w[0].slip_angle, T.w[2].slip_angle,
            T.w[0].travel, T.w[1].travel, T.w[2].travel, T.w[3].travel, T.w[0].steer, T.w[1].steer);
    fseek(CSV, -1, SEEK_CUR);
    for (int k = 0; k < 4; k++) fprintf(CSV, ",%.1f,%.1f", T.w[k].fx, T.w[k].fy);
    for (int k = 0; k < 4; k++) fprintf(CSV, ",%.5f", T.w[k].slip_angle_ss);
    fprintf(CSV, ",%.4f,%.4f\n", T.w[2].slip_ratio_ss, T.w[3].slip_ratio_ss);
}
#define SUM(name, val, unit) printf("%-10s %-28s %12.4f %s\n", tag, name, (double)(val), unit)
static const char* tag = "run";

/* path helpers (static buffers; the driver keeps a pointer) */
static double PATH[2 * 4096];
static void circle_path(double R, int n) { for (int i = 0; i < n; i++) { double a = 2 * M_PI * i / n; PATH[2 * i] = R * sin(a); PATH[2 * i + 1] = R * (1 - cos(a)); } dc_driver_set_path(D, PATH, n, 1); }

static void m_settle(const char* dir) {
    start(0, 0); csv_open(dir, "settle");
    dc_driver_cmd cmd = {0}; cmd.steer_mode = DC_DRV_HANDS_OFF; cmd.pedal_mode = DC_PED_DIRECT; cmd.brake = 0.3; dc_driver_command(D, &cmd);
    for (int i = 0; i < 360; i++) { tick(0); if (i % 4 == 0) csv_row(); }
    fclose(CSV);
    SUM("settle_fz_front", T.w[0].fz + T.w[1].fz, "N");
    SUM("settle_fz_rear", T.w[2].fz + T.w[3].fz, "N");
    SUM("settle_front_pct", 100 * (T.w[0].fz + T.w[1].fz) / (T.w[0].fz + T.w[1].fz + T.w[2].fz + T.w[3].fz), "%");
    SUM("settle_travel_f", T.w[0].travel * 1000, "mm");
    SUM("settle_travel_r", T.w[2].travel * 1000, "mm");
}

static void m_bounce(const char* dir) {   /* drop-release body bounce: body heave frequency and damping from travel */
    start(0, 0); csv_open(dir, "bounce");
    dc_driver_cmd cmd = {0}; cmd.steer_mode = DC_DRV_HANDS_OFF; cmd.pedal_mode = DC_PED_DIRECT; cmd.brake = 0.3; dc_driver_command(D, &cmd);
    for (int i = 0; i < 240; i++) tick(0);
    /* 40 mm road step under all wheels by moving the bump: emulate with a long plateau */
    BUMP.bx = -100; BUMP.len = 1e9; BUMP.h = 0; /* no-op */
    /* heave: kick the body with a 30 mm/s.. use road step: simplest is a lift then release = drop from 30 mm */
    double t0 = SIMT; double tr[360]; int nt = 0;
    BUMP.bx = -1e6; BUMP.len = 2e6; BUMP.h = 0.03;   /* plateau: 30 mm road step under all four wheels */
    H_FIRST = 1;   /* the teleported road injects tyre energy this step: external work, not an audit failure */
    for (int i = 0; i < 360; i++) { tick(0); csv_row(); dc_xform px[DC_PART_COUNT]; dc_get_parts(C, px, DC_PART_COUNT); tr[nt++] = px[0].pos[2]; }
    BUMP.h = 0; fclose(CSV);
    /* body heave response to the step: final value, first max, following min -> half damped period, overshoot */
    double zf = tr[nt - 1], z0 = tr[0]; int imax = 0, imin = -1;
    for (int i = 1; i < nt / 2; i++) if (tr[i] > tr[imax]) imax = i;
    for (int i = imax + 1; i < nt - 1; i++) if (tr[i] < tr[i - 1] && tr[i] <= tr[i + 1]) { imin = i; break; }
    double os = (tr[imax] - zf) / fmax(fabs(zf - z0), 1e-6);
    SUM("bounce_overshoot", 100 * os, "% body heave overshoot on 30 mm step");
    if (imin > 0) SUM("bounce_period", 2 * (imin - imax) * DT, "s (2x max->min)");
    SUM("bounce_zeta_est", os > 1e-4 ? -log(os) / sqrt(M_PI * M_PI + log(os) * log(os)) : 1.0, "from overshoot");
    (void)t0;
}

static void m_skidpad(const char* dir) {
    double R = 40; start(8, 2); csv_open(dir, "skidpad"); circle_path(R, 720);
    dc_driver_cmd cmd = {0}; cmd.steer_mode = DC_DRV_PATH; cmd.pedal_mode = DC_PED_SPEED; cmd.speed = 8; dc_driver_command(D, &cmd);
    double ay_max = 0, sw_at[3] = {0}, tq_at[3] = {0}; double ay_lvl[3] = {3.0, 5.0, 7.0}; int got[3] = {0};
    double sw_ay_lo = 0, ay_lo = 0; int lo_set = 0, lost = 0;
    /* SAE J266 constant-radius method: R fixed, so the Ackermann angle L/R is constant and the slope of steer angle
     * vs ay is the understeer gradient. Least-squares over the linear range 0.2-0.45 g (the start at 8 m/s is already 0.16 g, so lower samples are the start transient) (single first-crossing samples
     * are dominated by path-driver hunting). Reported at the steering wheel and at the road wheels (mean front steer). */
    double fn = 0, fx = 0, fxx = 0, fsw = 0, fxsw = 0, frw = 0, fxrw = 0;
    for (int i = 0; i < 120 * 60; i++) {
        double tt = SIMT; cmd.speed = 8 + 0.25 * tt; dc_driver_command(D, &cmd);
        tick(1);
        if (i % 6 == 0) csv_row();
        dc_driver_state st; dc_driver_get_state(D, &st);
        double ayf = fabs(T.ay);
        if (fabs(st.lat_err) < 2.0 && ayf > ay_max) ay_max = ayf;
        for (int k = 0; k < 3; k++) if (!got[k] && ayf >= ay_lvl[k]) { got[k] = 1; sw_at[k] = T.sw_angle * DEG; tq_at[k] = T.column_torque; }
        if (!lo_set && ayf >= 3.0) { lo_set = 1; sw_ay_lo = T.sw_angle * DEG; ay_lo = ayf; }
        if (SIMT > 3.0 && ayf >= 0.2 * 9.81 && ayf <= 0.45 * 9.81 && fabs(st.lat_err) < 1.0) {
            double g = ayf / 9.81, sw = fabs(T.sw_angle) * DEG, rw = fabs(0.5 * (T.w[0].steer + T.w[1].steer)) * DEG;
            fn += 1; fx += g; fxx += g * g; fsw += sw; fxsw += g * sw; frw += rw; fxrw += g * rw; }
        if (fabs(st.lat_err) > 4.0) { lost = 1; break; }
    }
    fclose(CSV);
    SUM("skidpad_ay_max", ay_max / 9.81, "g");
    for (int k = 0; k < 3; k++) { char nm[64]; snprintf(nm, sizeof nm, "skidpad_sw_at_%.0fms2", ay_lvl[k]); SUM(nm, sw_at[k], "deg");
        snprintf(nm, sizeof nm, "skidpad_tq_at_%.0fms2", ay_lvl[k]); SUM(nm, tq_at[k], "Nm"); }
    if (got[2] && lo_set) SUM("understeer_grad_sw", (sw_at[2] - sw_ay_lo) / ((ay_lvl[2] - ay_lo) / 9.81), "deg sw/g, OLD: two single samples 3->7 m/s2, noisy");
    { double den = fn * fxx - fx * fx;
      if (fn > 20 && den > 1e-9) {
        double ksw = (fn * fxsw - fx * fsw) / den, krw = (fn * fxrw - fx * frw) / den;
        SUM("usg_sw_fit", ksw, "deg sw/g, LSQ 0.2-0.45 g, constant R (J266 steering-wheel angle gradient)");
        SUM("usg_rw_fit", krw, "deg actual road-wheel angle/g, LSQ 0.2-0.45 g (tyre + rear/roll steer only; excludes front compliance steer)");
        SUM("usg_overall_ratio", krw > 1e-6 ? ksw / krw : 0, "sw/rw gradient ratio (rack ratio + column compliance)");
        /* J266 reference steer angle gradient = SW gradient / static overall ratio. This is the understeer gradient K
         * as published for road cars: it includes front compliance steer, which moves the wheel relative to the rack
         * and so never appears in the actual road-wheel angle (usg_rw_fit only sees tyre + rear/roll steer). */
        SUM("usg_K_j266", ksw / P.steer_ratio, "deg/g, usg_sw_fit / steer_ratio (SAE J266 reference steer gradient)");
        SUM("usg_fit_n", fn, "samples"); } }
    SUM("skidpad_lost_path", lost, "");
}

static void m_step(const char* dir) {
    start(80 / 3.6, 3); csv_open(dir, "step");
    dc_driver_cmd cmd = {0}; cmd.steer_mode = DC_DRV_ANGLE; cmd.steer_target = 0; cmd.pedal_mode = DC_PED_SPEED; cmd.speed = 80 / 3.6; dc_driver_command(D, &cmd);
    for (int i = 0; i < 120; i++) tick(0);
    cmd.steer_target = 40 / DEG; dc_driver_command(D, &cmd);
    double t0 = SIMT, rmax = 0, r_ss = 0, t90 = -1; double buf[600]; int n = 0;
    for (int i = 0; i < 480; i++) { tick(0); csv_row(); buf[n++] = T.yaw_rate; if (fabs(T.yaw_rate) > rmax) rmax = fabs(T.yaw_rate); }
    r_ss = fabs(buf[n - 1]);
    for (int i = 0; i < n; i++) if (fabs(buf[i]) >= 0.9 * r_ss) { t90 = (i + 1) * DT; break; }
    fclose(CSV); (void)t0;
    SUM("step_yaw_rate_ss", r_ss * DEG, "deg/s");
    SUM("step_t90", t90, "s");
    SUM("step_overshoot", 100 * (rmax / fmax(r_ss, 1e-6) - 1), "%");
    SUM("step_ay_ss", T.ay / 9.81, "g");
}

static void m_liftoff(const char* dir) {
    double R = 40; start(8, 2); csv_open(dir, "liftoff"); circle_path(R, 720);
    dc_driver_cmd cmd = {0}; cmd.steer_mode = DC_DRV_PATH; cmd.pedal_mode = DC_PED_SPEED; cmd.speed = 8; dc_driver_command(D, &cmd);
    for (int i = 0; i < 120 * 30; i++) { double v = fmin(8 + 0.3 * SIMT, sqrt(0.7 * 9.81 * R));   /* steady 0.7 g, then lift (ISO 9816 style) */ cmd.speed = v; dc_driver_command(D, &cmd); tick(1); if (i % 6 == 0) csv_row(); }
    /* hold steer, lift */
    double sw = T.sw_angle; cmd.steer_mode = DC_DRV_ANGLE; cmd.steer_target = sw; cmd.pedal_mode = DC_PED_DIRECT; cmd.throttle = 0; dc_driver_command(D, &cmd);
    double r0 = T.yaw_rate, b0 = atan2(T.vy, T.vx), rmax = fabs(r0), bmax = fabs(b0), ay0 = T.ay;
    for (int i = 0; i < 240; i++) { tick(0); csv_row(); if (fabs(T.yaw_rate) > rmax) rmax = fabs(T.yaw_rate); double b = fabs(atan2(T.vy, T.vx)); if (b > bmax) bmax = b; }
    fclose(CSV);
    SUM("liftoff_ay0", fabs(ay0) / 9.81, "g");
    SUM("liftoff_yawrate_gain", 100 * (rmax / fmax(fabs(r0), 1e-6) - 1), "% over pre-lift");
    SUM("liftoff_beta_max", bmax * DEG, "deg");
}

static void m_handsoff(const char* dir) {
    /* provoke a power slide (steer + handbrake flick + full throttle in 2nd), then release the rim; measure how far the
       wheel follows the front axle's course (hands-off countersteer) */
    GRIP = 0.55;   /* wet low-friction pad (drift-school style), the car can actually be provoked into a slide */
    start(45 / 3.6, 2); csv_open(dir, "handsoff");
    dc_driver_cmd cmd = {0}; cmd.steer_mode = DC_DRV_ANGLE; cmd.steer_target = 0; cmd.pedal_mode = DC_PED_DIRECT; cmd.throttle = 0.3; dc_driver_command(D, &cmd);
    for (int i = 0; i < 60; i++) { tick(0); csv_row(); }
    cmd.steer_target = 150 / DEG; cmd.throttle = 1.0; dc_driver_command(D, &cmd);
    for (int i = 0; i < 30; i++) { tick(0); csv_row(); }
    cmd.handbrake = 1; cmd.throttle = 0; dc_driver_command(D, &cmd);
    for (int i = 0; i < 40; i++) { tick(0); csv_row(); }
    cmd.handbrake = 0; cmd.throttle = 1.0; dc_driver_command(D, &cmd);
    for (int i = 0; i < 12; i++) { tick(0); csv_row(); }
    cmd.steer_mode = DC_DRV_HANDS_OFF; cmd.throttle = getenv("HO_THR") ? atof(getenv("HO_THR")) : 0.7; dc_driver_command(D, &cmd);
    double sw_min = 1e9, beta_max = 0, corr_n = 0, corr_e = 0, bf_min = 1e9;
    for (int i = 0; i < 360; i++) { tick(0); csv_row();
        double beta = atan2(T.vy, fmax(fabs(T.vx), 0.5));
        if (T.sw_angle < sw_min) sw_min = T.sw_angle;
        if (fabs(beta) > beta_max) beta_max = fabs(beta);
        /* course of the front axle relative to the chassis (a = 1.0 m CG to front axle) */
        double bf = atan2(T.vy + 1.0 * T.yaw_rate, fmax(fabs(T.vx), 0.5));
        if (bf < bf_min) bf_min = bf;
        if (fabs(beta) > 5 / DEG && T.speed > 3) { corr_e += fabs(0.5 * (T.w[0].steer + T.w[1].steer) - bf); corr_n++; }
    }
    fclose(CSV); GRIP = 1.0;
    SUM("handsoff_beta_max", beta_max * DEG, "deg");
    SUM("handsoff_front_course_min", bf_min * DEG, "deg");
    SUM("handsoff_sw_counter_max", -sw_min * DEG, "deg (opposite lock)");
    SUM("handsoff_frontwheel_vs_course", corr_n > 0 ? corr_e / corr_n * DEG : -1, "deg mean |steer - front-axle course| while beta>5");
}

static void m_bump(const char* dir) {
    /* single left-wheel cosine bump 40 mm x 0.6 m at 60 km/h, hands with arm impedance holding 0 */
    BUMP.bx = 30; BUMP.len = 0.6; BUMP.h = 0.04; BUMP.side = 1;
    start(60 / 3.6, 3); csv_open(dir, "bump");
    dc_driver_cmd cmd = {0}; cmd.steer_mode = DC_DRV_ANGLE; cmd.steer_target = 0; cmd.pedal_mode = DC_PED_SPEED; cmd.speed = 60 / 3.6; dc_driver_command(D, &cmd);
    double y0 = 0, yaw0 = 0, tqmax = 0, swmax = 0; int armed = 0;
    for (int i = 0; i < 480; i++) { tick(0); csv_row(); double x, y; pos_of(&x, &y);
        if (!armed && x > 25) { armed = 1; y0 = y; yaw0 = yaw_of(); }
        if (armed) { if (fabs(T.column_torque) > tqmax) tqmax = fabs(T.column_torque); if (fabs(T.sw_angle) > swmax) swmax = fabs(T.sw_angle); } }
    double x, y; pos_of(&x, &y);
    fclose(CSV); BUMP.h = 0;
    SUM("bump_col_torque_peak", tqmax, "Nm");
    SUM("bump_sw_peak", swmax * DEG, "deg");
    SUM("bump_heading_change", (yaw_of() - yaw0) * DEG, "deg");
    SUM("bump_lateral_drift", y - y0, "m");
}

static void m_slalom(const char* dir) {
    start(60 / 3.6, 3); csv_open(dir, "slalom");
    int n = 0; for (int i = 0; i < 2000; i++) { double x = i * 0.25; double y = x < 40 ? 0 : 1.5 * sin(2 * M_PI * (x - 40) / 36.0); PATH[2 * n] = x; PATH[2 * n + 1] = y; n++; }
    dc_driver_set_path(D, PATH, n, 0);
    dc_driver_cmd cmd = {0}; cmd.steer_mode = DC_DRV_PATH; cmd.pedal_mode = DC_PED_SPEED; cmd.speed = 60 / 3.6; dc_driver_command(D, &cmd);
    double emax = 0, aymax = 0, tqmax = 0;
    for (int i = 0; i < 120 * 20; i++) { tick(0); csv_row(); double x, y; pos_of(&x, &y); if (x > 450) break;
        dc_driver_state st; dc_driver_get_state(D, &st); if (x > 60) { if (fabs(st.lat_err) > emax) emax = fabs(st.lat_err); if (fabs(T.ay) > aymax) aymax = fabs(T.ay); if (fabs(T.column_torque) > tqmax) tqmax = fabs(T.column_torque); } }
    fclose(CSV);
    SUM("slalom_lat_err_max", emax, "m");
    SUM("slalom_ay_max", aymax / 9.81, "g");
    SUM("slalom_col_torque_max", tqmax, "Nm");
}

/* steering hysteresis: a stiff rim servo (steering robot, not the human arm model) drives a slow sine
 * theta0 + A sin(2 pi f t) about the held rim angle, straight (~0 g) and on a 40 m circle at ~0.6 g.
 * Rim torque is fitted over whole cycles as a + b sin + c cos: c is the part in phase with rim velocity;
 * for Coulomb friction c = 4 Tf / pi, so pi c / 4 is the equivalent rim friction. Vehicle lag also adds to c,
 * so compare joint_friction 1 vs 0: the difference is the joint friction seen at the rim. */
static void hyst_case(const char* dir, const char* name, double ay_target) {
    double v = 20, A = 3 / DEG, f = 0.1;
    start(v, 3); csv_open(dir, name);
    dc_driver_cmd cmd = {0}; cmd.steer_mode = DC_DRV_OFF; cmd.pedal_mode = DC_PED_SPEED; cmd.speed = v; dc_driver_command(D, &cmd);
    IN.steer_mode = DC_STEER_ANGLE; IN.hands_k = 3000; IN.hands_d = 30; IN.hands_tmax = 500;
    /* find the rim angle for the target ay: slow integral trim (25 s), then hold 5 s */
    double th0 = 0;
    for (int i = 0; i < 120 * 30; i++) {
        if (i < 120 * 25 && ay_target > 0) th0 = fmin(th0 + 0.03 * (ay_target * 9.81 - fabs(T.ay)) * DT * fmin(1, i * DT / 3), 3.0);
        IN.steer_target = th0; tick(1);
    }
    double S[3][3] = {{0}}, Y[3] = {0}, rf = 0, cf = 0, ays = 0; int n = 0;
    int ncyc = 3, pre = (int)(120 / f);   /* one cycle run-in (drops the start transient), then ncyc fitted */
    for (int i = 0; i < pre + (int)(120 * ncyc / f); i++) {
        double ph = 2 * M_PI * f * (i * DT);
        IN.steer_target = th0 + A * sin(ph);
        tick(1); if (i % 6 == 0) csv_row();
        if (i >= pre) { double r[3] = {1, sin(ph), cos(ph)}, y = T.driver_torque;
            for (int a = 0; a < 3; a++) { Y[a] += r[a] * y; for (int b = 0; b < 3; b++) S[a][b] += r[a] * r[b]; }
            rf += fabs(T.rack_fric_force); cf += fabs(T.col_fric_torque); ays += fabs(T.ay); n++; }
    }
    fclose(CSV);
    /* 3x3 normal equations (Cramer) */
    double det = S[0][0] * (S[1][1] * S[2][2] - S[1][2] * S[2][1]) - S[0][1] * (S[1][0] * S[2][2] - S[1][2] * S[2][0]) + S[0][2] * (S[1][0] * S[2][1] - S[1][1] * S[2][0]);
    double c = (S[0][0] * (S[1][1] * Y[2] - Y[1] * S[2][1]) - S[0][1] * (S[1][0] * Y[2] - Y[1] * S[2][0]) + Y[0] * (S[1][0] * S[2][1] - S[1][1] * S[2][0])) / det;
    double b = (S[0][0] * (Y[1] * S[2][2] - S[1][2] * Y[2]) - Y[0] * (S[1][0] * S[2][2] - S[1][2] * S[2][0]) + S[0][2] * (S[1][0] * Y[2] - Y[1] * S[2][0])) / det;
    double a = (Y[0] - S[0][1] * b - S[0][2] * c) / S[0][0];
    char nm[64];
    snprintf(nm, sizeof nm, "%s_ay", name); SUM(nm, ays / n / 9.81, "g mean");
    snprintf(nm, sizeof nm, "%s_rim0", name); SUM(nm, th0 * DEG, "deg");
    snprintf(nm, sizeof nm, "%s_tq_mean", name); SUM(nm, a, "Nm");
    snprintf(nm, sizeof nm, "%s_tq_grad", name); SUM(nm, b / (A * DEG), "Nm/deg in phase with angle");
    snprintf(nm, sizeof nm, "%s_fric_eq", name); SUM(nm, M_PI * c / 4, "Nm equiv Coulomb at rim (pi c/4)");
    snprintf(nm, sizeof nm, "%s_rack_fric", name); SUM(nm, rf / n, "N mean |rack friction|");
    snprintf(nm, sizeof nm, "%s_col_fric", name); SUM(nm, cf / n, "Nm mean |column friction|");
}
static void m_hyst(const char* dir) { hyst_case(dir, "hyst0g", 0); hyst_case(dir, "hyst06g", 0.6); }

int main(int argc, char** argv) {
    if (argc < 3) { fprintf(stderr, "usage: mbcompare car.ini outdir [solver] [manoeuvre|all] [tag]\n"); return 1; }
    const char* dir = argv[2]; int solver = argc > 3 ? atoi(argv[3]) : 0; const char* what = argc > 4 ? argv[4] : "all";
    tag = argc > 5 ? argv[5] : "run";
    char err[256]; dc_default_params(&P);
    if (!dc_load_params(argv[1], &P, err, sizeof err)) { fprintf(stderr, "%s\n", err); return 1; }
#ifndef OLDCORE
    P.solver = solver;
#else
    (void)solver;
#endif
    const char* ps = getenv("PSET");
    if (ps && *ps) { static char buf[8192]; snprintf(buf, sizeof buf, "%s", ps);
        for (char* tok = strtok(buf, ";"); tok; tok = strtok(NULL, ";")) { char* dot = strchr(tok, '.'); char* eq = strchr(tok, '='); if (!dot || !eq) continue; *dot = 0; *eq = 0;
            if (!dc_set_param(&P, tok, dot + 1, eq + 1)) fprintf(stderr, "bad PSET %s.%s\n", tok, dot + 1); } }
    char cmd[600]; snprintf(cmd, sizeof cmd, "mkdir -p '%s'", dir); if (system(cmd)) {}
#define RUN(nm, fn) if (!strcmp(what, "all") || !strcmp(what, nm)) fn(dir)
    RUN("settle", m_settle); RUN("bounce", m_bounce); RUN("skidpad", m_skidpad); RUN("step", m_step); RUN("liftoff", m_liftoff);
    RUN("handsoff", m_handsoff); RUN("bump", m_bump); RUN("slalom", m_slalom); if (!strcmp(what, "hyst")) m_hyst(dir);
    SUM("health_min_step_dissipation", H_DMIN, "J (all manoeuvres; < 0 = energy created)");
    printf("%-10s %-28s %12s t=%.3f s\n", tag, "health_min_step_at", H_DMIN_M, H_DMIN_T);
    SUM("health_valve_pmax", H_PMAX * 1e-6, "MPa peak |valve pressure|");
    SUM("health_valve_nfail", H_NFAIL, "valve Newton non-convergences (max per run)");
    SUM("cpu_per_sim_s", CPU / fmax(SIMT, 1e-9), "(last manoeuvre normalised; see total)");
    return 0;
}
