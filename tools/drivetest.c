/* drivetest.c - headless closed-loop driver harness (fishtail / slide-catch and road-disturbance kickback).
   Runs the EXACT demo tick (demo/sim.c, incl. its gamepad mapping) with a scripted device or an external driver model.
   Build: make drivetest   Run: ./build/drivetest [fishtail|kickback|all] [car.ini] [outdir]                        */
#include "../demo/sim.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG (180.0 / M_PI)
static double clampd(double v, double a, double b) { return v < a ? a : (v > b ? b : v); }
static double flat_road(void* u, double x, double y, double* nx, double* ny, double* nz, double* g) { *nx = 0; *ny = 0; *nz = 1; *g = 1; return 0; }
static double yaw_of(const dc_xform* c) { double q0 = c->quat[0], q1 = c->quat[1], q2 = c->quat[2], q3 = c->quat[3]; return atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)); }
static double wrap(double a) { while (a > M_PI) a -= 2 * M_PI; while (a < -M_PI) a += 2 * M_PI; return a; }

/* ---------------- driver models ---------------- */
enum { DRV_HANDSOFF, DRV_LOCKED, DRV_IDEAL, DRV_ARMS, DRV_GP_HOLD, DRV_GP_HUMAN, DRV_GP_RELEASE };
typedef struct {
    const char* name; int kind;
    double delay;          /* s, perception/reaction delay of the outer loop */
    double k, d, tmax;     /* hands: grip stiffness N m/rad, damping N m s/rad, max hand torque N m */
} driver_t;
#define HIST 4096
typedef struct { double t[HIST], r[HIST], beta[HIST], v[HIST], head[HIST], lat[HIST]; int n; } hist_t;
static void hpush(hist_t* H, double t, double r, double b, double v, double hd, double lat) { int i = H->n++ % HIST; H->t[i] = t; H->r[i] = r; H->beta[i] = b; H->v[i] = v; H->head[i] = hd; H->lat[i] = lat; }
static int hget(const hist_t* H, double delay, double dt) { int back = (int)(delay / dt + 0.5); if (back >= H->n) back = H->n - 1; if (back > HIST - 1) back = HIST - 1; return (H->n - 1 - back) % HIST; }

typedef struct {
    sim_t S; cfg_t cfg; track_t* T;
    double a_cg, wb, ratio, lock;     /* cg to front axle, wheelbase, overall steer ratio, lock (rad sw) */
    hist_t H;
    double stick_f;                  /* human thumb, low-passed */
} rig_t;

static track_t TRK;
static void rig_rebuild(rig_t* R);
static void rig_init(rig_t* R, const char* carini, const char* gp_mode) {
    memset(R, 0, sizeof *R);
    cfg_set(&R->cfg, "autopilot.enabled", "0");
    if (carini && *carini) cfg_set(&R->cfg, "car.ini", carini);
    if (gp_mode) cfg_set(&R->cfg, "gamepad.mode", gp_mode);
    { const char* cs = getenv("CFGSET");   /* "gamepad.arms_k=10;gamepad.arms_d=1" */
      if (cs && *cs) { char buf[1024]; snprintf(buf, sizeof buf, "%s", cs);
          for (char* tok = strtok(buf, ";"); tok; tok = strtok(NULL, ";")) { char* eq = strchr(tok, '='); if (eq) { *eq = 0; cfg_set(&R->cfg, tok, eq + 1); } } } }
    char err[256];
    if (!sim_init(&R->S, &R->cfg, &TRK, NULL, err, sizeof err)) { fprintf(stderr, "sim_init: %s\n", err); exit(1); }
    dc_set_road(R->S.car, flat_road, NULL);
    const char* ps = getenv("PSET");   /* "car.key=val;front.key=val" parameter variants for attribution runs */
    if (ps && *ps) {
        char buf[1024]; snprintf(buf, sizeof buf, "%s", ps);
        for (char* tok = strtok(buf, ";"); tok; tok = strtok(NULL, ";")) {
            char* dot = strchr(tok, '.'); char* eq = strchr(tok, '='); if (!dot || !eq) continue; *dot = 0; *eq = 0;
            if (!strcmp(tok, "axle")) { dc_set_param(&R->S.P, "front", dot + 1, eq + 1); dc_set_param(&R->S.P, "rear", dot + 1, eq + 1); }
            else if (!dc_set_param(&R->S.P, tok, dot + 1, eq + 1)) fprintf(stderr, "bad PSET %s.%s\n", tok, dot + 1);
        }
        rig_rebuild(R);
    }
    R->a_cg = R->S.P.axle[0].hub[0]; R->wb = R->S.P.axle[0].hub[0] - R->S.P.axle[1].hub[0];
    R->ratio = R->S.P.steer_ratio; R->lock = R->S.P.steer_lock_deg / DEG;
}
static void rig_rebuild(rig_t* R) {   /* after editing R->S.P */
    dc_destroy(R->S.car); R->S.car = dc_create(&R->S.P); dc_set_road(R->S.car, flat_road, NULL);
}

/* the "catch" law: align the front wheels with the front-axle velocity (zero front slip), damp yaw, hold heading */
static double catch_law(const rig_t* R, double r, double beta, double v, double head_err, double lat_err, double kr) {
    v = fmax(v, 3);
    double d_road = beta + R->a_cg * r / v - kr * r - 0.25 * head_err - 0.0 * lat_err;
    return d_road * R->ratio;          /* steering-wheel angle */
}
/* gamepad stick that a human who knows the mapping would push to ask for steering-wheel angle sw (demo raw mapping) */
static double stick_for(const rig_t* R, double sw, double v) {   /* inverse of the demo's stick -> rim-target map */
    const sim_t* S = &R->S;
    double lo = S->gp_mode ? S->gp_range_lo : 0.25, vv = S->gp_mode ? S->gp_range_v : 12, e = S->gp_mode ? S->gp_expo : 0.5;
    double lim = S->P.steer_lock_deg / DEG * (lo + (1 - lo) / (1 + v / vv));
    double y = clampd(sw / lim, -1, 1), ay = fabs(y);
    double x = e > 1e-9 ? (-(1 - e) + sqrt((1 - e) * (1 - e) + 4 * e * ay)) / (2 * e) : ay;   /* (1-e)x + e x^2 = |y| */
    x = clampd(x, 0, 1);
    if (S->gp_mode) { double dz = S->gp_deadzone; if (x > 0) x = dz + x * (1 - dz); }
    return copysign(x, y);
}

typedef struct { int swings, spin; double zeta, settle, peak_r, peak_beta, peak_sw_rate, peak_col, rms_drv; double p[8]; int np; } metr_t;
static void metrics(const double* t, const double* r, const double* b, const double* swr, const double* col, const double* drv, int n, metr_t* M) {
    memset(M, 0, sizeof *M);
    double thr = 4.0;   /* deg/s */
    int sgn = 0; double pk = 0;
    for (int i = 0; i < n; i++) {
        double x = r[i] * DEG; int s = x > 0 ? 1 : (x < 0 ? -1 : 0);
        if (s && sgn && s != sgn) { if (pk > thr && M->np < 8) M->p[M->np++] = pk; pk = 0; }
        if (s) sgn = s;
        if (fabs(x) > pk) pk = fabs(x);
        if (fabs(r[i]) * DEG > M->peak_r) M->peak_r = fabs(r[i]) * DEG;
        if (fabs(b[i]) * DEG > M->peak_beta) M->peak_beta = fabs(b[i]) * DEG;
        if (fabs(swr[i]) * DEG > M->peak_sw_rate) M->peak_sw_rate = fabs(swr[i]) * DEG;
        if (fabs(col[i]) > M->peak_col) M->peak_col = fabs(col[i]);
        M->rms_drv += drv[i] * drv[i];
    }
    if (pk > thr && M->np < 8) M->p[M->np++] = pk;
    M->rms_drv = sqrt(M->rms_drv / n);
    M->swings = M->np > 0 ? M->np - 1 : 0;   /* reversals after the initial rotation */
    M->spin = M->peak_beta > 45;
    if (M->np >= 2) {   /* half-cycle log decrement over the reversals */
        double s = 0; int k = 0;
        for (int i = 0; i + 1 < M->np && i < 4; i++) { double d = log(M->p[i] / M->p[i + 1]); s += d; k++; }
        double d = s / k; M->zeta = d / sqrt(M_PI * M_PI + d * d); if (d < 0) M->zeta = -fabs(M->zeta);
    } else M->zeta = 1.0;   /* no reversal: >= critically damped */
    M->settle = -1;
    for (int i = 0; i < n; i++) {
        int ok = 1;
        for (int j = i; j < n && t[j] < t[i] + 1.0; j++) if (fabs(r[j]) * DEG > 3 || fabs(b[j]) * DEG > 2) { ok = 0; break; }
        if (ok && t[n - 1] - t[i] >= 1.0) { M->settle = t[i] - t[0]; break; }
    }
}

/* ---------------- fishtail scenarios ---------------- */
typedef struct { const char* name; double R, v; int power; double tev; } scen_t;
#define NREC 12000
static double RT[NREC], RR[NREC], RB[NREC], RSW[NREC], RSWR[NREC], RCOL[NREC], RDRV[NREC], RRV[NREC], RAS[NREC], RCMD[NREC], RST[NREC];
static int run_fishtail(rig_t* R, const scen_t* sc, const driver_t* D, const char* csv, metr_t* M, int verbose) {
    sim_t* S = &R->S;
    const double dt = 0.001, cx = 300, cy = 150;
    double x0 = cx, y0 = cy - sc->R;
    sim_reset_at(S, x0, y0, 0, sc->v * 0.9); S->gear = sc->v < 15 ? 2 : 3; if (sc->v < 9) S->gear = 1;
    S->ext_pedals = 1; S->ext_steer = 1; S->dev.kind = 0; S->dev.steer = 0;
    R->H.n = 0; R->stick_f = 0;
    double ti = 0, held_stick = 0, head_ref = 0; int nrec = 0;
    double tend = sc->tev + 9.0;
    FILE* f = csv ? fopen(csv, "w") : NULL;
    if (f) fprintf(f, "t,yaw_rate_dps,beta_deg,sw_deg,sw_rate_dps,column_nm,driver_nm,rack_vel,assist_n,sw_cmd_deg,stick,speed\n");
    for (double tt = 0; tt < tend; tt += dt) {
        dc_telemetry t; dc_get_telemetry(S->car, &t);
        dc_xform ch[DC_PART_COUNT]; dc_get_parts(S->car, ch, DC_PART_COUNT);
        double yaw = yaw_of(&ch[0]), v = fmax(t.vx, 0.5), beta = atan2(t.vy, fmax(t.vx, 0.5));
        hpush(&R->H, tt, t.yaw_rate, beta, v, yaw, 0);
        double sw_cmd = 0, stick = 0;
        if (tt < sc->tev) {
            /* setup driver: pure pursuit on the circle (CCW), stiff hands, PI speed */
            double px = ch[0].pos[0] - cx, py = ch[0].pos[1] - cy, ph = atan2(py, px), L = 5 + 0.35 * v;
            double tx = cx + sc->R * cos(ph + L / sc->R), ty = cy + sc->R * sin(ph + L / sc->R);
            double ex = tx - ch[0].pos[0], ey = ty - ch[0].pos[1];
            double lx = cos(yaw) * ex + sin(yaw) * ey, ly = -sin(yaw) * ex + cos(yaw) * ey;
            double delta = atan2(2 * R->wb * ly, lx * lx + ly * ly);
            sw_cmd = delta * R->ratio;
            S->ext_in.steer_mode = DC_STEER_ANGLE; S->ext_in.steer_target = sw_cmd; S->ext_in.hands_k = 300; S->ext_in.hands_d = 3; S->ext_in.hands_tmax = 80;
            double e = sc->v - t.vx; ti = clampd(ti + e * dt * 0.3, -0.3, 0.6);
            S->ext_in.throttle = clampd(0.25 + 0.6 * e + ti, 0, 1); S->ext_in.brake = clampd(-0.3 * e, 0, 0.5);
            held_stick = stick_for(R, t.sw_angle, v); head_ref = yaw;
            R->stick_f = held_stick;
        } else {
            double te = tt - sc->tev;
            /* pedals: lift-off = throttle to 0 over 50 ms; power = floor it 0.8 s, then back to 15 % */
            if (!sc->power) S->ext_in.throttle = fmax(0, S->ext_in.throttle - dt / 0.05);
            else S->ext_in.throttle = te < 0.8 ? 1.0 : 0.15;
            S->ext_in.brake = 0;
            if (te < 1e-9) head_ref = yaw;
            int k = hget(&R->H, D->delay, dt);
            double rd = R->H.r[k], bd = R->H.beta[k], vd = R->H.v[k], hd = wrap(R->H.head[k] - head_ref);
            /* the catch target: after the event the driver wants to stop the rotation; heading hold ramps in after 1.5 s */
            double hw = clampd((te - 1.5) / 1.5, 0, 1);
            sw_cmd = catch_law(R, rd, bd, vd, hw * hd * 0 + 0, 0, 0.10);
            (void)hd;
            switch (D->kind) {
            case DRV_HANDSOFF: S->ext_steer = 1; S->ext_in.steer_mode = DC_STEER_TORQUE; S->ext_in.steer_torque = 0; break;
            case DRV_LOCKED: S->ext_steer = 1; S->ext_in.steer_mode = DC_STEER_ANGLE; S->ext_in.steer_target = RSW[nrec > 0 ? 0 : 0]; S->ext_in.hands_k = 3000; S->ext_in.hands_d = 30; S->ext_in.hands_tmax = 500; break;
            case DRV_IDEAL: case DRV_ARMS:
                S->ext_steer = 1; S->ext_in.steer_mode = DC_STEER_ANGLE; S->ext_in.steer_target = clampd(sw_cmd, -R->lock, R->lock);
                S->ext_in.hands_k = D->k; S->ext_in.hands_d = D->d; S->ext_in.hands_tmax = D->tmax; break;
            case DRV_GP_HOLD: S->ext_steer = 0; S->dev.kind = 2; S->dev.steer = held_stick; stick = held_stick; break;
            case DRV_GP_RELEASE: S->ext_steer = 0; S->dev.kind = 2; S->dev.steer = 0; break;
            case DRV_GP_HUMAN: {
                S->ext_steer = 0; S->dev.kind = 2;
                double want = stick_for(R, sw_cmd, v);
                R->stick_f += (want - R->stick_f) * (1 - exp(-dt * 2 * M_PI * 6.0));  /* thumb bandwidth ~6 Hz */
                S->dev.steer = clampd(R->stick_f, -1, 1); stick = S->dev.steer; break; }
            }
            if (D->kind == DRV_LOCKED && te < 1e-9) { S->ext_in.steer_target = t.sw_angle; }
            if (D->kind == DRV_LOCKED) { static double lk; if (te < 1e-9) lk = t.sw_angle; S->ext_in.steer_target = lk; }
            if (nrec < NREC) {
                RT[nrec] = tt; RR[nrec] = t.yaw_rate; RB[nrec] = beta; RSW[nrec] = t.sw_angle; RSWR[nrec] = t.sw_rate; RCOL[nrec] = t.column_torque;
                RDRV[nrec] = t.driver_torque; RRV[nrec] = t.rack_vel; RAS[nrec] = t.assist_force; RCMD[nrec] = sw_cmd; RST[nrec] = stick; nrec++;
            }
        }
        if (f && ((int)(tt / dt + 0.5)) % 5 == 0)
            fprintf(f, "%.3f,%.3f,%.3f,%.2f,%.1f,%.3f,%.3f,%.4f,%.1f,%.2f,%.3f,%.2f\n", tt, t.yaw_rate * DEG, beta * DEG, t.sw_angle * DEG, t.sw_rate * DEG,
                    t.column_torque, t.driver_torque, t.rack_vel, t.assist_force, sw_cmd * DEG, stick, t.vx * 3.6);
        sim_tick(S, dt);
    }
    if (f) fclose(f);
    metrics(RT, RR, RB, RSWR, RCOL, RDRV, nrec, M);
    if (verbose) printf("  %-14s %-12s swings %d  zeta %5.2f  settle %5.2f s  peak r %5.1f dps  beta %5.1f  sw_rate %6.0f dps  col %5.2f Nm  %s\n",
                        sc->name, D->name, M->swings, M->zeta, M->settle, M->peak_r, M->peak_beta, M->peak_sw_rate, M->peak_col, M->spin ? "SPIN" : "");
    return nrec;
}

static const driver_t DRIVERS[] = {
    {"hands_off", DRV_HANDSOFF, 0, 0, 0, 0},
    {"locked", DRV_LOCKED, 0, 0, 0, 0},
    {"ideal", DRV_IDEAL, 0.0, 300, 3, 80},
    {"arms", DRV_ARMS, 0.2, 40, 1.5, 20},
    {"gp_hold", DRV_GP_HOLD, 0, 0, 0, 0},
    {"gp_release", DRV_GP_RELEASE, 0, 0, 0, 0},
    {"gp_human", DRV_GP_HUMAN, 0.2, 0, 0, 0},
};
#define NDRV ((int)(sizeof DRIVERS / sizeof DRIVERS[0]))
static const scen_t SCEN[] = {
    {"lift_45kmh_R20", 20, 12.8, 0, 8},
    {"lift_60kmh_R32", 32, 16.6, 0, 8},
    {"lift_85kmh_R65", 65, 23.6, 0, 9},
    {"power_40kmh_R20", 20, 11.0, 1, 8},
    {"power_60kmh_R40", 40, 16.0, 1, 8},
};
#define NSCEN ((int)(sizeof SCEN / sizeof SCEN[0]))

int main(int argc, char** argv) {
    const char* what = argc > 1 ? argv[1] : "fishtail";
    const char* car = argc > 2 ? argv[2] : "";
    const char* out = argc > 3 ? argv[3] : "results/drivetest";
    const char* gpm = getenv("GP_MODE");
    char cmd[512]; snprintf(cmd, sizeof cmd, "mkdir -p %s", out); if (system(cmd)) {}
    track_build(&TRK, 1.0, TRACK_PROVING);
    static rig_t R;
    if (!strcmp(what, "fishtail") || !strcmp(what, "all")) {
        char path[512]; snprintf(path, sizeof path, "%s/fishtail_summary.csv", out);
        FILE* s = fopen(path, "w"); fprintf(s, "scenario,driver,swings,zeta,settle_s,peak_r_dps,peak_beta_deg,peak_sw_rate_dps,peak_col_nm,spin\n");
        double vs = getenv("VSCALE") ? atof(getenv("VSCALE")) : 1.0;
        const char* only = getenv("DRV");
        for (int i = 0; i < NSCEN; i++) for (int j = 0; j < NDRV; j++) {
            if (only && !strstr(only, DRIVERS[j].name)) continue;
            if (getenv("SC") && !strstr(getenv("SC"), SCEN[i].name)) continue;
            rig_init(&R, car, gpm);
            scen_t sc = SCEN[i]; sc.v *= vs;
            char csv[512]; snprintf(csv, sizeof csv, "%s/fish_%s_%s.csv", out, SCEN[i].name, DRIVERS[j].name);
            metr_t M; run_fishtail(&R, &sc, &DRIVERS[j], csv, &M, 1);
            fprintf(s, "%s,%s,%d,%.3f,%.3f,%.2f,%.2f,%.1f,%.3f,%d\n", SCEN[i].name, DRIVERS[j].name, M.swings, M.zeta, M.settle, M.peak_r, M.peak_beta, M.peak_sw_rate, M.peak_col, M.spin);
        }
        fclose(s);
    }
    return 0;
}
