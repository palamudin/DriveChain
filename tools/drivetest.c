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
static int RIG_ROAD = 0;   /* 0 flat plane, 1 proving-ground road */
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
    if (!RIG_ROAD) dc_set_road(R->S.car, flat_road, NULL);
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
    dc_destroy(R->S.car); R->S.car = dc_create(&R->S.P);
    if (!RIG_ROAD) dc_set_road(R->S.car, flat_road, NULL); else dc_set_road(R->S.car, track_road_fn, R->S.trk);
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


/* ---------------- road-disturbance kickback ---------------- */
typedef struct { const char* name; int where; double u0, u1, v; int feat; } kev_t;   /* where: 0 straight1 (s), 1 straight2 (u = s - s_str2), 2 west arc (s - s_arc2) */
static const kev_t KEV[] = {
    {"bump_left", 1, 119.3, 120.7, 0, PGF_BUMP_L}, {"bump_right", 1, 169.3, 170.7, 0, PGF_BUMP_R},
    {"pothole_left", 1, 219.1, 220.9, 0, PGF_POT_L}, {"pothole_right", 1, 269.1, 270.9, 0, PGF_POT_R},
    {"crossfall_change", 1, 320, 400, 0, PGF_CROSSFALL}, {"ruts", 1, 440, 530, 0, PGF_RUTS},
    {"undul_25m", 0, 340, 440, 0, PGF_UNDUL25}, {"undul_9m", 0, 450, 504, 0, PGF_UNDUL9}, {"undul_3m_twist", 0, 508, 540, 0, PGF_TWIST},
    {"bumpy_corner", 2, 40, 160, 19.4, PGF_BUMPYARC},
};
#define NKEV ((int)(sizeof KEV / sizeof KEV[0]))
typedef struct { const char* name; int kind; double delay, k, d, tmax; } kdrv_t;
enum { KD_ARMS, KD_HANDSOFF, KD_LOCKED, KD_GP };
static const kdrv_t KDRV[] = {
    {"arms20", KD_ARMS, 0.20, 40, 1.5, 20}, {"arms15_slow", KD_ARMS, 0.25, 30, 1.2, 15}, {"arms25_quick", KD_ARMS, 0.15, 60, 2.0, 25},
    {"hands_off", KD_HANDSOFF, 0, 0, 0, 0}, {"locked", KD_LOCKED, 0, 3000, 30, 500}, {"gamepad_arms", KD_GP, 0.20, 0, 0, 0},
};
#define NKDRV ((int)(sizeof KDRV / sizeof KDRV[0]))
typedef struct {
    double col_pk, rim_pk, rim_rate_pk, hand_pk; int hand_sat;
    double dpsi_recentre, dpsi_pk, dlat_end, t_recover;
    double imp[5];   /* attribution impulse: integral |dM| dt over entry .. exit + 0.5 s (N m s) */
    double m_fy, m_fx, m_mz, m_fz, m_gyro, toe_pk;   /* peak |delta| kingpin moments (sum of both fronts) N m, toe deg */
    double fz_pk;
} kres_t;
static double s_from_ev(const track_t* T, const kev_t* e, double u) { return e->where == 0 ? u : (e->where == 1 ? T->s_str2 + u : T->s_arc2 + u); }
static double lane_wrap_s(const track_t* T, double ds) { while (ds > T->length / 2) ds -= T->length; while (ds < -T->length / 2) ds += T->length; return ds; }

static void run_kick(rig_t* R, const kev_t* E, double v_set, const kdrv_t* D, const char* csv, kres_t* K) {
    sim_t* S = &R->S; const track_t* T = R->T;
    const double dt = 0.001;
    memset(K, 0, sizeof *K); K->t_recover = -1;
    double s_in = s_from_ev(T, E, E->u0), s_out = s_from_ev(T, E, E->u1);
    double s_lim = E->where == 0 ? T->s_arc1 - 35 : (E->where == 1 ? T->s_arc2 - 35 : T->s_arc2 + 440);
    track_isolate_feature = E->feat;   /* only this feature on the road: no neighbouring event inside the 4 s window */
    double s0 = s_in - 100; double x, y, yaw; track_point(T, s0, 0, &x, &y, &yaw);
    sim_reset_at(S, x, y, yaw, v_set); S->gear = v_set < 15 ? 2 : (v_set < 23 ? 3 : 4);
    S->ext_pedals = 1; S->ext_steer = 1; S->dev.kind = 0; R->H.n = 0; R->stick_f = 0;
    double Is = S->P.axle[0].tyre.spin_inertia;
    FILE* f = csv ? fopen(csv, "w") : NULL;
    if (f) fprintf(f, "t,ds_front,sw_deg,sw_rate_dps,column_nm,driver_nm,head_err_deg,lat_m,fzL,fzR,M_Fy,M_Fx,M_Mz,M_Fz,M_gyro,toeL_deg,toeR_deg\n");
    double ti = 0, t_in = -1, t_out = -1, lock_sw = 0, ax_prev[2] = {0, 0}; int have_ax = 0;
    /* baseline accumulators (0.5 s before entry) */
    double b_n = 0, b_sw = 0, b_col = 0, b_hd = 0, b_lat = 0, b_drv = 0, b_m[5] = {0}, b_toe[2] = {0};
    int have_base = 0; double base_sw = 0, base_col = 0, base_hd = 0, base_lat = 0, base_drv = 0, base_m[5] = {0}, base_toe[2] = {0};
    double t_rim_pk = -1, ring[600]; int nring = 0; (void)ring; (void)nring;
    double tmax_run = 0; int capped = 0;
    for (double tt = 0; tt < 60; tt += dt) {
        dc_telemetry t; dc_get_telemetry(S->car, &t);
        dc_xform ch[DC_PART_COUNT]; dc_get_parts(S->car, ch, DC_PART_COUNT);
        double cyaw = yaw_of(&ch[0]);
        double fxp = ch[0].pos[0] + R->a_cg * cos(cyaw), fyp = ch[0].pos[1] + R->a_cg * sin(cyaw);
        double dlat, sc; track_height(T, ch[0].pos[0], ch[0].pos[1], NULL, NULL, &dlat, &sc);
        double dlf, sf; track_height(T, fxp, fyp, NULL, NULL, &dlf, &sf);
        double tx, ty, tyaw; track_point(T, sc, 0, &tx, &ty, &tyaw);
        double hd = wrap(cyaw - tyaw), v = fmax(t.vx, 1);
        hpush(&R->H, tt, t.yaw_rate, 0, v, hd, dlat);
        double ds_in = lane_wrap_s(T, sf - s_in), ds_out = lane_wrap_s(T, sc - (R->wb - R->a_cg) - s_out);   /* rear axle past exit */
        if (t_in < 0 && ds_in >= 0) t_in = tt;
        if (t_in >= 0 && t_out < 0 && ds_out >= 0) t_out = tt;
        if (t_out >= 0 && tt > t_out + 4.0) break;
        if (t_in >= 0 && lane_wrap_s(T, sf - s_lim) >= 0) { capped = 1; break; }   /* stop before the next bend's bank ramp: a straight-line driver cannot be judged in a bend */
        /* curvature feed-forward from the centreline */
        double xa, ya, ya1, xb, yb, yb1; track_point(T, sc - 2, 0, &xa, &ya, &ya1); track_point(T, sc + 2, 0, &xb, &yb, &yb1); (void)xa; (void)ya; (void)xb; (void)yb;
        double kap = wrap(yb1 - ya1) / 4.0;
        /* speed hold */
        double e = v_set - t.vx; ti = clampd(ti + e * dt * 0.3, -0.3, 0.6);
        S->ext_in.throttle = clampd(0.2 + 0.5 * e + ti, 0, 1); S->ext_in.brake = clampd(-0.3 * e, 0, 0.5);
        /* lane-keeping intent on DELAYED perception (preview / pure-pursuit form) */
        int k = hget(&R->H, D->delay, dt);
        double L = 6 + 0.8 * v, hdd = R->H.head[k], ld = R->H.lat[k];
        double delta = kap * R->wb - 2 * R->wb * (ld + L * sin(hdd)) / (L * L);
        double sw_cmd = clampd(delta * R->ratio, -R->lock, R->lock);
        int ev_phase = t_in >= 0;
        if (!ev_phase) {   /* approach: everybody drives with a good grip so they enter identically */
            S->ext_steer = 1; S->ext_in.steer_mode = DC_STEER_ANGLE; S->ext_in.steer_target = sw_cmd;
            S->ext_in.hands_k = 40; S->ext_in.hands_d = 1.5; S->ext_in.hands_tmax = 20;
            lock_sw = t.sw_angle;
            if (tt > 0 && sf > s_in - 0.3 * v) {   /* baseline window: last 0.3 s before entry */
                double mfx = 0, mfy = 0, mmz = 0, mfz = 0, mg = 0;
                for (int w = 0; w < 2; w++) { const dc_wheel_telem* W = &t.w[w]; double sd = w == 0 ? 1 : -1;
                    mfy += -W->fy * W->trail; mfx += -sd * W->fx * W->scrub; mmz += W->mz; mfz += -W->fz * (W->scrub * sin(W->kpi) + W->trail * sin(W->caster)) * sin(W->steer); }
                b_m[0] += mfy; b_m[1] += mfx; b_m[2] += mmz; b_m[3] += mfz; b_m[4] += mg;
                b_sw += t.sw_angle; b_col += t.column_torque; b_hd += hd; b_lat += dlat; b_drv += t.driver_torque;
                b_toe[0] += t.w[0].toe; b_toe[1] += t.w[1].toe; b_n++;
            }
        } else {
            if (!have_base) { have_base = 1; double n = fmax(b_n, 1);
                base_sw = b_sw / n; base_col = b_col / n; base_hd = b_hd / n; base_lat = b_lat / n; base_drv = b_drv / n;
                for (int q = 0; q < 5; q++) base_m[q] = b_m[q] / n; base_toe[0] = b_toe[0] / n; base_toe[1] = b_toe[1] / n; }
            switch (D->kind) {
            case KD_ARMS: S->ext_steer = 1; S->ext_in.steer_mode = DC_STEER_ANGLE; S->ext_in.steer_target = sw_cmd;
                S->ext_in.hands_k = D->k; S->ext_in.hands_d = D->d; S->ext_in.hands_tmax = D->tmax; break;
            case KD_HANDSOFF: S->ext_steer = 1; S->ext_in.steer_mode = DC_STEER_TORQUE; S->ext_in.steer_torque = 0; break;
            case KD_LOCKED: S->ext_steer = 1; S->ext_in.steer_mode = DC_STEER_ANGLE; S->ext_in.steer_target = lock_sw;
                S->ext_in.hands_k = D->k; S->ext_in.hands_d = D->d; S->ext_in.hands_tmax = D->tmax; break;
            case KD_GP: { S->ext_steer = 0; S->dev.kind = 2;
                double want = stick_for(R, sw_cmd, v);
                R->stick_f += (want - R->stick_f) * (1 - exp(-dt * 2 * M_PI * 6.0));
                S->dev.steer = clampd(R->stick_f, -1, 1); break; }
            }
            /* attribution: moments about each front kingpin (+ = turns the wheel left), minus pre-event equilibrium */
            double mfx = 0, mfy = 0, mmz = 0, mfz = 0, mg = 0;
            for (int w = 0; w < 2; w++) { const dc_wheel_telem* W = &t.w[w]; double sd = w == 0 ? 1 : -1;
                mfy += -W->fy * W->trail; mfx += -sd * W->fx * W->scrub; mmz += W->mz;
                mfz += -W->fz * (W->scrub * sin(W->kpi) + W->trail * sin(W->caster)) * sin(W->steer);
                /* gyroscopic: H = Is w a (a = axle unit vector); reaction on the knuckle -dH/dt; its component about the
                   steer axis (~chassis z) is -Is w (da/dt . z). a is taken in the chassis frame from the wheel's pose. */
                { const dc_xform* X = &ch[DC_PART_WHEEL0 + w]; double q0 = X->quat[0], q1 = X->quat[1], q2 = X->quat[2], q3 = X->quat[3];
                  double ay_w[3] = {2 * (q1 * q2 - q0 * q3), 1 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 + q0 * q1)};   /* R e_y */
                  const dc_xform* C0 = &ch[0]; double c0 = C0->quat[0], c1 = C0->quat[1], c2 = C0->quat[2], c3 = C0->quat[3];
                  double zc[3] = {2 * (c1 * c3 + c0 * c2), 2 * (c2 * c3 - c0 * c1), 1 - 2 * (c1 * c1 + c2 * c2)};      /* chassis z */
                  double az = ay_w[0] * zc[0] + ay_w[1] * zc[1] + ay_w[2] * zc[2];
                  if (have_ax) mg += -Is * W->omega * (az - ax_prev[w]) / dt;
                  ax_prev[w] = az; }
                K->fz_pk = fmax(K->fz_pk, W->fz);
                K->toe_pk = fmax(K->toe_pk, fabs(W->toe - base_toe[w]) * DEG);
            }
            have_ax = 1;
            double dm[5] = {mfy - base_m[0], mfx - base_m[1], mmz - base_m[2], mfz - base_m[3], mg};
            K->m_fy = fmax(K->m_fy, fabs(dm[0])); K->m_fx = fmax(K->m_fx, fabs(dm[1])); K->m_mz = fmax(K->m_mz, fabs(dm[2]));
            K->m_fz = fmax(K->m_fz, fabs(dm[3])); K->m_gyro = fmax(K->m_gyro, fabs(dm[4]));
            if (t_out < 0 || tt < t_out + 0.5) for (int q = 0; q < 5; q++) K->imp[q] += fabs(dm[q]) * dt;
            double rim = fabs(t.sw_angle - base_sw) * DEG;
            if (rim > K->rim_pk) { K->rim_pk = rim; t_rim_pk = tt; }
            K->col_pk = fmax(K->col_pk, fabs(t.column_torque - base_col));
            K->rim_rate_pk = fmax(K->rim_rate_pk, fabs(t.sw_rate) * DEG);
            K->hand_pk = fmax(K->hand_pk, fabs(t.driver_torque - base_drv));
            if (D->kind == KD_ARMS && fabs(t.driver_torque) >= 0.98 * D->tmax) K->hand_sat = 1;
            if (D->kind == KD_GP && fabs(t.driver_torque) >= 0.98 * S->gp_tmax) K->hand_sat = 1;
            double dpsi = (hd - base_hd) * DEG;
            K->dpsi_pk = fabs(dpsi) > fabs(K->dpsi_pk) ? dpsi : K->dpsi_pk;
            if (t_rim_pk > 0 && K->dpsi_recentre == 0 && tt > t_rim_pk && rim < 1.0) K->dpsi_recentre = dpsi + 1e-9;
            /* recovery (s after the rear axle leaves the event): heading within 0.3 deg of the pre-event value, yaw rate within 0.5 deg/s of the path, rim within 2 deg, held 1 s */
            if (t_out >= 0) {
                if (fabs(dpsi) < 0.3 && fabs(t.yaw_rate) * DEG < fabs(kap * v) * DEG + 0.5 && rim < 2.0) { if (K->t_recover < 0) K->t_recover = tt; }
                else K->t_recover = -1;
            }
            K->dlat_end = dlat - base_lat; tmax_run = tt;
            if (f && ((int)(tt / dt + 0.5)) % 2 == 0)
                fprintf(f, "%.3f,%.3f,%.3f,%.1f,%.3f,%.3f,%.4f,%.4f,%.0f,%.0f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f\n", tt - t_in, lane_wrap_s(T, sf - s_in),
                        (t.sw_angle - base_sw) * DEG, t.sw_rate * DEG, t.column_torque, t.driver_torque, dpsi, dlat - base_lat,
                        t.w[0].fz, t.w[1].fz, dm[0], dm[1], dm[2], dm[3], dm[4], (t.w[0].toe - base_toe[0]) * DEG, (t.w[1].toe - base_toe[1]) * DEG);
        }
        sim_tick(S, dt);
    }
    if (f) fclose(f);
    track_isolate_feature = -1;
    if (K->t_recover >= 0) { if (tmax_run - K->t_recover >= (capped ? 0.5 : 1.0))   /* window capped before a bend: accept a 0.5 s hold */ K->t_recover = fmax(0, K->t_recover - t_out); else K->t_recover = -1; }
}

/* ---- fishtail-settle regression (make fishtest): exit code != 0 on failure ----
   Reference "before" = legacy raw gamepad path + no orifice back-drive (results/fishtail/default_car_before.txt):
   gp_release lift_60: 3 swings zeta 0.16 settle 1.46 s; lift_85: 7 swings zeta -0.13 settle 6.71 s (beta 38 deg). */
static int fishcheck(const char* car, const char* out) {
    static rig_t R; int fails = 0, known = 0;
    printf("fishtail-settle check (car %s)\n", car && *car ? car : "built-in default");
    for (int i = 0; i < NSCEN; i++) for (int j = 0; j < NDRV; j++) {
        const driver_t* D = &DRIVERS[j];
        int is_rel = D->kind == DRV_GP_RELEASE, is_hum = D->kind == DRV_GP_HUMAN, is_ide = D->kind == DRV_IDEAL, is_ho = D->kind == DRV_HANDSOFF;
        if (!(is_rel || is_hum || is_ide || is_ho)) continue;
        for (int raw = 0; raw <= (is_rel ? 1 : 0); raw++) {
            rig_init(&R, car, raw ? "raw" : NULL);
            if (raw) { R.S.P.assist_backdrive = 0; rig_rebuild(&R); }
            char csv[512]; snprintf(csv, sizeof csv, "%s/fishcheck_%s_%s%s.csv", out, SCEN[i].name, D->name, raw ? "_BEFORE" : "");
            metr_t M; run_fishtail(&R, &SCEN[i], D, csv, &M, 0);
            const char* verdict = "ok"; int bad = 0;
            if (raw) verdict = "(before, info)";
            else if (is_rel) bad = M.spin || M.swings > 1 || M.settle < 0 || M.settle > 1.0 || (M.swings >= 1 && M.zeta < 0.3);
            else if (is_hum) bad = M.spin || M.zeta < 0.05 || M.settle < 0 || M.settle > 2.5;
            else if (is_ide) bad = M.spin || M.swings > 1 || M.settle < 0 || M.settle > 0.6;
            else if (is_ho) {
                bad = M.spin || M.zeta < 0.1;
                if (bad && SCEN[i].v > 20) { bad = 0; known++; verdict = "KNOWN RESIDUAL (free wheel, 85 km/h)"; }
            }
            if (bad) { fails++; verdict = "FAIL"; }
            printf("  %-16s %-10s%s swings %d zeta %5.2f settle %5.2f s beta %5.1f %s  -> %s\n", SCEN[i].name, D->name, raw ? " RAW" : "    ",
                   M.swings, M.zeta, M.settle, M.peak_beta, M.spin ? "SPIN" : "    ", verdict);
        }
    }
    printf("fishtail-settle: %s (%d failures, %d known residuals)\n", fails ? "FAIL" : "PASS", fails, known);
    return fails ? 1 : 0;
}

int main(int argc, char** argv) {
    const char* what = argc > 1 ? argv[1] : "fishtail";
    const char* car = argc > 2 ? argv[2] : "";
    const char* out = argc > 3 ? argv[3] : "results/drivetest";
    const char* gpm = getenv("GP_MODE");
    char cmd[512]; snprintf(cmd, sizeof cmd, "mkdir -p %s", out); if (system(cmd)) {}
    track_build(&TRK, 1.0, TRACK_PROVING);
    static rig_t R;
    if (!strcmp(what, "fishcheck")) return fishcheck(car, out);
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
    if (!strcmp(what, "kickback") || !strcmp(what, "all")) {
        RIG_ROAD = 1;
        char path[512]; snprintf(path, sizeof path, "%s/kickback_summary.csv", out);
        FILE* s = fopen(path, "w");
        fprintf(s, "event,kmh,driver,col_pk_nm,rim_pk_deg,rim_rate_pk_dps,hand_pk_nm,hand_saturated,dpsi_at_recentre_deg,dpsi_pk_deg,dlat_end_m,recover_s,M_Fy_trail,M_Fx_scrub,M_Mz,M_Fz_jack,M_gyro,toe_pk_deg,fz_pk_n,I_Fy_trail,I_Fx_scrub,I_Mz,I_Fz_jack,I_gyro\n");
        const char* only = getenv("DRV"); const char* sel = getenv("SC");
        double speeds[2] = {60 / 3.6, 100 / 3.6};
        for (int i = 0; i < NKEV; i++) for (int vi = 0; vi < 2; vi++) for (int j = 0; j < NKDRV; j++) {
            if (only && !strstr(only, KDRV[j].name)) continue;
            if (sel && !strstr(sel, KEV[i].name)) continue;
            double v = KEV[i].v > 0 ? KEV[i].v : speeds[vi];
            if (KEV[i].v > 0 && vi) continue;
            rig_init(&R, car, gpm); R.T = &TRK;
            char csv[512]; snprintf(csv, sizeof csv, "%s/kick_%s_%.0f_%s.csv", out, KEV[i].name, v * 3.6, KDRV[j].name);
            kres_t K; run_kick(&R, &KEV[i], v, &KDRV[j], csv, &K);
            printf("  %-16s %3.0f %-13s col %5.2f  rim %6.2f deg (%5.0f dps)  hand %5.2f%s  dpsi@ctr %6.2f pk %6.2f deg  dlat %6.3f m  rec %5.2f s | Fy*t %5.1f Fx*s %5.1f Mz %5.1f Fz %5.1f gyro %5.2f toe %.3f | imp Fy %.3f Fx %.3f Mz %.3f gyro %.3f\n",
                   KEV[i].name, v * 3.6, KDRV[j].name, K.col_pk, K.rim_pk, K.rim_rate_pk, K.hand_pk, K.hand_sat ? "*SAT" : "    ", K.dpsi_recentre, K.dpsi_pk, K.dlat_end, K.t_recover,
                   K.m_fy, K.m_fx, K.m_mz, K.m_fz, K.m_gyro, K.toe_pk, K.imp[0], K.imp[1], K.imp[2], K.imp[4]);
            fprintf(s, "%s,%.0f,%s,%.3f,%.3f,%.1f,%.3f,%d,%.3f,%.3f,%.4f,%.3f,%.2f,%.2f,%.2f,%.2f,%.3f,%.4f,%.0f,%.4f,%.4f,%.4f,%.4f,%.4f\n", KEV[i].name, v * 3.6, KDRV[j].name,
                    K.col_pk, K.rim_pk, K.rim_rate_pk, K.hand_pk, K.hand_sat, K.dpsi_recentre, K.dpsi_pk, K.dlat_end, K.t_recover, K.m_fy, K.m_fx, K.m_mz, K.m_fz, K.m_gyro, K.toe_pk, K.fz_pk, K.imp[0], K.imp[1], K.imp[2], K.imp[3], K.imp[4]);
            fflush(stdout);
        }
        fclose(s);
    }
    return 0;
}
