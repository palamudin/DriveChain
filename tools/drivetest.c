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
        static char buf[16384]; snprintf(buf, sizeof buf, "%s", ps);
        for (char* tok = strtok(buf, ";"); tok; tok = strtok(NULL, ";")) {
            char* dot = strchr(tok, '.'); char* eq = strchr(tok, '='); if (!dot || !eq) continue; *dot = 0; *eq = 0;
            if (!strcmp(tok, "axle")) { dc_set_param(&R->S.P, "front", dot + 1, eq + 1); dc_set_param(&R->S.P, "rear", dot + 1, eq + 1); }
            else if (!dc_set_param(&R->S.P, tok, dot + 1, eq + 1)) fprintf(stderr, "bad PSET %s.%s\n", tok, dot + 1);
        }
        rig_rebuild(R);
    }
    if (getenv("DT_VERBOSE")) fprintf(stderr, "rig: solver param %d active %d\n", R->S.P.solver, dc_active_solver(R->S.car));
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
static double stick_for(const rig_t* R, double sw, double v) {   /* inverse of the demo's stick -> rim-target map (linear, full lock) */
    const sim_t* S = &R->S; (void)v;
    double x = clampd(sw / (S->P.steer_lock_deg / DEG), -1, 1), dz = S->gp_deadzone;
    if (fabs(x) > 1e-12) x = copysign(dz + fabs(x) * (1 - dz), x);
    return x;
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
typedef struct { const char* name; double R, v; int power; double tev, tpow, relb; int gear; } scen_t;
/* tpow: full-throttle time of a power event (0 = 0.8 s). relb > 0: "slide-out" event - the setup driver keeps holding the
   corner steer while the slide is provoked (power: full throttle; lift: throttle off + handbrake), and lets go (hands off /
   stick to zero) once |beta| > relb deg (or after 2.5 s); afterwards power keeps full throttle for tpow s then 15 %. */
static int REL_IDX = 0;
#define NREC 12000
static double RBF[NREC], RDL[NREC], RFS[NREC], RHT[NREC];   /* front-axle sideslip, road-wheel angle (mean FL/FR), front slip, hand torque */
static double RT[NREC], RR[NREC], RB[NREC], RSW[NREC], RSWR[NREC], RCOL[NREC], RDRV[NREC], RRV[NREC], RAS[NREC], RCMD[NREC], RST[NREC];
static int run_fishtail(rig_t* R, const scen_t* sc, const driver_t* D, const char* csv, metr_t* M, int verbose) {
    sim_t* S = &R->S;
    const double dt = 0.001, cx = 300, cy = 150;
    double x0 = cx, y0 = cy - sc->R;
    sim_reset_at(S, x0, y0, 0, sc->v * 0.9); S->gear = sc->v < 15 ? 2 : 3; if (sc->v < 9) S->gear = 1;
    S->ext_pedals = 1; S->ext_steer = 1; S->dev.kind = 0; S->dev.steer = 0;
    R->H.n = 0; R->stick_f = 0;
    double ti = 0, held_stick = 0, head_ref = 0, held_sw = 0, t_rel = 0; int nrec = 0, released = 0; REL_IDX = 0;
    if (sc->gear) S->gear = sc->gear;
    double tend = sc->tev + (sc->relb > 0 ? 7.0 : 9.0);
    FILE* f = csv ? fopen(csv, "w") : NULL;
    if (f) fprintf(f, "t,yaw_rate_dps,beta_deg,sw_deg,sw_rate_dps,column_nm,driver_nm,rack_vel,assist_n,sw_cmd_deg,stick,speed,beta_front_deg,delta_deg,front_slip_deg\n");
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
        } else if (sc->relb > 0 && !released) {   /* provoke the slide with the corner steer held */
            double te = tt - sc->tev;
            if (te < 1e-9) { held_sw = t.sw_angle; REL_IDX = 0; }
            S->ext_in.steer_mode = DC_STEER_ANGLE; S->ext_in.steer_target = held_sw; S->ext_in.hands_k = 300; S->ext_in.hands_d = 3; S->ext_in.hands_tmax = 80;
            if (sc->power) { S->ext_in.throttle = 1.0; S->ext_in.brake = 0; } else { S->ext_in.throttle = 0; S->ext_in.handbrake = 1; }
            held_stick = stick_for(R, t.sw_angle, v); R->stick_f = held_stick;
            if (fabs(beta) * DEG > sc->relb || te > 2.5) { released = 1; t_rel = tt; REL_IDX = nrec; S->ext_in.handbrake = 0; }
            if (nrec < NREC) {
                RT[nrec] = tt; RR[nrec] = t.yaw_rate; RB[nrec] = beta; RSW[nrec] = t.sw_angle; RSWR[nrec] = t.sw_rate; RCOL[nrec] = t.column_torque;
                RDRV[nrec] = t.driver_torque; RRV[nrec] = t.rack_vel; RAS[nrec] = t.assist_force; RCMD[nrec] = held_sw; RST[nrec] = 0;
                RBF[nrec] = atan2(t.vy + R->a_cg * t.yaw_rate, fmax(fabs(t.vx), 0.5)); RDL[nrec] = 0.5 * (t.w[0].steer + t.w[1].steer);
                RFS[nrec] = 0.5 * (t.w[0].slip_angle + t.w[1].slip_angle); RHT[nrec] = t.driver_torque; nrec++;
            }
        } else {
            double te = sc->relb > 0 ? tt - t_rel : tt - sc->tev;
            /* pedals: lift-off = throttle to 0 over 50 ms; power = floor it 0.8 s, then back to 15 % */
            if (!sc->power) S->ext_in.throttle = fmax(0, S->ext_in.throttle - dt / 0.05);
            else S->ext_in.throttle = te < (sc->tpow > 0 ? sc->tpow : 0.8) ? 1.0 : 0.15;
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
                RDRV[nrec] = t.driver_torque; RRV[nrec] = t.rack_vel; RAS[nrec] = t.assist_force; RCMD[nrec] = sw_cmd; RST[nrec] = stick;
                RBF[nrec] = atan2(t.vy + R->a_cg * t.yaw_rate, fmax(fabs(t.vx), 0.5)); RDL[nrec] = 0.5 * (t.w[0].steer + t.w[1].steer);
                RFS[nrec] = 0.5 * (t.w[0].slip_angle + t.w[1].slip_angle); RHT[nrec] = t.driver_torque; nrec++;
            }
        }
        if (f && ((int)(tt / dt + 0.5)) % 5 == 0)
            fprintf(f, "%.3f,%.3f,%.3f,%.2f,%.1f,%.3f,%.3f,%.4f,%.1f,%.2f,%.3f,%.2f,%.3f,%.3f,%.3f\n", tt, t.yaw_rate * DEG, beta * DEG, t.sw_angle * DEG, t.sw_rate * DEG,
                    t.column_torque, t.driver_torque, t.rack_vel, t.assist_force, sw_cmd * DEG, stick, t.vx * 3.6,
                    atan2(t.vy + R->a_cg * t.yaw_rate, fmax(fabs(t.vx), 0.5)) * DEG, 0.5 * (t.w[0].steer + t.w[1].steer) * DEG, 0.5 * (t.w[0].slip_angle + t.w[1].slip_angle) * DEG);
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


/* ---------------- countersteer tracking (hands off / zero stick) ----------------
   A free steering system should turn the front wheels into the front-axle velocity: caster trail x Fy (+ pneumatic trail,
   + the valve boosting that torque) spins the rim until the front lateral force ~ 0, i.e. road-wheel angle delta ~ front-axle
   sideslip beta_f = atan((vy + a r) / vx), until the rack stop. Metrics over the slide (|beta_f| > 4 deg):
   err = delta - clamp(beta_f, +-road-wheel lock); t_track = first time after onset with |err| < 3 deg for 0.2 s;
   rms/max err from onset + 0.3 s to the slide end; lag = shift of delta that best matches beta_f. */
typedef struct { double onset, t_track, rms_err, max_err, lag, max_bf, max_delta, max_rim, max_rim_rate, max_hand, end; int ok; } trk_t;
static void track_metrics(const rig_t* R, int n, double tev, trk_t* K) {
    memset(K, 0, sizeof *K); K->onset = K->t_track = K->end = -1;
    double dl = R->lock / R->ratio;   /* road-wheel lock (mean, rad) */
    int i0 = -1, i1 = -1;
    for (int i = REL_IDX; i < n; i++) if (fabs(RBF[i]) > 4 / DEG) { if (i0 < 0) i0 = i; i1 = i; }
    for (int i = REL_IDX; i < n; i++) {
        if (fabs(RBF[i]) * DEG > K->max_bf) K->max_bf = fabs(RBF[i]) * DEG;
        if (fabs(RDL[i]) * DEG > K->max_delta) K->max_delta = fabs(RDL[i]) * DEG;
        if (fabs(RSW[i]) * DEG > K->max_rim) K->max_rim = fabs(RSW[i]) * DEG;
        if (fabs(RSWR[i]) * DEG > K->max_rim_rate) K->max_rim_rate = fabs(RSWR[i]) * DEG;
        if (fabs(RHT[i]) > K->max_hand) K->max_hand = fabs(RHT[i]);
    }
    if (i0 < 0) return;
    K->onset = RT[i0] - RT[REL_IDX]; K->end = RT[i1] - RT[REL_IDX]; (void)tev;
    for (int i = i0; i <= i1; i++) {
        int ok = 1;
        for (int j = i; j <= i1 && RT[j] < RT[i] + 0.2; j++) if (fabs(RDL[j] - clampd(RBF[j], -dl, dl)) > 3 / DEG) { ok = 0; break; }
        if (ok) { K->t_track = RT[i] - RT[i0]; break; }
    }
    double s = 0; int m = 0;
    for (int i = i0; i <= i1; i++) if (RT[i] >= RT[i0] + 0.3) { double e = fabs(RDL[i] - clampd(RBF[i], -dl, dl)) * DEG; s += e * e; m++; if (e > K->max_err) K->max_err = e; }
    K->rms_err = m ? sqrt(s / m) : 0;
    double best = 1e30;
    for (int L = 0; L <= 500; L += 5) {   /* ms */
        double q = 0; int c = 0;
        for (int i = i0; i + L <= i1; i++) { double e = RDL[i + L] - clampd(RBF[i], -dl, dl); q += e * e; c++; }
        if (c > 50 && q / c < best) { best = q / c; K->lag = L * 1e-3; }
    }
    K->ok = K->t_track >= 0 && K->t_track < 0.3 && K->rms_err < 4;
}
static const scen_t CSCEN[] = {   /* slide-outs: provoke with the corner steer held, then let go at |beta| = relb */
    {"lift_40kmh_R20", 20, 11.4, 0, 8, 0, 8, 0},
    {"lift_60kmh_R32", 32, 16.6, 0, 8, 0, 8, 0},
    {"lift_85kmh_R65", 65, 23.6, 0, 9, 0, 8, 0},
    {"power_40kmh_R20", 20, 11.0, 1, 8, 1.5, 8, 2},
    {"power_60kmh_R40", 40, 16.6, 1, 8, 1.5, 8, 2},
    {"power_85kmh_R65", 65, 23.6, 1, 9, 1.5, 8, 3},
};
#define NCSCEN ((int)(sizeof CSCEN / sizeof CSCEN[0]))
static const driver_t CDRV[] = { {"hands_off", DRV_HANDSOFF, 0, 0, 0, 0}, {"gp_zero", DRV_GP_RELEASE, 0, 0, 0, 0} };
static int counter(const char* car, const char* out, const char* tag) {
    static rig_t R; int fails = 0;
    char path[512]; snprintf(path, sizeof path, "%s/countersteer_%s.csv", out, tag);
    FILE* s = fopen(path, "w");
    fprintf(s, "scenario,driver,gp_mode,onset_s,t_track_s,rms_err_deg,max_err_deg,lag_s,max_beta_front_deg,max_delta_deg,max_rim_deg,max_rim_rate_dps,max_hand_nm,slide_end_s,swings,zeta,peak_beta_deg,spin,pass\n");
    const char* gpm = getenv("GP_MODE");
    printf("countersteer tracking [%s] (car %s, gamepad mode %s, PSET %s)\n", tag, car && *car ? car : "default", gpm ? gpm : "ini", getenv("PSET") ? getenv("PSET") : "-");
    for (int i = 0; i < NCSCEN; i++) for (int j = 0; j < (int)(sizeof CDRV / sizeof CDRV[0]); j++) {
        if (getenv("SC") && !strstr(getenv("SC"), CSCEN[i].name)) continue;
        if (getenv("DRV") && !strstr(getenv("DRV"), CDRV[j].name)) continue;
        rig_init(&R, car, gpm);
        char csv[512]; snprintf(csv, sizeof csv, "%s/cs_%s_%s_%s.csv", out, tag, CSCEN[i].name, CDRV[j].name);
        metr_t M; int n = run_fishtail(&R, &CSCEN[i], &CDRV[j], csv, &M, 0);
        trk_t K; track_metrics(&R, n, CSCEN[i].tev, &K);
        if (!K.ok) fails++;
        printf("  %-16s %-9s onset %5.2f  track %5.2f s  rms %5.1f  max %5.1f deg  lag %4.2f s | beta_f %5.1f  delta %5.1f  rim %5.0f deg  %5.0f dps  hand %5.1f Nm | swings %d zeta %5.2f beta %5.1f %s %s\n",
               CSCEN[i].name, CDRV[j].name, K.onset, K.t_track, K.rms_err, K.max_err, K.lag, K.max_bf, K.max_delta, K.max_rim, K.max_rim_rate, K.max_hand,
               M.swings, M.zeta, M.peak_beta, M.spin ? "SPIN" : "    ", K.ok ? "ok" : "--");
        fprintf(s, "%s,%s,%s,%.3f,%.3f,%.2f,%.2f,%.3f,%.2f,%.2f,%.1f,%.0f,%.2f,%.3f,%d,%.3f,%.2f,%d,%d\n", CSCEN[i].name, CDRV[j].name, R.S.gp_mode ? "arms" : "raw",
                K.onset, K.t_track, K.rms_err, K.max_err, K.lag, K.max_bf, K.max_delta, K.max_rim, K.max_rim_rate, K.max_hand, K.end, M.swings, M.zeta, M.peak_beta, M.spin, K.ok);
        fflush(stdout);
    }
    fclose(s);
    return fails;
}


/* ---------------- sustained power drift (skidpad) + transition ----------------
   Closed-loop "human" drift driver: everything it sees is delayed by DR_DELAY (perception + neuromuscular).
   Steering: point the front wheels a small slip angle inside the front-axle velocity (delta = beta_f + alpha_des),
   alpha_des trims the path radius. Throttle: holds the body slip angle at beta* (PI + beta-rate damping), foot rate-limited.
   Hands: finite arms (k 40 N m/rad, d 1.5, tmax 20 N m) on the rim, or the demo's gamepad path (linear full-lock stick, 6 Hz thumb). */
typedef struct { double t, beta, bf, r, v, d, dbeta; } dsamp_t;
#define DHIST 1024
typedef struct { double bstar, R, v0, delay, hold_s, alpha0, kd_path, kp, ki, kdb; int gear, gp, transition; double kbs, krs, hb_s; } drift_cfg_t;
typedef struct { double hold, hold_t0, beta_m, beta_sd, delta_m, delta_max, rim_m, rim_max, af_m, fy_use_m, fy_use_max, comb_use_m, rear_sr_m, rear_sa_m, thr_m, thr_sd, v_m, rad_m, rpm_m, lsd_dw_m, req_m, req_max; int spin; double tr_ok, tr_time, tr_hold; } drift_res_t;
static void drift_stats(const double* T, const double* B, int n, double t0, double t1, double lo, double hi, double* best, double* bt0) {
    *best = 0; *bt0 = -1; double start = -1, lastin = -1;
    for (int i = 0; i < n; i++) {
        if (T[i] < t0 || T[i] > t1) continue;
        double b = fabs(B[i]) * DEG; int in = b >= lo && b <= hi;
        if (in) { if (start < 0 || T[i] - lastin > 0.3) start = T[i]; lastin = T[i]; if (lastin - start > *best) { *best = lastin - start; *bt0 = start; } }
    }
}
#define DREC 80000
static double DT_[DREC], DB_[DREC], DDL_[DREC], DSW_[DREC], DAF_[DREC], DFU_[DREC], DCU_[DREC], DSR_[DREC], DSA_[DREC], DTH_[DREC], DV_[DREC], DRAD_[DREC], DRPM_[DREC], DLSD_[DREC], DREQ_[DREC], DBF_[DREC];
static int run_drift(rig_t* R, const drift_cfg_t* C, const char* csv, drift_res_t* O) {
    sim_t* S = &R->S; const double dt = 0.001;
    double cx = 300, cy = 150, sgn_ = 1;       /* +1 = CCW (left-hand drift, beta < 0) */
    memset(O, 0, sizeof *O); O->tr_ok = 0; O->tr_time = -1;
    sim_reset_at(S, cx, cy - C->R, 0, C->v0 * 0.9); S->gear = C->gear;
    S->ext_pedals = 1; S->ext_steer = 1; S->dev.kind = 0; S->dev.steer = 0; R->stick_f = 0;
    static dsamp_t Hs[DHIST]; int hn = 0;
    double tev = 8, ttr = tev + 4 + C->hold_s, tend = C->transition ? ttr + 12 : tev + 4 + C->hold_s;
    double ti = 0, I = 0.5, thr = 0, prev_bs = 0; int n = 0;
    FILE* f = csv ? fopen(csv, "w") : NULL;
    if (f) fprintf(f, "t,beta_deg,beta_front_deg,delta_deg,sw_deg,sw_rate_dps,column_nm,hand_nm,throttle,speed_kmh,yaw_rate_dps,radius_m,front_slip_deg,front_fy_use,front_comb_use,rear_slip_ratio,rear_slip_deg,rpm,lsd_dw_rads,beta_target_deg,delta_req_deg\n");
    for (double tt = 0; tt < tend; tt += dt) {
        dc_telemetry t; dc_get_telemetry(S->car, &t);
        dc_xform ch[DC_PART_COUNT]; dc_get_parts(S->car, ch, DC_PART_COUNT);
        double yaw = yaw_of(&ch[0]), vx = fmax(fabs(t.vx), 0.5), beta = atan2(t.vy, vx), bf = atan2(t.vy + R->a_cg * t.yaw_rate, vx);
        double px = ch[0].pos[0] - cx, py = ch[0].pos[1] - cy, dist = sqrt(px * px + py * py);
        if (C->transition && tt >= ttr && sgn_ > 0) {   /* figure-8: reflect the circle centre through the car, reverse the orbit */
            cx += 2 * px; cy += 2 * py; sgn_ = -1; px = -px; py = -py;
        }
        double bs = -sgn_ * beta;                       /* drift angle, + = tail out of the current circle */
        dsamp_t cur = {tt, bs, -sgn_ * bf, sgn_ * t.yaw_rate, t.speed, dist - C->R, (bs - prev_bs) / dt}; prev_bs = bs;
        Hs[hn++ % DHIST] = cur;
        double hand = 0;
        if (tt < tev) {   /* grip entry on the circle: pure pursuit, stiff hands, PI speed */
            double ph = atan2(py, px), L = 5 + 0.35 * vx;
            double tx = cx + C->R * cos(ph + L / C->R), ty = cy + C->R * sin(ph + L / C->R);
            double ex = tx - ch[0].pos[0], ey = ty - ch[0].pos[1];
            double lx = cos(yaw) * ex + sin(yaw) * ey, ly = -sin(yaw) * ex + cos(yaw) * ey;
            double sw = atan2(2 * R->wb * ly, lx * lx + ly * ly) * R->ratio;
            S->ext_steer = 1; S->ext_in.steer_mode = DC_STEER_ANGLE; S->ext_in.steer_target = sw; S->ext_in.hands_k = 300; S->ext_in.hands_d = 3; S->ext_in.hands_tmax = 80;
            double e = C->v0 - t.vx; ti = clampd(ti + e * dt * 0.3, -0.3, 0.6);
            thr = clampd(0.25 + 0.6 * e + ti, 0, 1); S->ext_in.throttle = thr; S->ext_in.brake = 0;
        } else {
            int back = (int)(C->delay / dt + 0.5); if (back > hn - 1) back = hn - 1;
            dsamp_t h = Hs[(hn - 1 - back) % DHIST];
            /* anticipation (McRuer lead): a driver extrapolates the delayed percept over his own reaction time */
            { double tp = (getenv("DRIFT_PRED") ? atof(getenv("DRIFT_PRED")) : 1.0) * C->delay;
              dsamp_t h1 = Hs[(hn - 2 - back + DHIST) % DHIST]; double dbf = (h.bf - h1.bf) / dt;
              h.beta += tp * h.dbeta; h.bf += tp * dbf; }
            double bstar = C->bstar / DEG;
            /* steering: front wheels a little inside the front-axle velocity; radius trim */
            /* front slip target: base + radius trim, minus beta error / beta rate (more countersteer = less front force = less yaw) */
            double ad = clampd(C->alpha0 / DEG + C->kd_path / DEG * clampd(h.d, -6, 6) - C->kbs * (h.beta - C->bstar / DEG) - C->krs * h.dbeta, -12 / DEG, 8 / DEG);
            if (!getenv("DRIFT_LAW") || strcmp(getenv("DRIFT_LAW"), "beta")) {   /* default yaw-rate law: hold a yaw-rate reference (path rate + beta correction) with front slip */
                static double ai; double rref = h.v / C->R + (getenv("DRIFT_KB2") ? atof(getenv("DRIFT_KB2")) : 1.5) * (bstar - h.beta);
                double kr2 = getenv("DRIFT_KR2") ? atof(getenv("DRIFT_KR2")) : 0.2;   /* deg front slip per deg/s yaw-rate error */
                if (tt - tev < dt * 1.5) ai = 0;
                ai = clampd(ai + (getenv("DRIFT_KI2") ? atof(getenv("DRIFT_KI2")) : 0.5) * (bstar - h.beta) * dt, -4 / DEG, 4 / DEG);
                ad = clampd(C->alpha0 / DEG + ai + kr2 * (rref - h.r) + C->kd_path / DEG * clampd(h.d, -6, 6) * (getenv("DRIFT_PATH") ? atof(getenv("DRIFT_PATH")) : 0), -12 / DEG, 9 / DEG);
            }
            double delta = h.bf - ad;                    /* in drift-frame sign (+ = countersteer) */
            double sw = clampd(-sgn_ * delta * R->ratio, -R->lock, R->lock);
            /* throttle: hold beta* */
            double e = (h.beta - bstar) * DEG;           /* + = too much angle */
            I = clampd(I - C->ki * e * dt, 0, 1);
            double want = clampd(I - C->kp * e - C->kdb * h.dbeta * DEG, 0, 1);
            /* entry: a 1.6 NA Miata does not power over from a steady 60 km/h corner (rear has slack, front saturates first),
               so like real drivers it is initiated with a handbrake pull (0.4 s) while the foot stays in, then held on throttle */
            int entry = tt - tev < C->hb_s;
            S->ext_in.handbrake = entry ? 1 : 0;
            if (entry) { want = 0; thr = 0; I = 0.7; }   /* foot off during the pull (a real driver dips the clutch) */
            thr += clampd(want - thr, -dt / 0.12, dt / 0.12);   /* foot: full travel in 0.12 s */
            { static double frz_sw, frz_thr; double frz = getenv("DRIFT_FREEZE") ? atof(getenv("DRIFT_FREEZE")) : 0;   /* open-loop probe */
              if (frz > 0 && tt - tev >= frz) { sw = frz_sw; thr = frz_thr; } else { frz_sw = sw; frz_thr = thr; } }
            S->ext_in.throttle = thr; S->ext_in.brake = 0;
            /* a drifting driver short of revs upshifts instead of riding the limiter (the rear can no longer out-spin the road) */
            if ((getenv("DRIFT_SHIFT") ? atoi(getenv("DRIFT_SHIFT")) : 1) && t.engine_rpm > 6900 && S->gear == C->gear) S->gear = C->gear + 1;
            if (!C->gp) {
                S->ext_steer = 1; S->ext_in.steer_mode = DC_STEER_ANGLE; S->ext_in.steer_target = sw; S->ext_in.hands_k = getenv("DRIFT_HK") ? atof(getenv("DRIFT_HK")) : 40; S->ext_in.hands_d = 1.5; S->ext_in.hands_tmax = 20;
                if (getenv("DRIFT_STIFF")) { S->ext_in.hands_k = 300; S->ext_in.hands_d = 3; S->ext_in.hands_tmax = 80; }   /* ideal servo: physics-feasibility check */
            } else {
                S->ext_steer = 0; S->dev.kind = 2;
                double st = stick_for(R, sw, vx);
                R->stick_f += (st - R->stick_f) * (1 - exp(-dt * 2 * M_PI * 6.0));
                S->dev.steer = clampd(R->stick_f, -1, 1);
            }
        }
        hand = t.driver_torque;
        double fu = 0, cu = 0, af = 0, sr = 0, sa = 0;
        for (int w = 0; w < 2; w++) { double m = fmax(t.w[w].mu * t.w[w].fz, 1); fu += 0.5 * fabs(t.w[w].fy) / m; cu += 0.5 * hypot(t.w[w].fx, t.w[w].fy) / m; af += 0.5 * t.w[w].slip_angle_ss; }
        for (int w = 2; w < 4; w++) { sr += 0.5 * t.w[w].slip_ratio_ss; sa += 0.5 * t.w[w].slip_angle_ss; }
        double dl = 0.5 * (t.w[0].steer + t.w[1].steer), lsd = t.w[2].omega - t.w[3].omega;
        double req = -sgn_ * bf * DEG + 0.0;           /* countersteer needed to align the fronts with their velocity */
        if (n < DREC) {
            DT_[n] = tt; DB_[n] = -sgn_ * beta; DBF_[n] = -sgn_ * bf; DDL_[n] = -sgn_ * dl; DSW_[n] = -sgn_ * t.sw_angle; DAF_[n] = sgn_ * af; DFU_[n] = fu; DCU_[n] = cu;
            DSR_[n] = sr; DSA_[n] = -sgn_ * sa; DTH_[n] = S->ext_in.throttle; DV_[n] = t.speed; DRAD_[n] = dist; DRPM_[n] = t.engine_rpm; DLSD_[n] = lsd; DREQ_[n] = req; n++;
        }
        if (f && ((int)(tt / dt + 0.5)) % 10 == 0)
            fprintf(f, "%.3f,%.2f,%.2f,%.2f,%.1f,%.0f,%.2f,%.2f,%.3f,%.1f,%.1f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.0f,%.2f,%.1f,%.2f\n", tt, -sgn_ * beta * DEG, -sgn_ * bf * DEG, -sgn_ * dl * DEG,
                    -sgn_ * t.sw_angle * DEG, -sgn_ * t.sw_rate * DEG, t.column_torque, hand, S->ext_in.throttle, t.speed * 3.6, sgn_ * t.yaw_rate * DEG, dist,
                    sgn_ * af * DEG, fu, cu, sr, -sgn_ * sa * DEG, t.engine_rpm, lsd, tt < tev ? 0 : C->bstar, req);
        if (fabs(beta) * DEG > 75 && tt > tev) { O->spin = 1; if (!C->transition || tt < ttr) break; }
        sim_tick(S, dt);
    }
    if (f) fclose(f);
    double tlim = C->transition ? ttr : tend;
    drift_stats(DT_, DB_, n, tev, tlim, 20, 35, &O->hold, &O->hold_t0);
    double s[16] = {0}, ss[4] = {0}; int m = 0;
    for (int i = 0; i < n; i++) {
        if (O->hold <= 0 || DT_[i] < O->hold_t0 || DT_[i] > O->hold_t0 + O->hold) continue;
        double b = DB_[i] * DEG;
        s[0] += b; ss[0] += b * b; s[1] += DDL_[i] * DEG; s[2] += DSW_[i] * DEG; s[3] += DAF_[i] * DEG; s[4] += DFU_[i]; s[5] += DCU_[i]; s[6] += DSR_[i]; s[7] += DSA_[i] * DEG;
        s[8] += DTH_[i]; ss[1] += DTH_[i] * DTH_[i]; s[9] += DV_[i]; s[10] += DRAD_[i]; s[11] += DRPM_[i]; s[12] += DLSD_[i]; s[13] += DREQ_[i];
        if (DDL_[i] * DEG > O->delta_max) O->delta_max = DDL_[i] * DEG;
        if (DSW_[i] * DEG > O->rim_max) O->rim_max = DSW_[i] * DEG;
        if (DFU_[i] > O->fy_use_max) O->fy_use_max = DFU_[i];
        if (DREQ_[i] > O->req_max) O->req_max = DREQ_[i];
        m++;
    }
    if (m) {
        O->beta_m = s[0] / m; O->beta_sd = sqrt(fmax(ss[0] / m - O->beta_m * O->beta_m, 0)); O->delta_m = s[1] / m; O->rim_m = s[2] / m; O->af_m = s[3] / m;
        O->fy_use_m = s[4] / m; O->comb_use_m = s[5] / m; O->rear_sr_m = s[6] / m; O->rear_sa_m = s[7] / m; O->thr_m = s[8] / m;
        O->thr_sd = sqrt(fmax(ss[1] / m - O->thr_m * O->thr_m, 0)); O->v_m = s[9] / m * 3.6; O->rad_m = s[10] / m; O->rpm_m = s[11] / m; O->lsd_dw_m = s[12] / m; O->req_m = s[13] / m;
    }
    if (C->transition) {   /* opposite-sign drift reached and held */
        double best, b0; drift_stats(DT_, DB_, n, ttr, tend, 15, 40, &best, &b0);
        O->tr_hold = best; O->tr_time = b0 >= 0 ? b0 - ttr : -1; O->tr_ok = best > 4 && !O->spin;
    }
    return n;
}
static int drift(const char* car, const char* out, const char* tag) {
    static rig_t R; int fails = 0;
    char path[512]; snprintf(path, sizeof path, "%s/drift_%s.csv", out, tag);
    FILE* s = fopen(path, "w");
    fprintf(s, "run,beta_target,hold_s,beta_mean,beta_sd,delta_mean,delta_max,rim_mean,rim_max,lock_rim,req_countersteer_mean,req_countersteer_max,front_slip_mean,front_fy_use_mean,front_fy_use_max,front_comb_use_mean,rear_slip_ratio_mean,rear_slip_deg_mean,throttle_mean,throttle_sd,speed_kmh,radius_m,rpm,lsd_dw,spin,tr_time_s,tr_hold_s,tr_ok\n");
    printf("sustained drift [%s] (car %s, PSET %s)\n", tag, car && *car ? car : "default", getenv("PSET") ? getenv("PSET") : "-");
    double bst[3] = {22, 27, 32};   /* inside the 20-35 deg hold window, so noise around the target does not leave it */ const char* hn[2] = {"arms", "gp"};
    for (int h = 0; h < 2; h++) for (int b = 0; b < 4; b++) {
        drift_cfg_t C = {b < 3 ? bst[b] : 27, 40, 60 / 3.6, 0.15, 12, 5, 0.5, 0.02, 0.02, 0, 2, h, b == 3};
        if (getenv("DRIFT_DELAY")) C.delay = atof(getenv("DRIFT_DELAY"));
        if (getenv("DRIFT_R")) C.R = atof(getenv("DRIFT_R"));            /* power-margin probe: smaller/slower circle */
        if (getenv("DRIFT_V0")) C.v0 = atof(getenv("DRIFT_V0")) / 3.6;
        if (getenv("DRIFT_GEAR")) C.gear = atoi(getenv("DRIFT_GEAR"));
        C.kbs = getenv("DRIFT_KB") ? atof(getenv("DRIFT_KB")) : 0.35; C.krs = getenv("DRIFT_KR") ? atof(getenv("DRIFT_KR")) : 0.20;
        C.hb_s = getenv("DRIFT_HB") ? atof(getenv("DRIFT_HB")) : 0.35;
        if (getenv("DRIFT_A0")) C.alpha0 = atof(getenv("DRIFT_A0"));
        if (getenv("DRIFT_KP")) C.kp = atof(getenv("DRIFT_KP"));
        if (getenv("DRIFT_KI")) C.ki = atof(getenv("DRIFT_KI"));
        if (getenv("DRIFT_KD")) C.kdb = atof(getenv("DRIFT_KD"));
        if (getenv("DRIFT_ONLY") && atoi(getenv("DRIFT_ONLY")) != b) continue;
        if (getenv("DRV") && !strstr(getenv("DRV"), hn[h])) continue;
        rig_init(&R, car, NULL);
        char nm[64]; snprintf(nm, sizeof nm, "%s_b%.0f%s", hn[h], C.bstar, C.transition ? "_transition" : "");
        if (getenv("SC") && !strstr(getenv("SC"), nm)) continue;
        char csv[512]; snprintf(csv, sizeof csv, "%s/drift_%s_%s.csv", out, tag, nm);
        drift_res_t O; run_drift(&R, &C, csv, &O);
        int ok = O.hold >= 10 && !O.spin; if (C.transition) ok = O.tr_ok && O.hold >= 10;
        if (!ok) fails++;
        printf("  %-20s hold %5.1f s  beta %5.1f+-%4.1f  delta %5.1f (max %5.1f, req %5.1f/%5.1f, lock %4.1f)  rim %4.0f/%4.0f  a_f %5.1f  Fy/muFz %4.2f (max %4.2f, comb %4.2f)  rear sr %5.2f sa %5.1f  thr %4.2f+-%4.2f  v %4.1f  R %5.1f  rpm %4.0f  lsd %5.1f  %s%s",
               nm, O.hold, O.beta_m, O.beta_sd, O.delta_m, O.delta_max, O.req_m, O.req_max, R.lock / R.ratio * DEG, O.rim_m, O.rim_max, O.af_m, O.fy_use_m, O.fy_use_max, O.comb_use_m,
               O.rear_sr_m, O.rear_sa_m, O.thr_m, O.thr_sd, O.v_m, O.rad_m, O.rpm_m, O.lsd_dw_m, O.spin ? "SPIN " : "", ok ? "ok" : "--");
        if (C.transition) printf("  | transition: reached in %5.2f s, held %4.1f s", O.tr_time, O.tr_hold);
        printf("\n"); fflush(stdout);
        fprintf(s, "%s,%.0f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.2f,%.3f,%.3f,%.1f,%.2f,%.0f,%.2f,%d,%.2f,%.2f,%d\n", nm, C.bstar, O.hold, O.beta_m, O.beta_sd,
                O.delta_m, O.delta_max, O.rim_m, O.rim_max, R.lock * DEG, O.req_m, O.req_max, O.af_m, O.fy_use_m, O.fy_use_max, O.comb_use_m, O.rear_sr_m, O.rear_sa_m, O.thr_m, O.thr_sd,
                O.v_m, O.rad_m, O.rpm_m, O.lsd_dw_m, O.spin, O.tr_time, O.tr_hold, (int)O.tr_ok);
    }
    fclose(s);
    return fails;
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
            else if (is_hum) bad = M.spin || M.zeta < 0.05 || M.settle < 0 || M.settle > 2.5;
            else if (is_ide) bad = M.spin || M.swings > 1 || M.settle < 0 || M.settle > 0.6;
            else if (is_ho || is_rel) {   /* a released stick (inside the deadzone) is hands off: no recentre servo, same physics, same criterion */
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
    if (!strcmp(what, "drift")) { int fl = drift(car, out, getenv("TAG") ? getenv("TAG") : "run"); return getenv("DRIFT_STRICT") ? (fl ? 1 : 0) : 0; }
    if (!strcmp(what, "counter")) { counter(car, out, getenv("TAG") ? getenv("TAG") : "run"); return 0; }
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
