





#define _POSIX_C_SOURCE 200809L
/* dc_tests.c - headless verification suite. Writes results/ CSV files and results/summary.txt.
 * Plots: python3 tests/plot.py.  Uses internal headers only for the tyre rig and kinematic sweeps. */
#include "dc_internal.h"
#include "dc_tyre.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdarg.h>

static FILE* SUM;
static int NFAIL = 0;
static double AUD_F = 0, AUD_T = 0;   /* worst Newton-3 residuals seen in any dynamic test */
#define D2R (M_PI / 180.0)
#define R2D (180.0 / M_PI)

static const char* RDIR(void) { const char* r = getenv("DC_RESULTS"); return r && *r ? r : "results"; }
/* car under test: defaults, or the car file named by $DC_CAR (e.g. DC_CAR=private/miata_ac.ini) */
static void car_params(dc_car_params* P) {
    dc_default_params(P);
    const char* f = getenv("DC_CAR");
    if (f && *f) { char e[256]; if (!dc_load_params(f, P, e, sizeof e)) { fprintf(stderr, "DC_CAR: %s\n", e); exit(2); } }
}
static void check(const char* name, int ok, const char* fmt, ...) {
    char buf[512]; va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof buf, fmt, ap); va_end(ap);
    fprintf(SUM, "[%s] %-34s %s\n", ok ? "PASS" : "FAIL", name, buf);
    printf("[%s] %-34s %s\n", ok ? "PASS" : "FAIL", name, buf);
    if (!ok) NFAIL++;
}
static void note(const char* fmt, ...) {
    char buf[512]; va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof buf, fmt, ap); va_end(ap);
    fprintf(SUM, "       %s\n", buf); printf("       %s\n", buf);
}
static FILE* csv(const char* name, const char* header) {
    char p[256]; snprintf(p, sizeof p, "%s/%s", RDIR(), name);
    FILE* f = fopen(p, "w"); if (!f) { perror(p); exit(1); }
    fprintf(f, "%s\n", header); return f;
}
static double yaw_of(const dc_car* c) { qt q = c->q; return atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)); }
static void audit(const dc_telemetry* T, double mg) {
    double f = sqrt(T->resid_force[0] * T->resid_force[0] + T->resid_force[1] * T->resid_force[1] + T->resid_force[2] * T->resid_force[2]) / mg;
    double t = sqrt(T->resid_torque[0] * T->resid_torque[0] + T->resid_torque[1] * T->resid_torque[1] + T->resid_torque[2] * T->resid_torque[2]);
    if (f > AUD_F) AUD_F = f; if (t > AUD_T) AUD_T = t;
}

/* ---------------- roads ---------------- */
typedef struct { int kind; } road_t;
static double road_fn(void* u, double x, double y, double* nx, double* ny, double* nz, double* grip) {
    road_t* r = u; *nx = 0; *ny = 0; *nz = 1; *grip = 1;
    double h = 0;
    if (r->kind == 1) {       /* kerb/bump strip: sharp-edged rumble kerb under the left wheels, plank under the right */
        if (x > 40 && x < 52 && y > 0.35) { double ph = fmod(x - 40, 0.8); if (ph < 0.4) h = 0.025; }
        if (x > 70 && x < 70.5 && y < -0.2) h = 0.040;
        if (x > 100 && x < 102) { double s = sin(M_PI * (x - 100) / 2); h = 0.06 * s * s;
            double dh = 0.06 * 2 * s * cos(M_PI * (x - 100) / 2) * M_PI / 2; double l = sqrt(1 + dh * dh); *nx = -dh / l; *nz = 1 / l; }
    } else if (r->kind == 2 || r->kind == 3) {  /* rough terrain, smooth random-ish (3 = 35% amplitude) */
        double A = r->kind == 3 ? 0.35 : 1.0;
        h = 0.03 * sin(0.9 * x) * sin(1.3 * y) + 0.015 * sin(2.7 * x + 1.1 * y) + 0.01 * sin(5.3 * x - 3.1 * y);
        double hx = 0.03 * 0.9 * cos(0.9 * x) * sin(1.3 * y) + 0.015 * 2.7 * cos(2.7 * x + 1.1 * y) + 0.01 * 5.3 * cos(5.3 * x - 3.1 * y);
        double hy = 0.03 * 1.3 * sin(0.9 * x) * cos(1.3 * y) + 0.015 * 1.1 * cos(2.7 * x + 1.1 * y) - 0.01 * 3.1 * cos(5.3 * x - 3.1 * y);
        h *= A; hx *= A; hy *= A;
        double l = sqrt(1 + hx * hx + hy * hy); *nx = -hx / l; *ny = -hy / l; *nz = 1 / l;
    }
    return h;
}

/* ---------------- drivers ---------------- */
static double ratio_of(const dc_car* c) { return c->P.steer_ratio; }
static int auto_gear(const dc_car* c, double v) {
    const dc_car_params* P = &c->P; double R = P->axle[1].tyre.radius;
    int g = 1;
    for (int i = 1; i <= P->ngears; i++) { double rpm = v / R * P->gear[i - 1] * P->final_drive * 30 / M_PI; if (rpm > 2600) g = i; }
    return g;
}
typedef struct { double i; } pi_t;
static void speed_ctl(const dc_car* c, dc_input* in, double vt, double v, pi_t* s, double h) {
    double e = vt - v; s->i += e * h; if (s->i > 2) s->i = 2; if (s->i < -2) s->i = -2;
    double u = 0.25 * e + 0.15 * s->i;
    in->throttle = u > 0 ? fmin(u, 1) : 0; in->brake = u < 0 ? fmin(-u * 0.5, 1) : 0;
    in->gear = auto_gear(c, v); in->clutch = 0;
}
static void hands_angle(dc_input* in, double target) {
    in->steer_mode = DC_STEER_ANGLE; in->steer_target = target; in->hands_k = 120; in->hands_d = 4; in->hands_tmax = 40;
}
static dc_car* mk(dc_car_params* P, road_t* rd) {
    dc_car* c = dc_create(P);
    if (rd) dc_set_road(c, road_fn, rd);
    return c;
}

/* ================= 0. kinematics: camber/toe/caster vs travel; Ackermann ================= */
static void test_kinematics(void) {
    dc_car_params P; car_params(&P); dc_car* c = mk(&P, NULL);
    FILE* f = csv("kin_travel.csv", "axle,travel_mm,camber_deg,toe_deg,caster_deg,kpi_deg,scrub_mm,trail_mm,spring_len_mm,motion_ratio,track_change_mm");
    for (int ax = 0; ax < 2; ax++) {
        int i = ax * 2; dc_kin* K = &c->K[i];
        double y0 = 0;
        for (int k = 0; k <= 32; k++) {
            double s = -0.08 + 0.005 * k;
            dc_kin_full(K, s, 0, 0, 0, &c->pose[i]);
            dc_pose* p = &c->pose[i];
            v3 a = mv(&p->R, K->spin_axis_u);
            memset(&c->tel.w[i], 0, sizeof c->tel.w[i]);
            c->x = V3(0, 0, 0); c->q = (qt){1, 0, 0, 0};
            dc_telemetry T; dc_get_telemetry(c, &T);
            if (k == 16) y0 = p->c.y;
            double cam = -asin(a.z) * R2D, toe = atan2(-a.x, a.y) * R2D;
            fprintf(f, "%d,%.1f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.3f,%.4f,%.3f\n", ax, s * 1000, cam, toe, T.w[i].caster * R2D, T.w[i].kpi * R2D,
                    T.w[i].scrub * 1000, T.w[i].trail * 1000, p->Ls * 1000, p->dL[0], 0.0);
        }
        (void)y0;
    }
    fclose(f);
    /* re-read to evaluate gains at design */
    dc_pose p0 = c->pose[0], p1 = c->pose[0], pm = c->pose[0];
    dc_kin_full(&c->K[0], 0.0, 0, 0, 0, &p0); dc_kin_full(&c->K[0], 0.02, 0, 0, 0, &p1); dc_kin_full(&c->K[0], -0.02, 0, 0, 0, &pm);
    v3 a1 = mv(&p1.R, c->K[0].spin_axis_u), am = mv(&pm.R, c->K[0].spin_axis_u);
    double cg = (-asin(a1.z) + asin(am.z)) * R2D / 40.0;           /* deg/mm */
    double tg = (atan2(-a1.x, a1.y) - atan2(-am.x, am.y)) * R2D / 40.0;
    note("front camber gain %.3f deg/10mm bump, bump steer %.4f deg/10mm (+ = toe-out on left)", cg * 10, tg * 10);
    check("kin: camber gain sign (DWB)", cg < 0, "%.3f deg/10mm (negative = gains negative camber in bump)", cg * 10);
    check("kin: bump steer small", fabs(tg * 10) < 0.15, "%.4f deg/10mm", tg * 10);
    /* Ackermann: rack sweep */
    f = csv("kin_rack.csv", "rack_mm,sw_deg,delta_L_deg,delta_R_deg,ideal_outer_deg,ackermann_pct");
    double L = c->K[0].hub0.x - c->K[2].hub0.x, tw = 2 * c->K[0].hub0.y;
    double ack20 = 0, sw_lock = 0, dL_lock = 0, dR_lock = 0;
    for (int k = -60; k <= 60; k++) {
        double r = c->rack_max * k / 60.0;
        dc_pose pl = c->pose[0], pr = c->pose[1]; dc_kin_full(&c->K[0], 0, r, 0, 0, &pl); dc_kin_full(&c->K[1], 0, r, 0, 0, &pr);
        v3 al = mv(&pl.R, c->K[0].spin_axis_u), ar = mv(&pr.R, c->K[1].spin_axis_u);
        double dl = atan2(-al.x, al.y), dr = atan2(-ar.x, ar.y);
        double din = r > 0 ? dl : dr, dout = r > 0 ? dr : dl;  /* r>0 = left turn: left wheel inner */
        double ideal = 0, pct = 0;
        if (fabs(din) > 0.02) {
            ideal = atan(1.0 / (1.0 / tan(fabs(din)) + tw / L)) * (din > 0 ? 1 : -1);
            pct = (fabs(din) - fabs(dout)) / (fabs(din) - fabs(ideal)) * 100;
        }
        double sw = r / c->r_p;
        fprintf(f, "%.2f,%.2f,%.4f,%.4f,%.4f,%.2f\n", r * 1000, sw * R2D, dl * R2D, dr * R2D, ideal * R2D, pct);
        if (fabs(din * R2D - 20) < 1.5 && r > 0) ack20 = pct;
        if (k == 60) { sw_lock = sw * R2D; dL_lock = dl * R2D; dR_lock = dr * R2D; }
    }
    fclose(f);
    note("lock: sw %.0f deg -> inner %.1f deg, outer %.1f deg; Ackermann at ~20 deg inner: %.0f %% (emergent from tie-rod geometry)", sw_lock, dL_lock, dR_lock, ack20);
    note("overall ratio at centre %.2f:1 (pinion radius %.2f mm, rack travel +-%.1f mm)", P.steer_ratio, c->r_p * 1000, c->rack_max * 1000);
    check("kin: Ackermann positive", ack20 > 10, "%.0f %%", ack20);
    dc_destroy(c);
}

/* ================= 1. static settle vs analytic ================= */
static void test_static(void) {
    dc_car_params P; car_params(&P);
    for (int pass = 0; pass < 3; pass++) {
        dc_car* c = mk(&P, NULL);
        double pos[3] = {0, 0, 0.0};
        dc_reset(c, pos, 0, 0);
        c->x.z += 0.03;                         /* drop from 3 cm above design */
        /* pass 0: frictionless ground (analytic statics are friction-free); 1: + no damping (ride frequencies);
           2: with friction (tyre scrub under track change holds some jacking: the real 'bounce the car' effect) */
        if (pass == 1) dc_debug_flags(c, DC_DBG_NO_DAMPING | DC_DBG_ENGINE_OFF | DC_DBG_NO_FRICTION);
        else if (pass == 0) dc_debug_flags(c, DC_DBG_ENGINE_OFF | DC_DBG_NO_FRICTION);
        else dc_debug_flags(c, DC_DBG_ENGINE_OFF);
        dc_input in = {0}; in.brake = 0.3; dc_set_input(c, &in);
        FILE* f = csv(pass == 1 ? "static_nodamp.csv" : pass == 0 ? "static_settle.csv" : "static_settle_friction.csv", "t,z,pitch_deg,fz0,fz1,fz2,fz3,s0_mm,s2_mm,defl0_mm,defl2_mm");
        double h = 1.0 / P.substep_hz; dc_telemetry T;
        for (int n = 0; n < (int)(5.0 / h); n++) {
            dc_substep(c); dc_get_telemetry(c, &T);
            if (n % 4 == 0) {
                double pitch = asin(fmax(-1, fmin(1, 2 * (c->q.w * c->q.y - c->q.z * c->q.x)))) * R2D;
                fprintf(f, "%.4f,%.6f,%.5f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f\n", T.time, c->x.z, pitch, T.w[0].fz, T.w[1].fz, T.w[2].fz, T.w[3].fz,
                        T.w[0].travel * 1000, T.w[2].travel * 1000, T.w[0].deflection * 1000, T.w[2].deflection * 1000);
            }
        }
        fclose(f);
        if (pass == 2) note("with tyre friction (drop from 3 cm, brakes on): travel F %.2f mm R %.2f mm held by tyre scrub (physical)", T.w[0].travel * 1000, T.w[2].travel * 1000);
        if (pass == 0) {
            double xf = P.axle[0].hub[0], xr = P.axle[1].hub[0], mg = P.mass * P.gravity;
            double Wf = mg * (-xr) / (xf - xr) / 2, Wr = mg * xf / (xf - xr) / 2;
            double ef = (T.w[0].fz - Wf) / Wf * 100, er = (T.w[2].fz - Wr) / Wr * 100;
            double sum = T.w[0].fz + T.w[1].fz + T.w[2].fz + T.w[3].fz;
            check("static: total load = m g", fabs(sum - mg) / mg < 1e-3, "%.1f N vs %.1f N", sum, mg);
            check("static: front corner load", fabs(ef) < 1.0, "%.1f N vs analytic %.1f N (%.2f %%)", T.w[0].fz, Wf, ef);
            check("static: rear corner load", fabs(er) < 1.0, "%.1f N vs analytic %.1f N (%.2f %%)", T.w[2].fz, Wr, er);
            /* tyre deflection analytic: per row F/n = k/n d (1 + 0.5 (d/dlim)^2) -> solve */
            const dc_tyre_params* ty = &P.axle[0].tyre; double dlim = ty->radius - ty->rim_radius, d = Wf / ty->kz_ref;
            for (int it = 0; it < 30; it++) { double g = ty->kz_ref * d * (1 + 0.5 * d * d / dlim / dlim) - Wf, dg = ty->kz_ref * (1 + 1.5 * d * d / dlim / dlim); d -= g / dg; }
            note("front tyre deflection: sim %.2f mm (camber-weighted rows), analytic flat %.2f mm", T.w[0].deflection * 1000, d * 1000);
            check("static: design ride height reached", fabs(T.w[0].travel) < 0.002 && fabs(T.w[2].travel) < 0.002,
                  "travel F %.2f mm R %.2f mm (springs preloaded for design height)", T.w[0].travel * 1000, T.w[2].travel * 1000);
            /* heave/pitch analytic 2-dof (ride rate = wheel rate in series with tyre) */
            double kf = 2 * 1 / (1 / P.axle[0].wheel_rate + 1 / ty->kz_ref), kr = 2 * 1 / (1 / P.axle[1].wheel_rate + 1 / P.axle[1].tyre.kz_ref);
            double a = xf - c->P.sprung_cg[0], b = xr - c->P.sprung_cg[0];
            double m = c->m_s, I = c->I_s_body.y;
            double K11 = kf + kr, K12 = -(kf * a + kr * b), K22 = kf * a * a + kr * b * b;
            double A = K11 / m, B = K12 / sqrt(m * I), C = K22 / I;
            double l1 = 0.5 * (A + C) - sqrt(0.25 * (A - C) * (A - C) + B * B), l2 = 0.5 * (A + C) + sqrt(0.25 * (A - C) * (A - C) + B * B);
            FILE* g = csv("static_analytic.csv", "f_mode1_hz,f_mode2_hz,Wf,Wr");
            fprintf(g, "%.4f,%.4f,%.2f,%.2f\n", sqrt(l1) / (2 * M_PI), sqrt(l2) / (2 * M_PI), Wf, Wr); fclose(g);
            note("analytic ride modes (sprung 2-dof, series tyre): %.3f Hz, %.3f Hz -> compared by FFT in plot.py", sqrt(l1) / (2 * M_PI), sqrt(l2) / (2 * M_PI));
        }
        dc_destroy(c);
    }
}

/* ================= 2. momentum / energy ================= */
static void test_conservation(void) {
    dc_car_params P; car_params(&P);
    P.rack_friction = 0; P.column_friction = 0; P.assist = 0; P.torsion_c = 0;
    for (int a = 0; a < 2; a++) { P.axle[a].brake_torque = 0; }
    const char* names[2] = {"cons_flat.csv", "cons_rough.csv"};
    for (int pass = 0; pass < 2; pass++) {
        road_t rd = {pass == 0 ? 0 : 3};
        dc_car* c = mk(&P, &rd);
        double pos[3] = {0, 0, 0}; dc_reset(c, pos, 0, 15);
        dc_debug_flags(c, DC_DBG_NO_DAMPING | DC_DBG_NO_FRICTION | DC_DBG_NO_AERO | DC_DBG_ENGINE_OFF);
        if (pass == 0) { c->x.z += 0.04; c->v.y = 2.0; c->w = V3(0.2, -0.1, 0.8); c->thd = 3.0; }   /* drop + yaw, roll, pitch, steering */
        else { c->v.y = 1.0; c->w = V3(0, 0, 0.3); c->thd = 2.0; }
        dc_input in = {0}; in.gear = 0; in.clutch = 1; dc_set_input(c, &in);
        FILE* f = csv(names[pass], "t,E,KE,PE,px,py,pz,Lx,Ly,Lz,resF,resT,sw_deg");
        double h = 1.0 / P.substep_hz; dc_telemetry T; double E0 = 0, p0x = 0, p0y = 0, L0 = 0, dE = 0, dE6 = 0, dP = 0, dL = 0, rf = 0, rt = 0;
        int N = (int)((pass ? 20.0 : 10.0) / h);
        for (int n = 0; n < N; n++) {
            dc_substep(c); dc_get_telemetry(c, &T);
            double E = T.kinetic + T.potential + T.dissipated - T.proj_work;
            if (n == 0) { E0 = E; p0x = T.momentum[0]; p0y = T.momentum[1]; L0 = T.ang_momentum[2]; }
            dE = fmax(dE, fabs(E - E0) / E0);
            if (T.time <= 6.0) dE6 = dE;
            if (pass && getenv("DC_PROBE") && T.time > atof(getenv("DC_PROBE")) && T.time < atof(getenv("DC_PROBE")) + 0.1 && n % 4 == 0) {
                printf("t %.4f E %.0f PE %.0f rack %.4f sw %.1f |", T.time, E, T.potential, T.rack_pos, T.sw_angle * 57.3);
                for (int w = 0; w < 4; w++) printf(" [%d tr %.3f fz %.0f rim %.0f bs %.0f sp %.0f defl %.3f]", w, T.w[w].travel, T.w[w].fz, T.w[w].rim_force, T.w[w].bumpstop_force, T.w[w].spring_force, T.w[w].deflection);
                printf("\n");
            }
            dP = fmax(dP, fmax(fabs(T.momentum[0] - p0x), fabs(T.momentum[1] - p0y)));
            dL = fmax(dL, fabs(T.ang_momentum[2] - L0));
            double F = sqrt(T.resid_force[0] * T.resid_force[0] + T.resid_force[1] * T.resid_force[1] + T.resid_force[2] * T.resid_force[2]);
            double Tq = sqrt(T.resid_torque[0] * T.resid_torque[0] + T.resid_torque[1] * T.resid_torque[1] + T.resid_torque[2] * T.resid_torque[2]);
            rf = fmax(rf, F); rt = fmax(rt, Tq);
            if (n % 10 == 0) fprintf(f, "%.4f,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3e,%.3e,%.3f\n", T.time, E, T.kinetic, T.potential,
                                     T.momentum[0], T.momentum[1], T.momentum[2], T.ang_momentum[0], T.ang_momentum[1], T.ang_momentum[2], F, Tq, T.sw_angle * R2D);
        }
        fclose(f);
        double pmag = P.mass * 15;
        if (pass == 0) {
            check("conserve: horizontal momentum", dP / pmag < 1e-9, "max |dP| = %.2e kg m/s (%.1e rel)", dP, dP / pmag);
            check("conserve: yaw ang. momentum Lz", dL / fabs(L0) < 1e-3, "max |dLz| = %.3e (L0 = %.1f, %.1e rel)", dL, L0, dL / fabs(L0));
            check("conserve: energy drift (flat)", dE < 0.005, "max |dE|/E0 = %.2e over 10 s (E0 = %.0f J)", dE, E0);
        } else {
            check("conserve: energy (rough, 6 s)", dE6 < 0.005, "max |dE|/E0 = %.2e over 6 s on +-2 cm rough terrain, undamped & frictionless", dE6);
            note("conserve: rough 20 s (undamped resonance builds to wheel-lift/droop-stop/bump-stop slams): max |dE|/E0 = %.2e, projection work %.1f J", dE, T.proj_work);
        }
        check(pass ? "newton3: residuals (rough)" : "newton3: residuals (flat)", rf < 1e-6 * P.mass * 9.81 && rt < 1e-4,
              "max |sum m a - F_ext| = %.2e N, |sum dL/dt - T_ext| = %.2e N m", rf, rt);
        dc_destroy(c);
    }
}

/* ================= 3a. tyre rig (calibration vs AC targets) ================= */
static void rig_point(dc_car* c, double Fz, double alpha, double kappa, double gamma, double V, double* Fx, double* Fy, double* Mz, double* tp, double* al_tr) {
    double h = 1.0 / c->P.substep_hz; const dc_tyre_params* T = &c->P.axle[0].tyre;
    dc_tyre_state* S = &c->ty[0];
    S->ex = S->ey = S->phi = 0;
    v3 a = V3(0, cos(gamma), -sin(gamma));
    v3 vc = V3(V * cos(alpha), -V * sin(alpha), 0);
    double z = T->radius - Fz / T->kz_ref;
    dc_tc t;
    for (int n = 0; n < 6000; n++) {
        v3 cw = V3(0, 0, z);
        memset(&t, 0, sizeof t);
        S->omega = 0; dc_tyre_phaseA(c, 0, cw, a, vc, V3(0, 0, 0), h, &t);
        double Om = vdot(vc, t.xc) * (1 + kappa) / t.Re;
        S->omega = Om;
        memset(&t, 0, sizeof t);
        dc_tyre_phaseA(c, 0, cw, a, vc, V3(0, 0, 0), h, &t);
        dc_tyre_phaseB(c, 0, cw, a, Om, h, &t, 1);
        z += (t.Ntot - Fz) / T->kz_ref * 0.5;
    }
    *Fx = t.Fx; *Fy = t.Fy; *Mz = t.Mz_brush + t.Mt; *tp = t.tp; *al_tr = atan(S->ey / t.sig_y);
}
static void test_tyre_rig(void) {
    dc_car_params P; car_params(&P); dc_car* c = mk(&P, NULL);
    const dc_tyre_params* T = &P.axle[0].tyre;
    FILE* f = csv("tyre_lateral.csv", "Fz,alpha_deg,Fy,Mz,trail_mm,mu");
    double loads[3] = {0.5 * T->fz0, T->fz0, 2 * T->fz0};
    double pkFy[3], pkA[3], pkMzA[3];
    for (int l = 0; l < 3; l++) {
        pkFy[l] = 0; pkA[l] = 0; pkMzA[l] = 0; double pkMz = 0;
        for (int k = 0; k <= 60; k++) {
            double al = 0.25 * k * D2R, Fx, Fy, Mz, tp, at;
            rig_point(c, loads[l], al, 0, 0, 15, &Fx, &Fy, &Mz, &tp, &at);
            fprintf(f, "%.0f,%.3f,%.2f,%.3f,%.2f,%.4f\n", loads[l], al * R2D, Fy, Mz, tp * 1000, Fy / loads[l]);
            if (Fy > pkFy[l]) { pkFy[l] = Fy; pkA[l] = al * R2D; }
            if (fabs(Mz) > pkMz) { pkMz = fabs(Mz); pkMzA[l] = al * R2D; }   /* aligning moment is negative for +alpha */
        }
    }
    fclose(f);
    f = csv("tyre_long.csv", "Fz,kappa,Fx");
    double pkFx = 0, pkK = 0;
    for (int k = -40; k <= 40; k++) {
        double kap = 0.01 * k, Fx, Fy, Mz, tp, at;
        rig_point(c, T->fz0, 0, kap, 0, 15, &Fx, &Fy, &Mz, &tp, &at);
        fprintf(f, "%.0f,%.3f,%.2f\n", T->fz0, kap, Fx);
        if (Fx > pkFx) { pkFx = Fx; pkK = kap; }
    }
    fclose(f);
    f = csv("tyre_combined.csv", "kappa,alpha_deg,Fx,Fy");
    for (int ki = 0; ki < 4; ki++) {
        double kap = (double[]){0, 0.03, 0.06, 0.12}[ki];
        for (int k = 0; k <= 40; k++) { double al = 0.4 * k * D2R, Fx, Fy, Mz, tp, at;
            rig_point(c, T->fz0, al, kap, 0, 15, &Fx, &Fy, &Mz, &tp, &at);
            fprintf(f, "%.3f,%.3f,%.2f,%.2f\n", kap, al * R2D, Fx, Fy); }
    }
    fclose(f);
    double Fx, Fy0, Mz, tp, at, Fyc;
    rig_point(c, T->fz0, 0, 0, 0, 15, &Fx, &Fy0, &Mz, &tp, &at);
    rig_point(c, T->fz0, 0, 0, -3 * D2R, 15, &Fx, &Fyc, &Mz, &tp, &at);
    /* the brush curve is a plateau after full sliding: report the first slip angle reaching 99.5 % of the peak */
    double a995 = 0, asl = 0, mzpk = 0;
    { char gp[256]; snprintf(gp, sizeof gp, "%s/tyre_lateral.csv", RDIR()); FILE* g = fopen(gp, "r"); char ln[256]; double fz, al, fy, mz, tr, mu;
      if (g) { if (fgets(ln, sizeof ln, g)) {} while (fgets(ln, sizeof ln, g)) if (sscanf(ln, "%lf,%lf,%lf,%lf,%lf,%lf", &fz, &al, &fy, &mz, &tr, &mu) == 6 && fabs(fz - loads[1]) < 1) {
          if (a995 == 0 && fy >= 0.995 * pkFy[1]) a995 = al;
          if (fabs(mz) > mzpk) mzpk = fabs(mz); else if (asl == 0 && fabs(mz) < 0.005 * mzpk) asl = al;   /* full sliding: Mz -> 0 */
      } fclose(g); } }
    double mu_l = pkFy[0] / loads[0], mu_0 = pkFy[1] / loads[1], mu_h = pkFy[2] / loads[2];
    double ls_fit = 1 + log(mu_h / mu_l) / log(4.0);
    note("tyre @Fz0=%.0f: peak Fy/Fz %.3f at %.2f deg (99.5%% of peak at %.2f deg, full sliding (Mz->0) at %.2f deg; target cfg mu_y %.3f, peak %.2f deg); |Mz| peaks at %.2f deg", T->fz0, mu_0, pkA[1], a995, asl, T->mu_y, T->peak_slip_deg, pkMzA[1]);
    note("tyre load sensitivity: mu %.3f / %.3f / %.3f at 0.5/1/2 Fz0 -> fitted exponent %.3f (cfg %.3f)", mu_l, mu_0, mu_h, ls_fit, T->ls_exp_y);
    note("tyre longitudinal: peak Fx/Fz %.3f at kappa %.3f (cfg mu_x %.3f)", pkFx / T->fz0, pkK, T->mu_x);
    note("camber thrust at -3 deg (top in, left wheel => pushes -y): Fy = %.1f N  (%.1f N/deg)", Fyc - Fy0, (Fyc - Fy0) / -3);
    check("tyre: Mz peaks before Fy", pkMzA[1] < pkA[1], "Mz peak %.2f deg < Fy peak %.2f deg", pkMzA[1], pkA[1]);
    check("tyre: peak mu within 3% of cfg", fabs(mu_0 / T->mu_y - 1) < 0.03, "%.3f vs %.3f", mu_0, T->mu_y);
    check("tyre: peak (full-sliding) slip within 1 deg of cfg", fabs(asl - T->peak_slip_deg) < 1.0, "%.2f deg (Mz->0; 99.5%% of max Fy at %.2f, flat-plateau max at %.2f) vs %.2f deg", asl, a995, pkA[1], T->peak_slip_deg);
    check("tyre: load sensitivity", fabs(ls_fit - T->ls_exp_y) < 0.05, "fitted %.3f vs %.3f", ls_fit, T->ls_exp_y);
    /* relaxation: step slip angle at 15 m/s, Fy(t) */
    f = csv("tyre_relax.csv", "dist_m,Fy,Fy_ss");
    {
        double h = 1.0 / P.substep_hz; dc_tyre_state* S = &c->ty[0]; S->ex = S->ey = S->phi = 0;
        double V = 15, al = 2 * D2R; v3 a = V3(0, 1, 0), vc = V3(V * cos(al), -V * sin(al), 0);
        double Fyss, dum; rig_point(c, T->fz0, al, 0, 0, V, &dum, &Fyss, &dum, &dum, &dum);
        S->ex = S->ey = S->phi = 0;
        double z = T->radius - T->fz0 / T->kz_ref; dc_tc t; double d63 = -1;
        for (int it = 0; it < 400; it++) { memset(&t, 0, sizeof t); S->omega = V / 0.28; dc_tyre_phaseA(c, 0, V3(0,0,z), a, V3(V,0,0), V3(0,0,0), h, &t); z += (t.Ntot - T->fz0) / T->kz_ref * 0.5; S->ex = S->ey = S->phi = 0; }
        for (int n = 0; n < 1000; n++) {
            memset(&t, 0, sizeof t); dc_tyre_phaseA(c, 0, V3(0, 0, z), a, vc, V3(0, 0, 0), h, &t);
            double Om = vdot(vc, t.xc) / t.Re; S->omega = Om;
            dc_tyre_phaseB(c, 0, V3(0, 0, z), a, Om, h, &t, 1);
            double d = V * h * (n + 1);
            fprintf(f, "%.4f,%.2f,%.2f\n", d, t.Fy, Fyss);
            if (d63 < 0 && t.Fy > 0.632 * Fyss) d63 = d;
        }
        note("relaxation length (63%% distance at 2 deg step): %.3f m (sigma_y = C/k_carcass)", d63);
    }
    fclose(f);
    dc_destroy(c);
}

/* ================= 3b. skidpad ================= */
static void test_skidpad(void) {
    dc_car_params P; car_params(&P); dc_car* c = mk(&P, NULL);
    double R0 = 40, pos[3] = {0, 0, 0}; dc_reset(c, pos, 0, 8);
    FILE* f = csv("skidpad.csv", "t,v,ay_g,sw_deg,delta_deg,beta_deg,r,radius,fz0,fz1,fz2,fz3,alpha_f_deg,alpha_r_deg,col_torque,mz0,mz1");
    double h = 1.0 / P.substep_hz, L = c->K[0].hub0.x - c->K[2].hub0.x, ie = 0; pi_t sp = {0};
    dc_telemetry T; dc_input in = {0};
    double ay_max = 0, sw_lin[2] = {0, 0}, ay_lin[2] = {0, 0}; int nlin = 0;
    double sxy = 0, sxx = 0, sx = 0, sy = 0; int n_fit = 0;
    double vt = 8;
    for (int n = 0; n < (int)(140.0 / h); n++) {
        double t = n * h;
        vt = fmin(8 + 0.12 * fmax(0, t - 5), 30);
        double cx = 0, cy = R0;
        double X = c->x.x, Y = c->x.y, psi = yaw_of(c);
        double phi = atan2(Y - cy, X - cx), rad = hypot(X - cx, Y - cy);
        double Ld = fmax(6, 0.8 * c->v.x);
        double pt = phi + Ld / R0; double tx = cx + R0 * cos(pt), ty = cy + R0 * sin(pt);
        double al = atan2(ty - Y, tx - X) - psi; while (al > M_PI) al -= 2 * M_PI; while (al < -M_PI) al += 2 * M_PI;
        ie += (rad - R0) * h;
        double delta = atan(L * 2 * sin(al) / Ld) + 0.002 * ie;
        if (n % 2 == 0) { dc_get_telemetry(c, &T); }
        hands_angle(&in, delta * ratio_of(c));
        speed_ctl(c, &in, vt, T.speed, &sp, h);
        dc_set_input(c, &in); dc_substep(c);
        dc_get_telemetry(c, &T); audit(&T, P.mass * 9.81);
        if (n % 20 == 0 && t > 5) {
            double ayg = T.ay / 9.81, beta = atan2(T.vy, T.vx);
            double dl = 0.5 * (T.w[0].steer + T.w[1].steer);
            fprintf(f, "%.3f,%.3f,%.4f,%.3f,%.4f,%.3f,%.4f,%.3f,%.1f,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%.2f,%.2f\n", t, T.speed, ayg, T.sw_angle * R2D, dl * R2D, beta * R2D, T.yaw_rate, rad,
                    T.w[0].fz, T.w[1].fz, T.w[2].fz, T.w[3].fz, 0.5 * (T.w[0].slip_angle + T.w[1].slip_angle) * R2D, 0.5 * (T.w[2].slip_angle + T.w[3].slip_angle) * R2D,
                    T.column_torque, T.w[0].mz, T.w[1].mz);
            if (fabs(rad - R0) < 1.0 && ayg > ay_max) ay_max = ayg;
            if (ayg > 0.1 && ayg < 0.4 && fabs(rad - R0) < 0.5) { sxy += ayg * dl * R2D; sxx += ayg * ayg; sx += ayg; sy += dl * R2D; n_fit++; }
        }
        if (fabs(rad - R0) > 4) break;
    }
    fclose(f);
    (void)sw_lin; (void)ay_lin; (void)nlin;
    double K = (n_fit * sxy - sx * sy) / (n_fit * sxx - sx * sx);
    note("skidpad R=%.0f m: max steady ay %.3f g; understeer gradient %.2f deg/g (road-wheel), %.1f deg/g at the steering wheel", R0, ay_max, K, K * P.steer_ratio);
    check("skidpad: max lateral accel plausible", ay_max > 0.80 && ay_max < 1.10, "%.3f g (street tyre target ~0.85-0.95 g)", ay_max);
    check("skidpad: understeer (K>0)", K > 0.3 && K < 6, "%.2f deg/g", K);
    dc_destroy(c);
}

/* ================= 4. step steer ================= */
static void test_step_steer(void) {
    dc_car_params P; car_params(&P); dc_car* c = mk(&P, NULL);
    double pos[3] = {0, 0, 0}, V = 80 / 3.6; dc_reset(c, pos, 0, V);
    FILE* f = csv("step_steer.csv", "t,sw_deg,r_degs,ay_g,beta_deg,col_torque,driver_torque,fz0,fz1,fz2,fz3,roll_deg");
    double h = 1.0 / P.substep_hz; pi_t sp = {0}; dc_input in = {0}; dc_telemetry T; memset(&T, 0, sizeof T);
    double t0 = 2.0, sw_amp = 40 * D2R, r_ss = 0, r_pk = 0, t90 = -1, ay_ss = 0;
    double rbuf[20000]; int nb = 0;
    for (int n = 0; n < (int)(7.0 / h); n++) {
        double t = n * h;
        double sw = t < t0 ? 0 : fmin((t - t0) / 0.15, 1) * sw_amp;
        hands_angle(&in, sw);
        if (t < t0) speed_ctl(c, &in, V, T.speed, &sp, h);   /* then hold throttle constant (ISO 7401) */
        dc_set_input(c, &in); dc_substep(c); dc_get_telemetry(c, &T); audit(&T, P.mass * 9.81);
        if (n % 4 == 0) {
            m3 Rm = qtom(c->q); double roll = asin(fmax(-1, fmin(1, Rm.m[2][1]))) * R2D;
            fprintf(f, "%.4f,%.3f,%.3f,%.4f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f,%.3f\n", t, T.sw_angle * R2D, T.yaw_rate * R2D, T.ay / 9.81, atan2(T.vy, T.vx) * R2D,
                    T.column_torque, T.driver_torque, T.w[0].fz, T.w[1].fz, T.w[2].fz, T.w[3].fz, roll);
            if (t > t0 && nb < 20000) rbuf[nb++] = T.yaw_rate;
        }
        if (t > t0 && T.yaw_rate > r_pk) r_pk = T.yaw_rate;
        if (t > 6.5) { r_ss = T.yaw_rate; ay_ss = T.ay; }
    }
    fclose(f);
    for (int k = 0; k < nb; k++) if (rbuf[k] > 0.9 * r_ss) { t90 = k * 4 * h - 0.075; break; }   /* from 50% steer */
    double over = (r_pk / r_ss - 1) * 100;
    note("step steer 80 km/h, sw 40 deg in 0.15 s: yaw rate ss %.2f deg/s, ay ss %.3f g, 90%% rise %.3f s (from 50%% sw), overshoot %.1f %%", r_ss * R2D, ay_ss / 9.81, t90, over);
    check("step steer: stable + well damped", over < 30 && t90 > 0.05 && t90 < 0.6, "overshoot %.1f %%, t90 %.3f s", over, t90);
    dc_destroy(c);
}

/* ================= 5. hands-off oversteer recovery ================= */
#define HO_SW 90   /* hands-off scenario: turn-in steering-wheel angle, deg */
#define HO_HB 0.3  /* handbrake pulse, s */
static void test_hands_off(void) {
    const char* nm[3] = {"handsoff.csv", "handsoff_held.csv", "handsoff_manual.csv"};
    double res[3][4];
    for (int pass = 0; pass < 3; pass++) {
        dc_car_params P; car_params(&P); if (pass == 2) P.assist = 0; dc_car* c = mk(&P, NULL);
        double pos[3] = {0, 0, 0}, V = 22; dc_reset(c, pos, 0, V);
        FILE* f = csv(nm[pass], "t,sw_deg,r_degs,beta_deg,alpha_f_deg,alpha_r_deg,col_torque,mz0,mz1,ay_g,v");
        double h = 1.0 / P.substep_hz; pi_t sp = {0}; dc_input in = {0}; dc_telemetry T; memset(&T, 0, sizeof T);
        double sw_min = 0, t_cs = -1, beta_max = 0; int countersteered = 0;
        for (int n = 0; n < (int)(9.0 / h); n++) {
            double t = n * h;
            in.handbrake = 0; in.clutch = 0;
            if (t < 1.0) { hands_angle(&in, 0); speed_ctl(c, &in, V, T.speed, &sp, h); }
            else if (t < 1.6) { hands_angle(&in, HO_SW * D2R); in.throttle = 0.0; in.brake = 0; in.handbrake = (t > 1.2 && t < 1.2 + HO_HB) ? 1 : 0; in.clutch = in.handbrake; }
            else {
                if (pass != 1) { in.steer_mode = DC_STEER_TORQUE; in.steer_torque = 0; }   /* hands off */
                else hands_angle(&in, HO_SW * D2R);                                          /* hands clamped at the turn-in angle */
                in.throttle = 0.15; in.brake = 0;
            }
            dc_set_input(c, &in); dc_substep(c); dc_get_telemetry(c, &T); audit(&T, P.mass * 9.81);
            double beta = atan2(T.vy, fmax(T.vx, 0.1));
            if (fabs(beta) > beta_max) beta_max = fabs(beta);
            if (t > 1.6 && T.sw_angle < sw_min) sw_min = T.sw_angle;
            if (t > 1.6 && !countersteered && T.sw_angle < 0 && T.yaw_rate > 0) { countersteered = 1; t_cs = t; }
            if (n % 4 == 0) fprintf(f, "%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.4f,%.3f\n", t, T.sw_angle * R2D, T.yaw_rate * R2D, beta * R2D,
                                    0.5 * (T.w[0].slip_angle + T.w[1].slip_angle) * R2D, 0.5 * (T.w[2].slip_angle + T.w[3].slip_angle) * R2D,
                                    T.column_torque, T.w[0].mz, T.w[1].mz, T.ay / 9.81, T.speed);
        }
        fclose(f);
        res[pass][0] = beta_max * R2D; res[pass][1] = sw_min * R2D; res[pass][2] = T.yaw_rate * R2D; res[pass][3] = atan2(T.vy, fmax(T.vx, .1)) * R2D;
        if (pass == 0) {
            note("hands-off: peak |beta| %.1f deg; wheel self-steered to %.1f deg (countersteer, first at t=%.2f s while yaw rate still +); final r %.2f deg/s, beta %.2f deg, sw %.1f deg",
                 res[0][0], res[0][1], t_cs, res[0][2], res[0][3], T.sw_angle * R2D);
            check("hands-off: emergent countersteer", countersteered && sw_min < -5 * D2R * R2D / R2D, "min sw %.1f deg", sw_min * R2D);
            check("hands-off: recovers + self-centres", fabs(res[0][2]) < 3 && fabs(res[0][3]) < 2 && fabs(T.sw_angle * R2D) < 15,
                  "r %.2f deg/s, beta %.2f deg, sw %.1f deg at t=9 s", res[0][2], res[0][3], T.sw_angle * R2D);
        } else if (pass == 1) note("comparison, hands clamped at %d deg: peak |beta| %.1f deg, final r %.1f deg/s (keeps rotating)", HO_SW, res[1][0], res[1][2]);
        else note("comparison, hands off with MANUAL rack: peak |beta| %.1f deg, min sw %.1f deg, final r %.2f deg/s, beta %.2f deg (hydraulic valve boosts the wheel-inertia torque -> slower self-steer)", res[2][0], res[2][1], res[2][2], res[2][3]);
        dc_destroy(c);
    }
}

/* ================= 6. kerb strike + bump ================= */
static void test_kerb(void) {
    const char* nm[2] = {"kerb_fan.csv", "kerb_single.csv"};
    double pk[2][4];
    for (int pass = 0; pass < 2; pass++) {
        dc_car_params P; car_params(&P);
        if (pass == 1) for (int a = 0; a < 2; a++) { P.axle[a].tyre.n_rays = 1; P.axle[a].tyre.n_rows = 1; }
        road_t rd = {1}; dc_car* c = mk(&P, &rd);
        double pos[3] = {0, 0, 0}, V = 60 / 3.6; dc_reset(c, pos, 0, V);
        FILE* f = csv(nm[pass], "t,x,fz0,fz1,fz2,fz3,fx0,col_torque,travel0_mm,travel1_mm,az_g,rim0");
        double h = 1.0 / P.substep_hz; pi_t sp = {0}; dc_input in = {0}; dc_telemetry T; memset(&T, 0, sizeof T);
        double fzp = -1, dmax = 0, fmax_ = 0, ffb_pk = 0, ffb_base = 0; v3 vprev = c->v;
        for (int n = 0; n < (int)(7.5 / h); n++) {
            hands_angle(&in, 0); speed_ctl(c, &in, V, T.speed, &sp, h);
            dc_set_input(c, &in); dc_substep(c); dc_get_telemetry(c, &T); audit(&T, P.mass * 9.81);
            double az = (c->v.z - vprev.z) / h / 9.81; vprev = c->v;
            if (fzp >= 0 && c->x.x > 38) { double d = fabs(T.w[0].fz - fzp) / h; if (d > dmax) dmax = d; }
            fzp = T.w[0].fz;
            if (c->x.x > 38 && T.w[0].fz > fmax_) fmax_ = T.w[0].fz;
            if (c->x.x > 38 && c->x.x < 55 && fabs(T.column_torque) > ffb_pk) ffb_pk = fabs(T.column_torque);
            if (c->x.x > 20 && c->x.x < 37 && fabs(T.column_torque) > ffb_base) ffb_base = fabs(T.column_torque);
            if (n % 2 == 0) fprintf(f, "%.4f,%.4f,%.1f,%.1f,%.1f,%.1f,%.1f,%.3f,%.2f,%.2f,%.3f,%.1f\n", T.time, c->x.x, T.w[0].fz, T.w[1].fz, T.w[2].fz, T.w[3].fz,
                                    T.w[0].fx, T.column_torque, T.w[0].travel * 1000, T.w[1].travel * 1000, az, T.w[0].rim_force);
        }
        fclose(f);
        pk[pass][0] = fmax_; pk[pass][1] = dmax; pk[pass][2] = ffb_pk; pk[pass][3] = ffb_base;
        dc_destroy(c);
    }
    note("kerb (25 mm sharp rumble, 60 km/h): fan envelope peak Fz %.0f N, max dFz/dt %.0f kN/s, |FFB| peak %.2f N m", pk[0][0], pk[0][1] / 1000, pk[0][2]);
    note("                                     single-ray      peak Fz %.0f N, max dFz/dt %.0f kN/s, |FFB| peak %.2f N m", pk[1][0], pk[1][1] / 1000, pk[1][2]);
    check("kerb: envelope removes edge spikes", pk[0][1] < 0.5 * pk[1][1] && pk[0][0] < pk[1][0], "dFz/dt ratio %.2f, peak ratio %.2f", pk[0][1] / pk[1][1], pk[0][0] / pk[1][0]);
    check("kerb: FFB shows the hit", pk[0][2] > 5 * pk[0][3] && pk[0][2] > 0.5, "column torque peak %.2f N m on the kerb vs %.3f N m on smooth tarmac before it", pk[0][2], pk[0][3]);
}

/* ================= 7. puncture ================= */
static void test_puncture(void) {
    dc_car_params P; car_params(&P); dc_car* c = mk(&P, NULL);
    double pos[3] = {0, 0, 0}, V = 22; dc_reset(c, pos, 0, V);
    FILE* f = csv("puncture.csv", "t,pressure_kpa,fz1,defl1_mm,patch1_mm,rim1,y,r_degs,col_torque,sw_deg,fx1,omega_ratio");
    double h = 1.0 / P.substep_hz; pi_t sp = {0}; dc_input in = {0}; dc_telemetry T; memset(&T, 0, sizeof T);
    double rimmax = 0, colmax = 0, p_end = 0, a0 = 0, a1 = 0;
    for (int n = 0; n < (int)(8.0 / h); n++) {
        double t = n * h;
        if (n == (int)(1.0 / h)) dc_puncture(c, 1, 1.0);
        hands_angle(&in, 0); in.hands_k = 60; in.hands_tmax = 15; speed_ctl(c, &in, V, T.speed, &sp, h);
        dc_set_input(c, &in); dc_substep(c); dc_get_telemetry(c, &T); audit(&T, P.mass * 9.81);
        if (t < 1) a0 = T.w[1].patch_half_len;
        if (T.w[1].rim_force > rimmax) rimmax = T.w[1].rim_force;
        if (t > 1 && fabs(T.column_torque) > colmax) colmax = fabs(T.column_torque);
        p_end = T.w[1].pressure; a1 = T.w[1].patch_half_len;
        if (n % 10 == 0) fprintf(f, "%.3f,%.2f,%.1f,%.2f,%.2f,%.1f,%.4f,%.3f,%.3f,%.3f,%.1f,%.4f\n", t, T.w[1].pressure / 1000, T.w[1].fz, T.w[1].deflection * 1000,
                                 T.w[1].patch_half_len * 2000, T.w[1].rim_force, c->x.y, T.yaw_rate * R2D, T.column_torque, T.sw_angle * R2D, T.w[1].fx,
                                 T.w[1].omega * 0.289 / fmax(T.speed, 0.1));
    }
    fclose(f);
    note("puncture FR (tau 1 s) at 80 km/h: pressure %.0f kPa at 8 s, patch length %.0f -> %.0f mm, rim contact force peak %.0f N, column torque peak %.2f N m, lateral drift %.2f m",
         p_end / 1000, a0 * 2000, a1 * 2000, rimmax, colmax, c->x.y);
    check("puncture: patch grows + rim contacts", a1 > 1.5 * a0 && rimmax > 100, "patch x%.2f, rim %.0f N", a1 / a0, rimmax);
    dc_destroy(c);
}

/* ================= 8. assist on vs off ================= */
static void test_assist(void) {
    double park[2] = {0, 0}, road[2] = {0, 0};
    for (int as = 0; as < 2; as++) {
        dc_car_params P; car_params(&P); P.assist = as; dc_car* c = mk(&P, NULL);
        double pos[3] = {0, 0, 0}; dc_reset(c, pos, 0, 0);
        char nm[64]; snprintf(nm, sizeof nm, "assist_%s.csv", as ? "on" : "off");
        FILE* f = csv(nm, "phase,t,sw_deg,driver_torque,col_torque,assist_force,ay_g,speed");
        double h = 1.0 / P.substep_hz; dc_input in = {0}; dc_telemetry T; pi_t sp = {0}; memset(&T, 0, sizeof T);
        in.gear = 0;
        /* parking: settle 1 s, then wheel to 360 deg over 4 s */
        for (int n = 0; n < (int)(5.0 / h); n++) {
            double t = n * h; in.brake = 1;
            hands_angle(&in, t < 1 ? 0 : fmin((t - 1) / 4, 1) * 2 * M_PI); in.hands_tmax = 60; in.hands_k = 200;
            dc_set_input(c, &in); dc_substep(c); dc_get_telemetry(c, &T);
            if (t > 1.5 && fabs(T.driver_torque) > park[as]) park[as] = fabs(T.driver_torque);
            if (n % 10 == 0) fprintf(f, "park,%.3f,%.2f,%.3f,%.3f,%.1f,%.4f,%.3f\n", t, T.sw_angle * R2D, T.driver_torque, T.column_torque, T.assist_force, T.ay / 9.81, T.speed);
        }
        dc_destroy(c);
        /* road: 20 m/s, 0.5 Hz sine 45 deg */
        c = mk(&P, NULL); dc_reset(c, pos, 0, 20); memset(&T, 0, sizeof T); sp.i = 0;
        for (int n = 0; n < (int)(8.0 / h); n++) {
            double t = n * h;
            hands_angle(&in, t < 1 ? 0 : 45 * D2R * sin(M_PI * (t - 1))); in.hands_tmax = 60; in.hands_k = 200;
            speed_ctl(c, &in, 20, T.speed, &sp, h); in.brake = 0;
            dc_set_input(c, &in); dc_substep(c); dc_get_telemetry(c, &T); audit(&T, P.mass * 9.81);
            if (t > 2 && fabs(T.driver_torque) > road[as]) road[as] = fabs(T.driver_torque);
            if (n % 10 == 0) fprintf(f, "road,%.3f,%.2f,%.3f,%.3f,%.1f,%.4f,%.3f\n", t, T.sw_angle * R2D, T.driver_torque, T.column_torque, T.assist_force, T.ay / 9.81, T.speed);
        }
        fclose(f);
        dc_destroy(c);
    }
    note("assist off/on: parking peak driver torque %.1f / %.1f N m; 20 m/s 45deg sine peak %.2f / %.2f N m", park[0], park[1], road[0], road[1]);
    check("assist: reduces effort", park[1] < 0.5 * park[0] && road[1] < road[0], "parking ratio %.2f, road ratio %.2f", park[1] / park[0], road[1] / road[0]);
}

/* ================= 9. performance ================= */
static double now_s(void) { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return ts.tv_sec + 1e-9 * ts.tv_nsec; }
static void test_perf(void) {
    FILE* f = csv("perf.csv", "config,us_per_substep,realtime_factor_at_hz");
    for (int cfg = 0; cfg < 3; cfg++) {
        dc_car_params P; car_params(&P); road_t rd = {2};
        if (cfg == 1) for (int a = 0; a < 2; a++) { P.axle[a].tyre.n_rays = 7; P.axle[a].tyre.n_rows = 1; }
        dc_car* c = mk(&P, cfg == 2 ? &rd : NULL);
        double pos[3] = {0, 0, 0}; dc_reset(c, pos, 0, 20);
        dc_input in = {0}; dc_telemetry T; memset(&T, 0, sizeof T); pi_t sp = {0};
        double h = 1.0 / P.substep_hz; int N = 40000;
        double t0 = now_s();
        for (int n = 0; n < N; n++) {
            hands_angle(&in, 0.6 * sin(n * h)); if (n % 50 == 0) { dc_get_telemetry(c, &T); speed_ctl(c, &in, 20, T.speed, &sp, h * 50); }
            dc_set_input(c, &in); dc_substep(c);
        }
        double us = (now_s() - t0) / N * 1e6;
        const char* nm[3] = {"default 3x13 rays flat", "1x7 rays flat", "3x13 rays rough terrain"};
        fprintf(f, "%s,%.3f,%.1f\n", nm[cfg], us, 1e6 / us / P.substep_hz);
        note("perf %-26s %.2f us/substep -> %.0fx real time at %.0f Hz (one core)", nm[cfg], us, 1e6 / us / P.substep_hz, P.substep_hz);
        if (cfg == 0) check("perf: >= 5x real time at substep rate (< 100 us)", us < 1e6 / (5 * P.substep_hz), "%.2f us/substep, %.1fx real time", us, 1e6 / us / P.substep_hz);
        dc_destroy(c);
    }
    fclose(f);
}

int main(int argc, char** argv) {
    { char sp[256]; snprintf(sp, sizeof sp, "%s/summary.txt", RDIR()); SUM = fopen(sp, "w"); }
    if (!SUM) { perror("results/summary.txt (run from repo root)"); return 1; }
    const char* only = argc > 1 ? argv[1] : NULL;
#define RUN(n, fn) if (!only || strstr(only, n)) { printf("--- %s\n", n); fprintf(SUM, "--- %s\n", n); fn(); fflush(SUM); }
    RUN("kin", test_kinematics);
    RUN("static", test_static);
    RUN("conserve", test_conservation);
    RUN("tyre", test_tyre_rig);
    RUN("skidpad", test_skidpad);
    RUN("step", test_step_steer);
    RUN("handsoff", test_hands_off);
    RUN("kerb", test_kerb);
    RUN("puncture", test_puncture);
    RUN("assist", test_assist);
    RUN("perf", test_perf);
    fprintf(SUM, "--- audit\n");
    check("newton3: worst residual all tests", AUD_F < 1e-6 && AUD_T < 1e-3, "|sum m a - F_ext|/mg = %.2e, |sum dL/dt - T_ext| = %.2e N m", AUD_F, AUD_T);
    fprintf(SUM, "FAILURES: %d\n", NFAIL); printf("FAILURES: %d\n", NFAIL);
    fclose(SUM);
    return NFAIL ? 2 : 0;
}
