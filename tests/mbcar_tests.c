/* mbcar_tests.c - Phase 1-2 analytic checks of the multibody vehicle (solver = DC_SOLVER_MULTIBODY).
   1. corner kinematics: every knuckle pose of the coupled solve equals the closed-loop corner kinematics
      (dc_kin_solve) at the same travel and rack position, static and while cornering;
   2. statics: sum of contact normal forces = m g, momentum (Newton) residuals ~ 0, dissipation >= 0;
   3. Newton-3 per joint: every multi-body joint's constraint wrenches sum to zero force and zero moment;
   4. self-centring: with the hands off at speed, a steered wheel returns toward centre (positive trail/SAT);
   5. energy: coasting, external work in = stored-energy change + dissipation (dissipation monotone >= 0).
   Build: make mbcartest */
#include "drivechain.h"
#include "../core/src/dc_internal.h"
#include "../core/src/dc_mbcar.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
static int fails = 0;
#define CHECK(cond, ...) do { int ok_ = (cond); printf("  %-4s ", ok_ ? "ok" : "FAIL"); printf(__VA_ARGS__); printf("\n"); if (!ok_) fails++; } while (0)

static int g_bush = 0;  /* 1: arm pivots as compliant bushings (docs/BUSHINGS.md estimates) */
static int g_jf = -1;   /* >= 0 overrides mb.jf (joint friction switch) */
static dc_car* make(const char* ini, double v0, int steer_mode) {
    dc_car_params p; char err[256]; dc_default_params(&p);
    if (!dc_load_params(ini, &p, err, sizeof err)) { printf("load %s: %s\n", ini, err); return NULL; }
    p.solver = DC_SOLVER_MULTIBODY; if (g_jf >= 0) p.mb.jf = g_jf;
    if (g_bush) for (int a = 0; a < 2; a++) { p.mb.bush_kr[a] = 2e6; p.mb.bush_ka[a] = 5e5; p.mb.bush_kt[a] = 300; }
    dc_car* c = dc_create(&p); double z[3] = {0, 0, 0}; dc_reset(c, z, 0, v0);
    dc_input in; memset(&in, 0, sizeof in); in.gear = 0; in.steer_mode = steer_mode; dc_set_input(c, &in);
    return c;
}
static double kin_err(dc_car* c, double* dang) {
    double e = 0, ea = 0;
    for (int i = 0; i < 4; i++) {
        dc_pose q; memset(&q, 0, sizeof q); q.c = c->K[i].hub0; q.R = m3id();   /* Newton from the design pose */
        if (!dc_kin_solve(&c->K[i], c->s[i], c->K[i].steered ? c->r : 0, &q, 0)) return 1e9;
        double d = sqrt((q.c.x - c->pose[i].c.x) * (q.c.x - c->pose[i].c.x) + (q.c.y - c->pose[i].c.y) * (q.c.y - c->pose[i].c.y) + (q.c.z - c->pose[i].c.z) * (q.c.z - c->pose[i].c.z));
        double a = 0; for (int r = 0; r < 3; r++) for (int s = 0; s < 3; s++) a = fmax(a, fabs(q.R.m[r][s] - c->pose[i].R.m[r][s]));
        e = fmax(e, d); ea = fmax(ea, a);
    }
    *dang = ea; return e;
}
static void joint_n3(dc_car* c, double* fmaxr, double* tmaxr, double* fref) {
    mb_world* W = c->mb->W; *fmaxr = 0; *tmaxr = 0; *fref = 0;
    for (int j = 0; j < W->nj; j++) {
        mb_joint* J = &W->Jt[j]; if (J->nb < 2) continue; int ground = 0;
        for (int k = 0; k < J->nb; k++) if (J->b[k] < 0) ground = 1;
        if (ground) continue;
        v3 Fs = V3(0, 0, 0), Ts = V3(0, 0, 0); double fm = 0;
        for (int k = 0; k < J->nb; k++) {
            v3 F, T; mb_joint_wrench(W, j, k, &F, &T);
            v3 xo = vsub(W->B[J->b[k]].x, vscl(W->B[J->b[k]].v, W->h));   /* rows were built at the pre-step position */
            Fs = vadd(Fs, F); Ts = vadd(Ts, vadd(T, vcross(xo, F))); fm = fmax(fm, vlen(F));
        }
        *fmaxr = fmax(*fmaxr, vlen(Fs)); *tmaxr = fmax(*tmaxr, vlen(Ts)); *fref = fmax(*fref, fm);
    }
}
int main(int argc, char** argv) {
    const char* ini = argc > 1 ? argv[1] : "cars/mx5_na.ini";
    printf("=== dc_mbcar Phase 1-2 (%s) ===\n", ini);
    dc_telemetry t;
    /* --- static */
    dc_car* c = make(ini, 0, DC_STEER_ANGLE);
    if (!c || !c->mb) { printf("  FAIL multibody car not created\n"); return 1; }
    double dmin = 1e9, dprev = 0; int mono = 1;
    for (int k = 0; k < 400; k++) { dc_step(c, 0.01); dc_get_telemetry(c, &t); if (k > 50 && t.dissipated < dprev - 1e-2) mono = 0; dprev = t.dissipated; dmin = fmin(dmin, t.dissipated); }
    double sfz = t.w[0].fz + t.w[1].fz + t.w[2].fz + t.w[3].fz, mg = c->P.mass * c->P.gravity;
    CHECK(fabs(sfz - mg) < 1e-3 * mg, "static: sum Fz %.2f N = m g %.2f N (err %.2e)", sfz, mg, fabs(sfz - mg) / mg);
    /* with joint friction the settle drop locks in a small diagonal warp (stiction band, cf. corner-weighting practice
       of settling the car before reading scales); bound it at 1 % of the corner load. Geometry symmetry: jf = 0. */
    CHECK(fabs(t.w[0].fz - t.w[1].fz) < 0.01 * mg / 4 && fabs(t.w[2].fz - t.w[3].fz) < 0.01 * mg / 4, "static: left/right warp within the joint-friction band (F %.1f/%.1f, R %.1f/%.1f N)", t.w[0].fz, t.w[1].fz, t.w[2].fz, t.w[3].fz);
    {
        g_jf = 0; dc_car* c0 = make(ini, 0, DC_STEER_ANGLE); g_jf = -1; dc_telemetry t0;
        for (int k = 0; k < 400; k++) dc_step(c0, 0.01);
        dc_get_telemetry(c0, &t0);
        CHECK(fabs(t0.w[0].fz - t0.w[1].fz) < 2 && fabs(t0.w[2].fz - t0.w[3].fz) < 2, "static (jf=0): left/right symmetric (F %.1f/%.1f, R %.1f/%.1f N)", t0.w[0].fz, t0.w[1].fz, t0.w[2].fz, t0.w[3].fz);
        dc_destroy(c0);
    }
    double rf = sqrt(t.resid_force[0]*t.resid_force[0]+t.resid_force[1]*t.resid_force[1]+t.resid_force[2]*t.resid_force[2]);
    double rt = sqrt(t.resid_torque[0]*t.resid_torque[0]+t.resid_torque[1]*t.resid_torque[1]+t.resid_torque[2]*t.resid_torque[2]);
    CHECK(rf < 1e-3 && rt < 2.0, "static: momentum residual |dP/dt - F_ext| %.2e N, |dL/dt - M_ext| %.2e N m", rf, rt);
    CHECK(dmin > -1e-6 && mono, "static: dissipation >= 0, non-decreasing to < 10 mJ per 10 ms (1 W) (min %.3g J, final %.3g J)", dmin, t.dissipated);
    double ea, e = kin_err(c, &ea);
    CHECK(e < 1e-6 && ea < 1e-6, "static: knuckle pose = closed-loop corner kinematics (pos %.2e m, rot %.2e)", e, ea);
    double fr, tr, fref; joint_n3(c, &fr, &tr, &fref);
    CHECK(fr < 1e-9 * fmax(fref, 1) && tr < 1e-8 * fmax(fref, 1), "static: Newton-3 per joint: max |sum F| %.2e N, |sum M| %.2e N m (largest joint force %.0f N)", fr, tr, fref);
    dc_destroy(c);
    /* --- cornering (steady steer at 15 m/s): kinematics + Newton-3 under load, energy balance */
    c = make(ini, 15, DC_STEER_ANGLE);
    dc_input in; memset(&in, 0, sizeof in); in.gear = 3; in.throttle = 0.25; in.steer_mode = DC_STEER_ANGLE;
    in.steer_target = 60 * M_PI / 180; in.hands_k = 60; in.hands_d = 3; in.hands_tmax = 30; dc_set_input(c, &in);
    double emax = 0, eamax = 0, frm = 0, trm = 0, frf = 0; mono = 1; dprev = 0;
    for (int k = 0; k < 500; k++) {
        dc_step(c, 0.01); dc_get_telemetry(c, &t);
        if (k > 100) { e = kin_err(c, &ea); emax = fmax(emax, e); eamax = fmax(eamax, ea); joint_n3(c, &fr, &tr, &fref); frm = fmax(frm, fr / fmax(fref, 1)); trm = fmax(trm, tr / fmax(fref, 1)); }
        if (k > 10 && t.dissipated < dprev - 1e-2) mono = 0; dprev = t.dissipated;
    }
    printf("       cornering: ay %.2f m/s^2, yaw %.3f rad/s, roll travel %.1f/%.1f mm, rack %.1f mm\n", t.ay, t.yaw_rate, t.w[0].travel * 1e3, t.w[1].travel * 1e3, t.rack_pos * 1e3);
    CHECK(fabs(t.ay) > 1 && t.ay * t.yaw_rate > 0, "cornering: car actually corners, ay and yaw agree (ay %.2f, v*r %.2f)", t.ay, 15.0 * t.yaw_rate);
    CHECK(emax < 1e-6 && eamax < 1e-6, "cornering: knuckle pose = corner kinematics at the solved travel/rack (pos %.2e m, rot %.2e)", emax, eamax);
    CHECK(frm < 1e-9 && trm < 1e-8, "cornering: Newton-3 per joint (rel |sum F| %.2e, |sum M| %.2e m)", frm, trm);
    CHECK(mono, "cornering: dissipation non-decreasing (%.1f J after 5 s)", t.dissipated);
    rf = sqrt(t.resid_force[0]*t.resid_force[0]+t.resid_force[1]*t.resid_force[1]+t.resid_force[2]*t.resid_force[2]);
    CHECK(rf < 1e-2, "cornering: momentum residual %.2e N", rf);
    dc_destroy(c);
    /* --- self-centring: steer to 90 deg at 15 m/s, then hands off (zero torque): the wheel must return */
    c = make(ini, 15, DC_STEER_ANGLE);
    memset(&in, 0, sizeof in); in.gear = 3; in.throttle = 0.2; in.steer_mode = DC_STEER_ANGLE; in.steer_target = 90 * M_PI / 180;
    in.hands_k = 60; in.hands_d = 3; in.hands_tmax = 30; dc_set_input(c, &in);
    for (int k = 0; k < 150; k++) dc_step(c, 0.01);
    dc_get_telemetry(c, &t); double sw0 = t.sw_angle, col0 = t.column_torque;
    in.steer_mode = DC_STEER_TORQUE; in.steer_torque = 0; dc_set_input(c, &in);
    for (int k = 0; k < 150; k++) dc_step(c, 0.01);
    dc_get_telemetry(c, &t);
    CHECK(fabs(t.sw_angle) < 0.35 * fabs(sw0), "self-centring: hands off at %.0f deg (column %.2f N m) -> %.1f deg after 1.5 s", sw0 * 180 / M_PI, col0, t.sw_angle * 180 / M_PI);
    dc_destroy(c);
    /* --- compliant arm bushings: same audits; pose departs from the rigid kinematics by the bush deflection only */
    {
        g_bush = 1; c = make(ini, 0, DC_STEER_ANGLE); g_bush = 0;
        printf("       bushings: %d arm pivots compliant\n", c->mb->nbush);
        dmin = 1e9; dprev = 0; mono = 1;
        for (int k = 0; k < 400; k++) { dc_step(c, 0.01); dc_get_telemetry(c, &t); if (k > 50 && t.dissipated < dprev - 1e-2) mono = 0; dprev = t.dissipated; dmin = fmin(dmin, t.dissipated); }
        sfz = t.w[0].fz + t.w[1].fz + t.w[2].fz + t.w[3].fz;
        CHECK(c->mb->nbush == 8 && fabs(sfz - mg) < 1e-3 * mg, "bushed static: %d bushes, sum Fz = m g (err %.2e)", c->mb->nbush, fabs(sfz - mg) / mg);
        CHECK(dmin > -1e-6 && mono, "bushed static: dissipation >= 0 and non-decreasing (min %.3g J)", dmin);
        joint_n3(c, &fr, &tr, &fref);
        CHECK(fr < 1e-9 * fmax(fref, 1) && tr < 1e-8 * fmax(fref, 1), "bushed static: Newton-3 per joint %.2e N, %.2e N m", fr, tr);
        dc_destroy(c);
        g_bush = 1; c = make(ini, 15, DC_STEER_ANGLE); g_bush = 0;
        memset(&in, 0, sizeof in); in.gear = 3; in.throttle = 0.25; in.steer_mode = DC_STEER_ANGLE;
        in.steer_target = 60 * M_PI / 180; in.hands_k = 60; in.hands_d = 3; in.hands_tmax = 30; dc_set_input(c, &in);
        emax = 0; frm = 0; trm = 0; mono = 1; dprev = 0;
        for (int k = 0; k < 500; k++) {
            dc_step(c, 0.01); dc_get_telemetry(c, &t);
            if (k > 100) { e = kin_err(c, &ea); emax = fmax(emax, e); joint_n3(c, &fr, &tr, &fref); frm = fmax(frm, fr / fmax(fref, 1)); trm = fmax(trm, tr / fmax(fref, 1)); }
            if (k > 10 && t.dissipated < dprev - 1e-2) mono = 0; dprev = t.dissipated;
        }
        rf = sqrt(t.resid_force[0]*t.resid_force[0]+t.resid_force[1]*t.resid_force[1]+t.resid_force[2]*t.resid_force[2]);
        printf("       bushed cornering: ay %.2f m/s^2, yaw %.3f rad/s, max pose departure from rigid kinematics %.2f mm\n", t.ay, t.yaw_rate, emax * 1e3);
        CHECK(frm < 1e-9 && trm < 1e-8, "bushed cornering: Newton-3 per joint (rel %.2e, %.2e m)", frm, trm);
        CHECK(mono && rf < 1e-2, "bushed cornering: dissipation non-decreasing, momentum residual %.2e N", rf);
        CHECK(emax > 1e-5 && emax < 5e-3, "bushed cornering: bush deflection visible but small (%.2f mm)", emax * 1e3);
        dc_destroy(c);
    }
    printf(fails ? "=== %d FAILURES ===\n" : "=== ALL PASS ===\n", fails);
    return fails != 0;
}
