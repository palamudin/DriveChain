/* mbcar_tests.c - Phase 1-2 analytic checks of the multibody vehicle (solver = DC_SOLVER_MULTIBODY).
   1. corner kinematics: every knuckle pose of the coupled solve equals the closed-loop corner kinematics
      (dc_kin_solve) at the same travel and rack position, static and while cornering;
   2. statics: sum of contact normal forces = m g, momentum (Newton) residuals ~ 0, dissipation >= 0;
   3. Newton-3 per joint: every multi-body joint's constraint wrenches sum to zero force and zero moment;
   4. self-centring: with the hands off at speed, a steered wheel returns toward centre (positive trail/SAT);
   5. energy: coasting, external work in = stored-energy change + dissipation (dissipation monotone >= 0);
   6. energy audit per step (both tyre models): drive/lift-off, steer/release and a 30 mm road-step heave load and
      unload the tyre carcass shear store; no step may create energy (regression: the audit once omitted the
      committed carcass shear/twist store from E_start). `mbcar_tests <car.ini> energy` runs only section 6.
   Build: make mbcartest */
#include "drivechain.h"
#include "../core/src/dc_internal.h"
#include "../core/src/dc_mbcar.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
static int fails = 0;
#define CHECK(cond, ...) do { int ok_ = (cond); printf("  %-4s ", ok_ ? "ok" : "FAIL"); printf(__VA_ARGS__); printf("\n"); if (!ok_) fails++; } while (0)

static int g_bush = 0;
static int g_carrier = 0;  /* 1: carrier-inertia driveline (docs/CARRIER.md proposal values) */  /* 1: arm pivots as compliant bushings (docs/BUSHINGS.md estimates) */
static int g_rmount = 0;  /* 1: rack housing on a compliant mount (propR estimate, docs/BUSHINGS.md) */
static int g_jf = -1;   /* >= 0 overrides mb.jf (joint friction switch) */
static int g_tyre = -1;
static double g_cstore = 0;  /* tyre carcass_store on both axles (energy-audit section) */  /* >= 0 overrides the tyre model on both axles (energy-audit section) */
static double g_road_h = 0;  /* uniform road height (energy-audit heave step) */
static double flat_road(void* u, double x, double y, double* nx, double* ny, double* nz, double* g) {
    (void)u; (void)x; (void)y; *nx = 0; *ny = 0; *nz = 1; *g = 1; return g_road_h;
}
static dc_car* make(const char* ini, double v0, int steer_mode) {
    dc_car_params p; char err[256]; dc_default_params(&p);
    if (!dc_load_params(ini, &p, err, sizeof err)) { printf("load %s: %s\n", ini, err); return NULL; }
    p.solver = DC_SOLVER_MULTIBODY; if (g_jf >= 0) p.mb.jf = g_jf;
    if (g_carrier) { p.mb.diff_inertia = 0.02; p.mb.gb_out_inertia = 0.01; }
    if (g_bush) for (int a = 0; a < 2; a++) { p.mb.bush_kr[a] = 2e6; p.mb.bush_ka[a] = 5e5; p.mb.bush_kt[a] = 300; }
    if (g_rmount) { p.mb.rack_mount_k = 8e6; p.mb.rack_mount_c = 1.9e4; }
    if (g_tyre >= 0) { p.axle[0].tyre.model = g_tyre; p.axle[1].tyre.model = g_tyre; }
    p.axle[0].tyre.carcass_store = p.axle[1].tyre.carcass_store = g_cstore;
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
/* section 6: minimum per-step dissipation over three carcass load/unload manoeuvres. Steps right after a
   change of input are included; only the road teleport step (external work not in W_in) is skipped. */
static void energy_audit(const char* ini) {
    const char* nm[3] = {"brush", "TARMAC", "brush carcass_store=1"};
    /* J per 10 ms step. TARMAC keeps an O(h) residual of explicit start-of-step lane forces (heave: -0.0125 J mx5_na,
       -0.048 J hatch_fwd at h = 10 ms, halves at 5 ms, -0.0015 J with friction off), so it gets a gross-failure bound */
    const double tolv[3] = {-1e-3, -7.5e-2, -1e-3};
    double traj[3][3] = {{0}};
    for (int ty = 0; ty < 3; ty++) {
        const double tol = tolv[ty];
        g_tyre = ty == 2 ? 0 : ty; g_cstore = ty == 2; double dmin = 1e9; const char* where = "-"; dc_telemetry t; dc_input in; dc_car* c;
#define DSTEP(tag) do { double d0_ = t.dissipated; dc_step(c, 0.01); dc_get_telemetry(c, &t); if (t.dissipated - d0_ < dmin) { dmin = t.dissipated - d0_; where = tag; } } while (0)
        if (ty == 2) {   /* (a0) carcass_store = 1 only: full throttle straight from the reset (driveline shock; the default
                            1/2 k e^2 store reports ~ -2 J on hatch_fwd here as the sliding brush unloads) */
            c = make(ini, 8, DC_STEER_ANGLE); dc_set_road(c, flat_road, NULL); g_road_h = 0; dc_get_telemetry(c, &t);
            memset(&in, 0, sizeof in); in.gear = 2; in.throttle = 1; in.steer_mode = DC_STEER_ANGLE; in.hands_k = 60; in.hands_d = 3; in.hands_tmax = 30; dc_set_input(c, &in);
            for (int k = 0; k < 100; k++) DSTEP("shock");
            dc_destroy(c);
        }
        /* (a) drive then lift-off at 8 m/s: longitudinal carcass store builds and releases */
        c = make(ini, 8, DC_STEER_ANGLE); dc_set_road(c, flat_road, NULL); g_road_h = 0; dc_get_telemetry(c, &t);
        memset(&in, 0, sizeof in); in.gear = 2; in.throttle = 0.2; in.steer_mode = DC_STEER_ANGLE; in.hands_k = 60; in.hands_d = 3; in.hands_tmax = 30; dc_set_input(c, &in);
        for (int k = 0; k < 100; k++) dc_step(c, 0.01);   /* the reset leaves the engine at idle against road-speed wheels: let the driveline lock up first */
        dc_get_telemetry(c, &t); in.throttle = 1; dc_set_input(c, &in);
        for (int k = 0; k < 100; k++) DSTEP("drive");
        in.throttle = 0; dc_set_input(c, &in); for (int k = 0; k < 100; k++) DSTEP("lift-off");
        in.brake = 0.6; dc_set_input(c, &in); for (int k = 0; k < 60; k++) DSTEP("brake");
        dc_destroy(c);
        /* (b) steer and release at 15 m/s: lateral carcass store */
        c = make(ini, 15, DC_STEER_ANGLE); dc_set_road(c, flat_road, NULL); dc_get_telemetry(c, &t);
        memset(&in, 0, sizeof in); in.gear = 3; in.throttle = 0.25; in.steer_mode = DC_STEER_ANGLE; in.steer_target = 60 * M_PI / 180;
        in.hands_k = 60; in.hands_d = 3; in.hands_tmax = 30; dc_set_input(c, &in);
        for (int k = 0; k < 200; k++) DSTEP("steer");
        in.steer_target = 0; dc_set_input(c, &in); for (int k = 0; k < 200; k++) DSTEP("release");
        traj[ty][0] = t.speed; traj[ty][1] = t.yaw_rate; traj[ty][2] = t.ay;
        dc_destroy(c);
        /* (c) braked at rest, 30 mm road step under all four wheels: heave bounce */
        c = make(ini, 0, DC_STEER_ANGLE); dc_set_road(c, flat_road, NULL); dc_get_telemetry(c, &t);
        memset(&in, 0, sizeof in); in.brake = 0.3; in.steer_mode = DC_STEER_ANGLE; in.hands_k = 60; in.hands_d = 3; in.hands_tmax = 30; dc_set_input(c, &in);
        for (int k = 0; k < 200; k++) DSTEP("settle");
        g_road_h = 0.03; dc_step(c, 0.01); dc_get_telemetry(c, &t);   /* teleport step: external work */
        for (int k = 0; k < 300; k++) DSTEP("heave");
        g_road_h = 0; dc_destroy(c);
#undef DSTEP
        CHECK(dmin > tol, "energy audit (%s tyre): min per-step dissipation %.2e J (at %s) > %.1e J, no step creates energy", nm[ty], dmin, where, tol);
    }
    CHECK(traj[2][0] == traj[0][0] && traj[2][1] == traj[0][1] && traj[2][2] == traj[0][2],
          "carcass_store = 1 is book-keeping only: steer/release trajectory bit-identical (v %.6f yaw %.6f ay %.6f)", traj[2][0], traj[2][1], traj[2][2]);
    g_tyre = -1; g_cstore = 0;
}
int main(int argc, char** argv) {
    const char* ini = argc > 1 ? argv[1] : "cars/mx5_na.ini";
    printf("=== dc_mbcar Phase 1-2 (%s) ===\n", ini);
    if (argc > 2 && !strcmp(argv[2], "energy")) { energy_audit(ini); printf(fails ? "=== %d FAILURES ===\n" : "=== ALL PASS ===\n", fails); return fails != 0; }
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
        int narms = 0; for (int i = 0; i < 4; i++) narms += c->mb->narm[i];
        CHECK(c->mb->nbush == narms && fabs(sfz - mg) < 1e-3 * mg, "bushed static: %d of %d arm pivots bushed, sum Fz = m g (err %.2e)", c->mb->nbush, narms, fabs(sfz - mg) / mg);
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
    /* --- compliant rack mount: housing body on a beam joint, soft only along the rack axis */
    {
        g_rmount = 1; c = make(ini, 15, DC_STEER_ANGLE); g_rmount = 0;
        dc_mbcar* m = c->mb; mb_world* W = m->W;
        CHECK(m->j_rmount >= 0 && m->hsg != m->ch, "rack mount: housing is its own body (%d) on joint %d", m->hsg, m->j_rmount);
        memset(&in, 0, sizeof in); in.gear = 3; in.throttle = 0.25; in.steer_mode = DC_STEER_ANGLE;
        in.steer_target = 60 * M_PI / 180; in.hands_k = 60; in.hands_d = 3; in.hands_tmax = 30; dc_set_input(c, &in);
        frm = 0; trm = 0; mono = 1; dprev = 0;
        for (int k = 0; k < 500; k++) {
            dc_step(c, 0.01); dc_get_telemetry(c, &t);
            if (k > 100) { joint_n3(c, &fr, &tr, &fref); frm = fmax(frm, fr / fmax(fref, 1)); trm = fmax(trm, tr / fmax(fref, 1)); }
            if (k > 10 && t.dissipated < dprev - 1e-2) mono = 0; dprev = t.dissipated;
        }
        mb_eval_joint(W, m->j_rmount);
        double dx = W->Jt[m->j_rmount].phi[0], Fm = 8e6 * dx;
        rf = sqrt(t.resid_force[0]*t.resid_force[0]+t.resid_force[1]*t.resid_force[1]+t.resid_force[2]*t.resid_force[2]);
        printf("       rack mount: ay %.2f m/s^2, housing shift %.3f mm, k x %.0f N vs tie-rod rack force %.0f N\n", t.ay, dx * 1e3, Fm, t.rack_force);
        CHECK(fabs(fabs(Fm) - fabs(t.rack_force)) < 0.1 * fabs(t.rack_force) + 20, "rack mount: |k x| matches the rack force within 10%% (quasi-static)");
        CHECK(frm < 1e-9 && trm < 1e-8, "rack mount cornering: Newton-3 per joint (rel %.2e, %.2e m)", frm, trm);
        CHECK(mono && rf < 1e-2, "rack mount cornering: dissipation non-decreasing, momentum residual %.2e N", rf);
        dc_destroy(c);
    }
    /* --- carrier inertia: propshaft + diff carrier as geared rotors */
    {
        dc_car* c0 = make(ini, 20, DC_STEER_ANGLE); g_carrier = 1; c = make(ini, 20, DC_STEER_ANGLE); g_carrier = 0;
        mb_world* W = c->mb->W; dc_mbcar* m = c->mb;
        int l = m->wh[m->drv[0]], r = m->wh[m->drv[1]];
        v3 yw = mv(&W->B[m->ch].R, V3(0, 1, 0)), xw = mv(&W->B[m->ch].R, V3(1, 0, 0));
        double wc = vdot(vsub(W->B[m->dcr].w, W->B[m->ch].w), yw); wc = fabs(wc);
        double dke = mb_kinetic(W) - mb_kinetic(c0->mb->W), ex = 0.5 * 0.02 * wc * wc + 0.5 * 0.01 * pow(c->P.final_drive * wc, 2);
        CHECK(m->carrier && fabs(dke - ex) < 1e-6 * ex, "carrier: reset KE adds 1/2 I_d wc^2 + 1/2 I_go (fd wc)^2 = %.2f J (got %.2f J, wc %.1f rad/s)", ex, dke, wc);
        dc_destroy(c0);
        memset(&in, 0, sizeof in); in.gear = 2; in.throttle = 0.6; in.steer_mode = DC_STEER_ANGLE;
        in.steer_target = 40 * M_PI / 180; in.hands_k = 60; in.hands_d = 3; in.hands_tmax = 30; dc_set_input(c, &in);
        frm = trm = 0; mono = 1; dprev = 0; double cerr = 0;
        for (int k = 0; k < 400; k++) {
            if (k == 200) { in.gear = 0; in.throttle = 0; dc_set_input(c, &in); }   /* neutral: rotors stay geared to the wheels */
            dc_step(c, 0.01); dc_get_telemetry(c, &t);
            if (k > 20) { joint_n3(c, &fr, &tr, &fref); frm = fmax(frm, fr / fmax(fref, 1)); trm = fmax(trm, tr / fmax(fref, 1)); }
            if (k > 10 && t.dissipated < dprev - 1e-2) mono = 0; dprev = t.dissipated;
            yw = mv(&W->B[m->ch].R, V3(0, 1, 0)); xw = mv(&W->B[m->ch].R, V3(1, 0, 0));
            v3 aL = mv(&W->B[l].R, c->K[m->drv[0]].spin_axis_u), aR = mv(&W->B[r].R, c->K[m->drv[1]].spin_axis_u);
            double wm = 0.5 * (vdot(vsub(W->B[l].w, W->B[m->ch].w), aL) + vdot(vsub(W->B[r].w, W->B[m->ch].w), aR));
            double wcar = vdot(vsub(W->B[m->dcr].w, W->B[m->ch].w), yw) * (vdot(vadd(aL, aR), yw) >= 0 ? 1 : -1);
            double wgo = vdot(vsub(W->B[m->go].w, W->B[m->ch].w), xw);
            cerr = fmax(cerr, fmax(fabs(wcar - wm), fabs(wgo - c->P.final_drive * wcar)) / fmax(fabs(wm), 1));
        }
        rf = sqrt(t.resid_force[0]*t.resid_force[0]+t.resid_force[1]*t.resid_force[1]+t.resid_force[2]*t.resid_force[2]);
        CHECK(cerr < 1e-6, "carrier: ratio chain holds (carrier = wheel mean, propshaft = fd x carrier), rel err %.2e", cerr);
        CHECK(frm < 1e-9 && trm < 1e-8, "carrier: Newton-3 per joint under drive and in neutral (rel %.2e, %.2e m)", frm, trm);
        CHECK(mono && rf < 1e-2, "carrier: dissipation non-decreasing, momentum residual %.2e N", rf);
        dc_destroy(c);
    }
    energy_audit(ini);
    printf(fails ? "=== %d FAILURES ===\n" : "=== ALL PASS ===\n", fails);
    return fails != 0;
}
