#include "sim.h"
#include "roadbake.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
static double now_us(void) { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return ts.tv_sec * 1e6 + ts.tv_nsec * 1e-3; }
static double clampd(double v, double a, double b) { return v < a ? a : (v > b ? b : v); }

/* Demo car setup (car.setup in demo.ini, or --set car.setup=...):
 *   default = the car file plus the documented proposal sets, applied where they were derived:
 *             propBR (compliant arm-pivot bushings kr 2e6 / ka 5e5 / kt 300 + rack mount 8e6 N/m, docs/BUSHINGS.md)
 *             on multibody-core cars, and the TARMAC tyre set propD (docs/TYRE_DATA.md) on 185/60R14 cars, the
 *             tyre it was fitted to. Other cars keep their own values.
 *   legacy  = the car file as-is (the setup before the proposals became the demo default).
 * An absent key means legacy, so the headless harnesses built on this file keep their baselines unless they ask. */
static const char* const SETUP_BR[] = {"mb_bush_kr", "2e6, 2e6", "mb_bush_ka", "5e5, 5e5", "mb_bush_kt", "300, 300",
                                       "mb_rack_mount_k", "8e6", "mb_rack_mount_c", "1.9e4", NULL};
static const char* const SETUP_D[] = {"tyre_mu_y", "0.955", "tyre_mu_x", "0.926", "tyre_ls_exp_y", "0.90", "tyre_ls_exp_x", "0.92",
                                      "tyre_trail_factor", "1.3", "tyre_model", "1", "tyre_tm_shape", "1", "tyre_tm_damp_mode", "1",
                                      "tyre_tm_load_sens", "0.10", "tyre_tm_slide_floor", "0.70", "tyre_tm_slide_exp", "2",
                                      "tyre_tm_slide_speed", "8", NULL};
int sim_apply_setup(dc_car_params* P, const char* setup, char* err, int errlen) {
    if (!strcmp(setup, "legacy")) { fprintf(stderr, "[car] setup legacy: car file as-is\n"); return 1; }
    if (strcmp(setup, "default")) { snprintf(err, errlen, "car.setup = '%s': use default or legacy", setup); return 0; }
    int br = P->solver == DC_SOLVER_MULTIBODY, d = 1;
    for (int a = 0; a < 2; a++) { const dc_tyre_params* t = &P->axle[a].tyre; d &= fabs(t->width - 0.185) < 0.003 && fabs(t->radius - 0.289) < 0.004; }
    for (int i = 0; br && SETUP_BR[i]; i += 2) if (!dc_set_param(P, "car", SETUP_BR[i], SETUP_BR[i + 1])) { snprintf(err, errlen, "setup key %s", SETUP_BR[i]); return 0; }
    for (int a = 0; d && a < 2; a++) for (int i = 0; SETUP_D[i]; i += 2)
        if (!dc_set_param(P, a ? "rear" : "front", SETUP_D[i], SETUP_D[i + 1])) { snprintf(err, errlen, "setup key %s", SETUP_D[i]); return 0; }
    fprintf(stderr, "[car] setup default: bushings + rack mount (propBR) %s, TARMAC tyre propD %s\n",
           br ? "on" : "n/a (legacy core)", d ? "on" : "n/a (not 185/60R14)");
    return 1;
}

/* [track] bake = 0 (live road) | 32 | 16: bake the road lattice under the drivable band once (prewarm); rebuilt when
   the track or the lattice pitch changes (F6 map switch rebuilds the track in place) */
static void bake_sync(sim_t* S) {
    const int bits = cfg_int(S->cfg, "track.bake", 0);
    const track_t* T = S->trk;
    double sig[5] = {T->layout, T->n, T->length, T->n ? T->px[0] + 7 * T->py[0] : 0, bits ? S->P.road_resolve * bits : 0};
    /* one bake per process, shared by every sim on the same track (test rigs build many cars; the tiles only read) */
    static roadbake* G_BAKE; static double G_SIG[5];
    memcpy(S->bake_sig, sig, sizeof sig);
    if (!bits) { S->bake = NULL; return; }
    if (G_BAKE && !memcmp(sig, G_SIG, sizeof sig)) { S->bake = G_BAKE; return; }
    roadbake_free(G_BAKE); G_BAKE = NULL; S->bake = NULL; memcpy(G_SIG, sig, sizeof sig);
    char err[160] = "";
    double band = cfg_num(S->cfg, "track.bake_band", T->half_w + T->kerb_w + 1.0);
    S->bake = roadbake_build(T, S->P.road_resolve, bits, band, err, sizeof err);
    if (!S->bake) { fprintf(stderr, "[track] bake off: %s\n", err); return; }
    G_BAKE = S->bake;
    roadbake_stats st; roadbake_get_stats(S->bake, &st);
    fprintf(stderr, "[track] baked road: %d-bit, band +-%.1f m, %lld tiles (%lld nodes, %lld exact zeros kept), %.1f MB, %.2f s, max err %.2e m rms %.2e m, %d grip values\n",
            st.bits, band, st.tiles, st.nodes, st.zeros, st.bytes / 1048576.0, st.bake_s, st.max_err, st.rms_err, st.grip_vals);
}
static int make_car(sim_t* S) {
    if (S->car) dc_destroy(S->car);
    S->car = dc_create(&S->P);
    if (!S->car) return 0;
    bake_sync(S);
    if (S->bake) dc_set_road(S->car, roadbake_road_fn, S->bake); else dc_set_road(S->car, track_road_fn, S->trk); dc_set_road_flags(S->car, DC_ROAD_HEIGHT_ONLY | DC_ROAD_STATIC);   /* tracks are static */
    { int nt = (int)cfg_num(S->cfg, "physics.threads", 1); S->threads = nt > 1 ? dc_set_threads(S->car, nt) : 1; }
    if (S->threads > 1) dc_set_pool_opts(S->car, cfg_num(S->cfg, "physics.pool_spin_us", 500), -2);   /* opt-in step worker threads (track_road_fn is pure) */
    /* thread placement: helpers pinned (or reset to the process mask: a new pool inherits the physics thread's pin)
       here on every car rebuild, the physics thread pins itself on its next tick.
       Harness default off (parallel test runs must not share cores); the interactive demo sets auto. */
    { const char* as = cfg_str(S->cfg, "physics.affinity", "off");
      int prio = dc_affinity_parse_prio(cfg_str(S->cfg, "physics.priority", "normal"));
      int c0 = S->aff.cpu[0], p0 = S->prio, had = S->aff.nthreads > 0;
      dc_affinity_plan(as, S->threads, &S->aff); S->prio = prio;
      S->aff_pool_ok = S->threads > 1 ? dc_set_pool_cpus(S->car, S->aff.cpu, S->threads, prio) : -1;
      S->aff_hprio = S->threads > 1 && prio ? -1 : 6;
      if (!had || c0 != S->aff.cpu[0] || p0 != prio) S->aff_self_res = -1; }
    S->offload_patch = dc_set_offload(S->car, DC_STAGE_PATCH, dc_offload_parse(cfg_str(S->cfg, "offload.patch", "ref")));   /* compute-offload lane (fast model) */
    dc_set_offload_xcheck(S->car, cfg_int(S->cfg, "offload.xcheck", 0));   /* 0 off; n = re-run every n-th non-cpu lane batch on cpu, same tick */
    { const char* fr = cfg_str(S->cfg, "physics.factor_reuse", "off");   /* tolerance factor reuse (fast model; off | on) */
      int on = !strcmp(fr, "on") || !strcmp(fr, "1") || !strcmp(fr, "true");
      dc_set_factor_reuse(S->car, on, cfg_num(S->cfg, "physics.factor_reuse_tol", 1e-3));
      dc_get_factor_reuse(S->car, &S->fr_st, 0); }
    return 1;
}

/* one line for the F7 overlay / perf dump: "affinity auto: CCD 0 (96 MB L3, 8 cores) cpus 2,4,6,8 | priority normal ok" */
void sim_affinity_desc(const sim_t* S, char* b, int n) {
    const dc_affinity_info* A = &S->aff; int k = 0;
    k += snprintf(b + k, n - k, "affinity %s", A->why[0] ? A->why : "off");
    if (A->pinned > 0 && k < n) {
        k += snprintf(b + k, n - k, ", %d cores", A->dom >= 0 && A->dom < 8 ? A->dom_cores[A->dom] : 0);
        k += snprintf(b + k, n - k, "; cpus phys %d, helpers", A->cpu[0]);
        for (int t = 1; t < A->nthreads && k < n; t++) k += snprintf(b + k, n - k, "%s%d", t > 1 ? "," : " ", A->cpu[t]);
        if (A->smt_shared && k < n) k += snprintf(b + k, n - k, " (SMT shared)");
    }
    if (k < n && A->ndom > 1) { k += snprintf(b + k, n - k, " [L3:"); for (int d = 0; d < A->ndom && d < 8 && k < n; d++) k += snprintf(b + k, n - k, " %.0fMB/%dc", A->l3_mb[d], A->dom_cores[d]); if (k < n) k += snprintf(b + k, n - k, "]"); }
    if (k < n) {
        int sr = S->aff_self_res, hp = S->aff_hprio;
        const char* pin = A->pinned <= 0 ? "" : (sr < 0 ? " pin pending" : ((sr & 1) && S->aff_pool_ok != 0 ? " pin ok" : " pin FAILED"));
        const char* pr = S->prio == 0 ? "" : (sr < 0 || hp < 0 ? " pending" : (((sr & 2) && (hp & 2)) ? ((sr & 4) ? " ok (MMCSS Games)" : " ok") : " denied"));
        snprintf(b + k, n - k, " |%s priority %s%s", pin, dc_affinity_prio_name(S->prio), pr);
    }
}
/* one line: "factor reuse on (tol 1e-03): 87.1 % hits, refactors 120 tol / 0 refine, dK max 9.8e-04, refine err max 2.1e-09" */
void sim_reuse_desc(const sim_t* S, char* b, int n) {
    const dc_factor_reuse_stats* F = &S->fr_st;
    if (!F->on) { snprintf(b, n, "factor reuse off"); return; }
    snprintf(b, n, "factor reuse on (tol %.0e): %.1f %% hits of %lld, refactors %lld tol / %lld refine, dK max %.1e, refine err max %.1e mean %.1e",
             F->tol, F->solves ? 100.0 * (double)F->hits / (double)F->solves : 0.0, F->solves, F->refactor_tol, F->refactor_refine, F->dev_max, F->err_max, F->err_mean);
}
int sim_init(sim_t* S, cfg_t* cfg, track_t* trk, audio_t* audio, char* err, int errlen) {
    memset(S, 0, sizeof *S);
    S->cfg = cfg; S->trk = trk; S->audio = audio;
    dc_default_params(&S->P);
    const char* carini = cfg_str(cfg, "car.ini", "");
    if (*carini && !dc_load_params(carini, &S->P, err, errlen)) return 0;
    /* host tick and mechanics rate: physics.tick_hz (default 1000) x physics.substeps; an explicit physics.substep_hz wins */
    { double thz = cfg_num(cfg, "physics.tick_hz", 1000); if (thz < 30) thz = 30; S->tick_dt = 1.0 / thz;
      int ns = cfg_int(cfg, "physics.substeps", 0);
      if (ns > 0) S->P.substep_hz = thz * ns;
      S->P.substep_hz = cfg_num(cfg, "physics.substep_hz", S->P.substep_hz);
      S->perf_log = cfg_int(cfg, "physics.perf_log", 0); }
    int as = cfg_int(cfg, "car.assist", -1); if (as >= 0) S->P.assist = as;
    int so = cfg_int(cfg, "car.solver", -1); if (so >= 0) S->P.solver = so;   /* --set car.solver=0 -> legacy core */
    { const char* pm = cfg_str(cfg, "physics.model", NULL); if (pm) S->P.model = !strcmp(pm, "fast") ? DC_MODEL_FAST : DC_MODEL_REFERENCE; }   /* reference = full numerics (default) */
    S->P.bricks = cfg_int(cfg, "physics.bricks", 0);   /* tabulated model-reduction bricks (bitmask, 0 = full model) */
    if (!sim_apply_setup(&S->P, cfg_str(cfg, "car.setup", "legacy"), err, errlen)) return 0;   /* after car.solver: propBR needs the MB core */
    if (!make_car(S)) { snprintf(err, errlen, "dc_create failed"); return 0; }
    S->rotation_rad = cfg_num(cfg, "input.rotation_deg", 2 * S->P.steer_lock_deg) * M_PI / 180;
    S->hands_k = cfg_num(cfg, "input.wheel_hands_k", 80); S->hands_d = cfg_num(cfg, "input.wheel_hands_d", 1.2);
    S->hands_tmax = cfg_num(cfg, "input.wheel_hands_tmax", 30);
    S->auto_clutch = cfg_int(cfg, "input.auto_clutch", -1);
    S->kb_hands_off = cfg_int(cfg, "input.keyboard_hands_off", 1);
    { const char* gm = cfg_str(cfg, "gamepad.mode", "arms"); S->gp_mode = strcmp(gm, "raw") != 0; }
    S->gp_k = cfg_num(cfg, "gamepad.arms_k", 10);          /* N m/rad at the rim: relaxed grip */
    S->gp_d = cfg_num(cfg, "gamepad.arms_d", 1.0);         /* N m s/rad: hand/forearm damping */
    S->gp_tmax = cfg_num(cfg, "gamepad.arms_tmax", 20);    /* N m: strongest the arms will push */
    S->gp_deadzone = cfg_num(cfg, "gamepad.deadzone", 0.04);     /* stick jitter isolation only: inside it the hands are off the rim */
    S->gp_grip_t = cfg_num(cfg, "gamepad.grip_time", 0.08);      /* s for the hands to close on the rim when the stick leaves the deadzone */
    S->auto_line = cfg_num(cfg, "autopilot.inside_offset", 4.8);
    S->auto_ay = cfg_num(cfg, "autopilot.lat_accel", 5.5);
    S->auto_speed_cap = cfg_num(cfg, "autopilot.speed_cap", 33);
    S->autopilot = cfg_int(cfg, "autopilot.enabled", 0);
    S->auto_straight = cfg_num(cfg, "autopilot.straight_speed", 0);
    {   /* plug-in driver defaults (overridable per command over the protocol) */
        dc_driver_params dp; dc_driver_default_params(&dp);
        dp.arm_k = cfg_num(cfg, "driver.arm_k", dp.arm_k); dp.arm_d = cfg_num(cfg, "driver.arm_d", dp.arm_d);
        dp.arm_tmax = cfg_num(cfg, "driver.arm_tmax", dp.arm_tmax); dp.delay = cfg_num(cfg, "driver.delay", dp.delay);
        dp.bandwidth_hz = cfg_num(cfg, "driver.bandwidth_hz", dp.bandwidth_hz);
        dp.preview_time = cfg_num(cfg, "driver.preview_time", dp.preview_time); dp.catch_gain = cfg_num(cfg, "driver.catch_gain", dp.catch_gain);
        dp.predict = cfg_num(cfg, "driver.predict", dp.predict); dp.lat_ki = cfg_num(cfg, "driver.lat_ki", dp.lat_ki);
        dp.yaw_damp = cfg_num(cfg, "driver.yaw_damp", dp.yaw_damp);
        S->drv = dc_driver_create(&dp);
        S->drv_takeover_enabled = cfg_int(cfg, "driver.takeover", 1);
        S->drv_takeover_stick = cfg_num(cfg, "driver.takeover_stick", 0.25);
        S->drv_takeover_wheel = cfg_num(cfg, "driver.takeover_wheel_deg", 45) * M_PI / 180;
        S->drv_takeover_pedal = cfg_num(cfg, "driver.takeover_pedal", 0.15);
    }
    input_open(cfg, &S->dev);
    if (S->auto_clutch < 0) S->auto_clutch = !(S->dev.kind == 1 && S->dev.hgear != -99);   /* H-shifter rigs: manual clutch */
    S->gear = 1;
    if (cfg_str(cfg, "start.x", NULL))
        sim_reset_at(S, cfg_num(cfg, "start.x", 0), cfg_num(cfg, "start.y", 0), cfg_num(cfg, "start.yaw_deg", 0) * M_PI / 180, cfg_num(cfg, "start.speed", 0));
    else if (track_spot_by_name(trk, cfg_str(cfg, "start.spot", cfg_str(cfg, "start.s", NULL) ? "" : (trk->layout == TRACK_MESH ? "start" : "hub"))) >= 0) {   /* default: proving-ground start hub */
        double x, y, yaw; track_spot(trk, track_spot_by_name(trk, cfg_str(cfg, "start.spot", cfg_str(cfg, "start.s", NULL) ? "" : (trk->layout == TRACK_MESH ? "start" : "hub"))), &x, &y, &yaw);
        sim_reset_at(S, x, y, yaw, cfg_num(cfg, "start.speed", 0));
    }
    else sim_reset_on_track(S, cfg_num(cfg, "start.s", 30), cfg_num(cfg, "start.d", 0), cfg_num(cfg, "start.speed", 0));
    { const char* rf = cfg_str(cfg, "replay.file", ""); char e2[300];   /* [replay] file = <recording>: replay it from the start */
      if (*rf && !sim_replay_load(S, rf, e2, sizeof e2)) { fprintf(stderr, "[replay] %s\n", e2); snprintf(S->rec_msg, sizeof S->rec_msg, "replay: %.180s", e2); } }
    if (cfg_int(cfg, "replay.record", 0) && !S->rep_on) sim_record_start(S);   /* record from the spawn (headless runs) */
    return 1;
}
void sim_reset_at(sim_t* S, double x, double y, double yaw, double speed) {
    double nx, ny, nz, g;
    double h = track_road_fn(S->trk, x, y, &nx, &ny, &nz, &g);
    double pos[3] = {x, y, h + 0.01};
    { struct roadbake* b0 = S->bake; bake_sync(S); if (S->bake != b0) dc_set_road(S->car, S->bake ? roadbake_road_fn : track_road_fn, S->bake ? (void*)S->bake : (void*)S->trk); }
    dc_reset(S->car, pos, yaw, speed);
    S->kb_sw = 0;
}
int sim_rebuild_car(sim_t* S) {
    dc_xform ch[DC_PART_COUNT]; dc_get_parts(S->car, ch, DC_PART_COUNT);
    dc_telemetry t0; dc_get_telemetry(S->car, &t0);
    double q0 = ch[0].quat[0], q1 = ch[0].quat[1], q2 = ch[0].quat[2], q3 = ch[0].quat[3];
    double yaw = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
    if (!make_car(S)) return 0;
    sim_reset_at(S, ch[0].pos[0], ch[0].pos[1], yaw, t0.vx); return 1;
}
void sim_driver_engage(sim_t* S, int on) {
    if (!S->drv) return;
    if (on && !S->drv_on) dc_driver_reset(S->drv);
    S->drv_on = on; if (on) S->drv_takeover = 0;
}
void sim_reset_on_track(sim_t* S, double s, double d, double speed) {
    double x, y, yaw; track_point(S->trk, s, d, &x, &y, &yaw);
    sim_reset_at(S, x, y, yaw, speed);
}

/* pure-pursuit + curvature speed autopilot (used for scripted screenshots / audio renders) */
static void autopilot(sim_t* S, const dc_telemetry* t, const dc_xform* ch, dc_input* in) {
    if (S->auto_straight > 0) {   /* hands off, hold a speed: shows pure emergent steering response on rough ground */
        double e = S->auto_straight - t->vx;
        in->steer_mode = DC_STEER_TORQUE; in->steer_torque = 0;
        in->throttle = clampd(e * 0.4, 0, 1); in->brake = clampd(-e * 0.3, 0, 1); return;
    }
    const track_t* T = S->trk;
    double px = ch->pos[0], py = ch->pos[1];
    double q0 = ch->quat[0], q1 = ch->quat[1], q2 = ch->quat[2], q3 = ch->quat[3];
    double yaw = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
    double d, s; track_height(T, px, py, NULL, NULL, &d, &s);
    double v = fmax(t->vx, 0);
    double L = 7 + 0.45 * v;
    /* line: towards the inside of upcoming curvature (puts the inside wheels onto the apex kerbs) */
    int k = (int)(fmod(s + L, T->length) / T->ds) % T->n;
    double cv = 0; for (int m = -6; m <= 6; m++) cv += T->curv[(k + m + T->n) % T->n]; cv /= 13;
    double dline = T->layout == TRACK_MESH ? 0 : clampd(cv * 400, -1, 1) * S->auto_line;   /* a mesh map's line is already a racing line */
    double tx, ty; track_point(T, s + L, dline, &tx, &ty, NULL);
    double ex = tx - px, ey = ty - py;
    double lx = cos(yaw) * ex + sin(yaw) * ey, ly = -sin(yaw) * ex + cos(yaw) * ey;
    double wb = S->P.axle[0].hub[0] - S->P.axle[1].hub[0];
    double delta = atan2(2 * wb * ly, lx * lx + ly * ly);
    in->steer_mode = DC_STEER_ANGLE; in->steer_target = clampd(delta * S->P.steer_ratio, -7, 7);
    in->hands_k = 40; in->hands_d = 1.0; in->hands_tmax = 25;
    /* speed: min over the next 80 m of sqrt(ay/|k|) with braking distance */
    double vt = S->auto_speed_cap;
    for (double a = 0; a < 160; a += 2) {
        int kk = (int)(fmod(s + a, T->length) / T->ds) % T->n;
        double c = fabs(T->curv[kk]);
        if (T->layout == TRACK_MESH) { c = 0; for (int m = -16; m <= 16; m++) c += T->curv[(kk + m + T->n) % T->n]; c = fabs(c) / 33; }   /* recorded line: curvature over +-8 m of arc */
        c += 1e-4; double vc = sqrt(S->auto_ay / c);
        double vb = sqrt(vc * vc + 2 * 4.5 * fmax(a - 0.4 * v, 0));
        if (vb < vt) vt = vb;
    }
    double e = vt - v;
    in->throttle = clampd(e * 0.35, 0, 1); in->brake = clampd(-e * 0.6, 0, 0.85);
    if (fabs(t->ay) > 0.8 * S->auto_ay * 1.3) in->throttle *= 0.5;
    /* recovery: autopilot that left the circuit (or spun) is put back on the centre line, like a marshal would */
    if (fabs(d) > T->half_w + T->kerb_w + 4 || (v < 2 && S->sim_time > 5)) {
        if (++S->auto_off_ms * S->tick_dt > 2.5 + 0.5 * S->tick_dt) {   /* 2.5 s off the road (auto_off_ms counts ticks) */ S->auto_off_ms = 0; sim_reset_on_track(S, s, 0, 8); S->gear = 1; }
    } else S->auto_off_ms = 0;
}

#ifdef DC_PROF
static prof_tick_t* PT; static prof_batch_t* PB; static prof_frame_t* PF; static int PT_cap, PB_cap, PF_cap, PT_n, PB_n, PF_n;
static int PT_ring; static long long PT_tot;   /* ring mode (perf overlay): PT_tot ticks written, slot PT_tot % PT_cap */
double prof_now_us(void) { return now_us(); }
void prof_init(int cap) {
    PT_cap = cap; PB_cap = cap; PF_cap = cap;
    PT = (prof_tick_t*)calloc(PT_cap, sizeof *PT); PB = (prof_batch_t*)calloc(PB_cap, sizeof *PB); PF = (prof_frame_t*)calloc(PF_cap, sizeof *PF);
    dc_prof_rec r; dc_prof_take(&r);   /* drop anything before the run */
}
void prof_init_ring(int cap) {
    PT_cap = cap; PT_ring = 1; PT_tot = 0;
    PT = (prof_tick_t*)calloc(PT_cap, sizeof *PT);   /* once, at start; nothing is allocated while running */
    dc_prof_rec r; dc_prof_take(&r);
}
long long prof_ring_copy(prof_tick_t* dst, int max) {
    /* newest ticks, oldest first; single writer (physics thread) publishes PT_tot after the record (release), the reader
       drops any slot the writer may have lapped while copying. Returns the count copied. */
    if (!PT || !PT_ring) return 0;
    long long n1 = __atomic_load_n(&PT_tot, __ATOMIC_ACQUIRE), k = n1 < max ? n1 : max;
    if (k > PT_cap - 64) k = PT_cap - 64;
    for (long long i = 0; i < k; i++) dst[i] = PT[(n1 - k + i) % PT_cap];
    long long n2 = __atomic_load_n(&PT_tot, __ATOMIC_ACQUIRE), lost = n2 - n1 - 64;   /* slots rewritten since the first load */
    if (lost > 0) { if (lost >= k) return 0; memmove(dst, dst + lost, (size_t)(k - lost) * sizeof *dst); k -= lost; }
    return k;
}
const prof_tick_t* prof_last(void) { if (PT_ring) return PT && PT_tot > 0 ? &PT[(PT_tot - 1) % PT_cap] : NULL; return PT && PT_n > 0 ? &PT[PT_n - 1] : NULL; }
void prof_rewind(void) { PT_n = PB_n = PF_n = 0; PT_tot = 0; }
void prof_batch(const prof_batch_t* b) { if (PB && PB_n < PB_cap) PB[PB_n++] = *b; }
void prof_frame(const prof_frame_t* f) { if (PF && PF_n < PF_cap) PF[PF_n++] = *f; }
static void prof_tick(sim_t* S, double w0, double w1) {
    dc_prof_rec r; dc_prof_take(&r);
    if (!PT || (!PT_ring && PT_n >= PT_cap)) return;
    prof_tick_t* p = PT_ring ? &PT[PT_tot % PT_cap] : &PT[PT_n++]; memset(p, 0, sizeof *p);
    p->tw = w0; p->simt = S->sim_time; p->tick_us = w1 - w0;
    if (S->pf_step1 > 0) {
        p->poll_us = S->pf_poll1 - S->pf_poll0; p->input_us = S->pf_step0 - w0 - p->poll_us; p->step_us = S->pf_step1 - S->pf_step0;
        p->ffb_us = S->pf_ffb1 - S->pf_step1; p->post_us = w1 - S->pf_ffb1;
    }
    S->pf_step1 = 0;
    for (int k = 0; k < DCP_S_N; k++) p->st_us[k] = (float)(r.st[k] * 1e6);
    for (int i = 0; i < 4; i++) { p->wh_us[i] = (float)(r.wh[i][DCP_W_TOTAL] * 1e6); for (int k = 0; k < DCP_W_TOTAL; k++) p->whs_us[k] += (float)(r.wh[i][k] * 1e6); }
    p->nsub = (int)r.substeps; p->solves = (int)r.solves; p->resolves = (int)r.resolves; p->sweeps = (int)r.pgs_sweeps;
    p->sweep_max = (int)r.pgs_sweep_max; p->capped = (int)r.pgs_capped; p->nbx_max = (int)r.nbx_max;
    p->pool_runs = (int)r.pool_runs; p->pool_wakes = (int)r.pool_wakes;
    p->pool_wait_us = (float)(r.pool_wait_s * 1e6); p->pool_wait_max_us = (float)(r.pool_wait_max_s * 1e6);
    const dc_telemetry* t = &S->snap.t; float sl = 0, sm = 0; int ns = 0;
    for (int i = 0; i < 4; i++) { sl += (float)t->w[i].sliding; if (t->w[i].sliding > 0.5) ns++; float a = (float)(fabs(t->w[i].slip_angle) * 57.2958); if (a > sm) sm = a; }
    p->slide = sl; p->nslide = ns; p->slip_max_deg = sm; p->speed = (float)fabs(t->speed);
    p->rack_n = (float)t->rack_force; p->column_nm = (float)t->column_torque; p->mz_fl = (float)t->w[0].mz; p->mz_fr = (float)t->w[1].mz; p->sw_deg = (float)(t->sw_angle * 57.2958);
    for (int i = 0; i < 4; i++) p->fz[i] = (float)t->w[i].fz;
    p->surf_mask = 0; for (int i = 0; i < 4; i++) if (S->snap.surf[i] >= 0 && S->snap.surf[i] < 31) p->surf_mask |= 1 << S->snap.surf[i];   /* bit per SURF_* id under any wheel */
    p->px = (float)S->snap.parts[0].pos[0]; p->py = (float)S->snap.parts[0].pos[1];
    {   /* input path: raw device values, the mapped dc_input given to dc_step, hands torque / saturation, road-wheel angles, brakes */
        const dc_input* in = &S->snap.in;
        p->dev_kind = S->dev.kind; p->stick = (float)S->dev.steer; p->steer_mode = in->steer_mode;
        p->steer_target_deg = in->steer_mode == DC_STEER_ANGLE ? (float)(in->steer_target * 57.2958) : 0;
        p->hands_nm = (float)t->driver_torque; p->hands_tmax = in->steer_mode == DC_STEER_ANGLE ? (float)in->hands_tmax : 0;
        p->hands_sat = p->hands_tmax > 0 && fabsf(p->hands_nm) >= 0.999f * p->hands_tmax;
        p->thr_raw = (float)S->dev.throttle_raw; p->brk_raw = (float)S->dev.brake_raw; p->thr = (float)in->throttle; p->brk = (float)in->brake; p->hb = (float)in->handbrake;
        p->st_deg[0] = (float)(t->w[0].steer * 57.2958); p->st_deg[1] = (float)(t->w[1].steer * 57.2958); p->yaw_dps = (float)(t->yaw_rate * 57.2958);
        for (int i = 0; i < 4; i++) { p->bt[i] = (float)t->w[i].brake_torque; p->om[i] = (float)t->w[i].omega; p->sr[i] = (float)t->w[i].slip_ratio_ss; }
    }
    {   /* cause tags for the spike list: kerb/rough surface under a wheel, wheel-load jump, fast steering */
        static float fz0[4], sw0; static int have;
        const double dt = S->tick_dt; float djump = 0;
        for (int i = 0; i < 4; i++) { float d = fabsf(p->fz[i] - fz0[i]); if (d > djump) djump = d; fz0[i] = p->fz[i]; }
        if (p->surf_mask & (1 << SURF_KERB | 1 << SURF_SAUSAGE | 1 << SURF_ROUGH | 1 << SURF_PAVE | 1 << SURF_GRAVEL | 1 << SURF_WALL)) p->tags |= PT_TAG_KERB;
        if (have && djump / dt > 2.0e5) p->tags |= PT_TAG_LOAD;              /* > 200 kN/s on a wheel (e.g. 200 N in one ms) */
        if (have && fabsf(p->sw_deg - sw0) / dt > 360) p->tags |= PT_TAG_STEER; /* > 360 deg/s at the rim */
        sw0 = p->sw_deg; have = 1;
    }
    if (PT_ring) __atomic_store_n(&PT_tot, PT_tot + 1, __ATOMIC_RELEASE);
}
void prof_write_ticks_csv(FILE* f, const prof_tick_t* T, int n) {
    static const char* SN[DCP_S_N] = {"begin", "phaseA", "loads", "assemble", "factor", "ksolve", "boxrows", "schur", "pgs", "subspace", "backsub", "resolve_chk", "commit", "integrate", "post", "step_total"};
    fprintf(f, "tw_us,simt,tick_us,poll_us,input_us,step_us,ffb_us,post_us");
    for (int k = 0; k < DCP_S_N; k++) fprintf(f, ",s_%s", SN[k]);
    fprintf(f, ",wh0,wh1,wh2,wh3,w_setup,w_rays,w_rim,w_brush,w_mz,w_cull,nsub,solves,resolves,sweeps,sweep_max,capped,nbx_max,pool_runs,pool_wakes,pool_wait_us,pool_wait_max_us,slide,nslide,slip_max_deg,speed,rack_n,column_nm,mz_fl,mz_fr,sw_deg,fz0,fz1,fz2,fz3,px,py,surf_mask,tags,dev_kind,stick,steer_mode,steer_target_deg,hands_nm,hands_tmax,hands_sat,thr_raw,brk_raw,thr,brk,hb,st0_deg,st1_deg,yaw_dps,bt0,bt1,bt2,bt3,om0,om1,om2,om3,sr0,sr1,sr2,sr3\n");
    for (int i = 0; i < n; i++) { const prof_tick_t* p = &T[i];
        fprintf(f, "%.1f,%.5f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", p->tw, p->simt, p->tick_us, p->poll_us, p->input_us, p->step_us, p->ffb_us, p->post_us);
        for (int k = 0; k < DCP_S_N; k++) fprintf(f, ",%.2f", p->st_us[k]);
        for (int k = 0; k < 4; k++) fprintf(f, ",%.2f", p->wh_us[k]);
        for (int k = 0; k < DCP_W_TOTAL; k++) fprintf(f, ",%.2f", p->whs_us[k]);
        fprintf(f, ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.3f,%d,%.2f,%.2f,%.3f,%.5f,%.4f,%.4f,%.4f,%.0f,%.0f,%.0f,%.0f,%.2f,%.2f,%d,%d", p->nsub, p->solves, p->resolves, p->sweeps, p->sweep_max, p->capped, p->nbx_max,
                p->pool_runs, p->pool_wakes, p->pool_wait_us, p->pool_wait_max_us, p->slide, p->nslide, p->slip_max_deg, p->speed, p->rack_n, p->column_nm, p->mz_fl, p->mz_fr, p->sw_deg, p->fz[0], p->fz[1], p->fz[2], p->fz[3], p->px, p->py, p->surf_mask, p->tags);
        fprintf(f, ",%d,%.4f,%d,%.2f,%.3f,%.1f,%d,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f", p->dev_kind, p->stick, p->steer_mode, p->steer_target_deg, p->hands_nm, p->hands_tmax, p->hands_sat,
                p->thr_raw, p->brk_raw, p->thr, p->brk, p->hb, p->st_deg[0], p->st_deg[1], p->yaw_dps);
        for (int k = 0; k < 4; k++) fprintf(f, ",%.1f", p->bt[k]);
        for (int k = 0; k < 4; k++) fprintf(f, ",%.2f", p->om[k]);
        for (int k = 0; k < 4; k++) fprintf(f, ",%.3f", p->sr[k]);
        fprintf(f, "\n"); }
}
void prof_dump(const char* prefix) {
    char path[512]; FILE* f;
    snprintf(path, sizeof path, "%s_ticks.csv", prefix);
    if ((f = fopen(path, "w"))) { prof_write_ticks_csv(f, PT, PT_n); fclose(f); }
    snprintf(path, sizeof path, "%s_batches.csv", prefix);
    if ((f = fopen(path, "w"))) { fprintf(f, "tw_us,sleep_req_us,sleep_us,lockwait_max_us,batch_us,backlog_ms,ticks,dropped,broke\n");
        for (int i = 0; i < PB_n; i++) { const prof_batch_t* b = &PB[i]; fprintf(f, "%.1f,%.1f,%.1f,%.1f,%.1f,%.3f,%d,%d,%d\n", b->tw, b->sleep_req_us, b->sleep_us, b->lockwait_max_us, b->batch_us, b->backlog_ms, b->ticks, b->dropped, b->broke); }
        fclose(f); }
    snprintf(path, sizeof path, "%s_frames.csv", prefix);
    if ((f = fopen(path, "w"))) { fprintf(f, "tw_us,frame_us,lock_us,draw_us,sim_time,new_ticks\n");
        for (int i = 0; i < PF_n; i++) { const prof_frame_t* q = &PF[i]; fprintf(f, "%.1f,%.1f,%.1f,%.1f,%.6f,%d\n", q->tw, q->frame_us, q->lock_us, q->draw_us, q->sim_time, q->new_ticks); }
        fclose(f); }
    fprintf(stderr, "[prof] %d ticks, %d batches, %d frames -> %s_*.csv\n", PT_n, PB_n, PF_n, prefix);
}
#endif
/* ---------------- input record / replay ---------------- */
static unsigned long long fnv(unsigned long long h, const void* p, size_t n) { const unsigned char* b = (const unsigned char*)p; for (size_t i = 0; i < n; i++) { h ^= b[i]; h *= 1099511628211ULL; } return h; }
unsigned long long sim_state_hash(sim_t* S) {
    unsigned long long h = 1469598103934665603ULL;
    dc_xform x[DC_PART_COUNT]; memset(x, 0, sizeof x); int n = dc_get_parts(S->car, x, DC_PART_COUNT);
    for (int i = 0; i < n && i < DC_PART_COUNT; i++) { h = fnv(h, x[i].pos, sizeof x[i].pos); h = fnv(h, x[i].quat, sizeof x[i].quat); }
    dc_telemetry t; memset(&t, 0, sizeof t); dc_get_telemetry(S->car, &t);
    for (int i = 0; i < 4; i++) { double v[5] = {t.w[i].fz, t.w[i].fx, t.w[i].fy, t.w[i].slip_ratio, t.w[i].slip_angle}; h = fnv(h, v, sizeof v); }
    double e[3] = {t.engine_rpm, t.sw_angle, t.column_torque}; return fnv(h, e, sizeof e);
}
static void rec_fill_hdr(sim_t* S, sim_rec_hdr* H, double x, double y, double yaw, double v) {
    memset(H, 0, sizeof *H); memcpy(H->magic, SIM_REC_MAGIC, 8); H->version = 1;
    H->tick_dt = S->tick_dt; H->substep_hz = S->P.substep_hz; H->x = x; H->y = y; H->yaw = yaw; H->speed = v; H->gear = S->gear;
    H->model = S->P.model; H->solver = S->P.solver; H->bricks = S->P.bricks; H->layout = S->trk->layout; H->threads = S->threads;
    H->backend = S->offload_patch; H->bake_bits = cfg_int(S->cfg, "track.bake", 0); H->track_len = S->trk->length;
    H->param_hash = fnv(1469598103934665603ULL, &S->P, sizeof S->P);   /* car params incl. setup (struct is memset before load) */
    snprintf(H->map, sizeof H->map, "%s", S->trk->layout == TRACK_MESH ? S->trk->name : track_layout_name(S->trk));
    snprintf(H->car, sizeof H->car, "%s", cfg_str(S->cfg, "car.ini", "(built-in)")); snprintf(H->setup, sizeof H->setup, "%s", cfg_str(S->cfg, "car.setup", "legacy"));
    snprintf(H->backend_cfg, sizeof H->backend_cfg, "%s", cfg_str(S->cfg, "offload.patch", "ref"));
}
static void rec_restart(sim_t* S, double x, double y, double yaw, double v, int gear) {   /* fresh car: no history carried in */
    make_car(S); sim_reset_at(S, x, y, yaw, v); S->gear = gear; S->shift_timer_ms = 0; S->shift_left_s = 0;
}
void sim_record_start(sim_t* S) {
    if (S->rec_on || S->rep_on) return;
    int cap = (int)(cfg_num(S->cfg, "replay.max_s", 600) / S->tick_dt);
    if (!S->rec_buf || S->rec_cap != cap) { free(S->rec_buf); S->rec_buf = (sim_rec_tick*)malloc(sizeof(sim_rec_tick) * (size_t)cap); S->rec_cap = S->rec_buf ? cap : 0; }
    if (!S->rec_buf) { snprintf(S->rec_msg, sizeof S->rec_msg, "record: out of memory"); return; }
    dc_xform ch[DC_PART_COUNT]; dc_get_parts(S->car, ch, DC_PART_COUNT);
    dc_telemetry t0; dc_get_telemetry(S->car, &t0);
    double q0 = ch[0].quat[0], q1 = ch[0].quat[1], q2 = ch[0].quat[2], q3 = ch[0].quat[3];
    double yaw = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
    rec_restart(S, ch[0].pos[0], ch[0].pos[1], yaw, t0.vx, S->gear);
    rec_fill_hdr(S, &S->rec_h, ch[0].pos[0], ch[0].pos[1], yaw, t0.vx);
    S->rec_n = 0; S->rec_ev = 0; S->rec_on = 1;
    snprintf(S->rec_msg, sizeof S->rec_msg, "REC (F9 stops)");
    fprintf(stderr, "[replay] recording from x %.2f y %.2f yaw %.1f deg v %.2f m/s gear %d\n", ch[0].pos[0], ch[0].pos[1], yaw * 180 / M_PI, t0.vx, S->gear);
}
void sim_record_stop(sim_t* S) {
    if (!S->rec_on) return;
    S->rec_on = 0; S->rec_h.nticks = S->rec_n; S->rec_h.end_hash = sim_state_hash(S);
    const char* fx = cfg_str(S->cfg, "replay.record_file", NULL);
    if (fx && *fx) snprintf(S->rec_path, sizeof S->rec_path, "%s", fx);
    else { time_t tt = time(NULL); struct tm* lt = localtime(&tt); char ts[32]; strftime(ts, sizeof ts, "%Y%m%d_%H%M%S", lt);
           snprintf(S->rec_path, sizeof S->rec_path, "%sreplay_%s.dcr", S->rec_dir, ts); }
    FILE* f = fopen(S->rec_path, "wb");
    int ok = f && fwrite(&S->rec_h, sizeof S->rec_h, 1, f) == 1 && (S->rec_n == 0 || fwrite(S->rec_buf, sizeof *S->rec_buf, (size_t)S->rec_n, f) == (size_t)S->rec_n);
    if (f) ok &= fclose(f) == 0;
    if (ok) snprintf(S->rec_msg, sizeof S->rec_msg, "recorded %.1f s -> %.170s", S->rec_n * S->tick_dt, S->rec_path);
    else snprintf(S->rec_msg, sizeof S->rec_msg, "record: cannot write %.170s", S->rec_path);
    fprintf(stderr, "[replay] %s (%d ticks, end hash %016llx)\n", S->rec_msg, S->rec_n, S->rec_h.end_hash);
}
static void rep_begin(sim_t* S) {
    const sim_rec_hdr* H = &S->rep_h;
    rec_restart(S, H->x, H->y, H->yaw, H->speed, H->gear);
    S->rep_i = 0; S->rep_on = 1; S->rep_done = 0; S->rep_match = -1; S->rep_hash = 0;
    snprintf(S->rec_msg, sizeof S->rec_msg, "REPLAY %.180s", S->rep_path);
}
int sim_replay_load(sim_t* S, const char* path, char* err, int errlen) {
    FILE* f = fopen(path, "rb"); sim_rec_hdr H;
    if (!f && S->rec_dir[0] && path[0] != '/' && path[0] != '\\' && !(path[0] && path[1] == ':')) {   /* relative: also try next to the exe */
        char p2[1100]; snprintf(p2, sizeof p2, "%s%s", S->rec_dir, path); f = fopen(p2, "rb"); }
    if (!f) { snprintf(err, errlen, "cannot open %s", path); return 0; }
    if (fread(&H, sizeof H, 1, f) != 1 || memcmp(H.magic, SIM_REC_MAGIC, 8) || H.version != 1 || H.nticks < 0) { fclose(f); snprintf(err, errlen, "%s: not a recording", path); return 0; }
    sim_rec_tick* b = (sim_rec_tick*)malloc(sizeof *b * (size_t)(H.nticks ? H.nticks : 1));
    if (!b || fread(b, sizeof *b, (size_t)H.nticks, f) != (size_t)H.nticks) { fclose(f); free(b); snprintf(err, errlen, "%s: truncated", path); return 0; }
    fclose(f);
    /* the physics must be the same run: refuse what would silently diverge */
    sim_rec_hdr C; rec_fill_hdr(S, &C, 0, 0, 0, 0);
    const char* why = H.tick_dt != C.tick_dt ? "physics.tick_hz" : H.substep_hz != C.substep_hz ? "physics.substeps" : H.model != C.model ? "physics.model" :
                      H.solver != C.solver ? "car.solver" : H.bricks != C.bricks ? "physics.bricks" : H.layout != C.layout || H.track_len != C.track_len || strcmp(H.map, C.map) ? "map" :
                      H.bake_bits != C.bake_bits ? "track.bake" : H.param_hash != C.param_hash ? "car / setup parameters" : NULL;
    if (why) { free(b); snprintf(err, errlen, "%s: recorded with a different %s (%s %s, %.0f Hz)", path, why, H.map, H.car, 1 / H.tick_dt); return 0; }
    if (H.backend != C.backend) {
        int hr = (H.backend == DC_OFFLOAD_REF), cr = (C.backend == DC_OFFLOAD_REF);
        fprintf(stderr, "[replay] note: recorded on backend %s, replaying on %s (%s)\n", dc_offload_name(H.backend), dc_offload_name(C.backend),
                hr != cr ? "ref and the lane backends round differently: expect a DIFFERENT end hash, set [offload] patch to match" : "the lane backends give the same bits");
    }
    free(S->rep_buf); S->rep_buf = b; S->rep_n = H.nticks; S->rep_h = H; snprintf(S->rep_path, sizeof S->rep_path, "%s", path);
    rep_begin(S);
    fprintf(stderr, "[replay] %s: %d ticks (%.1f s), recorded on %s, threads %d\n", path, H.nticks, H.nticks * H.tick_dt, dc_offload_name(H.backend), H.threads);
    return 1;
}
static void snap_rec(sim_t* S) {
    sim_snap* N = &S->snap;
    N->rec_on = S->rec_on; N->rep_on = S->rep_on; N->rep_done = S->rep_done; N->rep_match = S->rep_match; N->rep_i = S->rep_i; N->rep_n = S->rep_n; N->rec_n = S->rec_n;
    snprintf(N->rec_msg, sizeof N->rec_msg, "%s", S->rec_msg); snprintf(N->rep_path, sizeof N->rep_path, "%.250s", S->rep_path); snprintf(N->rec_path, sizeof N->rec_path, "%.250s", S->rec_path);
}
static void rep_finish(sim_t* S) {
    S->rep_on = 0; S->rep_done = 1; S->rep_hash = sim_state_hash(S); S->rep_match = S->rep_hash == S->rep_h.end_hash;
    snprintf(S->rec_msg, sizeof S->rec_msg, "replay done: %s (hash %016llx) - F9 replays again", S->rep_match ? "BIT-IDENTICAL" : "DIFFERS", S->rep_hash);
    fprintf(stderr, "[replay] end after %d ticks: hash %016llx, recorded %016llx -> %s\n", S->rep_n, S->rep_hash, S->rep_h.end_hash, S->rep_match ? "bit-identical" : "DIFFERS");
}
static void sim_tick_body(sim_t* S, double dt);
void sim_tick(sim_t* S, double dt) {
    double w0 = now_us();
    sim_tick_body(S, dt);
#ifdef DC_PROF
    prof_tick(S, w0, now_us());
#endif
    if (!S->perf_log) return;
    double w = now_us() - w0;
    S->perf_wall += w; S->perf_ticks++; if (w > S->perf_worst) S->perf_worst = w;
    S->perf_sub += S->tick_n; S->perf_step += S->tick_step_us;
    S->perf_t += dt;
    if (S->perf_t >= 1.0 - 1e-9) {
        double simms = S->perf_t * 1000;
        printf("[perf] t %6.1f s  v %5.1f km/h  tick %.0f Hz x %.2f sub (%.0f Hz) threads %d  whole tick %6.1f us/ms  dc_step %6.1f us/ms  worst tick %6.0f us\n",
               S->sim_time, fabs(S->snap.t.speed) * 3.6, 1.0 / dt, (double)S->perf_sub / S->perf_ticks, S->P.substep_hz, S->threads,
               S->perf_wall / simms, S->perf_step / simms, S->perf_worst);
        fflush(stdout);
        S->perf_t = S->perf_wall = S->perf_step = S->perf_worst = 0; S->perf_sub = S->perf_ticks = 0;
    }
}
static void sim_tick_body(sim_t* S, double dt) {
    if (S->aff_self_res < 0) {   /* first tick in the thread that runs the physics (demo: phys_thread) */
        S->aff_self_cpu = S->aff.cpu[0];
        S->aff_self_res = (S->aff.cpu[0] >= 0 || S->prio || S->aff_self_cpu > 0) ? dc_pin_thread(S->aff.cpu[0], S->prio) : 0;
    }
    if (S->aff_hprio < 0) S->aff_hprio = dc_pool_prio_status(S->car);   /* helpers apply the priority at their first run */
    dc_get_factor_reuse(S->car, &S->fr_st, 0);   /* snapshot for the overlay / dump (read in the physics thread only) */
    S->tick_n = 0; S->tick_step_us = 0;
    sim_ctl* C = &S->ctl;
    if (C->rec_toggle != S->ctl_seen.rec_toggle) {   /* F9: replay stop / again, else record start / stop */
        S->ctl_seen.rec_toggle = C->rec_toggle;
        if (S->rep_on) { S->rep_on = 0; snprintf(S->rec_msg, sizeof S->rec_msg, "replay stopped at tick %d / %d", S->rep_i, S->rep_n); }
        else if (S->rep_done) rep_begin(S);
        else if (S->rec_on) sim_record_stop(S);
        else sim_record_start(S);
    }
    if (S->rep_on || S->rep_done) {   /* replay owns the car: drop the one-shot requests (pause stays live) */
        S->ctl_seen.toggle_assist = C->toggle_assist; S->ctl_seen.reset = C->reset; S->ctl_seen.spot_req = C->spot_req; S->ctl_seen.puncture = C->puncture;
        if (S->rep_done) { S->snap.paused = 1; snap_rec(S); return; }   /* hold the end state for the dump (F9 replays again) */
    }
    if (S->rec_on && (C->toggle_assist != S->ctl_seen.toggle_assist || C->reset != S->ctl_seen.reset || C->spot_req != S->ctl_seen.spot_req))
        sim_record_stop(S);   /* a teleport / car rebuild ends the recording */
    /* one-shot requests */
    if (C->toggle_assist != S->ctl_seen.toggle_assist) {
        S->ctl_seen.toggle_assist = C->toggle_assist;
        dc_xform ch[DC_PART_COUNT]; dc_get_parts(S->car, ch, DC_PART_COUNT);
        dc_telemetry t0; dc_get_telemetry(S->car, &t0);
        double q0 = ch[0].quat[0], q1 = ch[0].quat[1], q2 = ch[0].quat[2], q3 = ch[0].quat[3];
        double yaw = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
        S->P.assist = !S->P.assist; make_car(S); sim_reset_at(S, ch[0].pos[0], ch[0].pos[1], yaw, t0.vx);
    }
    if (C->reset != S->ctl_seen.reset) {
        S->ctl_seen.reset = C->reset;
        dc_xform ch[DC_PART_COUNT]; dc_get_parts(S->car, ch, DC_PART_COUNT);
        double d, s; track_height(S->trk, ch[0].pos[0], ch[0].pos[1], NULL, NULL, &d, &s);
        for (int i = 0; i < 4; i++) dc_set_pressure(S->car, i, S->P.axle[i / 2].tyre.pressure);
        sim_reset_on_track(S, s, 0, 0); S->gear = 1;
    }
    if (C->spot_req != S->ctl_seen.spot_req) {
        S->ctl_seen.spot_req = C->spot_req;
        double x, y, yaw;
        if (track_spot(S->trk, C->spot, &x, &y, &yaw)) {
            for (int i = 0; i < 4; i++) dc_set_pressure(S->car, i, S->P.axle[i / 2].tyre.pressure);
            sim_reset_at(S, x, y, yaw, 0); S->gear = 1;
        }
    }
    if (C->puncture != S->ctl_seen.puncture) { S->ctl_seen.puncture = C->puncture; dc_puncture(S->car, (C->puncture - 1) & 3, 2.0); S->rec_ev = 1; S->rec_arg = (C->puncture - 1) & 3; }
    if (C->autopilot != S->ctl_seen.autopilot) { S->ctl_seen.autopilot = C->autopilot; S->autopilot = !S->autopilot; }
    if (C->drv_toggle != S->ctl_seen.drv_toggle) { S->ctl_seen.drv_toggle = C->drv_toggle; sim_driver_engage(S, !S->drv_on); }
    if (S->ext_hook) S->ext_hook(S, 0);
    if (C->pause) { S->snap.paused = 1; snap_rec(S); return; }
    S->snap.paused = 0;

#ifdef DC_PROF
    S->pf_poll0 = now_us();
#endif
    input_poll(&S->dev);
#ifdef DC_PROF
    S->pf_poll1 = now_us();
#endif
    dc_telemetry t; dc_get_telemetry(S->car, &t);
    dc_xform parts[DC_PART_COUNT]; dc_get_parts(S->car, parts, DC_PART_COUNT);
    dc_input in; memset(&in, 0, sizeof in);
    const input_state* D = &S->dev;
    double v = fabs(t.vx);
    /* ---- steering ---- */
    if (D->kind == 1) {          /* wheel: device angle is the steering-wheel angle (1:1), stiff hands; FFB = column torque */
        in.steer_mode = DC_STEER_ANGLE; in.steer_target = D->steer * S->rotation_rad * 0.5;
        in.hands_k = S->hands_k; in.hands_d = S->hands_d; in.hands_tmax = S->hands_tmax;
    } else if (D->kind == 2) {   /* gamepad: stick = rim target, linear over the FULL lock at every speed (no speed range, no expo) */
        double x = D->steer, dz = S->gp_deadzone, lock = S->P.steer_lock_deg * M_PI / 180;
        if (fabs(x) <= dz) {     /* stick centred: hands relaxed off the rim, caster trail x Fy / SAT / valve steer the wheel freely */
            in.steer_mode = DC_STEER_TORQUE; in.steer_torque = 0; S->gp_grip = 0;
        } else {
            x = (x - (x > 0 ? dz : -dz)) / (1 - dz);
            S->gp_grip = S->gp_grip_t > 0 ? fmin(1, S->gp_grip + dt / S->gp_grip_t) : 1;   /* hand grip onset, not a range limit */
            in.steer_mode = DC_STEER_ANGLE; in.steer_target = x * lock;
            if (S->gp_mode == 1) { in.hands_k = S->gp_k * S->gp_grip; in.hands_d = S->gp_d * S->gp_grip; in.hands_tmax = S->gp_tmax * S->gp_grip; }   /* arms */
            else { in.hands_k = 25 * S->gp_grip; in.hands_d = 0.8 * S->gp_grip; in.hands_tmax = 15 * S->gp_grip; }                                  /* raw: stiffer legacy hands */
        }
    } else {
        int dir = C->kb_left - C->kb_right;
        double lim = S->P.steer_lock_deg * M_PI / 180;   /* full lock at every speed */
        if (dir) {
            if (S->kb_sw * dir < 0 || fabs(S->kb_sw - t.sw_angle) > 0.6) S->kb_sw = t.sw_angle;   /* grab the wheel where it is */
            S->kb_sw = clampd(S->kb_sw + dir * 6.0 * dt, -lim, lim);
            in.steer_mode = DC_STEER_ANGLE; in.steer_target = S->kb_sw; in.hands_k = 25; in.hands_d = 0.8; in.hands_tmax = 15;
        } else if (S->kb_hands_off) {   /* hands off: caster/SAT centre the wheel and catch slides by themselves */
            in.steer_mode = DC_STEER_TORQUE; in.steer_torque = 0; S->kb_sw = t.sw_angle;
        } else {
            S->kb_sw *= 1 - 3 * dt; in.steer_mode = DC_STEER_ANGLE; in.steer_target = S->kb_sw; in.hands_k = 25; in.hands_d = 0.8; in.hands_tmax = 15;
        }
    }
    /* ---- pedals ---- */
    double thr = C->kb_up, brk = C->kb_down, clu = C->kb_clutch, hb = C->kb_hand;
    if (D->kind) { thr = fmax(thr, D->throttle); brk = fmax(brk, D->brake); clu = fmax(clu, D->clutch); hb = fmax(hb, D->handbrake); }
    in.throttle = thr; in.brake = brk; in.handbrake = hb;
    if (S->autopilot) { autopilot(S, &t, &parts[0], &in); }
    if (S->ext_steer) { in.steer_mode = S->ext_in.steer_mode; in.steer_torque = S->ext_in.steer_torque; in.steer_target = S->ext_in.steer_target;
                        in.hands_k = S->ext_in.hands_k; in.hands_d = S->ext_in.hands_d; in.hands_tmax = S->ext_in.hands_tmax; }
    if (S->ext_pedals) { in.throttle = S->ext_in.throttle; in.brake = S->ext_in.brake; in.handbrake = S->ext_in.handbrake; }
    /* ---- plug-in driver + live human takeover ---- */
    int drv_ped = 0;
    if (S->drv && S->drv_on) {
        int human = C->kb_left || C->kb_right || C->kb_up || C->kb_down;
        if (D->kind == 2) human |= fabs(D->steer) > S->drv_takeover_stick;
        if (D->kind == 1) human |= fabs(D->steer * S->rotation_rad * 0.5 - t.sw_angle) > S->drv_takeover_wheel;   /* wheel held away from the sim rim */
        if (D->kind) human |= D->throttle > S->drv_takeover_pedal || D->brake > S->drv_takeover_pedal;
        if (human && S->drv_takeover_enabled) { S->drv_on = 0; S->drv_takeover = 1; }   /* human input above already mapped into `in` */
        else {
            dc_driver_command(S->drv, &S->drv_cmd);
            dc_driver_update(S->drv, S->car, dt, &in);
            drv_ped = S->drv_cmd.pedal_mode != DC_PED_OFF;
        }
    }
    /* ---- gearbox ---- */
    int up = C->shift_up - S->ctl_seen.shift_up + D->shift_up, dn = C->shift_down - S->ctl_seen.shift_down + D->shift_down;
    S->ctl_seen.shift_up = C->shift_up; S->ctl_seen.shift_down = C->shift_down; S->dev.shift_up = S->dev.shift_down = 0;
    if (D->kind == 1 && D->hgear != -99) {
        if (D->hgear != S->gear) S->gear = D->hgear;
    } else {
        if (S->autopilot || drv_ped) {   /* autopilot / plug-in driver auto-shift */
            if (t.engine_rpm > 6600 && S->gear < S->P.ngears && S->shift_timer_ms <= 0) up = 1;
            if (t.engine_rpm < 2600 && S->gear > 1 && S->shift_timer_ms <= 0) dn = 1;
        }
        if (up > 0 && S->gear < S->P.ngears) { S->gear = S->gear < 0 ? 0 : S->gear + 1; S->shift_timer_ms = 160; }
        if (dn > 0 && S->gear > -1) { S->gear--; S->shift_timer_ms = 160; }
    }
    if (S->shift_timer_ms > 0) { double ms = dt * 1000 + S->shift_left_s; int k = (int)ms; S->shift_left_s = ms - k; S->shift_timer_ms -= k; }
    else S->shift_left_s = 0;
    if (S->auto_clutch || S->autopilot || drv_ped) {
        double c = S->shift_timer_ms > 0 ? 1.0 : 0.0;
        if (S->shift_timer_ms > 0 && S->shift_timer_ms < 60) c = S->shift_timer_ms / 60.0;
        /* launch / anti-stall: slip the clutch below ~1.6x idle */
        if (S->gear != 0 && t.engine_rpm < 2200) c = fmax(c, clampd((2200 - t.engine_rpm) / 1300.0, 0, 1) * (in.brake > 0.3 && v < 1 ? 1 : 0.85));
        if (S->gear != 0 && v < 0.5 && in.throttle < 0.05) c = 1;
        clu = fmax(clu, c);
        if (S->shift_timer_ms > 60) in.throttle = 0;
    }
    in.clutch = clu; in.gear = S->gear;
    if (S->rep_on) {   /* replay: the recorded input replaces everything mapped above */
        const sim_rec_tick* r = &S->rep_buf[S->rep_i++];
        if (r->ev == 1) dc_puncture(S->car, r->arg & 3, 2.0);
        in = r->in; S->gear = in.gear;
    } else if (S->rec_on) {
        sim_rec_tick* r = &S->rec_buf[S->rec_n++]; r->in = in; r->ev = S->rec_ev; r->arg = S->rec_arg;
    }
    S->rec_ev = 0;
    dc_set_input(S->car, &in);
    double t0 = now_us();
    int n = dc_step(S->car, dt);
    S->tick_step_us = now_us() - t0; S->tick_n = n;
#ifdef DC_PROF
    S->pf_step0 = t0; S->pf_step1 = t0 + S->tick_step_us;
#endif
    S->us_acc += S->tick_step_us; S->steps_acc += n;
    S->sim_time += dt;
    dc_get_telemetry(S->car, &t);
    /* ---- FFB ---- */
    input_ffb(&S->dev, t.column_torque, dt);
    if (D->kind == 2) {
        double sl = 0; for (int i = 0; i < 4; i++) sl += t.w[i].sliding * fmin(t.w[i].fz / 3000, 1.5);
        input_rumble(&S->dev, 0.15 * fabs(t.column_torque) / 5, 0.2 * sl);
    }
#ifdef DC_PROF
    S->pf_ffb1 = now_us();
#endif
    /* ---- surfaces + audio frame ---- */
    int surf[4];
    audio_frame af; memset(&af, 0, sizeof af);
    for (int i = 0; i < 4; i++) {
        track_height(S->trk, t.w[i].contact[0], t.w[i].contact[1], &surf[i], NULL, NULL, NULL);
        af.fz[i] = (float)t.w[i].fz; af.sliding[i] = (float)t.w[i].sliding; af.surf[i] = (float)surf[i]; af.rim[i] = (float)t.w[i].rim_force;
        double sv = fabs(t.speed) * hypot(t.w[i].slip_ratio, tan(fmin(fabs(t.w[i].slip_angle), 1.2)));
        af.slide[i] = (float)sv;
    }
    af.rpm = (float)t.engine_rpm; af.throttle = (float)in.throttle; af.speed = (float)fabs(t.speed);
    af.load = t.engine_running ? (float)(in.throttle > 0.02 ? in.throttle : -0.3) : 0;
    af.clutch = (float)in.clutch; af.gear = t.gear; af.engine_on = t.engine_running;
    af.limiter = t.engine_rpm > S->P.limiter_rpm - 80 ? 1.0f : 0.0f;
    if (S->audio) audio_push(S->audio, &af);
    /* ---- snapshot ---- */
    sim_snap* N = &S->snap;
    N->t = t; dc_get_parts(S->car, N->parts, DC_PART_COUNT); N->nseg = dc_get_segments(S->car, N->seg, SIM_MAXSEG);
    N->in = in; memcpy(N->surf, surf, sizeof surf); N->ffb_level = S->dev.ffb_out; N->sim_time = S->sim_time;
    N->beta = fabs(t.vx) > 1 ? atan2(t.vy, fabs(t.vx)) : 0;
    N->gear_req = S->gear; N->assist = S->P.assist; N->autopilot = S->autopilot; N->solver = S->car ? dc_active_solver(S->car) : 0;
    N->device_kind = S->dev.kind; N->has_ffb = S->dev.has_ffb; snprintf(N->device, sizeof N->device, "%s", S->dev.name);
    for (int a = 0; a < 8; a++) N->raw_axis[a] = S->dev.axis[a];
    N->thr_raw = S->dev.throttle_raw; N->brk_raw = S->dev.brake_raw;
    N->drv_on = S->drv_on; N->drv_takeover = S->drv_takeover; N->drv_mode = S->drv_cmd.steer_mode; N->drv_clients = S->ext_clients;
    if (S->drv) dc_driver_get_state(S->drv, &N->drv_st);
    N->raw_buttons = S->dev.buttons_raw; N->raw_naxes = S->dev.naxes; N->hgear = S->dev.hgear;
    if (S->ext_hook) S->ext_hook(S, 1);
    if (S->rep_on && S->rep_i >= S->rep_n) rep_finish(S);
    if (S->rec_on && S->rec_n >= S->rec_cap) sim_record_stop(S);   /* buffer full ([replay] max_s) */
    snap_rec(S);
    if (S->steps_acc >= 2000) { N->us_per_substep = S->us_acc / S->steps_acc; N->rt_factor = 1e6 / (N->us_per_substep * S->P.substep_hz); S->us_acc = 0; S->steps_acc = 0; }
}
void sim_shutdown(sim_t* S) { sim_record_stop(S); free(S->rec_buf); free(S->rep_buf); S->rec_buf = S->rep_buf = NULL; input_close(); S->bake = NULL; if (S->car) dc_destroy(S->car); S->car = NULL; if (S->drv) dc_driver_destroy(S->drv); S->drv = NULL; }
