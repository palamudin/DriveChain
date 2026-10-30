/* fastcheck.c - fidelity harness for physics.model=fast against the reference model.
   Two sims run side by side through the exact demo tick (demo/sim.c) with the same scripted, open-loop driver inputs
   (steering-wheel angle through the wheel "hands", pedals; a speed-hold PI where a manoeuvre is defined at constant
   speed, the same law on both) and are compared tick by tick: trajectory, yaw rate, transient slip angles, wheel loads,
   rack force and its spectrum (Welch band powers). Thresholds: docs/PERF.md (fixed before the fast path was tuned).
   Also times every tick of each model (wall clock and thread CPU time).
   Build: make fastcheck   Run: ./build/fastcheck [car.ini] [outdir]   (env MAN=name runs one manoeuvre, CFGSET=k=v;..) */
#define _GNU_SOURCE
#include "../demo/sim.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stddef.h>
#include <sys/resource.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG (180.0 / M_PI)
static double clampd(double v, double a, double b) { return v < a ? a : (v > b ? b : v); }
static double yaw_of(const dc_xform* c) { double q0 = c->quat[0], q1 = c->quat[1], q2 = c->quat[2], q3 = c->quat[3]; return atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)); }
static double now_s(int cpu) { struct timespec ts; clock_gettime(cpu ? CLOCK_THREAD_CPUTIME_ID : CLOCK_MONOTONIC, &ts); return ts.tv_sec + 1e-9 * ts.tv_nsec; }

static track_t TRK;
typedef struct { sim_t S; cfg_t cfg; double I; } rig_t;
static void rig_init(rig_t* R, const char* car, const char* model) {
    memset(R, 0, sizeof *R);
    cfg_set(&R->cfg, "autopilot.enabled", "0");
    cfg_set(&R->cfg, "car.setup", "default"); cfg_set(&R->cfg, "physics.tick_hz", "240"); cfg_set(&R->cfg, "physics.substeps", "4");
    if (car && *car) cfg_set(&R->cfg, "car.ini", car);
    { const char* cs = getenv("CFGSET");
      if (cs && *cs) { char buf[1024]; snprintf(buf, sizeof buf, "%s", cs);
          for (char* tok = strtok(buf, ";"); tok; tok = strtok(NULL, ";")) { char* eq = strchr(tok, '='); if (eq) { *eq = 0; cfg_set(&R->cfg, tok, eq + 1); } } } }
    cfg_set(&R->cfg, "physics.model", model);
    { const char* fs = getenv("FC_FAST_SET");   /* settings for the fast rig only (e.g. physics.bricks=1: fast + brick vs reference) */
      if (fs && !strcmp(model, "fast")) { char buf[512]; snprintf(buf, sizeof buf, "%s", fs);
          for (char* tok = strtok(buf, ";"); tok; tok = strtok(NULL, ";")) { char* eq = strchr(tok, '='); if (eq) { *eq = 0; cfg_set(&R->cfg, tok, eq + 1); } } } }
    if (getenv("FC_THREADS")) cfg_set(&R->cfg, "physics.threads", getenv("FC_THREADS"));   /* demo default is 4 (bit-identical results) */
#ifdef DC_PROF
    { extern int dc_prof_pgs_cap; dc_prof_pgs_cap = !strcmp(model, "fast") && getenv("FC_CAP_B") ? atoi(getenv("FC_CAP_B")) : 0; }   /* measurement: fast-model sweep cap override */
#endif
    char err[256];
    if (!sim_init(&R->S, &R->cfg, &TRK, NULL, err, sizeof err)) { fprintf(stderr, "sim_init: %s\n", err); exit(1); }
    { const char* fp = getenv("FC_FAST_PARAM");   /* car parameters for the fast rig only (sensitivity: is a brick's deviation inside parameter uncertainty?) */
      if (fp && !strcmp(model, "fast")) { char buf[512]; snprintf(buf, sizeof buf, "%s", fp);
          for (char* tok = strtok(buf, ";"); tok; tok = strtok(NULL, ";")) { char* eq = strchr(tok, '='); if (eq) { *eq = 0; if (!dc_set_param(&R->S.P, "car", tok, eq + 1)) { fprintf(stderr, "FC_FAST_PARAM %s\n", tok); exit(1); } } }
          if (!sim_rebuild_car(&R->S)) exit(1); } }
}

/* ---------------- manoeuvres: inputs as a function of time (+ speed for the hold) ---------------- */
enum { M_SLALOM, M_SKIDPAD, M_LIFTOFF, M_STEP, M_DRIFT, M_BRAKE, M_COBBLES, M_BUMP, M_N };
static const char* MN[M_N] = {"slalom", "skidpad_J266", "liftoff", "step_steer", "drift_entry_exit", "braking", "cobbles", "bump_pothole"};
typedef struct { double x, y, yaw, v; int gear; double T; } start_t;
static double POOL_D, POOL_DV;   /* pooled-statistics runs: lateral offset (m) and speed offset (km/h) */
static start_t man_start(int m) {
    start_t s = {100, 100, 0, 60 / 3.6, 3, 10};
    double x, y, yaw;
    switch (m) {
    case M_SLALOM: s.y = 150; s.T = 10; break;                       /* 18 m cone pitch at 60 km/h */
    case M_SKIDPAD: s.y = 60; s.v = 50 / 3.6; s.T = 14; break;        /* constant speed, slowly increasing steer (J266 constant speed) */
    case M_LIFTOFF: s.y = 60; s.v = 70 / 3.6; s.T = 8; break;
    case M_STEP: s.y = 60; s.v = 70 / 3.6; s.T = 6; break;           /* ISO 7401 step */
    case M_DRIFT: s.y = 60; s.v = 50 / 3.6; s.gear = 2; s.T = 8; break;
    case M_BRAKE: s.y = 150; s.v = 100 / 3.6; s.gear = 4; s.T = 6; break;
    case M_COBBLES: track_point(&TRK, 90, POOL_D, &x, &y, &yaw); s.x = x; s.y = y; s.yaw = yaw; s.v = (60 + POOL_DV) / 3.6; s.T = 12; break;   /* granite setts s 100..300 */
    case M_BUMP: track_point(&TRK, TRK.s_str2 + 100, 0, &x, &y, &yaw); s.x = x; s.y = y; s.yaw = yaw; s.v = 60 / 3.6; s.T = 12; break;   /* single-wheel bumps + potholes */
    }
    return s;
}
/* returns steering-wheel target (rad); sets pedals; hold = 1 -> speed-hold PI owns throttle/brake */
static double man_input(int m, double t, double* thr, double* brk, double* hb, int* hold) {
    double sw = 0, ts = 2.0; *thr = 0; *brk = 0; *hb = 0; *hold = 1;
    double u = t - ts;
    switch (m) {
    case M_SLALOM: if (u > 0) sw = 70 / DEG * sin(2 * M_PI * u / (36.0 / (60 / 3.6))) * clampd(u / 0.5, 0, 1); break;
    case M_SKIDPAD: if (u > 0) sw = fmin(u * 12, 140) / DEG; break;
    case M_LIFTOFF: if (u > 0) sw = fmin(u * 120, 60) / DEG; if (u > 3) { *hold = 0; *thr = 0; } break;
    case M_STEP: if (u > 0) sw = fmin(u / 0.15, 1) * 50 / DEG; if (u > 0) { *hold = 0; *thr = 0.35; } break;
    case M_DRIFT:
        if (u > 0) { *hold = 0; sw = fmin(u / 0.2, 1) * 160 / DEG; *thr = 1; if (u < 0.5) *hb = 1; }
        if (u > 2.0) sw = -60 / DEG;                         /* countersteer */
        if (u > 3.5) { sw = -60 / DEG * clampd(1 - (u - 3.5) / 1.0, 0, 1); *thr = 0.25; }   /* exit: lift, unwind */
        break;
    case M_BRAKE: if (u > 0) { *hold = 0; *brk = fmin(u / 0.15, 1) * 0.8; } break;
    default: break;   /* road features: straight ahead, wheel held centred by the hands, speed hold */
    }
    return sw;
}

/* ---------------- per-tick record ---------------- */
#define NMAX 24000   /* ticks per manoeuvre: 25 s at 960 Hz */
static double FS = 240;   /* host tick rate (Hz), set from the rig's tick_dt */
#ifndef DC_PROF
enum { DCP_S_N = 1, DCP_W_TOTAL = 1 };   /* stage arrays only filled in DC_PROF builds */
#endif
typedef struct { double x, y, r, sa[4], fz[4], rack, wall, cpu; long flt, sys_us; float st[DCP_S_N], wh[DCP_W_TOTAL]; int sweeps; } rec_t;   /* st/wh/sweeps: DC_PROF builds */
static rec_t RR[3][NMAX];

static double PERTURB;   /* FC_SELF: reference vs reference with the start yaw nudged by this (rad) -> the manoeuvre's chaos floor */
static void run(rig_t* R, int m, rec_t* out, int* n_out) {
    sim_t* S = &R->S; start_t st = man_start(m); st.yaw += PERTURB;
    sim_reset_at(S, st.x, st.y, st.yaw, st.v); S->gear = st.gear;
    S->ext_steer = 1; S->ext_pedals = 1; R->I = 0.2;
    double dt = S->tick_dt; int n = 0; FS = 1.0 / dt;
    for (double t = 0; t < st.T - 1e-9 && n < NMAX; t += dt) {
        dc_telemetry tl; dc_get_telemetry(S->car, &tl);
        double thr, brk, hb; int hold;
        double sw = man_input(m, t, &thr, &brk, &hb, &hold);
        if (hold) {   /* speed hold PI (same law on both models) */
            double e = st.v - tl.vx; R->I = clampd(R->I + 0.08 * e * dt, 0, 1);
            double c = 0.25 * e + R->I; thr = clampd(c, 0, 1); brk = clampd(-c * 0.5, 0, 1);
        }
        dc_input* in = &S->ext_in; memset(in, 0, sizeof *in);
        in->steer_mode = DC_STEER_ANGLE; in->steer_target = sw; in->hands_k = 80; in->hands_d = 1.2; in->hands_tmax = 30;
        in->throttle = thr; in->brake = brk; in->handbrake = hb;
        struct rusage u0, u1; getrusage(RUSAGE_THREAD, &u0);
        double w0 = now_s(0), c0 = now_s(1);
        sim_tick(S, dt);
        double w1 = now_s(0), c1 = now_s(1);
        getrusage(RUSAGE_THREAD, &u1);
        dc_get_telemetry(S->car, &tl);
        dc_xform ch[DC_PART_COUNT]; dc_get_parts(S->car, ch, DC_PART_COUNT);
        rec_t* o = &out[n++];
#ifdef DC_PROF
        { static int pinit; if (!pinit) { prof_init(64); pinit = 1; } const prof_tick_t* pt = prof_last();   /* sim_tick takes the core record into the ring */
          if (pt) { memcpy(o->st, pt->st_us, sizeof o->st); memcpy(o->wh, pt->whs_us, sizeof o->wh); o->sweeps = pt->sweeps; } prof_rewind(); }
#endif
        o->x = ch[0].pos[0]; o->y = ch[0].pos[1]; o->r = tl.yaw_rate; o->rack = tl.rack_force;
        for (int k = 0; k < 4; k++) { o->sa[k] = tl.w[k].slip_angle; o->fz[k] = tl.w[k].fz; }
        o->flt = (u1.ru_minflt - u0.ru_minflt) + 1000 * (u1.ru_majflt - u0.ru_majflt); o->sys_us = (u1.ru_stime.tv_sec - u0.ru_stime.tv_sec) * 1000000L + (u1.ru_stime.tv_usec - u0.ru_stime.tv_usec); o->wall = (w1 - w0) * 1e6; o->cpu = (c1 - c0) * 1e6;
        (void)yaw_of;
    }
    *n_out = n;
}

/* Welch band powers of x (fs = host tick rate), 1 s Hann segments, 50 % overlap */
static void bands(const double* x, int n, double fs, double out[4]) {
    static const double be[5] = {0, 5, 15, 40, 120.01};
    int L = (int)fs, hop = L / 2; memset(out, 0, 4 * sizeof(double));
    if (n < L) return;
    double w[1024], seg[1024], wn = 0; for (int i = 0; i < L; i++) { w[i] = 0.5 - 0.5 * cos(2 * M_PI * i / L); wn += w[i] * w[i]; }
    int ns = 0;
    for (int s0 = 0; s0 + L <= n; s0 += hop, ns++) {
        double mean = 0; for (int i = 0; i < L; i++) mean += x[s0 + i]; mean /= L;
        for (int i = 0; i < L; i++) seg[i] = (x[s0 + i] - (0 * mean)) * w[i];
        for (int f = 0; f <= L / 2; f++) {
            double re = 0, im = 0; for (int i = 0; i < L; i++) { double a = 2 * M_PI * f * i / L; re += seg[i] * cos(a); im -= seg[i] * sin(a); }
            double p = (re * re + im * im) / (wn * fs) * (f == 0 || f == L / 2 ? 1 : 2), fr = f * fs / L;
            for (int b = 0; b < 4; b++) if (fr >= be[b] && fr < be[b + 1]) out[b] += p * fs / L;
        }
    }
    for (int b = 0; b < 4; b++) out[b] /= ns;   /* mean band power, N^2 */
}

typedef struct { double pos_max, r_rms, r_max, sa_rms, sa_max, fz_rms, fz_max, rk_rms, rk_max, rk_sig, db[4], bp[4]; int pass, fails, first_diff, floor_k, pooled_pass, tick_pass; double pool_db[4], pool_rel[4]; char why[256]; double ref_mean, ref_p99, ref_max, f_mean, f_p99, f_max, ref_cpu_max, f_cpu_max; } res_t;
static int cmpd(const void* a, const void* b) { double x = *(const double*)a, y = *(const double*)b; return x < y ? -1 : x > y; }
static void stats(const rec_t* r, int n, int cpu, double* mean, double* p99, double* mx) {
    static double v[NMAX]; double s = 0;
    for (int i = 0; i < n; i++) { v[i] = cpu ? r[i].cpu : r[i].wall; s += v[i]; }
    qsort(v, n, sizeof(double), cmpd); *mean = s / n; *p99 = v[(int)(0.99 * (n - 1))]; *mx = v[n - 1];
}

/* F (optional): the manoeuvre's chaos floor, the element-wise worst metric of K reference runs whose start yaw differs
   by 1e-15 rad. Where the reference does not even reproduce itself tick by tick (contact chatter on setts), fast cannot
   be asked to be closer to it than it is to itself: each limit becomes max(fixed threshold, floor). */
static void compare(int m, const rec_t* a, const rec_t* b, int n, res_t* R, FILE* csv, const res_t* F) {
    memset(R, 0, sizeof *R);
    int drift = m == M_DRIFT, rough = m == M_COBBLES || m == M_BUMP;
    double er = 0, es = 0, ef = 0, ek = 0, sk = 0;
    for (int i = 0; i < n; i++) {
        double dp = hypot(a[i].x - b[i].x, a[i].y - b[i].y); if (dp > R->pos_max) R->pos_max = dp;
        double d = fabs(a[i].r - b[i].r) * DEG; er += d * d; if (d > R->r_max) R->r_max = d;
        for (int k = 0; k < 4; k++) {
            d = fabs(a[i].sa[k] - b[i].sa[k]) * DEG; es += d * d; if (d > R->sa_max) R->sa_max = d;
            d = fabs(a[i].fz[k] - b[i].fz[k]); ef += d * d; if (d > R->fz_max) R->fz_max = d;
        }
        d = fabs(a[i].rack - b[i].rack); ek += d * d; if (d > R->rk_max) R->rk_max = d; sk += a[i].rack * a[i].rack;
        if (csv) fprintf(csv, "%s,%d,%.6f,%.6f,%.6f,%.6f,%.5f,%.5f,%.3f,%.3f,%.2f,%.2f\n", MN[m], i, a[i].x, a[i].y, b[i].x, b[i].y, a[i].r * DEG, b[i].r * DEG, a[i].rack, b[i].rack, a[i].fz[0], b[i].fz[0]);
    }
    R->first_diff = -1;
    for (int i = 0; i < n && R->first_diff < 0; i++) if (memcmp(&a[i], &b[i], offsetof(rec_t, wall))) R->first_diff = i;
    R->r_rms = sqrt(er / n); R->sa_rms = sqrt(es / (4 * n)); R->fz_rms = sqrt(ef / (4 * n)); R->rk_rms = sqrt(ek / n); R->rk_sig = sqrt(sk / n);
    static double xa[NMAX], xb[NMAX]; double pa[4], pb[4];
    for (int i = 0; i < n; i++) { xa[i] = a[i].rack; xb[i] = b[i].rack; }
    bands(xa, n, FS, pa); bands(xb, n, FS, pb);
    int bad = 0; char* w = R->why; *w = 0;
#define LIM(th, f) fmax(th, F ? F->f : 0)
#define CHK(cond, ...) do { if (!(cond)) { bad++; w += snprintf(w, R->why + sizeof R->why - w, __VA_ARGS__); } } while (0)
    for (int k = 0; k < 4; k++) {
        R->bp[k] = sqrt(pa[k]); R->db[k] = 10 * log10((pb[k] + 1e-30) / (pa[k] + 1e-30));
        if (sqrt(pa[k]) < 1 && sqrt(pb[k]) < 1) R->db[k] = 0;   /* both bands below 1 N rms: below feel, ratio meaningless */
        CHK(fabs(R->db[k]) <= fmax(0.5, F ? fabs(F->db[k]) : 0), "band%d %.2fdB ", k, R->db[k]);
    }
    CHK(R->pos_max <= LIM(drift ? 0.5 : 0.10, pos_max), "pos %.3f ", R->pos_max);
    CHK(R->r_rms <= LIM(drift ? 1.5 : 0.5, r_rms) && R->r_max <= LIM(drift ? 6 : 2, r_max), "yaw %.3f/%.3f ", R->r_rms, R->r_max);
    CHK(R->sa_rms <= LIM(drift ? 0.5 : 0.15, sa_rms) && R->sa_max <= LIM(drift ? 3 : 0.75, sa_max), "slip %.3f/%.3f ", R->sa_rms, R->sa_max);
    CHK(R->fz_rms <= LIM(rough ? 80 : 40, fz_rms) && R->fz_max <= LIM(rough ? 600 : 200, fz_max), "fz %.1f/%.1f ", R->fz_rms, R->fz_max);
    CHK(R->rk_rms <= LIM(fmax(15, 0.01 * R->rk_sig), rk_rms) && R->rk_max <= LIM(fmax(60, 0.04 * R->rk_sig), rk_max), "rack %.2f/%.2f ", R->rk_rms, R->rk_max);
#undef CHK
#undef LIM
    R->pass = !bad; R->fails = bad;
    stats(a, n, 0, &R->ref_mean, &R->ref_p99, &R->ref_max); stats(b, n, 0, &R->f_mean, &R->f_p99, &R->f_max);
    double d1, d2; stats(a, n, 1, &d1, &d2, &R->ref_cpu_max); stats(b, n, 1, &d1, &d2, &R->f_cpu_max);
}

/* pooled statistics for a chaotic manoeuvre: 12 independent passes (4 lateral offsets x 3 speeds) per model;
   rack-force Welch band powers summed over all passes (+-0.5 dB, as the fixed spectrum threshold), and the pooled rms of
   rack force, yaw rate, wheel-load fluctuation and slip angle (each within +-3 %, half the force JND) */
static int pooled(int m, const char* car, res_t* R) {
    static rig_t P; static const double dd[4] = {-1.2, -0.4, 0.4, 1.2}, dv[3] = {-10, 0, 10};
    double bp[2][4] = {{0}}, acc[2][4] = {{0}}; long cnt[2] = {0, 0};
    for (int q = 0; q < 2; q++) for (int a = 0; a < 4; a++) for (int b = 0; b < 3; b++) {
        POOL_D = dd[a]; POOL_DV = dv[b]; int n;
        rig_init(&P, car, q ? "fast" : "reference"); run(&P, m, RR[2], &n); dc_destroy(P.S.car);
        static double x[NMAX]; double pb[4]; for (int i = 0; i < n; i++) x[i] = RR[2][i].rack;
        bands(x, n, FS, pb); for (int k = 0; k < 4; k++) bp[q][k] += pb[k];
        double fzm[4] = {0}; for (int i = 0; i < n; i++) for (int k = 0; k < 4; k++) fzm[k] += RR[2][i].fz[k] / n;
        for (int i = 0; i < n; i++) {
            acc[q][0] += RR[2][i].rack * RR[2][i].rack; acc[q][1] += RR[2][i].r * RR[2][i].r;
            for (int k = 0; k < 4; k++) { double d = RR[2][i].fz[k] - fzm[k]; acc[q][2] += d * d / 4; acc[q][3] += RR[2][i].sa[k] * RR[2][i].sa[k] / 4; }
        }
        cnt[q] += n;
    }
    POOL_D = POOL_DV = 0;
    int ok = 1; static const char* qn[4] = {"rack", "yaw", "dFz", "slip"};
    printf("  %-17s pooled 12 passes/model: rack bands dB", MN[m]);
    for (int k = 0; k < 4; k++) { R->pool_db[k] = 10 * log10(bp[1][k] / bp[0][k]); if (fabs(R->pool_db[k]) > 0.5) ok = 0; printf(" %+.2f", R->pool_db[k]); }
    printf(" | rms ratio");
    for (int k = 0; k < 4; k++) { R->pool_rel[k] = sqrt(acc[1][k] / cnt[1]) / sqrt(acc[0][k] / cnt[0]) - 1; if (fabs(R->pool_rel[k]) > 0.03) ok = 0; printf(" %s %+.2f%%", qn[k], 100 * R->pool_rel[k]); }
    printf(" -> %s\n", ok ? "PASS" : "FAIL");
    return ok;
}

/* FC_REP=N: tick cost with the machine's noise separated out. The step is deterministic, so a tick that is expensive
   for a physical reason is expensive in every repeat; a scheduler/hypervisor hiccup is not. Per tick, the minimum over N
   repeats is the intrinsic cost; its maximum is the worst intrinsic tick. Raw (single-run) maxima are reported beside it. */
static float ST[2][NMAX][DCP_S_N], WH[2][NMAX][DCP_W_TOTAL]; static int SW[2][NMAX];
static const char* STN[DCP_S_N] = {"begin","phaseA","loads","assemble","factor","ksolve","boxrows","schur","pgs","subspace","backsub","resolve_chk","commit","integrate","post","step_total"};
static const char* WHN[DCP_W_TOTAL] = {"w_setup","w_rays","w_rim","w_brush","w_mz"};
static void timing(int m, const char* car) {
    int N = atoi(getenv("FC_REP")); static rig_t P; static double mn[2][NMAX], rawmax[2]; static int over[2]; int n = 0;
    for (int q = 0; q < 2; q++) { rawmax[q] = 0; over[q] = 0; for (int i = 0; i < NMAX; i++) mn[q][i] = 1e30; }
    for (int r = 0; r < N; r++) for (int q = 0; q < 2; q++) {
#ifdef DC_PROF
        { extern int dc_prof_pgs_cap; dc_prof_pgs_cap = q && getenv("FC_CAP_B") ? atoi(getenv("FC_CAP_B")) : 0; }
#endif
        rig_init(&P, car, q ? "fast" : "reference"); run(&P, m, RR[2], &n); dc_destroy(P.S.car);
        for (int i = 0; i < n; i++) { double c = RR[2][i].cpu; if (c < mn[q][i]) { memcpy(ST[q][i], RR[2][i].st, sizeof ST[q][i]); memcpy(WH[q][i], RR[2][i].wh, sizeof WH[q][i]); SW[q][i] = RR[2][i].sweeps; } if (c < mn[q][i]) mn[q][i] = c; if (c > rawmax[q]) rawmax[q] = c; if (c > 2000) over[q]++; }
    }
    for (int q = 0; q < 2; q++) {
        static double v[NMAX]; double s = 0; int wi = 0;
        for (int i = 0; i < n; i++) { v[i] = mn[q][i]; s += v[i]; if (mn[q][i] > mn[q][wi]) wi = i; }
        qsort(v, n, sizeof(double), cmpd);
        printf("  %-17s %-9s intrinsic tick (min of %d) mean %4.0f p99 %4.0f max %5.0f us at t=%.3f s | raw max %6.0f us, raw ticks >2 ms %d of %d\n",
               MN[m], q ? "fast" : "reference", N, s / n, v[(int)(0.99 * (n - 1))], v[n - 1], wi / FS, rawmax[q], over[q], n * N);
#ifdef DC_PROF
        if (getenv("FC_STAGES")) {   /* per-stage cost (us/tick) of the minimum-cost repeat: mean over the run, and at the worst tick */
            printf("    stages mean/worst:"); for (int k = 0; k < DCP_S_N; k++) { double a = 0; for (int i = 0; i < n; i++) a += ST[q][i][k]; printf(" %s %.0f/%.0f", STN[k], a / n, ST[q][wi][k]); }
            printf(" | wheels(sum of 4)"); for (int k = 0; k < DCP_W_TOTAL; k++) { double a = 0; for (int i = 0; i < n; i++) a += WH[q][i][k]; printf(" %s %.0f/%.0f", WHN[k], a / n, WH[q][wi][k]); }
            printf(" | pgs sweeps at worst tick %d\n", SW[q][wi]);
        }
#endif
    }
}

/* FC_LAP=<layout>: closed-loop lap with the demo autopilot (both models), from the start spot, standing start.
   Lap time = time until the centre-line distance travelled reaches one track length. Returns ticks recorded. */
static int LAPCAP;
static int run_lap(rig_t* R, rec_t* out, double* lap_t, double tmax) {
    sim_t* S = &R->S; double dt = S->tick_dt, s_prev = (FS = 1.0 / dt, -1), s_acc = 0; int n = 0; *lap_t = -1;
    for (double t = 0; t < tmax && n < NMAX * 8; t += dt) {
        struct timespec a, b; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &a);
        sim_tick(S, dt);
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &b);
        dc_telemetry tl; dc_get_telemetry(S->car, &tl); dc_xform ch[DC_PART_COUNT]; dc_get_parts(S->car, ch, DC_PART_COUNT);
        rec_t* o = &out[n++]; memset(o, 0, sizeof *o);
        o->x = ch[0].pos[0]; o->y = ch[0].pos[1]; o->r = tl.yaw_rate; o->rack = tl.rack_force;
        for (int k = 0; k < 4; k++) { o->sa[k] = tl.w[k].slip_angle; o->fz[k] = tl.w[k].fz; }
        o->cpu = o->wall = ((b.tv_sec - a.tv_sec) + 1e-9 * (b.tv_nsec - a.tv_nsec)) * 1e6;
        double d, sc; track_height(&TRK, o->x, o->y, NULL, NULL, &d, &sc);
        if (s_prev >= 0) { double ds = sc - s_prev; if (ds > TRK.length / 2) ds -= TRK.length; if (ds < -TRK.length / 2) ds += TRK.length; s_acc += ds; }
        s_prev = sc; LAPCAP = tl.pgs_capped;
        if (s_acc >= TRK.length && *lap_t < 0) { *lap_t = t + dt; break; }
    }
    return n;
}
static rec_t LAP[2][NMAX * 8];
/* FC_LAP_SELF=<rad>: the second run is the reference again with the start yaw nudged -> the lap's chaos floor */
static void lap_nudge(rig_t* P, double eps) {
    const char* sp = cfg_str(&P->cfg, "start.spot", "start"); int i = track_spot_by_name(&TRK, sp); double x, y, yaw;
    if (i < 0 || !track_spot(&TRK, i, &x, &y, &yaw)) { fprintf(stderr, "FC_LAP_SELF: no spot %s\n", sp); exit(1); }
    sim_reset_at(&P->S, x, y, yaw + eps, 0);
}
static void lap_check(const char* layout, const char* car) {
    double lself = getenv("FC_LAP_SELF") ? atof(getenv("FC_LAP_SELF")) : 0;
    int lcap = getenv("FC_LAP_SELF_CAP") ? atoi(getenv("FC_LAP_SELF_CAP")) : 0;   /* DC_PROF: second run = reference with this PGS sweep cap (solver floor) */
    if (lcap && !lself) lself = 1e-300;   /* reference again; the nudge is far below one ulp of the start yaw */
    char err[256]; track_free(&TRK); memset(&TRK, 0, sizeof TRK);
    if (!track_select(&TRK, layout, 1.0, err, sizeof err)) { fprintf(stderr, "track: %s\n", err); exit(1); }
    static rig_t P; double lt[2]; int n[2]; double tmax = getenv("FC_LAP_T") ? atof(getenv("FC_LAP_T")) : 300;
    for (int q = 0; q < 2; q++) {
        rig_init(&P, car, q && !lself ? "fast" : "reference"); cfg_set(&P.cfg, "autopilot.enabled", "1");
        dc_destroy(P.S.car); char e2[256]; cfg_set(&P.cfg, "start.spot", getenv("FC_SPOT") ? getenv("FC_SPOT") : "start");
        if (!sim_init(&P.S, &P.cfg, &TRK, NULL, e2, sizeof e2)) { fprintf(stderr, "sim_init: %s\n", e2); exit(1); }
        if (q && lself) lap_nudge(&P, lself);
#ifdef DC_PROF
        { extern int dc_prof_pgs_cap; if (q && lcap) dc_prof_pgs_cap = lcap; }
#endif
        n[q] = run_lap(&P, LAP[q], &lt[q], tmax); dc_destroy(P.S.car);
        printf("  %s: %d ticks, PGS solves ending at the sweep cap: %d\n", q ? (lself ? "reference+nudge" : "fast") : "reference", n[q], LAPCAP);
        int rep = getenv("FC_REP") ? atoi(getenv("FC_REP")) : 1; double rawmax = 0; int over = 0, wi = 0;
        for (int i = 0; i < n[q]; i++) { if (LAP[q][i].cpu > rawmax) rawmax = LAP[q][i].cpu; over += LAP[q][i].cpu > 2000; }
        for (int r = 1; r < rep; r++) {   /* deterministic: same ticks again; keep the per-tick minimum (machine noise out) */
            static rec_t T2[NMAX * 8]; double l2;
            rig_init(&P, car, q && !lself ? "fast" : "reference"); cfg_set(&P.cfg, "autopilot.enabled", "1"); cfg_set(&P.cfg, "start.spot", getenv("FC_SPOT") ? getenv("FC_SPOT") : "start"); dc_destroy(P.S.car);
            if (!sim_init(&P.S, &P.cfg, &TRK, NULL, e2, sizeof e2)) exit(1);
            if (q && lself) lap_nudge(&P, lself);
#ifdef DC_PROF
            { extern int dc_prof_pgs_cap; if (q && lcap) dc_prof_pgs_cap = lcap; }
#endif
            int n2 = run_lap(&P, T2, &l2, tmax); dc_destroy(P.S.car);
            for (int i = 0; i < n2 && i < n[q]; i++) { if (T2[i].cpu > rawmax) rawmax = T2[i].cpu; over += T2[i].cpu > 2000; if (T2[i].cpu < LAP[q][i].cpu) LAP[q][i].cpu = T2[i].cpu; }
        }
        for (int i = 0; i < n[q]; i++) if (LAP[q][i].cpu > LAP[q][wi].cpu) wi = i;
        if (rep > 1) printf("  %s: intrinsic (min of %d) worst tick %.0f us at t=%.3f s | raw max %.0f us, raw ticks > 2 ms: %d of %d\n", q ? "fast" : "reference", rep, LAP[q][wi].cpu, wi / FS, rawmax, over, n[q] * rep);
    }
    double pb[2][4], acc[2] = {0, 0}; static double x[NMAX * 8];
    for (int q = 0; q < 2; q++) {
        for (int i = 0; i < n[q]; i++) { x[i] = LAP[q][i].rack; acc[q] += x[i] * x[i]; }
        bands(x, n[q], FS, pb[q]);
    }
    printf("lap %s: lap time ref %.3f s fast %.3f s (diff %+.3f s) | rack band dB", layout, lt[0], lt[1], lt[1] - lt[0]);
    int ok = lt[0] > 0 && lt[1] > 0 && fabs(lt[1] - lt[0]) <= 0.10;
    for (int k = 0; k < 4; k++) { double db = 10 * log10(pb[1][k] / pb[0][k]); printf(" %+.2f", db); if (fabs(db) > 0.5) ok = 0; }
    double rr = sqrt(acc[1] / n[1]) / sqrt(acc[0] / n[0]) - 1; printf(" | rack rms ratio %+.2f%%", 100 * rr);
    for (int q = 0; q < 2; q++) {
        static double v[NMAX * 8]; double sm = 0; for (int i = 0; i < n[q]; i++) { v[i] = LAP[q][i].cpu; sm += v[i]; }
        qsort(v, n[q], sizeof(double), cmpd); printf(" | %s tick mean %.0f p99 %.0f max %.0f us", q ? "fast" : "ref", sm / n[q], v[(int)(0.99 * (n[q] - 1))], v[n[q] - 1]);
    }
    printf(" -> %s\n", ok ? "PASS" : "FAIL");
}

#ifdef DC_MB_TRACE
extern int mb_trace;
#endif
int main(int argc, char** argv) {
#ifdef DC_MB_TRACE
    mb_trace = getenv("FC_MBTRACE") ? atoi(getenv("FC_MBTRACE")) : 0;
#endif
    const char* car = argc > 1 ? argv[1] : "";
    const char* out = argc > 2 ? argv[2] : "results/fastcheck";
    char cmd[512]; snprintf(cmd, sizeof cmd, "mkdir -p %s", out); if (system(cmd)) {}
    track_build(&TRK, 1.0, TRACK_PROVING);
    if (getenv("FC_LAP")) { lap_check(getenv("FC_LAP"), car); return 0; }
    static rig_t A, B;
    char path[512]; snprintf(path, sizeof path, "%s/fastcheck_summary.csv", out);
    FILE* s = fopen(path, "w");
    fprintf(s, "manoeuvre,pass,chaos_floor_runs,pos_max_m,yaw_rms_dps,yaw_max_dps,slip_rms_deg,slip_max_deg,fz_rms_n,fz_max_n,rack_rms_n,rack_max_n,rack_sig_rms_n,db_0_5,db_5_15,db_15_40,db_40_120,ref_mean_us,ref_p99_us,ref_max_us,fast_mean_us,fast_p99_us,fast_max_us,ref_cpu_max_us,fast_cpu_max_us\n");
    FILE* tr = getenv("FC_TRACE") ? fopen(getenv("FC_TRACE"), "w") : NULL;
    if (tr) fprintf(tr, "man,i,xr,yr,xf,yf,r_ref,r_fast,rack_ref,rack_fast,fz0_ref,fz0_fast\n");
    int fails = 0;
    printf("fastcheck: physics.model=fast vs reference (car %s, 240 Hz x 4 unless CFGSET: %s)\n", *car ? car : "default", getenv("CFGSET") ? getenv("CFGSET") : "-");
    for (int m = 0; m < M_N; m++) {
        if (getenv("MAN") && !strstr(getenv("MAN"), MN[m])) continue;
        if (getenv("FC_REP")) { timing(m, car); continue; }
        int na, nb;
        rig_init(&A, car, "reference"); run(&A, m, RR[0], &na); dc_destroy(A.S.car);
        if (getenv("FC_REFDUMP")) {   /* reference-path regression: dump the reference run bit-exactly (%a) and skip the comparison */
            FILE* f = fopen(getenv("FC_REFDUMP"), "a"); if (!f) return 1;
            for (int i = 0; i < na; i++) { const rec_t* q = &RR[0][i]; fprintf(f, "%s %d %a %a %a %a", MN[m], i, q->x, q->y, q->r, q->rack);
                for (int k = 0; k < 4; k++) fprintf(f, " %a %a", q->sa[k], q->fz[k]); fputc('\n', f); }
            fclose(f); continue;
        }
        const char* self = getenv("FC_SELF"); PERTURB = self ? atof(self) : (getenv("FC_FP") ? atof(getenv("FC_FP")) : 0);
#ifdef DC_PROF
        { extern int dc_prof_pgs_cap; dc_prof_pgs_cap = getenv("FC_CAP_B") ? atoi(getenv("FC_CAP_B")) : 0; }
#endif
        rig_init(&B, car, self ? "reference" : "fast"); run(&B, m, RR[1], &nb); PERTURB = 0; dc_destroy(B.S.car);
#ifdef DC_PROF
        { extern int dc_prof_pgs_cap; dc_prof_pgs_cap = 0; }
#endif
        int n = na < nb ? na : nb; res_t R; compare(m, RR[0], RR[1], n, &R, tr, NULL);
        int ens = getenv("FC_ENS") ? atoi(getenv("FC_ENS")) : 8; R.floor_k = 0;
        if (!R.pass && ens > 0 && !self) {   /* is the manoeuvre itself chaotic? build the reference's own floor */
            static res_t F, E; memset(&F, 0, sizeof F);
            /* solver floor (DC_PROF builds): the reference's PGS stops at a sweep cap of 400 when a solve has not converged;
               the same reference with caps 800 and 1600 is an equally valid reference, so its spread is part of the floor */
            int nsf = 0, sfc[2] = {800, 1600};
#ifdef DC_PROF
            nsf = getenv("FC_NO_SOLVER_FLOOR") ? 0 : 2;
#endif
            for (int k = 0; k < ens + nsf; k++) {
                int ne; PERTURB = k < ens ? 1e-15 * (k + 1) : 0; rig_init(&B, car, "reference");
#ifdef DC_PROF
                { extern int dc_prof_pgs_cap; dc_prof_pgs_cap = k < ens ? 0 : sfc[k - ens]; }
#endif
                run(&B, m, RR[2], &ne); dc_destroy(B.S.car); PERTURB = 0;
#ifdef DC_PROF
                { extern int dc_prof_pgs_cap; dc_prof_pgs_cap = 0; }
#endif
                compare(m, RR[0], RR[2], n < ne ? n : ne, &E, NULL, NULL);
#define MX(f) F.f = fmax(F.f, E.f)
                MX(pos_max); MX(r_rms); MX(r_max); MX(sa_rms); MX(sa_max); MX(fz_rms); MX(fz_max); MX(rk_rms); MX(rk_max);
                for (int q = 0; q < 4; q++) F.db[q] = fmax(F.db[q], fabs(E.db[q]));
#undef MX
                if (!E.pass) F.floor_k++;
            }
            if (F.floor_k > 0) {
                double tw[3][3] = {{R.ref_mean, R.ref_p99, R.ref_max}, {R.f_mean, R.f_p99, R.f_max}, {R.ref_cpu_max, R.f_cpu_max, 0}};
                compare(m, RR[0], RR[1], n, &R, NULL, &F); R.floor_k = F.floor_k;
                R.ref_mean = tw[0][0]; R.ref_p99 = tw[0][1]; R.ref_max = tw[0][2]; R.f_mean = tw[1][0]; R.f_p99 = tw[1][1]; R.f_max = tw[1][2]; R.ref_cpu_max = tw[2][0]; R.f_cpu_max = tw[2][1];
                printf("  %-17s chaos+solver floor: %d/%d reference runs (start yaw +1e-15..%g rad; %d with PGS cap 800/1600) fail against the reference itself: pos %.3f m | yaw %.3f/%.3f | slip %.3f/%.3f | Fz %.1f/%.1f | rack %.2f/%.2f | |dB| %.2f %.2f %.2f %.2f\n",
                       MN[m], F.floor_k, ens + nsf, 1e-15 * ens, nsf, F.pos_max, F.r_rms, F.r_max, F.sa_rms, F.sa_max, F.fz_rms, F.fz_max, F.rk_rms, F.rk_max, F.db[0], F.db[1], F.db[2], F.db[3]);
                if (m == M_COBBLES) R.pooled_pass = pooled(m, car, &R);
            }
        }
        if (R.floor_k && m == M_COBBLES) { R.tick_pass = R.pass; R.pass = R.pooled_pass; }   /* chaotic: the verdict is the pooled statistics; tick-by-tick vs the floor is reported */
        if (!R.pass) fails++;
        printf("  %-17s %-11s bitwise-from %d | pos %.3f m | yaw %.3f/%.3f dps | slip %.3f/%.3f deg | Fz %.1f/%.1f N | rack %.2f/%.2f N (sig %.0f) | dB %+.2f %+.2f %+.2f %+.2f | tick us ref %.0f/%.0f/%.0f fast %.0f/%.0f/%.0f %s\n",
               MN[m], R.pass ? (R.floor_k ? "PASS(pool)" : "PASS") : "FAIL", R.first_diff, R.pos_max, R.r_rms, R.r_max, R.sa_rms, R.sa_max, R.fz_rms, R.fz_max, R.rk_rms, R.rk_max, R.rk_sig, R.db[0], R.db[1], R.db[2], R.db[3],
               R.ref_mean, R.ref_p99, R.ref_max, R.f_mean, R.f_p99, R.f_max, R.why);
        fprintf(s, "%s,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.3f,%.3f,%.1f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n", MN[m], R.pass, R.floor_k, R.pos_max, R.r_rms, R.r_max, R.sa_rms, R.sa_max, R.fz_rms, R.fz_max, R.rk_rms, R.rk_max, R.rk_sig,
                R.db[0], R.db[1], R.db[2], R.db[3], R.ref_mean, R.ref_p99, R.ref_max, R.f_mean, R.f_p99, R.f_max, R.ref_cpu_max, R.f_cpu_max);
        if (getenv("FC_SPIKES")) for (int q = 0; q < 2; q++) for (int i = 0; i < n; i++) if (RR[q][i].cpu > 1500) printf("    spike %s tick %d (t=%.3f s) %.0f us cpu, %ld faults, %ld us sys\n", q ? "fast" : "ref ", i, i / FS, RR[q][i].cpu, RR[q][i].flt, RR[q][i].sys_us);
        fflush(stdout);
    }
    fclose(s); if (tr) fclose(tr);
    printf("fastcheck: %s (%d failing manoeuvres)\n", fails ? "FAIL" : "PASS", fails);
    return fails ? 1 : 0;
}
