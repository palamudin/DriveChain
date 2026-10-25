/* mb_tests.c - Phase 0 validation of the multibody library (dc_mb): closed loops, reactions, energy, friction,
 * momentum, zero allocation in the step, benchmark. Writes results/mb/phase0_*.csv. */
#include "dc_mb.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/* allocation counter (glibc interposition) */
extern void* __libc_malloc(size_t); extern void* __libc_calloc(size_t, size_t); extern void* __libc_realloc(void*, size_t);
static long g_nalloc; static int g_count;
void* malloc(size_t n) { if (g_count) g_nalloc++; return __libc_malloc(n); }
void* calloc(size_t a, size_t b) { if (g_count) g_nalloc++; return __libc_calloc(a, b); }
void* realloc(void* p, size_t n) { if (g_count) g_nalloc++; return __libc_realloc(p, n); }

static int fails = 0;
static void check(const char* name, double val, double lim) {
    int ok = fabs(val) <= lim; if (!ok) fails++;
    printf("  %-48s %12.3e  (limit %.1e)  %s\n", name, val, lim, ok ? "PASS" : "FAIL");
}
static m3 diag3(double a, double b, double c) { m3 I; memset(&I, 0, sizeof I); I.m[0][0] = a; I.m[1][1] = b; I.m[2][2] = c; return I; }
static m3 rod_inertia(double m, double L, v3 axis) {   /* slender rod (radius 1 cm) along a coordinate axis */
    double t = m * L * L / 12, a = 0.5 * m * 1e-4;
    if (fabs(axis.x) > 0.5) return diag3(a, t, t);
    if (fabs(axis.y) > 0.5) return diag3(t, a, t);
    return diag3(t, t, a);
}
static const qt QI = {1, 0, 0, 0};
static double potential(const mb_world* W, double g) { double e = 0; for (int i = 0; i < W->nb; i++) e += W->B[i].m * g * W->B[i].x.z; return e; }
static void gravity(mb_world* W, double g) { for (int i = 0; i < W->nb; i++) W->B[i].F.z -= W->B[i].m * g; }
static double max_phi(mb_world* W) {
    double e = 0;
    for (int j = 0; j < W->nj; j++) { if (W->Jt[j].box) continue; mb_eval_joint(W, j); for (int r = 0; r < W->Jt[j].m; r++) if (fabs(W->Jt[j].phi[r]) > e) e = fabs(W->Jt[j].phi[r]); }
    return e;
}
static double slope(const double* t, const double* y, int n) {
    double st = 0, sy = 0, stt = 0, sty = 0; for (int i = 0; i < n; i++) { st += t[i]; sy += y[i]; stt += t[i] * t[i]; sty += t[i] * y[i]; }
    return (n * sty - st * sy) / (n * stt - st * st);
}

/* ---- 1. spherical pendulum: constraint, tension vs centripetal balance, energy drift */
static void t_pendulum(void) {
    printf("[pendulum] 1 kg rod-bob on 1 m spherical joint, released horizontal with out-of-plane speed\n");
    mb_world* W = mb_create();
    int b = mb_add_body(W, "bob", 1.0, diag3(0.01, 0.01, 0.01), V3(1, 0, 0), QI);
    W->B[b].v = V3(0, 1.0, 0);
    int j = mb_spherical(W, "pivot", b, -1, V3(0, 0, 0));
    mb_finalize(W);
    double h = 1.0 / 2000, g = 9.81, E0 = mb_kinetic(W) + potential(W, g), emax = 0, tmax = 0, phimax = 0;
    static double ts[40000], es[40000]; int n = 0;
    FILE* f = fopen("results/mb/phase0_pendulum.csv", "w"); if (f) fprintf(f, "t,x,y,z,E_rel,phi,tension_err\n");
    for (int s = 0; s < 40000; s++) {
        mb_begin(W, h); gravity(W, g); mb_solve(W);
        /* radial balance with the solved state: T - m g . (-r) = m |v_perp|^2 / L  (exact for a point mass on a sphere) */
        mb_body* B = &W->B[b];
        v3 rhat = vnorm(B->x), vm = B->v;
        mb_integrate(W, 1);       /* reaction = velocity-row impulse + projection (SHAKE) multiplier */
        v3 F, T; mb_joint_wrench(W, j, 0, &F, &T);
        vm = vscl(vadd(vm, B->v), 0.5);
        double Tn = -vdot(F, rhat), vt2 = vdot(vm, vm) - pow(vdot(vm, rhat), 2);
        double terr = Tn - (B->m * vt2 / vlen(B->x) - B->m * g * rhat.z);
        double E = mb_kinetic(W) + potential(W, g), er = (E - E0) / E0, ph = max_phi(W);
        if (fabs(er) > emax) emax = fabs(er);
        if (ph > phimax) phimax = ph;
        if (s > 10 && fabs(terr) / (B->m * g) > tmax) tmax = fabs(terr) / (B->m * g);
        ts[n] = s * h; es[n] = er; n++;
        if (f && s % 20 == 0) fprintf(f, "%.4f,%.6f,%.6f,%.6f,%.3e,%.3e,%.3e\n", s * h, B->x.x, B->x.y, B->x.z, er, ph, terr);
    }
    if (f) fclose(f);
    check("max constraint error |phi| [m]", phimax, 1e-6);
    check("energy drift rate [1/s] (fit over 20 s)", slope(ts, es, n), 2e-4);
    check("max |E-E0|/E0 (staggered-velocity oscillation, O(h))", emax, 3e-2);
    check("radial balance error / mg (O(h))", tmax, 1e-2);
    mb_destroy(W);
}

/* ---- 2. slider-crank (closed loop): closure and energy, no gravity */
static void t_slider_crank(void) {
    printf("[slider-crank] crank 0.1 m, rod 0.3 m, slider 0.5 kg; crank revolute, rod spherical-spherical, slider prismatic\n");
    mb_world* W = mb_create();
    double a = 60 * M_PI / 180, rc = 0.1, lr = 0.3;
    v3 P = V3(rc * cos(a), 0, rc * sin(a)); double xs = P.x + sqrt(lr * lr - P.z * P.z); v3 S = V3(xs, 0, 0);
    v3 cu = vnorm(P), ru = vnorm(vsub(S, P));
    m3 Rc = {{{cu.x, 0, -cu.z}, {0, 1, 0}, {cu.z, 0, cu.x}}};  /* local x along crank */
    m3 Rr = {{{ru.x, 0, -ru.z}, {0, 1, 0}, {ru.z, 0, ru.x}}};
    int c = mb_add_body(W, "crank", 1.0, rod_inertia(1.0, rc, V3(1, 0, 0)), vscl(P, 0.5), mtoq(&Rc));
    int r = mb_add_body(W, "rod", 0.8, rod_inertia(0.8, lr, V3(1, 0, 0)), vscl(vadd(P, S), 0.5), mtoq(&Rr));
    int s = mb_add_body(W, "slider", 0.5, diag3(1e-3, 1e-3, 1e-3), S, QI);
    W->B[c].w = V3(0, -20, 0); W->B[c].v = vcross(W->B[c].w, W->B[c].x);   /* spin; rod/slider start consistent via the solve */
    mb_revolute(W, "crank", c, -1, V3(0, 0, 0), V3(0, 1, 0));
    mb_spherical(W, "pin", c, r, P);
    mb_spherical(W, "wrist", r, s, S);
    mb_prismatic(W, "slide", s, -1, V3(1, 0, 0));
    mb_finalize(W);
    double h = 1.0 / 2000, phimax = 0;
    /* first step makes velocities consistent; measure energy from there */
    mb_begin(W, h); mb_solve(W); mb_integrate(W, 1);
    double E0 = mb_kinetic(W), emax = 0;
    static double ts[20000], es[20000]; int n = 0;
    FILE* f = fopen("results/mb/phase0_slider_crank.csv", "w"); if (f) fprintf(f, "t,crank_angle,slider_x,E_rel,phi\n");
    for (int k = 0; k < 20000; k++) {
        mb_begin(W, h); mb_solve(W); mb_integrate(W, 1);
        double E = mb_kinetic(W), er = (E - E0) / E0, ph = max_phi(W);
        if (fabs(er) > emax) emax = fabs(er); if (ph > phimax) phimax = ph;
        ts[n] = k * h; es[n] = er; n++;
        if (f && k % 10 == 0) fprintf(f, "%.4f,%.5f,%.6f,%.3e,%.3e\n", k * h, atan2(W->B[c].x.z, W->B[c].x.x), W->B[s].x.x, er, ph);
    }
    if (f) fclose(f);
    check("max loop closure error [m|rad]", phimax, 1e-6);
    check("energy drift rate [1/s]", slope(ts, es, n), 2e-4);
    check("max |E-E0|/E0", emax, 2e-2);
    mb_destroy(W);
}

/* ---- 3. four-bar under gravity (suspension-like: revolute, spherical, spherical, revolute) */
static void t_fourbar(void) {
    printf("[four-bar] ground 0.4, crank 0.15, coupler 0.4, rocker 0.25 under gravity\n");
    mb_world* W = mb_create();
    v3 A = V3(0, 0, 0), D = V3(0.4, 0, 0);
    double ac = 1.2;  v3 B = V3(0.15 * cos(ac), 0, 0.15 * sin(ac));
    /* C: |C-B| = 0.4, |C-D| = 0.25 (upper intersection) */
    double dx = D.x - B.x, dz = D.z - B.z, d = sqrt(dx * dx + dz * dz), l1 = 0.4, l2 = 0.25;
    double aa = (l1 * l1 - l2 * l2 + d * d) / (2 * d), hh = sqrt(l1 * l1 - aa * aa);
    v3 C = V3(B.x + aa * dx / d - hh * dz / d, 0, B.z + aa * dz / d + hh * dx / d);
    #define ROD(nm, m, P0, P1) ({ v3 u = vnorm(vsub(P1, P0)); m3 R = {{{u.x, 0, -u.z}, {0, 1, 0}, {u.z, 0, u.x}}}; \
        mb_add_body(W, nm, m, rod_inertia(m, vlen(vsub(P1, P0)), V3(1, 0, 0)), vscl(vadd(P0, P1), 0.5), mtoq(&R)); })
    int c = ROD("crank", 0.6, A, B), k = ROD("coupler", 1.0, B, C), r = ROD("rocker", 0.8, D, C);
    mb_revolute(W, "A", c, -1, A, V3(0, 1, 0));
    mb_spherical(W, "B", c, k, B);
    mb_spherical(W, "C", k, r, C);
    mb_revolute(W, "D", r, -1, D, V3(0, 1, 0));
    mb_finalize(W);
    double h = 1.0 / 2000, g = 9.81, E0 = mb_kinetic(W) + potential(W, g), emax = 0, phimax = 0;
    static double ts[40000], es[40000]; int n = 0;
    double ekin_max = 0;
    FILE* f = fopen("results/mb/phase0_fourbar.csv", "w"); if (f) fprintf(f, "t,crank_angle,E_rel,phi\n");
    for (int s = 0; s < 40000; s++) {
        mb_begin(W, h); gravity(W, g); mb_solve(W); mb_integrate(W, 1);
        double ke = mb_kinetic(W), E = ke + potential(W, g), ph = max_phi(W);
        if (ke > ekin_max) ekin_max = ke;
        double er = (E - E0) / (ekin_max > 1e-9 ? ekin_max : 1);
        if (s > 100 && fabs(er) > emax) emax = fabs(er); if (ph > phimax) phimax = ph;
        ts[n] = s * h; es[n] = er; n++;
        if (f && s % 20 == 0) fprintf(f, "%.4f,%.5f,%.3e,%.3e\n", s * h, atan2(W->B[c].x.z, W->B[c].x.x), er, ph);
    }
    if (f) fclose(f);
    check("max loop closure error", phimax, 1e-6);
    check("energy drift rate (rel. to max KE) [1/s]", slope(ts, es, n), 2e-4);
    check("max |E-E0|/KEmax", emax, 2e-2);
    mb_destroy(W);
}

/* ---- 4. statically determinate support: reactions vs hand statics */
static void t_static(void) {
    printf("[statics] 10 kg plate on a ball joint + 2 vertical two-force links; analytic reactions W/3 each\n");
    mb_world* W = mb_create();
    int b = mb_add_body(W, "plate", 10, diag3(0.3, 0.8, 1.0), V3(0.5, 0.1, 0), QI);
    int ja = mb_spherical(W, "ball", b, -1, V3(0, 0, 0));
    int j1 = mb_distance(W, "link1", b, V3(1, 0, 0), -1, V3(1, 0, 1));
    int j2 = mb_distance(W, "link2", b, V3(0.5, 0.3, 0), -1, V3(0.5, 0.3, 1));
    mb_finalize(W);
    double h = 1.0 / 2000, g = 9.81, Wt = 10 * g; v3 F, T;
    mb_begin(W, h); gravity(W, g); mb_solve(W);
    double e = 0;
    mb_joint_wrench(W, ja, 0, &F, &T); e = fmax(e, fabs(F.z - Wt / 3) / Wt); e = fmax(e, fmax(fabs(F.x), fabs(F.y)) / Wt);
    mb_joint_wrench(W, j1, 0, &F, &T); e = fmax(e, fabs(F.z - Wt / 3) / Wt);
    mb_joint_wrench(W, j2, 0, &F, &T); e = fmax(e, fabs(F.z - Wt / 3) / Wt);
    check("max |reaction - statics| / weight", e, 1e-6);
    check("residual velocity after one step [m/s]", vlen(W->B[b].v) + vlen(W->B[b].w), 1e-9);
    mb_destroy(W);
}

/* ---- 5. Coulomb box row: flywheel on a revolute with a friction torque cap (stiction after stop) */
static void t_friction(void) {
    printf("[box row] flywheel I=0.2, w0=10 rad/s, Coulomb torque 0.5 N m -> stops at t = 4.000 s, then sticks\n");
    mb_world* W = mb_create();
    int b = mb_add_body(W, "fly", 3, diag3(0.1, 0.1, 0.2), V3(0, 0, 0), QI);
    W->B[b].w = V3(0, 0, 10);
    mb_revolute(W, "axle", b, -1, V3(0, 0, 0), V3(0, 0, 1));
    int bb[1] = {b}; int fj = mb_custom(W, "fric", 1, 1, 1, bb);
    mb_finalize(W);
    double h = 1.0 / 2000, tstop = -1, err = 0;
    for (int s = 0; s < 12000; s++) {
        mb_begin(W, h);
        mb_joint* J = &W->Jt[fj]; memset(J->J, 0, sizeof J->J); J->J[0][0][5] = 1; J->lo[0] = -0.5 * h; J->hi[0] = 0.5 * h; J->t[0] = 0;
        W->B[b].T = V3(0.3, 0, 0.0);   /* a side torque the bearing must carry */
        mb_solve(W); mb_integrate(W, 1);
        double t = (s + 1) * h, wz = W->B[b].w.z, wa = t < 4 ? 10 - 2.5 * t : 0;
        if (fabs(wz - wa) > err) err = fabs(wz - wa);
        if (tstop < 0 && fabs(wz) < 1e-12) tstop = t;
    }
    check("max |w - analytic| [rad/s]", err, 1e-9);
    check("stop time error [s]", tstop - 4.0, 1e-9);
    check("off-axis spin from the side torque [rad/s]", fabs(W->B[b].w.x) + fabs(W->B[b].w.y), 1e-9);
    mb_destroy(W);
}

/* ---- 5b. load-dependent Coulomb rows (bound = pre + c |own normal reaction|, normal from the same coupled solve) */
static void t_jfric(void) {
    printf("[joint friction] journal: radial load 100 N -> 200 N at t=1 s, mu r = 5 mm: w = 10-2.5t, then stops at 2.5 s\n");
    {
        mb_world* W = mb_create();
        int b = mb_add_body(W, "fly", 3, diag3(0.1, 0.1, 0.2), V3(0, 0, 0), QI);
        W->B[b].w = V3(0, 0, 10);
        int jr = mb_revolute(W, "axle", b, -1, V3(0, 0, 0), V3(0, 0, 1));
        int bb[1] = {b}; int fj = mb_custom(W, "fric", 1, 1, 1, bb);
        mb_fric_source(W, fj, jr, 0, 3, 0.005, V3(0, 0, 0));
        mb_finalize(W);
        double h = 1.0 / 2000, tstop = -1, err = 0, berr = 0;
        for (int s = 0; s < 8000; s++) {
            double t0 = s * h, F = t0 < 1.0 - 1e-12 ? 100 : 200;
            mb_begin(W, h);
            mb_joint* J = &W->Jt[fj]; memset(J->J, 0, sizeof J->J); J->J[0][0][5] = 1; J->t[0] = 0; J->active = 1; J->pre = 0;
            W->B[b].F = V3(F, 0, 0);
            mb_solve(W); mb_integrate(W, 1);
            double t = (s + 1) * h, wz = W->B[b].w.z;
            double wa = t <= 1 ? 10 - 2.5 * t : fmax(0, 7.5 - 5 * (t - 1));
            if (fabs(wz - wa) > err) err = fabs(wz - wa);
            if (wz > 1e-9) { double be = fabs(J->bound / h - 0.005 * F); if (be > berr) berr = be; }
            if (tstop < 0 && fabs(wz) < 1e-9) tstop = t;
        }
        check("max |w - analytic| (no lag at the load step) [rad/s]", err, 1e-9);
        check("stop time error [s]", tstop - 2.5, 1e-9);
        check("bound - mu r |F_radial| while slipping [N m]", berr, 1e-9);
        mb_destroy(W);
    }
    printf("[joint friction] slider on a prismatic rail, side load 100 N, mu 0.2, push 30 N (slides, a = 5) / 15 N (sticks)\n");
    for (int c = 0; c < 2; c++) {
        mb_world* W = mb_create();
        int b = mb_add_body(W, "blk", 2, diag3(0.01, 0.01, 0.01), V3(0, 0, 0), QI);
        int jp = mb_prismatic(W, "rail", b, -1, V3(1, 0, 0));
        int bb[1] = {b}; int fj = mb_custom(W, "fric", 1, 1, 1, bb);
        mb_fric_source(W, fj, jp, 0, 2, 0.2, V3(0, 0, 0));
        mb_finalize(W);
        double h = 1.0 / 2000, Fx = c == 0 ? 30 : 15;
        for (int s = 0; s < 2000; s++) {
            mb_begin(W, h);
            mb_joint* J = &W->Jt[fj]; memset(J->J, 0, sizeof J->J); J->J[0][0][0] = 1; J->t[0] = 0; J->active = 1; J->pre = 0;
            W->B[b].F = V3(Fx, 100, 0);
            mb_solve(W); mb_integrate(W, 1);
        }
        double va = c == 0 ? 5.0 * 1.0 : 0;
        check(c == 0 ? "sliding: |v - a t| after 1 s [m/s]" : "sticking: |v| after 1 s [m/s]", W->B[b].v.x - va, 1e-9);
        mb_destroy(W);
    }
    printf("[joint friction] ball joint, isotropic (cone) friction torque 0.4 N m + mu r |F|: spin direction preserved\n");
    {
        mb_world* W = mb_create();
        int b = mb_add_body(W, "ball", 2, diag3(0.05, 0.05, 0.05), V3(0, 0, 0), QI);
        W->B[b].w = V3(8, 6, 0);
        int js = mb_spherical(W, "bj", b, -1, V3(0, 0, 0));
        int bb[1] = {b}; int fj = mb_custom(W, "bjf", 3, 1, 1, bb); W->Jt[fj].cone = 1;
        mb_fric_source(W, fj, js, 0, 3, 0.001, V3(0, 0, 0));
        mb_finalize(W);
        double h = 1.0 / 2000, derr = 0, merr = 0;
        for (int s = 0; s < 2000; s++) {
            mb_begin(W, h);
            mb_joint* J = &W->Jt[fj]; memset(J->J, 0, sizeof J->J);
            for (int q = 0; q < 3; q++) { J->J[q][0][3 + q] = 1; J->t[q] = 0; }
            J->active = 1; J->pre = 0.4 * h;
            W->B[b].F = V3(0, 0, -600);   /* bound 0.4 + 0.6 = 1.0 N m -> |dw/dt| = 20 rad/s^2 */
            mb_solve(W); mb_integrate(W, 1);
            v3 w = W->B[b].w; double n = vlen(w), t = (s + 1) * h;
            if (n > 1e-9) { double d = fabs(w.x / n - 0.8) + fabs(w.y / n - 0.6) + fabs(w.z); if (d > derr) derr = d; }
            double na = fmax(0, 10 - 20 * t); if (fabs(n - na) > merr) merr = fabs(n - na);
        }
        check("spin direction error (cone, not box)", derr, 1e-9);
        check("|w| - analytic (10 - 20 t) [rad/s]", merr, 1e-9);
        mb_destroy(W);
    }
    {   /* anisotropic inertia: the solved impulse must oppose the post-step relative angular velocity */
        mb_world* W = mb_create();
        int b = mb_add_body(W, "ball", 2, diag3(0.02, 0.08, 0.05), V3(0, 0, 0), QI);
        W->B[b].w = V3(8, 6, 3);
        int js = mb_spherical(W, "bj", b, -1, V3(0, 0, 0));
        int bb[1] = {b}; int fj = mb_custom(W, "bjf", 3, 1, 1, bb); W->Jt[fj].cone = 1;
        mb_fric_source(W, fj, js, 0, 3, 0.001, V3(0, 0, 0));
        mb_finalize(W);
        double h = 1.0 / 2000, aerr = 0, nerr = 0;
        for (int s = 0; s < 400; s++) {
            mb_begin(W, h);
            mb_joint* J = &W->Jt[fj]; memset(J->J, 0, sizeof J->J);
            for (int q = 0; q < 3; q++) { J->J[q][0][3 + q] = 1; J->t[q] = 0; }
            J->active = 1; J->pre = 0.4 * h; W->B[b].F = V3(0, 0, -600);
            mb_solve(W);
            v3 w = W->B[b].w, l = V3(J->lam[0], J->lam[1], J->lam[2]);    /* w here is the solved (end-of-step) velocity */
            double c = -vdot(w, l) / (vlen(w) * vlen(l));
            if (1 - c > aerr) aerr = 1 - c;
            if (fabs(vlen(l) - J->bound) > nerr) nerr = fabs(vlen(l) - J->bound);
            mb_integrate(W, 1);
        }
        check("anisotropic: 1 - cos(impulse, -w_rel) while sliding", aerr, 1e-9);
        check("anisotropic: |impulse| - bound [N m s]", nerr / h, 1e-9);
        mb_destroy(W);
    }
}

/* ---- 6. free-floating pair with an implicit spring element: momentum conservation, oscillation frequency */
static void t_momentum(void) {
    printf("[momentum] two free bodies, revolute + implicit torsion spring element, gyroscopic spin, no gravity\n");
    mb_world* W = mb_create();
    int a = mb_add_body(W, "a", 5, diag3(0.2, 0.3, 0.4), V3(0, 0, 0), QI);
    int b = mb_add_body(W, "b", 2, diag3(0.05, 0.02, 0.05), V3(0.3, 0.1, 0), QI);
    W->B[a].v = V3(0.3, -0.2, 0.1); W->B[a].w = V3(0.5, 1.0, -0.3);
    W->B[b].w = V3(0.5, 30, -0.3); W->B[b].v = vadd(W->B[a].v, vcross(W->B[a].w, V3(0.3, 0.1, 0)));
    mb_revolute(W, "hinge", b, a, V3(0.3, 0.1, 0), V3(0, 1, 0));
    mb_couple(W, a, b);
    mb_finalize(W);
    double h = 1.0 / 2000;
    #define MOM(P, L) do { P = V3(0, 0, 0); L = V3(0, 0, 0); for (int i = 0; i < W->nb; i++) { mb_body* B = &W->B[i]; m3 t = mmul(&B->R, &B->Ib), Rt = mtrans(&B->R), Iw = mmul(&t, &Rt); \
        P = vadd(P, vscl(B->v, B->m)); L = vadd(L, vadd(vscl(vcross(B->x, B->v), B->m), mv(&Iw, B->w))); } } while (0)
    v3 P0, L0, P, L; double ep = 0, el = 0;
    mb_begin(W, h); mb_solve(W); mb_integrate(W, 1);   /* consistent start */
    MOM(P0, L0);
    for (int s = 0; s < 10000; s++) {
        mb_begin(W, h);
        /* torsion spring k=50 about the hinge axis (world, from body a) between a and b */
        v3 ax = mv(&W->B[a].R, V3(0, 1, 0)); double jj[2][6]; int bs[2] = {b, a};
        mb_row_ang(ax, jj[0]); mb_row_ang(vneg(ax), jj[1]);
        qt qr = qmul((qt){W->B[a].q.w, -W->B[a].q.x, -W->B[a].q.y, -W->B[a].q.z}, W->B[b].q);
        double ang = 2 * atan2(qr.y, qr.w);
        mb_elem_add(W, 2, bs, jj, -50 * ang, -50, -0.0);
        mb_solve(W); mb_integrate(W, 1);
        MOM(P, L); ep = fmax(ep, vlen(vsub(P, P0))); el = fmax(el, vlen(vsub(L, L0)));
    }
    check("linear momentum change [N s]", ep, 1e-10);
    check("angular momentum change / |L| over 5 s (implicit gyro, O(h))", el / vlen(L0), 2e-3);
    mb_destroy(W);
}

/* ---- 7. zero allocation + benchmark on a car-sized tree (35 bodies, ~120 rows) */
static void t_bench(void) {
    printf("[bench] car-sized mechanism: chassis + 4 x (2 arms, knuckle, wheel, link) + 5 steering + 4 ARB + 3 driveline\n");
    mb_world* W = mb_create();
    int ch = mb_add_body(W, "chassis", 900, diag3(300, 1100, 1250), V3(0, 0, 0.3), QI); W->B[ch].root = 1;
    for (int c = 0; c < 4; c++) {
        double sx = c < 2 ? 1.1 : -1.1, sy = (c & 1) ? -1 : 1;
        int ua = mb_add_body(W, "uca", 1.2, diag3(0.01, 0.005, 0.01), V3(sx, sy * 0.45, 0.45), QI);
        int la = mb_add_body(W, "lca", 2.8, diag3(0.02, 0.01, 0.02), V3(sx, sy * 0.40, 0.18), QI);
        int kn = mb_add_body(W, "knuckle", 6, diag3(0.06, 0.05, 0.06), V3(sx, sy * 0.62, 0.3), QI);
        int wh = mb_add_body(W, "wheel", 20, diag3(0.42, 0.8, 0.42), V3(sx, sy * 0.70, 0.3), QI);
        int tr = mb_add_body(W, "link", 0.6, diag3(0.005, 0.0005, 0.005), V3(sx - 0.1, sy * 0.45, 0.25), QI);
        mb_revolute(W, "uca_in", ua, ch, V3(sx, sy * 0.30, 0.45), V3(1, 0, 0));
        mb_revolute(W, "lca_in", la, ch, V3(sx, sy * 0.20, 0.18), V3(1, 0, 0));
        mb_spherical(W, "ubj", ua, kn, V3(sx, sy * 0.58, 0.48));
        mb_spherical(W, "lbj", la, kn, V3(sx, sy * 0.62, 0.15));
        mb_spherical(W, "tr_o", tr, kn, V3(sx - 0.12, sy * 0.6, 0.25));
        mb_spherical(W, "tr_i", tr, ch, V3(sx - 0.12, sy * 0.3, 0.25));
        mb_revolute(W, "bearing", wh, kn, V3(sx, sy * 0.70, 0.3), V3(0, 1, 0));
        mb_couple(W, la, ch);
        int bs[1] = {wh}; int fj = mb_custom(W, "brake", 1, 1, 1, bs); (void)fj;
    }
    for (int i = 0; i < 9; i++) { int r = mb_add_body(W, "rot", 1, diag3(0.01, 0.01, 0.01), V3(0.5 - 0.1 * i, 0, 0.3), QI); mb_revolute(W, "rot", r, ch, W->B[r].x, V3(1, 0, 0)); mb_couple(W, r, ch); }
    mb_finalize(W);
    printf("  bodies %d, joints %d, nodes %d, unknowns %d, factor blocks %d doubles\n", W->nb, W->nj, W->n, W->N, W->nblk_d);
    double h = 1.0 / 2000;
    g_nalloc = 0; g_count = 1;
    struct timespec t0, t1; clock_gettime(CLOCK_MONOTONIC, &t0);
    int steps = 4000;
    for (int s = 0; s < steps; s++) {
        mb_begin(W, h); gravity(W, 9.81);
        for (int j = 0; j < W->nj; j++) if (W->Jt[j].box) { mb_joint* J = &W->Jt[j]; memset(J->J, 0, sizeof J->J); J->J[0][0][4] = 1; J->lo[0] = -100 * h; J->hi[0] = 100 * h; }
        for (int i = 0; i < W->nb; i++) if (!strcmp(W->B[i].name, "lca")) { double jj[2][6]; int bs[2] = {i, ch}; mb_row_point(&W->B[i], W->B[i].x, V3(0, 0, 1), jj[0]); mb_row_point(&W->B[ch], W->B[i].x, V3(0, 0, -1), jj[1]); mb_elem_add(W, 2, bs, jj, 0, -3e4, -2e3); }
        mb_solve(W); mb_integrate(W, 1);
    }
    clock_gettime(CLOCK_MONOTONIC, &t1); g_count = 0;
    double us = ((t1.tv_sec - t0.tv_sec) * 1e6 + (t1.tv_nsec - t0.tv_nsec) * 1e-3) / steps;
    check("heap allocations during 4000 steps", (double)g_nalloc, 0);
    printf("  %.1f us per step (target <= 250 us)\n", us);
    check("step time over 250 us budget [us]", us > 250 ? us - 250 : 0, 0);
    FILE* f = fopen("results/mb/phase0_bench.csv", "w"); if (f) { fprintf(f, "bodies,joints,unknowns,us_per_step\n%d,%d,%d,%.2f\n", W->nb, W->nj, W->N, us); fclose(f); }
    mb_destroy(W);
}

int main(void) {
    if (system("mkdir -p results/mb")) {}
    printf("=== dc_mb Phase 0 ===\n");
    t_pendulum(); t_slider_crank(); t_fourbar(); t_static(); t_friction(); t_jfric(); t_momentum(); t_bench();
    printf("=== %s (%d failures) ===\n", fails ? "FAIL" : "ALL PASS", fails);
    return fails ? 1 : 0;
}
