/* dc_car.c - vehicle assembly, multibody integrator, steering chain, driveline.
 *
 * Generalized velocities u = [v(3) ω(3) ṡ1..4 ṙ θ̇sw]  (world v/ω of the sprung body, hub travels,
 * rack displacement, steering-wheel angle).  Bodies: chassis, 4 corners (upright + non-spinning wheel),
 * rack, pinion rotor, steering-wheel rotor.  Each body b has a world velocity Jacobian J_b(q) and bias
 * acceleration a_b0 (centripetal/Coriolis + exact kinematic J̇ṡ).  Equations of motion (virtual power):
 *       ( Σ J_bᵀ M_b J_b ) u̇ = Σ J_bᵀ ( F_b - M_b a_b0 - [0, ω_b×I_bω_b] ) + Q_internal
 * Internal elements (springs, dampers, bump stops, ARB, torsion bar, assist, rack friction, rack stops)
 * enter only as generalized forces from relative coordinates, so every one of them is an equal-and-opposite
 * pair by construction (Newton 3) and internal reactions sum to zero in Σ m a.
 * Wheel spins Ω (absolute, about the spindle) and the driveline (engine, gearbox) are separate rotors
 * coupled by a capped-impulse PGS (clutch, open/LSD diff, centre diff, brakes) whose reactions go to the
 * chassis (housings) or the uprights (calipers); rotor gyroscopic torques go to their carriers.
 * Time stepping: symplectic Euler at the fixed substep rate, quaternion attitude, no allocation.
 */
#include "dc_tyre.h"
#include "dc_mbcar.h"
#include <stdlib.h>
#include <stdio.h>

static const v3 EX = {1, 0, 0}, EY = {0, 1, 0}, EZ = {0, 0, 1};

static double lut_eval(const dc_car_params* p, double rpm) {
    if (p->nlut <= 0) return 0;
    if (rpm <= p->lut_rpm[0]) return p->lut_torque[0];
    for (int i = 1; i < p->nlut; i++) if (rpm <= p->lut_rpm[i]) {
        double f = (rpm - p->lut_rpm[i - 1]) / (p->lut_rpm[i] - p->lut_rpm[i - 1]);
        return p->lut_torque[i - 1] + f * (p->lut_torque[i] - p->lut_torque[i - 1]);
    }
    return p->lut_torque[p->nlut - 1];
}

static double kz_of(const dc_tyre_params* T, double p) {
    return dc_tyre_kz(T, p);
}

static v3 rack_point(const dc_car* c) {   /* rack COM in chassis frame (between the inner tie-rod joints) */
    const dc_kin* k = &c->K[0];
    if (k->rack_link < 0) return V3(k->hub0.x, 0, k->hub0.z);
    v3 p = k->lc[k->rack_link]; p.y = c->r; return p;
}

static double wheel_steer(const dc_pose* p, const dc_kin* k) {
    v3 a = mv(&p->R, k->spin_axis_u); return atan2(-a.x, a.y);
}

DC_API const dc_car_params* dc_params(const dc_car* c) { return &c->P; }
DC_API void dc_debug_flags(dc_car* c, int f) { c->dbg = f; }
DC_API void dc_set_road(dc_car* c, dc_road_fn fn, void* user) { c->road = fn; c->road_user = user; c->road_flags = 0; }
DC_API void dc_set_road_flags(dc_car* c, int flags) { c->road_flags = flags; }
DC_API void dc_set_heightfield(dc_car* c, const dc_heightfield* hf) { c->hf = *hf; c->road = NULL; }
DC_API void dc_set_input(dc_car* c, const dc_input* in) { c->in = *in; }
DC_API double dc_ffb_torque(const dc_car* c) { return c->tel.column_torque; }
DC_API void dc_puncture(dc_car* c, int w, double tau) { if (w >= 0 && w < 4) c->ty[w].leak_tau = tau; }
DC_API void dc_set_pressure(dc_car* c, int w, double pa) { if (w >= 0 && w < 4) { c->ty[w].pressure = pa; c->ty[w].leak_tau = 0; } }

DC_API dc_car* dc_create(const dc_car_params* pin) {
    dc_car* c = (dc_car*)calloc(1, sizeof(dc_car));
    if (!c) return NULL;
    c->P = *pin;
    dc_car_params* P = &c->P;
    if (P->substep_hz < 200) P->substep_hz = 200;
    c->h = 1.0 / P->substep_hz;
    /* mass split: corners lumped at the hub, rack, the rest is sprung */
    v3 mom = V3(0, 0, 0); double mu = 0;
    for (int i = 0; i < 4; i++) {
        const dc_axle_params* A = &P->axle[i / 2];
        c->m_c[i] = A->unsprung_mass;
        v3 hp = V3(A->hub[0], (i & 1) ? -A->hub[1] : A->hub[1], A->hub[2]);
        mom = vadd(mom, vscl(hp, c->m_c[i])); mu += c->m_c[i];
        c->I_c_body[i] = V3(A->upright_inertia[0], A->upright_inertia[1], A->upright_inertia[2]);
    }
    c->m_rack = P->rack_mass > 0 ? P->rack_mass : 2.0;
    {
        const dc_axle_params* F = &P->axle[0];
        v3 rp = F->rack_link >= 0 ? V3(F->link_c[F->rack_link][0], 0, F->link_c[F->rack_link][2]) : V3(F->hub[0], 0, F->hub[2]);
        mom = vadd(mom, vscl(rp, c->m_rack)); mu += c->m_rack;
    }
    c->m_s = P->mass - mu;
    v3 scg = vscl(mom, -1.0 / c->m_s);   /* total CG = origin */
    P->sprung_cg[0] = scg.x; P->sprung_cg[1] = scg.y; P->sprung_cg[2] = scg.z;
    c->I_s_body = V3(P->sprung_inertia[0], P->sprung_inertia[1], P->sprung_inertia[2]);
    dc_kin_setup(c);
    dc_tyre_tm_init(c);
    for (int i = 0; i < 4; i++) {
        c->pose[i].c = c->K[i].hub0; c->pose[i].R = m3id();
        dc_kin_solve(&c->K[i], 0, 0, &c->pose[i], 1);
    }
    /* pinion radius from the linkage: overall ratio at centre */
    c->r_p = 0.007;
    if (c->K[0].steered && c->K[0].rack_link >= 0) {
        double e = 1e-4, d = 0;
        for (int i = 0; i < 2; i++) {
            dc_pose a = c->pose[i], b = c->pose[i];
            dc_kin_solve(&c->K[i], 0, e, &a, 0); dc_kin_solve(&c->K[i], 0, -e, &b, 0);
            d += 0.5 * (wheel_steer(&a, &c->K[i]) - wheel_steer(&b, &c->K[i])) / (2 * e);
        }
        if (fabs(d) > 1e-9) c->r_p = 1.0 / (P->steer_ratio * d);
    }
    c->rack_max = fabs(c->r_p) * P->steer_lock_deg * M_PI / 180.0;
    c->rack_hard = c->rack_max + (P->rack_stop_solid > 0 ? P->rack_stop_solid : 1.0);
    /* valid travel range of each linkage. A multi-link/DWB can pass through a toggle (rank loss of the link
       constraint Jacobian) inside the configured droop/bump travel; the dynamics would then see unbounded
       Jacobians. Scan by continuation (also at full lock for steered corners) and, if needed, pull the stop
       in so it engages >= 20 mm before the linkage degenerates (|d(rot)/ds| > 3 rad/m or solve failure). */
    c->kin_warn = 0;
    for (int i = 0; i < 4; i++) {
        const dc_axle_params* A = &P->axle[i / 2];
        c->s_dn[i] = A->bumpstop_dn; c->s_up[i] = A->bumpstop_up;
        int nr = c->K[i].steered ? 3 : 1;
        for (int dir = -1; dir <= 1; dir += 2) {
            double lim = (dir < 0 ? A->bumpstop_dn : A->bumpstop_up) + 0.06, bad = 1e9;
            for (int ir = 0; ir < nr; ir++) {
                double rl = P->rack_stop_solid > 0 ? c->rack_hard : c->rack_max, r = ir == 0 ? 0 : (ir == 1 ? rl : -rl);
                dc_pose p = c->pose[i];
                for (double s = 0; s <= lim + 1e-9; s += 0.002) {
                    int ok = dc_kin_solve(&c->K[i], dir * s, r, &p, 1);
                    if (!ok || vlen(p.Jw[0]) > 3.0 || vlen(p.Jp[0]) > 3.0) { if (s < bad) bad = s; break; }
                }
            }
            double* st = dir < 0 ? &c->s_dn[i] : &c->s_up[i];
            double need = dir < 0 ? *st : *st + 0.04;      /* bump side: progressive stop needs ~40 mm of room */
            if (bad < need + 0.02) {
                double v = bad - 0.02 - (dir < 0 ? 0 : 0.04); if (v < 0.01) v = 0.01;
                *st = v; c->kin_warn |= 1 << i;
            }
            /* solid stop: the bump rubber goes solid bumpstop_solid past contact (then metal-to-metal); always
               >= 20 mm inside the scanned valid range, so the linkage can never be driven into its toggle */
            if (dir > 0) c->s_hard[i] = A->bumpstop_solid > 0 ? fmin(*st + A->bumpstop_solid, fmin(bad, lim) - 0.02) : 1e9;
        }
    }
    c->I_sw = P->sw_inertia + P->column_inertia;
    c->I_pin = 2e-4;
    c->col_axis = vnorm(V3(-0.94, 0, 0.34));
    /* springs: rate from wheel rate & motion ratio, free length from static equilibrium at design */
    double xf = c->K[0].hub0.x + scg.x, xr = c->K[2].hub0.x + scg.x, g = P->gravity;
    double Wax[2] = {P->mass * g * (-xr) / (xf - xr), P->mass * g * xf / (xf - xr)};
    for (int i = 0; i < 4; i++) {
        const dc_axle_params* A = &P->axle[i / 2];
        dc_kin* k = &c->K[i]; dc_pose* p = &c->pose[i];
        double W = 0.5 * Wax[i / 2];
        c->static_fz[i] = W;
        double MR = -p->dL[0];
        if (MR < 0.05) MR = 0.05;
        k->MR0 = MR;
        k->k_spring = A->spring_rate > 0 ? A->spring_rate : A->wheel_rate / (MR * MR);
        double dl = W / kz_of(&A->tyre, A->tyre.pressure);
        v3 pc = V3(0, 0, -(A->tyre.radius - dl));
        double dzp = 1 + vcross(p->Jw[0], pc).z + p->Jp[0].z - 1;
        double Fsp = (W * dzp - c->m_c[i] * g) / MR;
        k->L_free = p->Ls + Fsp / k->k_spring + A->preload_travel * MR;
        k->c_bump = A->damp_bump / (MR * MR); k->c_fbump = A->damp_fast_bump / (MR * MR); k->v_bump = A->damp_bump_thr * MR;
        k->c_reb = A->damp_reb / (MR * MR); k->c_freb = A->damp_fast_reb / (MR * MR); k->v_reb = A->damp_reb_thr * MR;
        c->ty[i].pressure = A->tyre.pressure;
    }
    double z0[3] = {0, 0, 0};
    dc_reset(c, z0, 0, 0);
    c->mb = NULL;
    if (P->solver == DC_SOLVER_MULTIBODY) {
        c->mb = dc_mbcar_create(c);        /* NULL (unsupported layout, e.g. AWD): reduced core is used */
        if (c->mb) dc_reset(c, z0, 0, 0);
    }
    return c;
}

DC_API void dc_destroy(dc_car* c) { if (c && c->mb) dc_mbcar_destroy(c->mb); free(c); }
DC_API int dc_active_solver(const dc_car* c) { return c->mb ? DC_SOLVER_MULTIBODY : DC_SOLVER_REDUCED; }

DC_API void dc_reset(dc_car* c, const double pos[3], double yaw, double speed) {
    const dc_car_params* P = &c->P;
    for (int i = 0; i < 4; i++) {
        c->s[i] = 0; c->sd[i] = 0;
        c->pose[i].c = c->K[i].hub0; c->pose[i].R = m3id();
        dc_kin_solve(&c->K[i], 0, 0, &c->pose[i], 1);
        double p = c->ty[i].pressure;
        memset(&c->ty[i], 0, sizeof c->ty[i]);
        c->ty[i].pressure = p > 0 ? p : P->axle[i / 2].tyre.pressure;
        c->ty[i].omega = speed / P->axle[i / 2].tyre.radius;
    }
    c->r = c->rd = c->th = c->thd = 0;
    /* ride height: hub height above ground = loaded radius */
    const dc_tyre_params* T = &P->axle[0].tyre;
    double dl = c->static_fz[0] / kz_of(T, c->ty[0].pressure);
    double hub_z = c->K[0].hub0.z;   /* chassis frame (sprung CG origin) */
    double nx, g; v3 n; (void)nx;
    double ground = dc_road_eval(c, pos[0], pos[1], &n, &g);
    c->x = V3(pos[0], pos[1], ground + (T->radius - dl) - hub_z);
    c->q = qexp(V3(0, 0, yaw));
    m3 R = qtom(c->q);
    c->v = mv(&R, V3(speed, 0, 0)); c->w = V3(0, 0, 0);
    c->we = c->P.idle_rpm * M_PI / 30; c->wg = 0; c->engine_on = 1; c->have_mt = 0; c->proj_work = 0; c->body_audit = 0; c->body_w = 0; c->body_pe = 0;
    memset(c->lam, 0, sizeof c->lam);
    c->t = 0; c->tick = 0; c->dissipated = 0; c->acc_dt = 0;
    memset(c->body_set, 0, sizeof c->body_set);
    memset(&c->tel, 0, sizeof c->tel);
    if (c->mb) dc_mbcar_reset(c);
}

/* ------------------------------------------------------------------ generalized assembly helpers */
typedef struct { double Jl[3][NQ], Ja[3][NQ]; double m; m3 I; v3 a0, al0, F, T, wb, xb; int has_lin; } body;

static void setcol(double J[3][NQ], int col, v3 v) { J[0][col] = v.x; J[1][col] = v.y; J[2][col] = v.z; }
static void skewcols(double J[3][NQ], int col, v3 r) { /* J[:, col..col+2] = -[r]x  (v + ω×r) */
    J[0][col] = 0;    J[0][col + 1] = r.z;  J[0][col + 2] = -r.y;
    J[1][col] = -r.z; J[1][col + 1] = 0;    J[1][col + 2] = r.x;
    J[2][col] = r.y;  J[2][col + 1] = -r.x; J[2][col + 2] = 0;
}
static v3 Jmul(double J[3][NQ], const double* u) {
    v3 r = {0, 0, 0};
    for (int j = 0; j < NQ; j++) { r.x += J[0][j] * u[j]; r.y += J[1][j] * u[j]; r.z += J[2][j] * u[j]; }
    return r;
}
static void accumulate(double* M, double* rhs, const body* b) {
    /* linear */
    if (b->has_lin) {
        v3 f = vsub(b->F, vscl(b->a0, b->m));
        for (int i = 0; i < NQ; i++) {
            double ji0 = b->Jl[0][i], ji1 = b->Jl[1][i], ji2 = b->Jl[2][i];
            if (ji0 == 0 && ji1 == 0 && ji2 == 0) continue;
            rhs[i] += ji0 * f.x + ji1 * f.y + ji2 * f.z;
            for (int j = 0; j < NQ; j++) M[i * NQ + j] += b->m * (ji0 * b->Jl[0][j] + ji1 * b->Jl[1][j] + ji2 * b->Jl[2][j]);
        }
    }
    v3 Iw = mv(&b->I, b->wb);
    v3 t = vsub(vsub(b->T, mv(&b->I, b->al0)), vcross(b->wb, Iw));
    double IJ[3][NQ];
    for (int r = 0; r < 3; r++) for (int j = 0; j < NQ; j++)
        IJ[r][j] = b->I.m[r][0] * b->Ja[0][j] + b->I.m[r][1] * b->Ja[1][j] + b->I.m[r][2] * b->Ja[2][j];
    for (int i = 0; i < NQ; i++) {
        double ji0 = b->Ja[0][i], ji1 = b->Ja[1][i], ji2 = b->Ja[2][i];
        if (ji0 == 0 && ji1 == 0 && ji2 == 0) continue;
        rhs[i] += ji0 * t.x + ji1 * t.y + ji2 * t.z;
        for (int j = 0; j < NQ; j++) M[i * NQ + j] += ji0 * IJ[0][j] + ji1 * IJ[1][j] + ji2 * IJ[2][j];
    }
}
static m3 rot_inertia(const m3* R, v3 d) {
    m3 D; memset(&D, 0, sizeof D); D.m[0][0] = d.x; D.m[1][1] = d.y; D.m[2][2] = d.z;
    m3 t = mmul(R, &D); m3 Rt = mtrans(R); return mmul(&t, &Rt);
}
static m3 rank1(v3 a, double I) {
    m3 r; for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) r.m[i][j] = I * vget(a, i) * vget(a, j); return r;
}

/* scratch kept on the stack of dc_substep */
typedef struct {
    body b[8]; int nb;
    dc_tc tc[4];
    v3 cw[4], a[4], vc[4], wc[4];
    double udot[NQ];
    v3 Fcorner_ext[4], Tcorner_ext[4];
    double Qint[NQ];
    double Fspring_ax[4], Fdamp_ax[4], Fbump_w[4], Farb_w[4];
    double tau_tb, F_assist, F_rackfric, T_hands;
    double c_rack_imp;   /* linearly-implicit rack damping coefficient (valve back-drive orifice), N s/m */
} step_scratch;

static void build_bodies(dc_car* c, step_scratch* S, const m3* R) {
    const double* uq;
    double u[NQ];
    u[0] = c->v.x; u[1] = c->v.y; u[2] = c->v.z; u[3] = c->w.x; u[4] = c->w.y; u[5] = c->w.z;
    for (int i = 0; i < 4; i++) u[IS + i] = c->sd[i];
    u[IR] = c->rd; u[ISW] = c->thd; uq = u;
    memset(S->b, 0, sizeof S->b);
    int nb = 0;
    v3 w = c->w;
    /* chassis */
    body* b = &S->b[nb++];
    b->has_lin = 1; b->m = c->m_s; b->I = rot_inertia(R, c->I_s_body);
    setcol(b->Jl, 0, EX); setcol(b->Jl, 1, EY); setcol(b->Jl, 2, EZ);
    setcol(b->Ja, 3, EX); setcol(b->Ja, 4, EY); setcol(b->Ja, 5, EZ);
    b->wb = w; b->xb = c->x;
    /* corners */
    for (int i = 0; i < 4; i++) {
        dc_pose* p = &c->pose[i]; const dc_kin* k = &c->K[i];
        b = &S->b[nb++];
        b->has_lin = 1; b->m = c->m_c[i];
        v3 rho = mv(R, p->c);
        skewcols(b->Jl, 3, rho);
        setcol(b->Jl, 0, EX); setcol(b->Jl, 1, EY); setcol(b->Jl, 2, EZ);
        setcol(b->Jl, IS + i, mv(R, p->Jp[0]));
        setcol(b->Ja, 3, EX); setcol(b->Ja, 4, EY); setcol(b->Ja, 5, EZ);
        setcol(b->Ja, IS + i, mv(R, p->Jw[0]));
        double rd = 0;
        if (k->steered) { setcol(b->Jl, IR, mv(R, p->Jp[1])); setcol(b->Ja, IR, mv(R, p->Jw[1])); rd = c->rd; }
        v3 prel = vadd(vscl(p->Jp[0], c->sd[i]), vscl(p->Jp[1], rd));
        v3 wrel = vadd(vscl(p->Jw[0], c->sd[i]), vscl(p->Jw[1], rd));
        v3 prel_w = mv(R, prel), wrel_w = mv(R, wrel);
        b->a0 = vadd(vadd(vcross(w, vcross(w, rho)), vscl(vcross(w, prel_w), 2)), mv(R, p->bp));
        b->al0 = vadd(vcross(w, wrel_w), mv(R, p->bw));
        b->wb = vadd(w, wrel_w);
        m3 Ru = mmul(R, &p->R);
        b->I = rot_inertia(&Ru, c->I_c_body[i]);
        b->xb = vadd(c->x, rho);
        S->cw[i] = b->xb;
        S->a[i] = mv(&Ru, k->spin_axis_u);
        S->vc[i] = Jmul(b->Jl, uq);
        S->wc[i] = b->wb;
    }
    /* rack */
    b = &S->b[nb++];
    {
        b->has_lin = 1; b->m = c->m_rack;
        v3 rho = mv(R, rack_point(c));
        setcol(b->Jl, 0, EX); setcol(b->Jl, 1, EY); setcol(b->Jl, 2, EZ); skewcols(b->Jl, 3, rho);
        v3 yw = mv(R, EY);
        setcol(b->Jl, IR, yw);
        b->a0 = vadd(vcross(w, vcross(w, rho)), vscl(vcross(w, vscl(yw, c->rd)), 2));
        b->wb = w; b->I = rank1(EX, 0); b->xb = vadd(c->x, rho);
        setcol(b->Ja, 3, EX); setcol(b->Ja, 4, EY); setcol(b->Ja, 5, EZ);
    }
    /* pinion rotor (spins with ṙ/r_p about the column axis) */
    b = &S->b[nb++];
    {
        v3 ax = mv(R, c->col_axis);
        b->has_lin = 0; b->I = rank1(ax, c->I_pin);
        setcol(b->Ja, 3, EX); setcol(b->Ja, 4, EY); setcol(b->Ja, 5, EZ); setcol(b->Ja, IR, vscl(ax, 1.0 / c->r_p));
        b->wb = vadd(w, vscl(ax, c->rd / c->r_p)); b->al0 = vcross(w, vscl(ax, c->rd / c->r_p));
    }
    /* steering wheel rotor */
    b = &S->b[nb++];
    {
        v3 ax = mv(R, c->col_axis);
        b->has_lin = 0; b->I = rank1(ax, c->I_sw);
        setcol(b->Ja, 3, EX); setcol(b->Ja, 4, EY); setcol(b->Ja, 5, EZ); setcol(b->Ja, ISW, ax);
        b->wb = vadd(w, vscl(ax, c->thd)); b->al0 = vcross(w, vscl(ax, c->thd));
    }
    S->nb = nb;
}

/* internal generalized forces (all relative-coordinate potentials / dissipative pairs) */
static double damper_force(const dc_kin* k, double vcomp) {
    if (vcomp >= 0) return vcomp <= k->v_bump ? k->c_bump * vcomp : k->c_bump * k->v_bump + k->c_fbump * (vcomp - k->v_bump);
    double v = -vcomp;
    return -(v <= k->v_reb ? k->c_reb * v : k->c_reb * k->v_reb + k->c_freb * (v - k->v_reb));
}

static void internal_forces(dc_car* c, step_scratch* S, double* pe_out, double* pdiss) {
    const dc_car_params* P = &c->P;
    double* Q = S->Qint; memset(Q, 0, sizeof S->Qint);
    double pe = 0, diss = 0;
    int nodamp = c->dbg & DC_DBG_NO_DAMPING;
    for (int i = 0; i < 4; i++) {
        const dc_kin* k = &c->K[i]; const dc_pose* p = &c->pose[i]; const dc_axle_params* A = &P->axle[i / 2];
        double rd = k->steered ? c->rd : 0;
        /* coilover spring (axial) */
        double Fs = k->k_spring * (k->L_free - p->Ls);
        pe += 0.5 * k->k_spring * (k->L_free - p->Ls) * (k->L_free - p->Ls);
        double Ldot = p->dL[0] * c->sd[i] + p->dL[1] * rd;
        double Fd = nodamp ? 0 : damper_force(k, -Ldot);
        diss += Fd * (-Ldot);
        double Fax = Fs + Fd;
        Q[IS + i] += Fax * p->dL[0];
        if (k->steered) Q[IR] += Fax * p->dL[1];
        S->Fspring_ax[i] = Fs; S->Fdamp_ax[i] = Fd;
        /* bump stop (progressive) and droop stop, wheel units on the travel coordinate */
        double s = c->s[i], Fb = 0;
        if (s > c->s_up[i]) {
            double x = s - c->s_up[i], x0 = 0.01;
            Fb = A->bumpstop_rate * x * (1 + x / x0);
            pe += A->bumpstop_rate * (0.5 * x * x + x * x * x / (3 * x0));
            if (!nodamp && c->sd[i] > 0) { double fd = 0.1 * sqrt(A->bumpstop_rate * (1 + x / x0) * c->m_c[i]) * 2 * c->sd[i]; Fb += fd; diss += fd * c->sd[i]; }
        } else if (s < -c->s_dn[i]) {
            double x = s + c->s_dn[i];   /* negative */
            Fb = A->droop_rate * x;
            pe += 0.5 * A->droop_rate * x * x;
            if (!nodamp && c->sd[i] < 0) { double fd = 0.2 * sqrt(A->droop_rate * c->m_c[i]) * 2 * c->sd[i]; Fb += fd; diss += fd * c->sd[i]; }
        }
        Q[IS + i] -= Fb; S->Fbump_w[i] = Fb;
    }
    /* anti-roll bars: V = ½ k (sL - sR)² */
    for (int a = 0; a < 2; a++) {
        double d = c->s[2 * a] - c->s[2 * a + 1], k = P->axle[a].arb;
        Q[IS + 2 * a] -= k * d; Q[IS + 2 * a + 1] += k * d;
        S->Farb_w[2 * a] = k * d; S->Farb_w[2 * a + 1] = -k * d;
        pe += 0.5 * k * d * d;
    }
    /* steering column: torsion bar between steering wheel and pinion */
    double ktb = P->assist ? P->assist_torsion_k : P->torsion_k;
    double twist = c->th - c->r / c->r_p, twd = c->thd - c->rd / c->r_p;
    double tau = ktb * twist + (nodamp ? 0 : P->torsion_c * twd);
    pe += 0.5 * ktb * twist * twist; diss += (nodamp ? 0 : P->torsion_c * twd * twd);
    Q[ISW] -= tau; Q[IR] += tau / c->r_p;
    S->tau_tb = tau;
    /* hydraulic assist: open-centre rotary valve on torsion-bar torque, pump flow limit, hydraulic damping */
    S->F_assist = 0;
    if (P->assist) {
        double x = tau / P->assist_tref;
        double x4 = x * x * x * x;         /* rotary-valve boost curve: ~flat on centre (<1 bar at 1 N m), steep beyond ~2 N m */
        double dP = P->assist_pmax * sgn(x) * x4 / (1 + x4);
        double F = P->assist_area * dP * sgn(c->r_p);
        double rpm = c->we * 30 / M_PI;
        double flow = P->assist_flow * clampd(rpm / P->assist_rpm_full, 0, 1);
        double va = c->rd * sgn(F);
        /* open-centre bridge flow continuity. Rack moving WITH the assist: the cylinder demands load flow
           A*v that the pump must supply, so chamber pressure collapses as (1 - A v / Q) (pump flow limit).
           Rack BACK-DRIVEN against the assist (tyre SAT pushing the rack while the hands/inertia keep the
           torsion bar twisted): the displaced oil is forced through the same, now nearly closed, valve
           orifices on top of the pump flow, so the pressure RISES as (1 + A|v|/Q)^2 (orifice dP ~ Q^2),
           capped by the relief valve. Linearised about v = 0 this is a damper c = 2 |F| A / Q acting only
           while back-driven; it is applied linearly-implicitly in the mass matrix (it reaches ~50 kN s/m,
           far beyond the explicit limit). assist_backdrive scales it (0.5 = orifice law with the back-driven flow split across the two parallel legs of the open-centre bridge, 1 = single orifice, 0 = legacy). */
        if (va > 0) F *= clampd(1 - va * P->assist_area / fmax(flow, 1e-9), 0, 1);
        double cbd = 0;
        if (!nodamp && va < 0 && P->assist_backdrive > 0) {
            double Qe = fmax(flow, 0.1 * P->assist_flow);
            cbd = P->assist_backdrive * 2 * fabs(F) * P->assist_area / Qe;
            double Frel = 1.25 * P->assist_pmax * P->assist_area;     /* relief valve */
            double room = fmax(Frel - fabs(F), 0);
            if (cbd * fabs(c->rd) > room) cbd = room / fmax(fabs(c->rd), 1e-9);
        }
        double Fh = nodamp ? 0 : P->assist_damping * c->rd;
        Q[IR] += F - Fh; diss += Fh * c->rd;
        Q[IR] -= cbd * c->rd;               /* explicit part; implicit h*cbd added to M[IR,IR] at the solve */
        S->c_rack_imp = cbd;
        S->F_assist = F - cbd * c->rd;
    }
    /* rack + column Coulomb friction (regularised) */
    if (!nodamp) {
        double fr = P->rack_friction * tanh(c->rd / 0.005), fc = P->column_friction * tanh(c->thd / 0.05);
        Q[IR] -= fr; Q[ISW] -= fc; diss += fr * c->rd + fc * c->thd; S->F_rackfric = fr;
    }
    /* rack end stops */
    double ex = fabs(c->r) - c->rack_max;
    if (ex > 0) {
        double F = P->rack_stop_k * ex * sgn(c->r);
        if (!nodamp && c->rd * c->r > 0) F += P->rack_stop_c * c->rd;
        Q[IR] -= F; pe += 0.5 * P->rack_stop_k * ex * ex;
    }
    /* driver's hands (reaction is on the driver, i.e. the seat/chassis: internal) */
    double Th = 0;
    if (!(c->dbg & DC_DBG_FREEZE_STEER)) {
        if (c->in.steer_mode == DC_STEER_ANGLE) {
            Th = c->in.hands_k * (c->in.steer_target - c->th) - c->in.hands_d * c->thd;
            if (c->in.hands_tmax > 0) Th = clampd(Th, -c->in.hands_tmax, c->in.hands_tmax);
        } else Th = c->in.steer_torque;
    }
    Q[ISW] += Th; S->T_hands = Th;
    *pe_out = pe; *pdiss = diss;
}

/* ------------------------------------------------------------------ driveline PGS */
enum { CL_CLUTCH, CL_DIFF, CL_LSDF, CL_LSDR, CL_CENTER, CL_BRAKE0, CL_N = CL_BRAKE0 + 4 };

typedef struct { double coef[6]; double lo, hi; int active; } dcon;

static void driveline(dc_car* c, step_scratch* S, double* Ieff, double* Wfree, double Te,
                      double* imp /*6 shaft impulses out*/, v3* Tch, v3* Tup /*4*/) {
    const dc_car_params* P = &c->P;
    double h = c->h;
    m3 R = qtom(c->q);
    v3 xw = mv(&R, EX);
    double wx = vdot(c->w, xw);
    double v[6], inv[6], house[6];
    v[0] = c->we + h * Te / P->engine_inertia; inv[0] = 1 / P->engine_inertia; house[0] = wx;
    v[1] = c->wg; inv[1] = 1 / fmax(P->gearbox_inertia, 1e-4); house[1] = wx;
    for (int i = 0; i < 4; i++) { v[2 + i] = Wfree[i]; inv[2 + i] = 1 / Ieff[i]; house[2 + i] = vdot(c->w, S->a[i]); }
    dcon C[CL_N]; memset(C, 0, sizeof C);
    int gear = c->in.gear;
    double G = 0;
    if (gear > 0 && gear <= P->ngears) G = P->gear[gear - 1] * P->final_drive;
    else if (gear < 0) G = P->gear_reverse * P->final_drive;
    double cw[4] = {0, 0, 0, 0};
    double k = P->drive == DC_DRIVE_FWD ? 1.0 : (P->drive == DC_DRIVE_RWD ? 0.0 : P->awd_front_split);
    cw[0] = cw[1] = 0.5 * k; cw[2] = cw[3] = 0.5 * (1 - k);
    double clutch_cap = P->clutch_torque * clampd(1 - c->in.clutch, 0, 1) * h;
    C[CL_CLUTCH].active = 1; C[CL_CLUTCH].coef[0] = 1; C[CL_CLUTCH].coef[1] = -1;
    C[CL_CLUTCH].lo = -clutch_cap; C[CL_CLUTCH].hi = clutch_cap;
    if (G != 0) {
        C[CL_DIFF].active = 1; C[CL_DIFF].coef[1] = 1.0 / G;
        for (int i = 0; i < 4; i++) C[CL_DIFF].coef[2 + i] = -cw[i];
        C[CL_DIFF].lo = -1e30; C[CL_DIFF].hi = 1e30;
    }
    if (k > 0) { C[CL_LSDF].active = 1; C[CL_LSDF].coef[2] = 1; C[CL_LSDF].coef[3] = -1; }
    if (k < 1) { C[CL_LSDR].active = 1; C[CL_LSDR].coef[4] = 1; C[CL_LSDR].coef[5] = -1; }
    if (P->drive == DC_DRIVE_AWD) {
        C[CL_CENTER].active = 1; C[CL_CENTER].coef[2] = C[CL_CENTER].coef[3] = 0.5; C[CL_CENTER].coef[4] = C[CL_CENTER].coef[5] = -0.5;
    }
    for (int i = 0; i < 4; i++) {
        const dc_axle_params* A = &P->axle[i / 2];
        double cap = (clampd(c->in.brake, 0, 1) * A->brake_torque + clampd(c->in.handbrake, 0, 1) * A->handbrake_torque) * h;
        dcon* b = &C[CL_BRAKE0 + i];
        b->active = cap > 0; b->coef[2 + i] = 1; b->lo = -cap; b->hi = cap;
    }
    double bh[6]; memcpy(bh, house, sizeof bh);
    double brake_house[4];
    for (int i = 0; i < 4; i++) brake_house[i] = vdot(S->wc[i], S->a[i]);
    double* lam = c->lam;
    /* warm start */
    for (int n = 0; n < CL_N; n++) {
        if (!C[n].active) { lam[n] = 0; continue; }
        lam[n] *= 0.8;
        if (n == CL_CLUTCH || n >= CL_BRAKE0) lam[n] = clampd(lam[n], C[n].lo, C[n].hi);
        if ((n == CL_LSDF || n == CL_LSDR) && P->diff_viscous <= 0 && P->diff_preload <= 0) lam[n] = 0;
        for (int j = 0; j < 6; j++) v[j] += C[n].coef[j] * inv[j] * lam[n];
    }
    for (int it = 0; it < 30; it++) {
        for (int n = 0; n < CL_N; n++) {
            dcon* cn = &C[n]; if (!cn->active) continue;
            if (n == CL_LSDF || n == CL_LSDR || n == CL_CENTER) {
                double share = n == CL_LSDF ? cw[0] + cw[1] : (n == CL_LSDR ? cw[2] + cw[3] : 1.0);
                double Tin = fabs(lam[CL_DIFF]) * share / h;
                double Tw = -lam[CL_DIFF] * share;   /* impulse into the axle */
                double wav = n == CL_LSDF ? v[2] + v[3] : (n == CL_LSDR ? v[4] + v[5] : v[2] + v[3] + v[4] + v[5]);
                double lock = (n == CL_CENTER) ? P->center_lock : (Tw * wav >= 0 ? P->diff_power : P->diff_coast);
                double cap = (P->diff_preload + lock * Tin) * h;
                cn->lo = -cap; cn->hi = cap;
            }
            double err = 0, eff = 0;
            for (int j = 0; j < 6; j++) {
                if (cn->coef[j] == 0) continue;
                double hv = (n >= CL_BRAKE0) ? brake_house[j - 2] : bh[j];
                err += cn->coef[j] * (v[j] - hv); eff += cn->coef[j] * cn->coef[j] * inv[j];
            }
            if (eff <= 0) continue;
            double nl;
            if ((n == CL_LSDF || n == CL_LSDR) && P->diff_viscous > 0) {
                /* viscous coupling: implicit damper T = c*dw between the axle halves (soft row, gamma = 1/(c h)) */
                double gam = 1.0 / (P->diff_viscous * h);
                nl = lam[n] - (err + gam * lam[n]) / (eff + gam);
            } else nl = clampd(lam[n] - err / eff, cn->lo, cn->hi);
            double d = nl - lam[n];
            lam[n] = nl;
            if (d != 0) for (int j = 0; j < 6; j++) v[j] += cn->coef[j] * inv[j] * d;
        }
    }
    /* friction work of slipping clutch / LSD / brakes (dissipation bookkeeping) */
    {
        double dsum = 0;
        for (int n = 0; n < CL_N; n++) {
            if (!C[n].active || n == CL_DIFF) continue;
            double err = 0;
            for (int j = 0; j < 6; j++) if (C[n].coef[j] != 0) err += C[n].coef[j] * (v[j] - ((n >= CL_BRAKE0) ? brake_house[j - 2] : bh[j]));
            dsum += -lam[n] * err / h;
        }
        c->dissipated += fmax(dsum, 0) * h;
    }
    for (int j = 0; j < 6; j++) imp[j] = 0;
    *Tch = V3(0, 0, 0);
    for (int i = 0; i < 4; i++) Tup[i] = V3(0, 0, 0);
    v3 axes[6] = {xw, xw, S->a[0], S->a[1], S->a[2], S->a[3]};
    for (int n = 0; n < CL_N; n++) {
        if (!C[n].active) continue;
        for (int j = 0; j < 6; j++) {
            double pj = C[n].coef[j] * lam[n];
            if (pj == 0) continue;
            imp[j] += pj;
            v3 react = vscl(axes[j], -pj / h);
            if (n >= CL_BRAKE0) Tup[n - CL_BRAKE0] = vadd(Tup[n - CL_BRAKE0], react);
            else *Tch = vadd(*Tch, react);
        }
    }
    c->brake_imp[0] = lam[CL_BRAKE0]; c->brake_imp[1] = lam[CL_BRAKE0 + 1];
    c->brake_imp[2] = lam[CL_BRAKE0 + 2]; c->brake_imp[3] = lam[CL_BRAKE0 + 3];
    for (int i = 0; i < 4; i++) c->drive_imp[i] = imp[2 + i] - lam[CL_BRAKE0 + i] * (C[CL_BRAKE0 + i].active);
}

/* Joint/link reactions (telemetry): each link is a two-force member; the spring/ARB/bump-stop path is
 * idealised as one force along the chassis z at the hub (DWB/multilink) or along the strut axis (strut).
 * Solves  Σ f_j w_j = W_req  (6x6) where W_req is the wrench the upright needs from the chassis side. */
static void link_forces(dc_car* c, step_scratch* S, const double* ud, const m3* R) {
    for (int i = 0; i < 4; i++) {
        body* b = &S->b[1 + i]; const dc_kin* k = &c->K[i]; const dc_pose* p = &c->pose[i];
        v3 acc = vadd(Jmul(b->Jl, ud), b->a0), alp = vadd(Jmul(b->Ja, ud), b->al0);
        v3 Freq = vsub(vscl(acc, b->m), b->F);
        v3 Iw = mv(&b->I, b->wb);
        v3 Treq = vsub(vadd(mv(&b->I, alp), vcross(b->wb, Iw)), b->T);
        double A[36], rhs[6]; int col = 0;
        v3 cw = S->cw[i];
        m3 Ru = mmul(R, &p->R);
        for (int l = 0; l < k->nl && col < 6; l++, col++) {
            v3 Pu = vadd(cw, mv(&Ru, k->lu[l]));
            v3 Qc = k->lc[l]; if (l == k->rack_link && k->steered) Qc.y += c->r;
            v3 Qw = vadd(c->x, mv(R, Qc));
            v3 u = vnorm(vsub(Qw, Pu)), m = vcross(vsub(Pu, cw), u);
            A[0 * 6 + col] = u.x; A[1 * 6 + col] = u.y; A[2 * 6 + col] = u.z;
            A[3 * 6 + col] = m.x; A[4 * 6 + col] = m.y; A[5 * 6 + col] = m.z;
        }
        if (k->type == DC_SUSP_STRUT) {
            v3 Tw = vadd(c->x, mv(R, k->strut_top));
            v3 dirs[3] = {mv(&Ru, k->strut_e1), mv(&Ru, k->strut_e2), mv(&Ru, k->strut_d)};
            for (int j = 0; j < 3 && col < 6; j++, col++) {
                v3 m = vcross(vsub(Tw, cw), dirs[j]);
                A[0 * 6 + col] = dirs[j].x; A[1 * 6 + col] = dirs[j].y; A[2 * 6 + col] = dirs[j].z;
                A[3 * 6 + col] = m.x; A[4 * 6 + col] = m.y; A[5 * 6 + col] = m.z;
            }
        } else if (col < 6) {
            v3 z = mv(R, EZ);
            A[0 * 6 + col] = z.x; A[1 * 6 + col] = z.y; A[2 * 6 + col] = z.z;
            A[3 * 6 + col] = 0; A[4 * 6 + col] = 0; A[5 * 6 + col] = 0; col++;
        }
        for (; col < 6; col++) for (int r = 0; r < 6; r++) A[r * 6 + col] = (r == col) ? 1e-9 : 0;
        rhs[0] = Freq.x; rhs[1] = Freq.y; rhs[2] = Freq.z; rhs[3] = Treq.x; rhs[4] = Treq.y; rhs[5] = Treq.z;
        double A0[36], r0[6]; memcpy(A0, A, sizeof A); memcpy(r0, rhs, sizeof rhs);
        if (!dc_solve(6, A, rhs)) { memset(c->link_f[i], 0, sizeof c->link_f[i]); continue; }
        double res = 0;
        for (int r = 0; r < 6; r++) { double s2 = -r0[r]; for (int j = 0; j < 6; j++) s2 += A0[r * 6 + j] * rhs[j]; res += s2 * s2; }
        c->link_resid[i] = sqrt(res);
        for (int j = 0; j < 6 && j <= DC_MAX_LINKS; j++) c->link_f[i][j] = rhs[j];
    }
}

/* ------------------------------------------------------------------ one fixed substep */
DC_API void dc_substep(dc_car* c) {
    if (c->mb) { dc_mbcar_step(c); return; }
    const dc_car_params* P = &c->P;
    const double h = c->h, g = P->gravity;
    step_scratch S; S.c_rack_imp = 0;
    m3 R = qtom(c->q);
    /* 1. kinematics (poses, Jacobians, bias) */
    for (int i = 0; i < 4; i++) dc_kin_full(&c->K[i], c->s[i], c->r, c->sd[i], c->K[i].steered ? c->rd : 0, &c->pose[i]);
    build_bodies(c, &S, &R);
    /* 2. tyre contact, phase A */
    for (int i = 0; i < 4; i++) dc_tyre_phaseA(c, i, S.cw[i], S.a[i], S.vc[i], S.wc[i], h, &S.tc[i]);
    /* 3. engine torque */
    double rpm = c->we * 30 / M_PI;
    double thr = clampd(c->in.throttle, 0, 1);
    if (rpm > P->limiter_rpm) thr = 0;
    /* combustion only while running; stalls below 250 rpm; starter cranks with clutch in or neutral */
    int crank = (c->in.clutch > 0.5 || c->in.gear == 0) && rpm < 0.6 * P->idle_rpm;
    if (c->engine_on && rpm < 250 && !crank) c->engine_on = 0;
    if (!c->engine_on && rpm > 350) c->engine_on = 1;
    if (c->dbg & DC_DBG_ENGINE_OFF) { c->engine_on = 0; crank = 0; }
    double thr_idle = clampd((P->idle_rpm + 50 - rpm) / 200, 0, 1) * 0.6;
    if (thr_idle > thr) thr = thr_idle;
    if (!c->engine_on) thr = 0;
    double Te = lut_eval(P, fmax(rpm, 0)) * thr - P->coast_torque * clampd(rpm / P->coast_rpm, -1, 2) * (1 - thr);
    if (c->dbg & DC_DBG_ENGINE_OFF) Te = 0;
    if (crank) Te += 90;   /* starter motor (reaction on the block = chassis, like combustion torque) */
    /* 4. wheel spin: implicit tyre linearisation + driveline/brake PGS */
    double Ieff[4], Wfree[4], imp[6];
    v3 Tch_drive, Tup_brake[4];
    for (int i = 0; i < 4; i++) {
        dc_tc* t = &S.tc[i];
        double Is = P->axle[i / 2].tyre.spin_inertia;
        double add = h * (-t->arm_x) * t->kx * t->D;
        Ieff[i] = Is + (add > 0 ? add : 0);
        Wfree[i] = c->ty[i].omega + h * (t->Trest + t->arm_x * t->Fx_star) / Ieff[i];
    }
    driveline(c, &S, Ieff, Wfree, Te, imp, &Tch_drive, Tup_brake);
    double Wnew[4];
    for (int i = 0; i < 4; i++) {
        double Is = P->axle[i / 2].tyre.spin_inertia;
        double Wpgs = Wfree[i] + imp[2 + i] / Ieff[i];
        dc_tyre_phaseB(c, i, S.cw[i], S.a[i], Wpgs, h, &S.tc[i], 1);
        /* final spin from the torque actually applied (exact action/reaction with the corner body) */
        Wnew[i] = c->ty[i].omega + (h * vdot(S.tc[i].tau, S.a[i]) + imp[2 + i]) / Is;
    }
    /* 5. external loads */
    v3 Fext = V3(0, 0, 0), Text = V3(0, 0, 0);
    body* ch = &S.b[0];
    ch->F = V3(0, 0, -c->m_s * g);
    /* aero */
    if (!(c->dbg & DC_DBG_NO_AERO)) {
        double rho = 1.2, sp = vlen(c->v);
        v3 Fd = vscl(c->v, -0.5 * rho * P->cda * sp);
        v3 zb = mv(&R, EZ), xb = mv(&R, EX);
        double vx = vdot(c->v, xb), q = 0.5 * rho * vx * vx;
        v3 pf = mv(&R, V3(P->aero_front_x, 0, P->aero_z)), pr = mv(&R, V3(P->aero_rear_x, 0, P->aero_z));
        v3 Ff = vscl(zb, -q * P->cla_front), Fr = vscl(zb, -q * P->cla_rear);
        v3 pd = mv(&R, V3(0, 0, P->aero_z));
        ch->F = vadd(ch->F, vadd(Fd, vadd(Ff, Fr)));
        ch->T = vadd(ch->T, vadd(vcross(pd, Fd), vadd(vcross(pf, Ff), vcross(pr, Fr))));
    }
    /* body/ground sliding contacts */
    double pe_body = 0; v3 bF = V3(0, 0, 0), bT = V3(0, 0, 0);
    for (int k = 0; k < P->nbodypts; k++) {
        v3 rp = mv(&R, V3(P->bodypt[k][0] - P->sprung_cg[0], P->bodypt[k][1] - P->sprung_cg[1], P->bodypt[k][2] - P->sprung_cg[2]));
        v3 X = vadd(c->x, rp); v3 n; double gr;
        double pen = dc_body_depth(c, X, &n, &gr);
        if (pen <= 0) continue;
        v3 vp = vadd(c->v, vcross(c->w, rp));
        double vn = vdot(vp, n);
        double pe_k, wd, Nd; double N = dc_body_normal(c, k, pen, vn, (c->dbg & DC_DBG_NO_DAMPING) != 0, &pe_k, &wd, &Nd); (void)wd; (void)Nd;
        pe_body += pe_k; if (N <= 0) continue;
        v3 vt = vsub(vp, vscl(n, vn)); double vtl = vlen(vt);
        v3 F = vscl(n, N);
        if (vtl > 1e-9 && !(c->dbg & DC_DBG_NO_FRICTION)) F = vsub(F, vscl(vt, P->body_mu * gr * N * tanh(vtl / 0.05) / vtl));
        ch->F = vadd(ch->F, F); ch->T = vadd(ch->T, vcross(rp, F)); bF = vadd(bF, F); bT = vadd(bT, vcross(rp, F));
    }
    /* body-contact loss (crush, damping, scrape) for the energy audit: the work the contact forces did on the chassis
       over the last step (with the velocities the integrator actually moved it by) minus the change of the elastic
       energy they store. Exact for the discrete step, so a contact that returned more than it took shows up as a
       negative loss instead of hiding in an idealised formula. */
    if (c->body_audit) c->dissipated += -c->body_w - (pe_body - c->body_pe);
    c->body_pe = pe_body; c->body_audit = 1;
    Fext = ch->F; Text = vadd(vcross(c->x, ch->F), ch->T);
    /* driveline reactions on the housing + engine block reaction + rotor gyroscopic torques */
    v3 xw = mv(&R, EX);
    ch->T = vadd(ch->T, Tch_drive);
    ch->T = vadd(ch->T, vscl(xw, -Te));
    ch->T = vadd(ch->T, vscl(vcross(c->w, xw), -(P->engine_inertia * c->we + P->gearbox_inertia * c->wg)));
    for (int i = 0; i < 4; i++) {
        body* b = &S.b[1 + i];
        dc_tc* t = &S.tc[i];
        double Is = P->axle[i / 2].tyre.spin_inertia;
        v3 a = S.a[i];
        v3 tperp = vsub(t->tau, vscl(a, vdot(t->tau, a)));
        b->F = vadd(t->F, V3(0, 0, -c->m_c[i] * g));
        b->T = vadd(vadd(tperp, Tup_brake[i]), vscl(vcross(S.wc[i], a), -Is * c->ty[i].omega));
        Fext = vadd(Fext, b->F);
        Text = vadd(Text, vadd(vcross(S.cw[i], b->F), t->tau));
        c->Fc[i] = t->F; c->Pc[i] = t->p;
    }
    {
        body* b = &S.b[5];
        b->F = V3(0, 0, -c->m_rack * g);
        Fext = vadd(Fext, b->F); Text = vadd(Text, vcross(b->xb, b->F));
    }
    double pe_int, pdiss;
    internal_forces(c, &S, &pe_int, &pdiss);
    /* 6. assemble & solve */
    double M[NQ * NQ], rhs[NQ];
    memset(M, 0, sizeof M); memset(rhs, 0, sizeof rhs);
    for (int k = 0; k < S.nb; k++) accumulate(M, rhs, &S.b[k]);
    double u[NQ];
    u[0] = c->v.x; u[1] = c->v.y; u[2] = c->v.z; u[3] = c->w.x; u[4] = c->w.y; u[5] = c->w.z;
    for (int i = 0; i < 4; i++) u[IS + i] = c->sd[i];
    u[IR] = c->rd; u[ISW] = c->thd;
    /* energy and momentum snapshot (start of step) */
    {
        double ke = 0;
        for (int i = 0; i < NQ; i++) for (int j = 0; j < NQ; j++) ke += 0.5 * u[i] * M[i * NQ + j] * u[j];
        double ke_b0 = ke;
        double pe = pe_int + pe_body;
        pe += c->m_s * g * c->x.z + c->m_rack * g * S.b[5].xb.z;
        v3 Pm = V3(0, 0, 0), Lm = V3(0, 0, 0);
        for (int k = 0; k < S.nb; k++) {
            body* b = &S.b[k];
            if (b->has_lin) { v3 vb = Jmul(b->Jl, u); Pm = vadd(Pm, vscl(vb, b->m)); Lm = vadd(Lm, vcross(b->xb, vscl(vb, b->m))); }
            Lm = vadd(Lm, mv(&b->I, b->wb));
        }
        for (int i = 0; i < 4; i++) {
            double Is = P->axle[i / 2].tyre.spin_inertia;
            ke += 0.5 * Is * c->ty[i].omega * c->ty[i].omega;
            pe += c->m_c[i] * g * S.cw[i].z + S.tc[i].pe;
            Lm = vadd(Lm, vscl(S.a[i], Is * c->ty[i].omega));
        }
        ke += 0.5 * P->engine_inertia * c->we * c->we + 0.5 * P->gearbox_inertia * c->wg * c->wg;
        Lm = vadd(Lm, vscl(xw, P->engine_inertia * c->we + P->gearbox_inertia * c->wg));
        /* momentum projection: the generalized-coordinate integrator conserves the momentum balance only to
           O(h) (J(q) changes over the step). Every force is applied with its exact reaction (see audit), so the
           true discrete momentum is P_prev + h F_ext, L_prev + h T_ext; restore it with the minimum 6-DOF
           chassis velocity correction (solve the 6x6 momentum map of the base coordinates). */
        if (c->have_mt && !(c->dbg & DC_DBG_NO_PROJECTION)) {
            double A6[36], b6[6];
            memset(A6, 0, sizeof A6);
            for (int k = 0; k < S.nb; k++) {
                body* b = &S.b[k];
                for (int j = 0; j < 6; j++) {
                    v3 jl = V3(b->Jl[0][j], b->Jl[1][j], b->Jl[2][j]), ja = V3(b->Ja[0][j], b->Ja[1][j], b->Ja[2][j]);
                    v3 dp = b->has_lin ? vscl(jl, b->m) : V3(0, 0, 0);
                    v3 dl = vadd(b->has_lin ? vcross(b->xb, dp) : V3(0, 0, 0), mv(&b->I, ja));
                    A6[0 * 6 + j] += dp.x; A6[1 * 6 + j] += dp.y; A6[2 * 6 + j] += dp.z;
                    A6[3 * 6 + j] += dl.x; A6[4 * 6 + j] += dl.y; A6[5 * 6 + j] += dl.z;
                }
            }
            b6[0] = c->Pt[0] - Pm.x; b6[1] = c->Pt[1] - Pm.y; b6[2] = c->Pt[2] - Pm.z;
            b6[3] = c->Lt[0] - Lm.x; b6[4] = c->Lt[1] - Lm.y; b6[5] = c->Lt[2] - Lm.z;
            double Ac[36]; memcpy(Ac, A6, sizeof A6);
            if (dc_solve(6, Ac, b6)) {
                double dv = sqrt(b6[0] * b6[0] + b6[1] * b6[1] + b6[2] * b6[2]), dw = sqrt(b6[3] * b6[3] + b6[4] * b6[4] + b6[5] * b6[5]);
                if (dv < 2e-3 && dw < 2e-3) {   /* larger = impact step or external edit: re-seed instead */
                    for (int j = 0; j < 6; j++) u[j] += b6[j];
                    double ke1 = 0;
                    for (int ii = 0; ii < NQ; ii++) for (int jj = 0; jj < NQ; jj++) ke1 += 0.5 * u[ii] * M[ii * NQ + jj] * u[jj];
                    c->proj_work += ke1 - ke_b0; ke += ke1 - ke_b0;
                    Pm = V3(c->Pt[0], c->Pt[1], c->Pt[2]); Lm = V3(c->Lt[0], c->Lt[1], c->Lt[2]);
                }
            }
        }
        c->have_mt = 1;
        c->Pt[0] = Pm.x + h * Fext.x; c->Pt[1] = Pm.y + h * Fext.y; c->Pt[2] = Pm.z + h * Fext.z;
        c->Lt[0] = Lm.x + h * Text.x; c->Lt[1] = Lm.y + h * Text.y; c->Lt[2] = Lm.z + h * Text.z;
        c->tel.kinetic = ke; c->tel.potential = pe;
        c->tel.momentum[0] = Pm.x; c->tel.momentum[1] = Pm.y; c->tel.momentum[2] = Pm.z;
        c->tel.ang_momentum[0] = Lm.x; c->tel.ang_momentum[1] = Lm.y; c->tel.ang_momentum[2] = Lm.z;
        c->tel.ext_force[0] = Fext.x; c->tel.ext_force[1] = Fext.y; c->tel.ext_force[2] = Fext.z;
        c->tel.ext_torque[0] = Text.x; c->tel.ext_torque[1] = Text.y; c->tel.ext_torque[2] = Text.z;
    }
    for (int i = 0; i < NQ; i++) rhs[i] += S.Qint[i];
    /* linearly-implicit stiff dampers: (M + h C) ud = rhs, rhs already holds -C u, so the applied force is -C u_new */
    M[IR * NQ + IR] += h * S.c_rack_imp;
    double Mc[NQ * NQ]; memcpy(Mc, M, sizeof M);
    double ud[NQ]; memcpy(ud, rhs, sizeof rhs);
    if (!dc_chol_solve(NQ, Mc, ud)) { memcpy(Mc, M, sizeof M); memcpy(ud, rhs, sizeof rhs); dc_solve(NQ, Mc, ud); }
    M[IR * NQ + IR] -= h * S.c_rack_imp;
    if (c->dbg & DC_DBG_FREEZE_STEER) { ud[IR] = 0; ud[ISW] = 0; }
    /* 7. integrate (symplectic Euler) */
    for (int i = 0; i < NQ; i++) u[i] += h * ud[i];
    if (c->dbg & DC_DBG_FREEZE_STEER) { u[IR] = 0; u[ISW] = 0; }
    /* rigid end stops: bump-stop solid height on each travel coordinate and the rack's steel-on-steel end stop.
       Rigid, inelastic unilateral limits, speculative (a coordinate may close at most its remaining gap this step,
       never pass it): generalized impulses through the full mass matrix (wheel/rack AND body react, Newton 3),
       small PGS over the active set; the kinetic energy they remove is dissipated (plastic metal-to-metal). */
    {
        int qa[5], na = 0; double sg[5], tg[5];
        for (int i = 0; i < 4; i++) {
            c->hard_f[i] = 0;
            double gap = c->s_hard[i] - c->s[i];
            if (gap < 0.03) { qa[na] = IS + i; sg[na] = 1; tg[na] = gap > 0 ? gap / h : 0; na++; }
        }
        c->rack_hard_f = 0;
        if (P->rack_stop_solid > 0) {
            double gap = c->rack_hard - fabs(c->r);
            if (gap < 0.01) { qa[na] = IR; sg[na] = c->r >= 0 ? 1 : -1; tg[na] = gap > 0 ? gap / h : 0; na++; }
        }
        if (na) {
            double Z[5][NQ], p[5] = {0};
            for (int a = 0; a < na; a++) {
                double Mc2[NQ * NQ]; memcpy(Mc2, M, sizeof M);
                memset(Z[a], 0, sizeof Z[a]); Z[a][qa[a]] = sg[a];
                if (!dc_chol_solve(NQ, Mc2, Z[a])) { memcpy(Mc2, M, sizeof M); memset(Z[a], 0, sizeof Z[a]); Z[a][qa[a]] = sg[a]; dc_solve(NQ, Mc2, Z[a]); }
            }
            /* row velocity v_a = sg_a u[q_a] <= t_a; impulse sg_a p_a e_{q_a}, p_a <= 0 */
            for (int it = 0; it < 30; it++) {
                for (int a = 0; a < na; a++) {
                    double v = sg[a] * u[qa[a]]; for (int b2 = 0; b2 < na; b2++) v += sg[a] * Z[b2][qa[a]] * p[b2];
                    double d = sg[a] * Z[a][qa[a]]; if (d <= 0) continue;
                    double np = p[a] - (v - tg[a]) / d; if (np > 0) np = 0; p[a] = np;
                }
            }
            double uo[5]; for (int a = 0; a < na; a++) uo[a] = sg[a] * u[qa[a]];
            for (int a = 0; a < na; a++) for (int q = 0; q < NQ; q++) u[q] += Z[a][q] * p[a];
            double dke = 0;   /* KE change of an impulse set = sum p_a (v_old + v_new)/2 */
            for (int a = 0; a < na; a++) dke += p[a] * 0.5 * (uo[a] + sg[a] * u[qa[a]]);
            c->dissipated -= dke;
            for (int a = 0; a < na; a++) { if (qa[a] == IR) c->rack_hard_f = sg[a] * p[a] / h; else c->hard_f[qa[a] - IS] = -p[a] / h; }
        }
    }
    v3 vold = c->v, vold_w = c->w;
    c->v = V3(u[0], u[1], u[2]); c->w = V3(u[3], u[4], u[5]);
    for (int i = 0; i < 4; i++) c->sd[i] = u[IS + i];
    c->rd = u[IR]; c->thd = u[ISW];
    c->x = vmad(c->x, c->v, h);
    c->body_w = h * (vdot(bF, c->v) + vdot(bT, c->w));   /* body-contact work this step (audited next step) */
    c->q = qnorm(qmul(qexp(vscl(c->w, h)), c->q));
    for (int i = 0; i < 4; i++) c->s[i] += h * c->sd[i];
    c->r += h * c->rd; c->th += h * c->thd;
    c->dissipated += pdiss * h + S.c_rack_imp * u[IR] * u[IR] * h;   /* implicit damper work: F(u_new) . (h u_new) */
    /* Newton-3 / momentum audit: sum of all body inertial terms minus external loads */
    {
        v3 rf = vneg(Fext), rt = vneg(Text);
        for (int k = 0; k < S.nb; k++) {
            body* b = &S.b[k];
            v3 al = vadd(Jmul(b->Ja, ud), b->al0);
            v3 Iw = mv(&b->I, b->wb);
            rt = vadd(rt, vadd(mv(&b->I, al), vcross(b->wb, Iw)));
            if (b->has_lin) {
                v3 ab = vscl(vadd(Jmul(b->Jl, ud), b->a0), b->m);
                rf = vadd(rf, ab); rt = vadd(rt, vcross(b->xb, ab));
            }
        }
        for (int i = 0; i < 4; i++) {
            double Is = P->axle[i / 2].tyre.spin_inertia;
            double Wold = c->ty[i].omega;   /* not yet updated */
            rt = vadd(rt, vscl(S.a[i], Is * (Wnew[i] - Wold) / h));
            rt = vadd(rt, vscl(vcross(S.wc[i], S.a[i]), Is * Wold));
        }
        double Lr = P->engine_inertia * c->we + P->gearbox_inertia * c->wg;
        double we_n = c->we + h * Te / P->engine_inertia + imp[0] / P->engine_inertia;
        double wg_n = c->wg + imp[1] / fmax(P->gearbox_inertia, 1e-4);
        rt = vadd(rt, vscl(xw, (P->engine_inertia * (we_n - c->we) + fmax(P->gearbox_inertia, 1e-4) * (wg_n - c->wg)) / h));
        rt = vadd(rt, vscl(vcross(vold_w, xw), Lr));
        c->tel.resid_force[0] = rf.x; c->tel.resid_force[1] = rf.y; c->tel.resid_force[2] = rf.z;
        c->tel.resid_torque[0] = rt.x; c->tel.resid_torque[1] = rt.y; c->tel.resid_torque[2] = rt.z;
    }
    /* rotors */
    double we_old = c->we;
    c->we = c->we + h * Te / P->engine_inertia + imp[0] / P->engine_inertia;
    c->wg = c->wg + imp[1] / fmax(P->gearbox_inertia, 1e-4);
    for (int i = 0; i < 4; i++) {
        dc_tyre_state* ts = &c->ty[i];
        double rel = Wnew[i] - vdot(S.wc[i], S.a[i]);
        ts->angle = fmod(ts->angle + h * rel, 2 * M_PI);
        ts->omega = Wnew[i];
        if (ts->leak_tau > 0) ts->pressure *= exp(-h / ts->leak_tau);
    }
    /* 8. telemetry for this substep */
    dc_telemetry* T = &c->tel;
    memcpy(c->tel_udot, ud, sizeof ud);
    link_forces(c, &S, ud, &R);
    T->time = c->t; T->tick = c->tick;
    v3 xb = mv(&R, EX), yb = mv(&R, EY);
    v3 acc = vscl(vsub(c->v, vold), 1.0 / h);
    T->vx = vdot(c->v, xb); T->vy = vdot(c->v, yb); T->speed = vlen(c->v);
    T->yaw_rate = vdot(c->w, mv(&R, EZ));
    T->ax = vdot(acc, xb); T->ay = vdot(acc, yb);
    T->sw_angle = c->th; T->sw_rate = c->thd; T->rack_pos = c->r; T->rack_vel = c->rd;
    T->column_torque = S.tau_tb; T->driver_torque = S.T_hands; T->assist_force = S.F_assist;
    T->rack_force = S.Qint[IR];
    T->engine_rpm = c->we * 30 / M_PI; T->clutch_slip = (c->we - c->wg) * 30 / M_PI; T->gear = c->in.gear;
    T->dissipated = c->dissipated; T->engine_running = c->engine_on; T->proj_work = c->proj_work;
    (void)we_old;
    for (int i = 0; i < 4; i++) {
        dc_wheel_telem* W = &T->w[i]; dc_tc* t = &S.tc[i]; const dc_kin* k = &c->K[i]; dc_pose* p = &c->pose[i];
        W->fz = t->Ntot; W->fx = t->Fx; W->fy = t->Fy; W->mz = t->Mz_brush + t->Mt - t->ey_new * t->Fx;
        W->force_world[0] = t->F.x; W->force_world[1] = t->F.y; W->force_world[2] = t->F.z;
        W->contact[0] = t->p.x; W->contact[1] = t->p.y; W->contact[2] = t->p.z;
        W->normal[0] = t->n.x; W->normal[1] = t->n.y; W->normal[2] = t->n.z;
        W->slip_angle = t->contact ? atan(c->ty[i].ey / t->sig_y) : 0;
        W->slip_ratio = t->contact ? c->ty[i].ex / t->sig_x : 0;
        W->slip_angle_ss = t->slip_angle_ss; W->slip_ratio_ss = t->slip_ratio_ss;
        W->patch_half_len = t->a_eff; W->deflection = t->delta_eff; W->pressure = c->ty[i].pressure; W->rim_force = t->Fr;
        W->mu = t->mu_eff; W->sliding = fmin(t->lam, 1.0);
        W->travel = c->s[i]; W->travel_vel = c->sd[i];
        W->spring_force = S.Fspring_ax[i]; W->damper_force = S.Fdamp_ax[i]; W->bumpstop_force = S.Fbump_w[i] + c->hard_f[i];
        v3 ac = mv(&p->R, k->spin_axis_u);
        W->steer = atan2(-ac.x, ac.y); W->toe = W->steer;
        W->camber = t->contact ? ((i & 1) ? -t->camber_l : t->camber_l) : ((i & 1) ? asin(ac.z) : asin(-ac.z));
        W->omega = c->ty[i].omega; W->brake_torque = c->brake_imp[i] / h; W->drive_torque = (imp[2 + i] - c->brake_imp[i]) / h;
        W->tierod_force = k->rack_link >= 0 ? c->link_f[i][k->rack_link] : 0;
        for (int r = 0; r < DC_MAX_ROWS; r++) W->row_fz[r] = r < t->nrows ? t->N[r] : 0;
    }
    c->t += h; c->tick++;
}

DC_API int dc_step(dc_car* c, double dt) {
    c->acc_dt += dt;
    int n = 0;
    while (c->acc_dt >= c->h - 1e-12 && n < 200) { dc_substep(c); c->acc_dt -= c->h; n++; }
    return n;
}

int dc_travel_limits(const dc_car* c, int i, double* dn, double* up) {
    if (i < 0 || i > 3) return 0;
    if (dn) *dn = c->s_dn[i];
    if (up) *up = c->s_up[i];
    return (c->kin_warn >> i) & 1;
}
