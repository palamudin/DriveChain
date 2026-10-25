#include <stdio.h>
/* dc_mbcar.c - full vehicle as a maximal-coordinate multibody system (solver = DC_SOLVER_MULTIBODY).
 *
 * Every part is a rigid body with its own mass and inertia; every joint transmits only the reactions its design
 * allows; all of it is ONE coupled linearly-implicit solve per substep (dc_mb.c):
 *   chassis (sprung body, root) - per corner: A-arms (revolute to the body, ball joint to the knuckle) or two-force
 *   rods (distance rows), strut top (point-on-line), knuckle, wheel (revolute bearing, own spin + transverse inertia);
 *   steering: rim (+ upper column) revolute - U-joint - intermediate shaft - sliding U-joint - valve input shaft -
 *   torsion bar element - pinion - rack-and-pinion gear row - rack (prismatic in its housing) - tie rods (distance
 *   rows, massless two-force members) - knuckle steering arms;
 *   coil-over: spring + bilinear damper + bump/droop stops as one force element along the actual spring line
 *   between the lower seat body and the chassis (reaction on the chassis at the top mount);
 *   anti-roll bar: two bar halves on chassis revolutes, torsion element between them, drop links (distance rows)
 *   to the lower arms;
 *   driveline: engine and gearbox-input rotors (revolutes in the body: reaction torque, gyroscopics emerge),
 *   clutch (Coulomb box row), open diff (ratio row), clutch-pack LSD (box row) or viscous coupling (element),
 *   brakes (Coulomb box rows wheel vs knuckle: reaction into the knuckle and through the arms);
 *   tyre: envelope/brush model (dc_tyre.c) applied to the wheel body; the longitudinal force is linearly implicit
 *   in the coupled solve (element along the contact x at the patch).
 * The reduced core (dc_car.c) is untouched and remains selectable (solver = 0).
 */
#include "dc_mbcar.h"
#include <stdlib.h>
#include <string.h>

static const v3 EX = {1, 0, 0}, EY = {0, 1, 0}, EZ = {0, 0, 1};
static qt qid(void) { qt q = {1, 0, 0, 0}; return q; }
static qt qcj(qt q) { q.x = -q.x; q.y = -q.y; q.z = -q.z; return q; }
static m3 mdiag(double a, double b, double c) { m3 M = m3id(); M.m[0][0] = a; M.m[1][1] = b; M.m[2][2] = c; return M; }
static m3 axis_inertia(v3 e, double Ia, double It) {   /* It I + (Ia - It) e e^T */
    m3 M = mdiag(It, It, It); double d = Ia - It, E[3] = {e.x, e.y, e.z};
    for (int r = 0; r < 3; r++) for (int s = 0; s < 3; s++) M.m[r][s] += d * E[r] * E[s];
    return M;
}
static double wrapd(double a) { while (a > M_PI) a -= 2 * M_PI; while (a < -M_PI) a += 2 * M_PI; return a; }
static double lut_eval(const dc_car_params* p, double rpm) {
    if (p->nlut <= 0) return 0;
    if (rpm <= p->lut_rpm[0]) return p->lut_torque[0];
    for (int i = 1; i < p->nlut; i++) if (rpm <= p->lut_rpm[i]) {
        double f = (rpm - p->lut_rpm[i - 1]) / (p->lut_rpm[i] - p->lut_rpm[i - 1]);
        return p->lut_torque[i - 1] + f * (p->lut_torque[i] - p->lut_torque[i - 1]);
    }
    return p->lut_torque[p->nlut - 1];
}
static double damper_force(const dc_kin* k, double vcomp, double* slope) {
    if (vcomp >= 0) {
        if (vcomp <= k->v_bump) { *slope = k->c_bump; return k->c_bump * vcomp; }
        *slope = k->c_fbump; return k->c_bump * k->v_bump + k->c_fbump * (vcomp - k->v_bump);
    }
    double v = -vcomp;
    if (v <= k->v_reb) { *slope = k->c_reb; return -k->c_reb * v; }
    *slope = k->c_freb; return -(k->c_reb * k->v_reb + k->c_freb * (v - k->v_reb));
}
/* rotation angle of body b relative to the chassis about chassis-frame axis e (body design orientation qd) */
static double rel_angle(const mb_world* W, int ch, int b, qt qd, v3 e) {
    qt qr = qmul(qcj(W->B[ch].q), qmul(W->B[b].q, qcj(qd)));
    return 2 * atan2(qr.x * e.x + qr.y * e.y + qr.z * e.z, qr.w);
}
static void row_pt(const mb_world* W, int b, v3 p, v3 u, double s, double o[6]) {
    mb_row_point(&W->B[b], p, u, o); for (int i = 0; i < 6; i++) o[i] *= s;
}
static void row_an(v3 u, double s, double o[6]) { o[0] = o[1] = o[2] = 0; o[3] = s * u.x; o[4] = s * u.y; o[5] = s * u.z; }

static int addb(dc_mbcar* m, const char* nm, double mass, m3 Ib, v3 x, qt q) {
    int b = mb_add_body(m->W, nm, mass, Ib, x, q);
    if (b >= 0) { m->xd[b] = x; m->qd[b] = q; m->nbody = m->W->nb; }
    return b;
}
static v3 loc(const dc_mbcar* m, int b, v3 pw) { const mb_body* B = &m->W->B[b]; return mtv(&B->R, vsub(pw, B->x)); }

/* point P0 carried by the lower arm (or knuckle) of corner i: position at travel s (chassis frame), via dc_kin */
static v3 carried_point(const dc_car* c, int i, int on_arm, v3 P0, double s) {
    const dc_kin* k = &c->K[i]; dc_pose p = c->pose[i], p0 = c->pose[i];
    dc_kin_solve(k, 0, 0, &p0, 0);
    dc_kin_solve(k, s, 0, &p, 0);
    if (!on_arm) { m3 Rt = mtrans(&p0.R); v3 l = mv(&Rt, vsub(P0, p0.c)); return vadd(p.c, mv(&p.R, l)); }
    v3 e = k->arm_axis, A = k->arm_p;
    v3 b0 = vadd(p0.c, mv(&p0.R, k->lu[k->arm_a])), b = vadd(p.c, mv(&p.R, k->lu[k->arm_a]));
    v3 r0 = vsub(b0, A), r1 = vsub(b, A);
    r0 = vsub(r0, vscl(e, vdot(e, r0))); r1 = vsub(r1, vscl(e, vdot(e, r1)));
    double phi = atan2(vdot(e, vcross(r0, r1)), vdot(r0, r1));
    m3 Rot = mexp(vscl(e, phi));
    return vadd(A, mv(&Rot, vsub(P0, A)));
}


/* ------------------------------------------------------------------ hydraulic valve (docs/VALVE.md)
   Open-centre rotary valve = Wheatstone bridge of four variable orifices between the pump supply S, the tank T and
   the cylinder chambers A (pushes the rack +y) and B. Twist th > 0 (valve input ahead of the pinion in the +rack
   sense) closes the edges S->B and A->T and leaves S->A, B->T open, so A rises and B falls. Orifice flow
   Q = cd a sqrt(2|dp|/rho) (turbulent), regularised below ptr; chamber pressures follow the continuity equation
   (V/beta) dP/dt = Q_in - dV/dt with the effective bulk modulus (oil + hose). A relief valve caps the supply. */
void dc_valve_setup(dc_valve* v, const dc_car* c) {
    const dc_car_params* P = &c->P; const dc_mb_params* M = &P->mb;
    v->a0 = M->valve_a0; v->amin = M->valve_amin; v->thc = fmax(M->valve_theta_c, 1e-4); v->cd = M->valve_cd;
    v->rho = M->oil_rho; v->beta = M->oil_beta; v->Vc = M->chamber_vol; v->Vs = M->supply_vol;
    v->Cs = M->supply_vol / M->oil_beta + fmax(M->hose_compliance, 0);   /* supply capacitance: oil + hose expansion, m^3/Pa */
    v->Ap = P->assist_area; v->half = c->rack_hard > 0 ? c->rack_hard : 0.08;
    v->prel = P->assist_pmax; v->Qr = P->assist_flow;
    v->grel = P->assist_flow / (0.05 * P->assist_pmax);   /* relief passes the rated flow at 5 % over its setting */
    v->ktb = P->assist_torsion_k; v->tref = P->assist_tref; v->pmax = P->assist_pmax;
    v->ptr = 1e4;                                          /* laminar/turbulent transition pressure, Pa */
    v->edge = M->valve_model >= 1.5 ? 1 : 0;
}
double dc_valve_area(const dc_valve* v, double th) {
    /* opening edge (th < 0): grows from a0 as the chamfer uncovers, limited by the valve-body ports (Roesth 2007:
       levels off at 20-30 mm^2); closing edge (th > 0): contoured to the design boost law, or a linear chamfer */
    double ao = fmin(v->a0 * (1 + fabs(th) / v->thc), fmax(25e-6, v->a0));
    if (th <= 0) return ao;
    double a;
    if (v->edge == 0) {   /* dP = K (1/a_c^2 - 1/a_o^2) at rated flow, no rack motion (Merritt bridge) = pmax x^4/(1+x^4) */
        double x = v->ktb * th / v->tref, x4 = x * x * x * x, dp = v->pmax * x4 / (1 + x4);
        double q = v->Qr / (2 * v->cd), K = 0.5 * v->rho * q * q;
        a = 1 / sqrt(dp / K + 1 / (ao * ao));
    } else a = v->a0 * (1 - th / v->thc);
    return fmax(a, v->amin);
}
static double orif(const dc_valve* v, double a, double dp, double* dq) {
    double C = v->cd * a * sqrt(2 / v->rho), s = dp * dp + v->ptr * v->ptr, r = sqrt(sqrt(s));
    *dq = C * (0.5 * dp * dp + v->ptr * v->ptr) / (s * r);
    return C * dp / r;
}
static int solve3(double A[3][3], const double b[3], double x[3]) {
    double d = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
             + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    if (fabs(d) < 1e-300) return 0;
    for (int k = 0; k < 3; k++) {
        double B[3][3]; memcpy(B, A, sizeof B);
        for (int i = 0; i < 3; i++) B[i][k] = b[i];
        x[k] = (B[0][0] * (B[1][1] * B[2][2] - B[1][2] * B[2][1]) - B[0][1] * (B[1][0] * B[2][2] - B[1][2] * B[2][0])
              + B[0][2] * (B[1][0] * B[2][1] - B[1][1] * B[2][0])) / d;
    }
    return 1;
}
int dc_valve_solve(const dc_valve* v, const double P0[3], double th, double rd, double r, double Qp, double h, double P[3], double dPdv[3]) {
    const double pvap = -0.95e5;
    double aSA = dc_valve_area(v, -th), aAT = dc_valve_area(v, th), aSB = dc_valve_area(v, th), aBT = dc_valve_area(v, -th);
    double VA = v->Vc + v->Ap * fmax(v->half + r, 0), VB = v->Vc + v->Ap * fmax(v->half - r, 0);
    double CA = h > 0 ? VA / (v->beta * h) : 0, CB = h > 0 ? VB / (v->beta * h) : 0, CS = h > 0 ? v->Cs / h : 0;
    double J[3][3]; int it, ok = -1;
    for (int i = 0; i < 3; i++) P[i] = P0[i];
    for (it = 1; it <= 60; it++) {
        double gSA, gAT, gSB, gBT;
        double qSA = orif(v, aSA, P[2] - P[0], &gSA), qAT = orif(v, aAT, P[0], &gAT);
        double qSB = orif(v, aSB, P[2] - P[1], &gSB), qBT = orif(v, aBT, P[1], &gBT);
        double over = P[2] - v->prel, qrel = over > 0 ? v->grel * over : 0, grl = over > 0 ? v->grel : 0;
        double R[3] = { CA * (P[0] - P0[0]) - qSA + qAT + v->Ap * rd,
                        CB * (P[1] - P0[1]) - qSB + qBT - v->Ap * rd,
                        CS * (P[2] - P0[2]) - Qp + qSA + qSB + qrel };
        J[0][0] = CA + gSA + gAT; J[0][1] = 0;              J[0][2] = -gSA;
        J[1][0] = 0;              J[1][1] = CB + gSB + gBT; J[1][2] = -gSB;
        J[2][0] = -gSA;           J[2][1] = -gSB;           J[2][2] = CS + gSA + gSB + grl;
        double dx[3], nb[3] = {-R[0], -R[1], -R[2]};
        if (!solve3(J, nb, dx)) break;
        double mx = 0;
        for (int i = 0; i < 3; i++) { double lim = 0.25 * fmax(fabs(P[i]), 2e5); if (dx[i] > lim) dx[i] = lim; if (dx[i] < -lim) dx[i] = -lim; P[i] += dx[i]; mx = fmax(mx, fabs(dx[i])); }
        if (mx < 0.05) { ok = it; break; }
    }
    double b[3] = {-v->Ap, v->Ap, 0};
    if (!solve3(J, b, dPdv)) dPdv[0] = dPdv[1] = dPdv[2] = 0;
    for (int i = 0; i < 2; i++) if (P[i] < pvap) { P[i] = pvap; dPdv[i] = 0; }   /* cavitation: the chamber cannot pull below vapour */
    return ok;
}
double dc_valve_energy(const dc_valve* v, const double P[3], double r) {
    double VA = v->Vc + v->Ap * fmax(v->half + r, 0), VB = v->Vc + v->Ap * fmax(v->half - r, 0);
    return 0.5 / v->beta * (VA * P[0] * P[0] + VB * P[1] * P[1]) + 0.5 * v->Cs * P[2] * P[2];
}

static void phi_refresh(void* user);

dc_mbcar* dc_mbcar_create(dc_car* c) {
    const dc_car_params* P = &c->P; const dc_mb_params* M = &P->mb;
    if (P->drive == DC_DRIVE_AWD) return NULL;          /* AWD centre diff not ported yet: reduced core */
    dc_mbcar* m = (dc_mbcar*)calloc(1, sizeof(dc_mbcar));
    if (!m) return NULL;
    m->W = mb_create();
    if (!m->W) { free(m); return NULL; }
    mb_world* W = m->W;
    v3 off = V3(P->sprung_cg[0], P->sprung_cg[1], P->sprung_cg[2]);
    double msum = 0; v3 mom = V3(0, 0, 0);
    #define ACC(b) do { msum += W->B[b].m; mom = vadd(mom, vscl(W->B[b].x, W->B[b].m)); } while (0)
    /* chassis first (placeholder COM at the K origin, shifted to its true COM once the parts are known) */
    m->ch = addb(m, "chassis", 1, mdiag(P->sprung_inertia[0], P->sprung_inertia[1], P->sprung_inertia[2]), V3(0, 0, 0), qid());
    W->B[m->ch].root = 1;
    for (int i = 0; i < 4; i++) for (int l = 0; l < DC_MAX_LINKS; l++) m->j_link[i][l] = -1;
    m->j_pfric = -1; for (int i = 0; i < 4; i++) { m->j_stf[i] = m->j_tmf[i] = -1; for (int n = 0; n < 3; n++) m->j_bjf[i][n] = -1; }
    const int jf = M->jf > 0;
    /* ---------------- steering chain (built first so tie rods can attach to the rack) */
    const dc_kin* K0 = &c->K[0];
    m->steer = K0->steered && K0->rack_link >= 0;
    if (m->steer) {
        v3 rl = K0->lc[K0->rack_link];
        v3 rp = V3(rl.x, 0, rl.z);
        double mr = c->m_rack;
        m->rack = addb(m, "rack", mr, mdiag(mr * 0.04, 1e-3, mr * 0.04), rp, qid()); ACC(m->rack);
        v3 pp = V3(rp.x, M->pinion_y, rp.z);
        v3 u1 = vsub(V3(M->col_u1[0], M->col_u1[1], M->col_u1[2]), off);
        v3 u2 = vsub(V3(M->col_u2[0], M->col_u2[1], M->col_u2[2]), off);
        v3 rim = vsub(V3(M->col_rim[0], M->col_rim[1], M->col_rim[2]), off);
        m->e_in = vnorm(vsub(u2, pp)); m->e_is = vnorm(vsub(u1, u2)); m->e_rim = vnorm(vsub(rim, u1));
        m->pin = addb(m, "pinion", 0.4, axis_inertia(m->e_in, M->pinion_inertia, 2e-4), pp, qid()); ACC(m->pin);
        m->inp = addb(m, "input", 0.3, axis_inertia(m->e_in, M->input_inertia, 2e-4), vmad(pp, m->e_in, 0.05), qid()); ACC(m->inp);
        double Lis = vlen(vsub(u1, u2));
        m->ish = addb(m, "ishaft", 0.8, axis_inertia(m->e_is, M->ishaft_inertia, 0.8 * Lis * Lis / 12), vscl(vadd(u1, u2), 0.5), qid()); ACC(m->ish);
        m->rim = addb(m, "rim", 3.0, axis_inertia(m->e_rim, M->rim_inertia + P->column_inertia, 0.03), rim, qid()); ACC(m->rim);
        /* rack housing: rigid part of the chassis unless mb_rack_mount_k > 0, then its own body on a beam joint that is rigid
           except along the rack axis (rubber clamp bushings: lateral housing shift = steering-system compliance) */
        m->hsg = m->ch; m->j_rmount = -1;
        if (M->rack_mount_k > 0) {
            m->hsg = addb(m, "rack_housing", 3.0, mdiag(0.15, 2e-3, 0.15), rp, qid()); ACC(m->hsg);
            m->j_rmount = mb_beam(W, "rack_mount", m->hsg, m->ch, rp, EY, EX);
            W->Jt[m->j_rmount].sk[0] = M->rack_mount_k; W->Jt[m->j_rmount].sc[0] = M->rack_mount_c > 0 ? M->rack_mount_c : 0;
        }
        m->j_rack = mb_prismatic(W, "rack_hsg", m->rack, m->hsg, EY);
        m->j_pin = mb_revolute(W, "pinion_brg", m->pin, m->hsg, pp, m->e_in);
        m->j_inp = mb_revolute(W, "input_brg", m->inp, m->hsg, pp, m->e_in);
        m->j_rim = mb_revolute(W, "column_brg", m->rim, m->ch, rim, m->e_rim);
        m->j_u1 = mb_ujoint(W, "ujoint_up", m->rim, m->ish, u1, m->e_rim, m->e_is, 0);
        m->j_u2 = mb_ujoint(W, "ujoint_lo", m->ish, m->inp, u2, m->e_is, m->e_in, 1);
        int gb[3] = {m->rack, m->pin, m->hsg};
        m->j_gear = mb_custom(W, "rack_pinion", 1, 0, 3, gb); W->Jt[m->j_gear].proj = 1;
        int fb[2] = {m->rack, m->hsg}; m->j_rfric = mb_custom(W, "rack_fric", 1, 1, 2, fb);
        m->j_rstop = mb_custom(W, "rack_stop", 1, 1, 2, fb);
        int cb[2] = {m->rim, m->ch}; m->j_cfric = mb_custom(W, "col_fric", 1, 1, 2, cb);
        if (jf) {   /* load-dependent bounds (docs/JOINT_FRICTION.md); source order is relied on in the step */
            mb_fric_source(W, m->j_rfric, m->j_gear, 0, 1, M->yoke_mu * M->mesh_sep, V3(0, 0, 0));       /* yoke: separating force past the spring preload */
            mb_fric_source(W, m->j_rfric, m->j_rack, 0, 2, M->rack_bush_mu, V3(0, 0, 0));                 /* guide bushes: side load */
            mb_fric_source(W, m->j_rfric, m->j_rack, 2, 3, 2 * M->rack_bush_mu / M->rack_bush_span, EY);  /* misalignment moment -> edge loads */
            mb_fric_source(W, m->j_cfric, m->j_rim, 0, 3, M->brg_mu * M->col_brg_radius, V3(0, 0, 0));
            mb_fric_source(W, m->j_cfric, m->j_rim, 3, 2, 2 * M->brg_mu * M->col_brg_radius / M->col_brg_span, V3(0, 0, 0));
            int pb_[2] = {m->pin, m->hsg}; m->j_pfric = mb_custom(W, "pin_fric", 1, 1, 2, pb_);
            mb_fric_source(W, m->j_pfric, m->j_pin, 0, 3, M->brg_mu * M->pin_brg_radius, V3(0, 0, 0));
            mb_fric_source(W, m->j_pfric, m->j_pin, 3, 2, 2 * M->brg_mu * M->pin_brg_radius / M->pin_brg_span, V3(0, 0, 0));
        }
        mb_couple(W, m->inp, m->pin); mb_couple(W, m->rim, m->ch); mb_couple(W, m->rack, m->hsg);
        if (m->hsg != m->ch) { mb_couple(W, m->hsg, m->ch); mb_couple(W, m->rack, m->ch); }
    }
    /* ---------------- corners */
    for (int i = 0; i < 4; i++) {
        const dc_kin* k = &c->K[i]; int a = i / 2;
        dc_pose p0 = c->pose[i]; dc_kin_solve(k, 0, 0, &p0, 1);
        qt qk = mtoq(&p0.R);
        m->kn[i] = addb(m, "knuckle", M->knuckle_mass[a], mdiag(M->knuckle_inertia[a][0], M->knuckle_inertia[a][1], M->knuckle_inertia[a][2]), p0.c, qk); ACC(m->kn[i]);
        m->wh[i] = addb(m, "wheel", M->wheel_mass[a], axis_inertia(k->spin_axis_u, P->axle[a].tyre.spin_inertia, M->wheel_inertia_t[a]), p0.c, qk); ACC(m->wh[i]);
        m->j_hub[i] = mb_revolute(W, "hub_brg", m->wh[i], m->kn[i], p0.c, mv(&p0.R, k->spin_axis_u));
        int used[DC_MAX_LINKS] = {0};
        m->narm[i] = 0; m->lo_body[i] = m->kn[i];
        for (int l = 0; l < k->nl; l++) {
            if (used[l] || l == k->rack_link) continue;
            for (int q = l + 1; q < k->nl; q++) {
                if (used[q] || q == k->rack_link || vlen(vsub(k->lu0[l], k->lu0[q])) > 1e-6) continue;
                if (m->narm[i] >= 3) break;
                used[l] = used[q] = 1;
                v3 bj = vadd(p0.c, mv(&p0.R, k->lu[l]));
                int lower = (l == k->arm_a && q == k->arm_b) || (l == k->arm_b && q == k->arm_a);
                double ma = lower ? M->lca_mass[a] : M->uca_mass[a];
                v3 pts[3] = {k->lc[l], k->lc[q], bj}, cg = vscl(vadd(vadd(pts[0], pts[1]), pts[2]), 1.0 / 3);
                m3 Ib = mdiag(1e-4, 1e-4, 1e-4);
                for (int t = 0; t < 3; t++) {
                    v3 r = vsub(pts[t], cg); double R2 = vdot(r, r), Rv[3] = {r.x, r.y, r.z};
                    for (int u = 0; u < 3; u++) for (int w = 0; w < 3; w++) Ib.m[u][w] += ma / 3 * ((u == w ? R2 : 0) - Rv[u] * Rv[w]);
                }
                int n = m->narm[i]++;
                m->arm[i][n] = addb(m, lower ? "lca" : "uca", ma, Ib, cg, qid()); ACC(m->arm[i][n]);
                m->arm_l[i][n][0] = l; m->arm_l[i][n][1] = q;
                v3 pmid = vscl(vadd(pts[0], pts[1]), 0.5), pax = vsub(pts[1], pts[0]);
                if (M->bush_kr[a] > 0 && M->bush_ka[a] > 0 && M->bush_kt[a] > 0) {
                    /* two rubber bushes at the inner pivots, lumped at their midpoint as one compliant row set (docs/BUSHINGS.md):
                       translation 2 kr / 2 ka; tilt 2 kt + 2 kr (L/2)^2 (the pair resists tilt by opposite radial deflection);
                       rotation about the pivot axis free (torsion carried by the spring). Damping c = loss * k / (2 pi 10 Hz). */
                    double L = vlen(pax), kr = 2 * M->bush_kr[a], ka = 2 * M->bush_ka[a], kc = 2 * M->bush_kt[a] + 0.5 * M->bush_kr[a] * L * L;
                    int j = m->j_arm_rev[i][n] = mb_bushing(W, lower ? "lca_bush" : "uca_bush", m->arm[i][n], m->ch, pmid, pax);
                    double kk[5] = {ka, kr, kr, kc, kc}, cf = fmax(M->bush_damp, 0) / (2 * M_PI * 10);
                    for (int q = 0; q < 5; q++) { W->Jt[j].sk[q] = kk[q]; W->Jt[j].sc[q] = cf * kk[q]; }
                    m->nbush++;
                } else
                    m->j_arm_rev[i][n] = mb_revolute(W, lower ? "lca_pivot" : "uca_pivot", m->arm[i][n], m->ch, pmid, pax);
                m->j_arm_bj[i][n] = mb_spherical(W, lower ? "lca_bj" : "uca_bj", m->arm[i][n], m->kn[i], bj);
                if (jf) {   /* ball-in-socket friction torque <= T0 + mu r |F|, isotropic (all three relative rotations) */
                    int fb2[2] = {m->arm[i][n], m->kn[i]};
                    m->j_bjf[i][n] = mb_custom(W, "bj_fric", 3, 1, 2, fb2); W->Jt[m->j_bjf[i][n]].cone = 1;
                    mb_fric_source(W, m->j_bjf[i][n], m->j_arm_bj[i][n], 0, 3, M->bj_mu * M->bj_radius, V3(0, 0, 0));
                }
                if (lower && !k->spr_on_upright) m->lo_body[i] = m->arm[i][n];
                break;
            }
        }
        for (int l = 0; l < k->nl; l++) {
            if (used[l]) continue;
            v3 pu = vadd(p0.c, mv(&p0.R, k->lu[l]));
            int other = (l == k->rack_link && k->steered && m->steer) ? m->rack : m->ch;
            m->j_link[i][l] = mb_distance(W, l == k->rack_link ? "tierod" : "link", m->kn[i], pu, other, k->lc[l]);
        }
        if (k->type == DC_SUSP_STRUT) {
            v3 S = vadd(p0.c, mv(&p0.R, k->strut_S));
            m->j_strut[i] = mb_pointline(W, "strut_top", m->kn[i], S, mv(&p0.R, k->strut_d), m->ch, k->strut_top);
            if (jf) {   /* rod guide + piston sliding under the strut side load; top-mount bearing under the spring load */
                int sb[2] = {m->kn[i], m->ch};
                m->j_stf[i] = mb_custom(W, "strut_fric", 1, 1, 2, sb);
                mb_fric_source(W, m->j_stf[i], m->j_strut[i], 0, 2, M->strut_mu * M->strut_lever, V3(0, 0, 0));
                m->j_tmf[i] = mb_custom(W, "topmount_fric", 1, 1, 2, sb);
            }
        } else m->j_strut[i] = -1;
        m->lo_local[i] = loc(m, m->lo_body[i], p0.lo);
        mb_couple(W, m->lo_body[i], m->ch);
        /* stop engagement lengths along the spring line */
        dc_pose pu = p0, pd = p0;
        dc_kin_solve(k, c->s_up[i], 0, &pu, 1); dc_kin_solve(k, -c->s_dn[i], 0, &pd, 1);
        m->L_up[i] = pu.Ls; m->MR_up[i] = fmax(-pu.dL[0], 0.05);
        m->L_dn[i] = pd.Ls; m->MR_dn[i] = fmax(-pd.dL[0], 0.05);
        int bb[2] = {m->wh[i], m->kn[i]};
        m->j_brake[i] = mb_custom(W, "brake", 1, 1, 2, bb);
        /* bump-stop solid height: unilateral rigid row along the spring line (lower spring seat body vs chassis) */
        dc_pose ph = p0; dc_kin_solve(k, fmin(c->s_hard[i], 1.0), 0, &ph, 1); m->L_hard[i] = c->s_hard[i] < 1.0 ? ph.Ls : -1;
        { int hb[2] = {m->lo_body[i], m->ch}; m->j_hard[i] = mb_custom(W, "solid_stop", 1, 1, 2, hb); }
    }
    /* ---------------- anti-roll bars */
    for (int a = 0; a < 2; a++) {
        m->has_bar[a] = P->axle[a].arb > 0;
        if (!m->has_bar[a]) continue;
        double Gs = 0;
        for (int sd = 0; sd < 2; sd++) {
            int i = 2 * a + sd; const dc_kin* k = &c->K[i];
            int body = m->lo_body[i], on_arm = body != m->kn[i];
            if (!on_arm) {   /* no lower arm body: look for any arm, else the knuckle */
                for (int n = 0; n < m->narm[i]; n++) if ((m->arm_l[i][n][0] == k->arm_a) || (m->arm_l[i][n][1] == k->arm_a)) { body = m->arm[i][n]; on_arm = 1; }
            }
            v3 P0, bj = V3(0, 0, 0);
            if (on_arm) {
                bj = vadd(c->pose[i].c, mv(&c->pose[i].R, k->lu[k->arm_a]));
                v3 pin = vscl(vadd(k->lc[k->arm_a], k->lc[k->arm_b]), 0.5);
                P0 = vadd(pin, vscl(vsub(bj, pin), M->arb_link_frac[a]));
            } else P0 = vadd(c->pose[i].c, V3(0, 0, -0.08));
            double zb = (on_arm ? k->arm_p.z : P0.z) + M->arb_bar_dz[a];
            if (fabs(zb - P0.z) < 0.05) zb = P0.z + 0.05;
            double arm = fmax(M->arb_arm[a], 0.05);
            v3 E0 = V3(P0.x, P0.y, zb), C = V3(P0.x + (a == 0 ? arm : -arm), P0.y, zb);
            m->bar[a][sd] = addb(m, "arb_half", 1.2, mdiag(0.01, 0.002, 0.01), C, qid()); ACC(m->bar[a][sd]);
            m->j_bar[a][sd] = mb_revolute(W, "arb_bush", m->bar[a][sd], m->ch, C, EY);
            m->j_drop[a][sd] = mb_distance(W, "droplink", body, P0, m->bar[a][sd], E0);
            double e = 1e-4;
            double dz = (carried_point(c, i, on_arm, P0, e).z - carried_point(c, i, on_arm, P0, -e).z) / (2 * e);
            Gs += 0.5 * dz / arm;
        }
        m->G_bar[a] = Gs;
        m->k_bar[a] = fabs(Gs) > 1e-6 ? P->axle[a].arb / (Gs * Gs) : 0;
        mb_couple(W, m->bar[a][0], m->bar[a][1]);
    }
    /* ---------------- driveline rotors (axis = chassis x), mass 1 kg each */
    v3 ep = V3(c->K[0].hub0.x - 0.3, 0, c->K[0].hub0.z);
    m->eng = addb(m, "engine_rot", 1.0, mdiag(P->engine_inertia, 0.01, 0.01), ep, qid()); ACC(m->eng);
    m->gi = addb(m, "gbox_in", 1.0, mdiag(fmax(P->gearbox_inertia, 1e-4), 0.01, 0.01), vadd(ep, V3(-0.5, 0, 0)), qid()); ACC(m->gi);
    m->j_eng = mb_revolute(W, "crank_brg", m->eng, m->ch, ep, EX);
    m->j_gi = mb_revolute(W, "gbox_brg", m->gi, m->ch, vadd(ep, V3(-0.5, 0, 0)), EX);
    { int b[2] = {m->eng, m->gi}; m->j_clutch = mb_custom(W, "clutch", 1, 1, 2, b); }
    m->drv[0] = P->drive == DC_DRIVE_FWD ? 0 : 2; m->drv[1] = m->drv[0] + 1;
    /* carrier inertia (opt-in, docs/CARRIER.md): the gearbox output + propshaft and the diff carrier + ring gear become rotors
       on chassis bearings, joined by three ratio rows in series (gearbox g, final drive fd, open-diff mean) instead of one
       overall-ratio row; their bearing and mesh reactions then reach the chassis through the solve */
    m->carrier = M->diff_inertia > 0 || M->gb_out_inertia > 0; m->go = m->dcr = m->j_go = m->j_dcr = m->j_gbox = m->j_fd = -1;
    if (m->carrier) {
        const dc_kin* Kd = &c->K[m->drv[0]];
        v3 pg = vadd(ep, V3(-1.0, 0, 0)), pc = V3(Kd->hub0.x, 0, Kd->hub0.z);
        m->go = addb(m, "gbox_out", 1.0, mdiag(fmax(M->gb_out_inertia, 1e-4), 0.01, 0.01), pg, qid()); ACC(m->go);
        m->dcr = addb(m, "diff_carrier", 1.0, mdiag(0.01, fmax(M->diff_inertia, 1e-4), 0.01), pc, qid()); ACC(m->dcr);
        m->j_go = mb_revolute(W, "gbox_out_brg", m->go, m->ch, pg, EX);
        m->j_dcr = mb_revolute(W, "carrier_brg", m->dcr, m->ch, pc, EY);
        int bg[3] = {m->gi, m->go, m->ch}; m->j_gbox = mb_custom(W, "gearbox", 1, 0, 3, bg);
        int bf[3] = {m->go, m->dcr, m->ch}; m->j_fd = mb_custom(W, "final_drive", 1, 0, 3, bf);
    }
    {
        int wl = m->wh[m->drv[0]], wr = m->wh[m->drv[1]];
        int b4[4] = {m->carrier ? m->dcr : m->gi, wl, wr, m->ch}; m->j_diff = mb_custom(W, "diff", 1, 0, 4, b4);
        int b3[3] = {wl, wr, m->ch}; m->j_lsd = mb_custom(W, "lsd", 1, 1, 3, b3);
        mb_couple(W, wl, wr); mb_couple(W, wl, m->ch); mb_couple(W, wr, m->ch);
    }
    /* ---------------- chassis mass and COM: total mass and total CG as configured */
    double mch = P->mass - msum;
    if (mch < 0.3 * P->mass) { mb_destroy(W); free(m); return NULL; }
    v3 Kcg = vneg(off);
    m->cgc = vscl(vsub(vscl(Kcg, P->mass), mom), 1.0 / mch);
    W->B[m->ch].m = mch; W->B[m->ch].x = m->cgc; m->xd[m->ch] = m->cgc;
    for (int j = 0; j < W->nj; j++) {
        mb_joint* J = &W->Jt[j]; if (J->type == MBJ_CUSTOM) continue;
        if (J->b[0] == m->ch) J->pa = vsub(J->pa, m->cgc);
        if (J->b[1] == m->ch) J->pb = vsub(J->pb, m->cgc);
    }
    /* every force element spanning two bodies needs its off-diagonal stiffness block declared,
       otherwise the implicit k/c coupling is silently dropped from the system matrix */
    for (int i = 0; i < 4; i++) mb_couple(W, m->lo_body[i], m->ch);
    for (int a = 0; a < 2; a++) if (m->has_bar[a]) mb_couple(W, m->bar[a][0], m->bar[a][1]);
    if (m->steer) { mb_couple(W, m->inp, m->pin); mb_couple(W, m->rack, m->hsg); mb_couple(W, m->rim, m->ch);
                    if (m->hsg != m->ch) { mb_couple(W, m->hsg, m->ch); mb_couple(W, m->rack, m->ch); } }
    m->vmodel = (m->steer && P->assist && M->valve_model > 0.5) ? (M->valve_model >= 1.5 ? 2 : 1) : 0;
    if (m->vmodel) dc_valve_setup(&m->vlv, c);
    W->phi_cb = phi_refresh; W->cb_user = c;
    if (!mb_finalize(W)) { mb_destroy(W); free(m); return NULL; }
    return m;
    #undef ACC
}

void dc_mbcar_destroy(dc_mbcar* m) { if (!m) return; mb_destroy(m->W); free(m); }

void dc_mbcar_reset(dc_car* c) {
    dc_mbcar* m = c->mb; if (!m) return;
    mb_world* W = m->W;
    m3 R = qtom(c->q);
    for (int b = 0; b < W->nb; b++) {
        mb_body* B = &W->B[b];
        B->x = vadd(c->x, mv(&R, m->xd[b]));
        B->q = qnorm(qmul(c->q, m->qd[b])); B->R = qtom(B->q);
        B->v = c->v; B->w = c->w;
    }
    for (int i = 0; i < 4; i++) {
        mb_body* B = &W->B[m->wh[i]];
        v3 a = mv(&B->R, c->K[i].spin_axis_u);
        B->w = vadd(c->w, vscl(a, c->ty[i].omega));
    }
    v3 xw = mv(&R, EX);
    W->B[m->eng].w = vadd(c->w, vscl(xw, c->we));
    W->B[m->gi].w = vadd(c->w, vscl(xw, c->wg));
    if (m->carrier) {   /* carrier and propshaft geared to the driven wheels' mean speed */
        int l = m->wh[m->drv[0]], r = m->wh[m->drv[1]];
        v3 aL = mv(&W->B[l].R, c->K[m->drv[0]].spin_axis_u), aR = mv(&W->B[r].R, c->K[m->drv[1]].spin_axis_u), yw = mv(&R, EY);
        double wc = 0.5 * (vdot(vsub(W->B[l].w, c->w), aL) + vdot(vsub(W->B[r].w, c->w), aR));
        v3 ac = vdot(vadd(aL, aR), yw) >= 0 ? yw : vneg(yw);
        W->B[m->dcr].w = vadd(c->w, vscl(ac, wc));
        W->B[m->go].w = vadd(c->w, vscl(xw, c->P.final_drive * wc));
    }
    for (int j = 0; j < W->nj; j++) memset(W->Jt[j].lam, 0, sizeof W->Jt[j].lam);
    m->th_prev_raw = m->pin_prev_raw = m->inp_prev_raw = 0;
    m->th_acc = m->pin_acc = m->inp_acc = 0;
    m->lam_diff_prev = 0; m->diss = 0; m->audit_init = 0;
    m->vp[0] = m->vp[1] = m->vp[2] = 0; m->vQp = 0; m->v_fail = 0;
    if (m->vmodel) {   /* steady centred bridge at the current pump flow */
        const dc_car_params* P = &c->P; double z[3] = {0, 0, 0}, d[3];
        m->vQp = P->assist_flow * clampd(fabs(c->we * 30 / M_PI) / P->assist_rpm_full, 0, 1);
        dc_valve_solve(&m->vlv, z, 0, 0, c->r, m->vQp, 0, m->vp, d);
    }
    memset(m->extF, 0, sizeof m->extF); memset(m->extT, 0, sizeof m->extT);
}

/* steering angles (unwrapped, relative to the chassis); commit = 1 stores them as the new reference */
static void steer_angles(dc_car* c, double* th, double* thp, double* thi, double* r, int commit) {
    dc_mbcar* m = c->mb; mb_world* W = m->W;
    *th = *thp = *thi = *r = 0;
    if (!m->steer) return;
    double a1 = rel_angle(W, m->ch, m->rim, m->qd[m->rim], m->e_rim);
    double a2 = rel_angle(W, m->hsg, m->pin, m->qd[m->pin], m->e_in);
    double a3 = rel_angle(W, m->hsg, m->inp, m->qd[m->inp], m->e_in);
    *th = m->th_acc + wrapd(a1 - m->th_prev_raw);
    *thp = m->pin_acc + wrapd(a2 - m->pin_prev_raw);
    *thi = m->inp_acc + wrapd(a3 - m->inp_prev_raw);
    const mb_body* Ch = &W->B[m->ch];
    v3 d = mtv(&Ch->R, vsub(W->B[m->rack].x, Ch->x));
    *r = vadd(d, m->cgc).y - m->xd[m->rack].y;
    if (m->hsg != m->ch) {   /* rack travel in its housing = rack shift minus housing shift (both in the chassis frame) */
        v3 dh = mtv(&Ch->R, vsub(W->B[m->hsg].x, Ch->x));
        *r -= vadd(dh, m->cgc).y - m->xd[m->hsg].y;
    }
    if (commit) { m->th_acc = *th; m->pin_acc = *thp; m->inp_acc = *thi; m->th_prev_raw = a1; m->pin_prev_raw = a2; m->inp_prev_raw = a3; }
}
static void phi_refresh(void* user) {
    dc_car* c = (dc_car*)user; dc_mbcar* m = c->mb;
    if (!m->steer) return;
    double th, thp, thi, r;
    steer_angles(c, &th, &thp, &thi, &r, 0);
    m->W->Jt[m->j_gear].phi[0] = r - c->r_p * thp;
}

/* ------------------------------------------------------------------ one substep */
void dc_mbcar_step(dc_car* c) {
    dc_mbcar* m = c->mb; mb_world* W = m->W;
    const dc_car_params* P = &c->P; const dc_mb_params* M = &P->mb;
    const double h = c->h, g = P->gravity;
    int nodamp = c->dbg & DC_DBG_NO_DAMPING;
    mb_begin(W, h);
    mb_body* Ch = &W->B[m->ch];
    const m3 R = Ch->R;
    v3 xw = mv(&R, EX), yw = mv(&R, EY), zw = mv(&R, EZ);
    double E0 = mb_kinetic(W);
    /* ---- 1. tyres, phase A (wheel body state; knuckle = non-spinning carrier) */
    v3 a_w[4], cw[4];
    for (int i = 0; i < 4; i++) {
        mb_body* Wb = &W->B[m->wh[i]];
        a_w[i] = mv(&Wb->R, c->K[i].spin_axis_u); cw[i] = Wb->x;
        c->ty[i].omega = vdot(Wb->w, a_w[i]);
        dc_tyre_phaseA(c, i, cw[i], a_w[i], Wb->v, W->B[m->kn[i]].w, h, &m->tc[i]);
    }
    /* ---- 2. engine torque (identical logic to the reduced core) */
    double we = vdot(W->B[m->eng].w, xw), wx = vdot(Ch->w, xw);
    double rpm = (we - wx) * 30 / M_PI;
    double thr = clampd(c->in.throttle, 0, 1);
    if (rpm > P->limiter_rpm) thr = 0;
    int crank = (c->in.clutch > 0.5 || c->in.gear == 0) && rpm < 0.6 * P->idle_rpm;
    if (c->engine_on && rpm < 250 && !crank) c->engine_on = 0;
    if (!c->engine_on && rpm > 350) c->engine_on = 1;
    if (c->dbg & DC_DBG_ENGINE_OFF) { c->engine_on = 0; crank = 0; }
    double thr_idle = clampd((P->idle_rpm + 50 - rpm) / 200, 0, 1) * 0.6;
    if (thr_idle > thr) thr = thr_idle;
    if (!c->engine_on) thr = 0;
    double Te = lut_eval(P, fmax(rpm, 0)) * thr - P->coast_torque * clampd(rpm / P->coast_rpm, -1, 2) * (1 - thr);
    if (c->dbg & DC_DBG_ENGINE_OFF) Te = 0;
    if (crank) Te += 90;
    /* ---- 3. external loads: gravity on every body, aero + body contacts on the chassis, test hooks */
    v3 Fext = V3(0, 0, 0), Text = V3(0, 0, 0);
    for (int b = 0; b < W->nb; b++) {
        mb_body* B = &W->B[b];
        B->F = vadd(V3(0, 0, -B->m * g), m->extF[b]); B->T = m->extT[b];
        Fext = vadd(Fext, B->F); Text = vadd(Text, vadd(vcross(B->x, B->F), B->T));
    }
    v3 Kp = vsub(Ch->x, mv(&R, m->cgc));                 /* K-frame origin (old sprung CG) in world */
    double Win = 0;                                      /* external work this step (energy audit) */
    v3 F_chx = V3(0, 0, 0), T_chx = V3(0, 0, 0);         /* aero + body-contact wrench on the chassis (about its CG) */
    {
        v3 F = V3(0, 0, 0), T = V3(0, 0, 0);
        if (!(c->dbg & DC_DBG_NO_AERO)) {
            double rho = 1.2, sp = vlen(Ch->v);
            v3 Fd = vscl(Ch->v, -0.5 * rho * P->cda * sp);
            double vx = vdot(Ch->v, xw), q = 0.5 * rho * vx * vx;
            v3 pf = vadd(Kp, mv(&R, V3(P->aero_front_x, 0, P->aero_z)));   /* aero points: K frame (as the reduced core) */
            v3 pr = vadd(Kp, mv(&R, V3(P->aero_rear_x, 0, P->aero_z)));
            v3 pd = vadd(Kp, mv(&R, V3(0, 0, P->aero_z)));
            v3 Ff = vscl(zw, -q * P->cla_front), Fr = vscl(zw, -q * P->cla_rear);
            F = vadd(Fd, vadd(Ff, Fr));
            T = vadd(vcross(vsub(pd, Ch->x), Fd), vadd(vcross(vsub(pf, Ch->x), Ff), vcross(vsub(pr, Ch->x), Fr)));
        }
        F_chx = F; T_chx = T;   /* aero only in the external-work term: the ground is stationary, so body-contact work
                                   is internal and its crush/scrape loss appears in the audited residual (dissipation) */
        m->pe_body = 0;
        for (int k = 0; k < P->nbodypts; k++) {
            v3 X = vadd(Kp, mv(&R, vsub(V3(P->bodypt[k][0], P->bodypt[k][1], P->bodypt[k][2]), V3(P->sprung_cg[0], P->sprung_cg[1], P->sprung_cg[2]))));
            v3 n; double gr; double pen = dc_body_depth(c, X, &n, &gr); if (pen <= 0) continue;
            v3 vp = mb_point_vel(Ch, X); double vn = vdot(vp, n);
            double pe_k, wd_, Nd_; double N = dc_body_normal(c, k, pen, vn, nodamp, &pe_k, &wd_, &Nd_); (void)wd_; (void)Nd_; m->pe_body += pe_k; if (N <= 0) continue;   /* MB: dissipation is the audited energy residual */
            v3 vt = vsub(vp, vscl(n, vn)); double vtl = vlen(vt);
            v3 Fc = vscl(n, N);
            if (vtl > 1e-9 && !(c->dbg & DC_DBG_NO_FRICTION)) Fc = vsub(Fc, vscl(vt, P->body_mu * gr * N * tanh(vtl / 0.05) / vtl));
            F = vadd(F, Fc); T = vadd(T, vcross(vsub(X, Ch->x), Fc));
        }
        Ch->F = vadd(Ch->F, F); Ch->T = vadd(Ch->T, T);
        Fext = vadd(Fext, F); Text = vadd(Text, vadd(vcross(Ch->x, F), T));
    }
    /* engine: combustion torque on the crank, reaction on the block (chassis) */
    W->B[m->eng].T = vadd(W->B[m->eng].T, vscl(xw, Te));
    Ch->T = vadd(Ch->T, vscl(xw, -Te));
    /* tyres: normal + lateral forces and all moments explicit on the wheel body (Fx below as an implicit element) */
    for (int i = 0; i < 4; i++) {
        dc_tc* t = &m->tc[i]; mb_body* Wb = &W->B[m->wh[i]];
        if (!t->contact) continue;
        v3 F = vscl(t->yc, t->Fy);
        for (int r = 0; r < t->nrows; r++) if (t->N[r] > 0) F = vadd(F, vscl(t->nr[r], t->N[r]));
        v3 T = vadd(t->tau, vscl(t->n, -t->ey_new * t->Fx_star));
        Wb->F = vadd(Wb->F, F); Wb->T = vadd(Wb->T, T);
        m->T_tyre[i] = T;                                /* pure contact moments: external torque (Newton audit) */
    }
    /* ---- 4. constraint rows that change every step */
    double th, thp, thi, r;
    steer_angles(c, &th, &thp, &thi, &r, 1);
    if (m->steer) {
        mb_joint* J = &W->Jt[m->j_gear]; memset(J->J, 0, sizeof J->J);
        v3 xr = W->B[m->rack].x, ein = mv(&R, m->e_in);
        J->J[0][0][0] = yw.x; J->J[0][0][1] = yw.y; J->J[0][0][2] = yw.z;
        row_an(ein, -c->r_p, J->J[0][1]);
        double o[6]; row_pt(W, m->hsg, xr, yw, -1, o); for (int q = 0; q < 6; q++) J->J[0][2][q] = o[q];
        J->J[0][2][3] += c->r_p * ein.x; J->J[0][2][4] += c->r_p * ein.y; J->J[0][2][5] += c->r_p * ein.z;
        J->phi[0] = r - c->r_p * thp; J->t[0] = 0;
        /* Coulomb friction: rack in its housing, column in its bearings (exact stick/slip box rows) */
        mb_joint* F = &W->Jt[m->j_rfric]; memset(F->J, 0, sizeof F->J);
        F->J[0][0][0] = yw.x; F->J[0][0][1] = yw.y; F->J[0][0][2] = yw.z; row_pt(W, m->hsg, xr, yw, -1, F->J[0][1]);
        F->lo[0] = nodamp ? 0 : -P->rack_friction * h; F->hi[0] = -F->lo[0]; F->t[0] = 0;
        /* rack end stop solid (steel on steel): rigid inelastic unilateral row, speculative */
        mb_joint* Rs = &W->Jt[m->j_rstop]; memcpy(Rs->J, F->J, sizeof Rs->J);
        double rgap = c->rack_hard - fabs(r);
        if (P->rack_stop_solid > 0 && rgap < 0.01) {
            double tt = rgap > 0 ? rgap / h : 0;
            if (r >= 0) { Rs->t[0] = tt; Rs->lo[0] = -1e12; Rs->hi[0] = 0; } else { Rs->t[0] = -tt; Rs->lo[0] = 0; Rs->hi[0] = 1e12; }
        } else { Rs->lo[0] = Rs->hi[0] = 0; Rs->t[0] = 0; }
        mb_joint* G = &W->Jt[m->j_cfric]; memset(G->J, 0, sizeof G->J);
        v3 er = mv(&R, m->e_rim); row_an(er, 1, G->J[0][0]); row_an(er, -1, G->J[0][1]);
        G->lo[0] = nodamp ? 0 : -P->column_friction * h; G->hi[0] = -G->lo[0]; G->t[0] = 0;
        if (F->nsrc > 0) {   /* load-dependent: seals (rack_friction) + yoke preload, then normals from the coupled solve */
            F->active = !nodamp; F->pre = (P->rack_friction + M->yoke_mu * M->yoke_preload) * h;
            F->src[0].d = M->yoke_preload / M->mesh_sep * h; F->src[2].ax = yw;
            G->active = !nodamp; G->pre = P->column_friction * h;
        }
        if (m->j_pfric >= 0) {
            mb_joint* Pf = &W->Jt[m->j_pfric]; memset(Pf->J, 0, sizeof Pf->J);
            row_an(ein, 1, Pf->J[0][0]); row_an(ein, -1, Pf->J[0][1]); Pf->t[0] = 0;
            Pf->active = !nodamp; Pf->pre = M->pin_torque * h;
        }
    }
    for (int i = 0; i < 4; i++) for (int n = 0; n < m->narm[i]; n++) {
        if (m->j_bjf[i][n] < 0) continue;
        mb_joint* Bf = &W->Jt[m->j_bjf[i][n]]; memset(Bf->J, 0, sizeof Bf->J);
        for (int q = 0; q < 3; q++) { v3 e = V3(q == 0, q == 1, q == 2); row_an(e, 1, Bf->J[q][0]); row_an(e, -1, Bf->J[q][1]); Bf->t[q] = 0; }
        Bf->active = !nodamp; Bf->pre = M->bj_torque * h;
    }
    int gear = c->in.gear; double Gr = 0;
    if (gear > 0 && gear <= P->ngears) Gr = P->gear[gear - 1] * P->final_drive;
    else if (gear < 0) Gr = P->gear_reverse * P->final_drive;
    int wl = m->wh[m->drv[0]], wr = m->wh[m->drv[1]];
    v3 aL = a_w[m->drv[0]], aR = a_w[m->drv[1]];
    {
        mb_joint* J = &W->Jt[m->j_clutch]; memset(J->J, 0, sizeof J->J);
        row_an(xw, 1, J->J[0][0]); row_an(xw, -1, J->J[0][1]);
        double cap = P->clutch_torque * clampd(1 - c->in.clutch, 0, 1) * h;
        J->lo[0] = -cap; J->hi[0] = cap; J->t[0] = 0;
        mb_joint* D = &W->Jt[m->j_diff]; memset(D->J, 0, sizeof D->J); D->t[0] = 0; D->phi[0] = 0;
        if (m->carrier) {
            /* carrier axis = chassis lateral, signed so that forward rolling of the driven wheels spins it positive */
            v3 yw_ = mv(&Ch->R, EY), ac = vdot(vadd(aL, aR), yw_) >= 0 ? yw_ : vneg(yw_);
            double fd = P->final_drive, g = Gr / fd;
            mb_joint* G = &W->Jt[m->j_gbox]; memset(G->J, 0, sizeof G->J); G->t[0] = 0; G->phi[0] = 0;
            if (Gr != 0) {   /* w_in / g - w_out = 0 (relative to the casing) */
                row_an(xw, 1.0 / g, G->J[0][0]); row_an(xw, -1, G->J[0][1]); row_an(xw, 1 - 1.0 / g, G->J[0][2]); G->compl = 1e-11;
            } else G->compl = 1.0;   /* neutral: row inert, the output stays geared to the wheels */
            mb_joint* F = &W->Jt[m->j_fd]; memset(F->J, 0, sizeof F->J); F->t[0] = 0; F->phi[0] = 0;
            row_an(xw, 1.0 / fd, F->J[0][0]); row_an(ac, -1, F->J[0][1]); row_an(vsub(ac, vscl(xw, 1.0 / fd)), 1, F->J[0][2]); F->compl = 1e-11;
            row_an(ac, 1, D->J[0][0]); row_an(aL, -0.5, D->J[0][1]); row_an(aR, -0.5, D->J[0][2]);
            row_an(vsub(vscl(vadd(aL, aR), 0.5), ac), 1, D->J[0][3]); D->compl = 1e-11;
        } else if (Gr != 0) {
            row_an(xw, 1.0 / Gr, D->J[0][0]); row_an(aL, -0.5, D->J[0][1]); row_an(aR, -0.5, D->J[0][2]);
            v3 cwv = vadd(vscl(xw, -1.0 / Gr), vscl(vadd(aL, aR), 0.5)); row_an(cwv, 1, D->J[0][3]);
            D->compl = 1e-11;
        } else D->compl = 1.0;
        mb_joint* L = &W->Jt[m->j_lsd]; memset(L->J, 0, sizeof L->J); L->t[0] = 0;
        row_an(aL, 1, L->J[0][0]); row_an(aR, -1, L->J[0][1]); row_an(vsub(aR, aL), 1, L->J[0][2]);
        double cap2 = 0;
        if (P->diff_viscous <= 0) {
            double Tin = fabs(m->lam_diff_prev) / h, Tw = -m->lam_diff_prev;
            double wav = vdot(W->B[wl].w, aL) + vdot(W->B[wr].w, aR);
            double lock = Tw * wav >= 0 ? P->diff_power : P->diff_coast;
            cap2 = (P->diff_preload + lock * Tin) * h;
        }
        L->lo[0] = -cap2; L->hi[0] = cap2;
    }
    for (int i = 0; i < 4; i++) {
        mb_joint* B = &W->Jt[m->j_brake[i]]; memset(B->J, 0, sizeof B->J); B->t[0] = 0;
        row_an(a_w[i], 1, B->J[0][0]); row_an(a_w[i], -1, B->J[0][1]);
        const dc_axle_params* A = &P->axle[i / 2];
        double cap = (clampd(c->in.brake, 0, 1) * A->brake_torque + clampd(c->in.handbrake, 0, 1) * A->handbrake_torque) * h;
        B->lo[0] = -cap; B->hi[0] = cap;
    }
    /* ---- 5. force elements */
    double pe = m->pe_body;   /* body-contact elastic store (its crush loss then appears in the audited residual) */
    for (int i = 0; i < 4; i++) {
        const dc_kin* k = &c->K[i]; const dc_axle_params* A = &P->axle[i / 2];
        mb_body* Lb = &W->B[m->lo_body[i]];
        v3 plo = mb_point_world(Lb, m->lo_local[i]);
        v3 phi_ = vadd(Kp, mv(&R, k->spr_hi));
        v3 d = vsub(plo, phi_); double L = vlen(d); v3 u = vscl(d, 1 / L);
        int bb[2] = {m->lo_body[i], m->ch}; double jj[2][6];
        row_pt(W, m->lo_body[i], plo, u, 1, jj[0]); row_pt(W, m->ch, phi_, u, -1, jj[1]);
        double Ld = mb_row_vel(W, 2, bb, jj);
        double Fs = k->k_spring * (k->L_free - L), kk = -k->k_spring, cc = 0, slope = 0;
        pe += 0.5 * k->k_spring * (k->L_free - L) * (k->L_free - L);
        double Fd = nodamp ? 0 : damper_force(k, -Ld, &slope);
        if (!nodamp) cc -= slope;
        double Fb = 0;
        double mcx = c->m_c[i];
        if (L < m->L_up[i]) {             /* progressive bump stop (rate in wheel units) */
            double MR = m->MR_up[i], x = (m->L_up[i] - L) / MR, x0 = 0.01, rate = A->bumpstop_rate;
            double Fw = rate * x * (1 + x / x0);
            Fb = Fw / MR; kk -= rate * (1 + 2 * x / x0) / (MR * MR);
            pe += rate * (0.5 * x * x + x * x * x / (3 * x0));
            if (!nodamp && Ld < 0) { double cw_ = 0.2 * sqrt(rate * (1 + x / x0) * mcx); Fb += cw_ * (-Ld / MR) / MR; cc -= cw_ / (MR * MR); }
        } else if (L > m->L_dn[i]) {      /* droop stop */
            double MR = m->MR_dn[i], x = (m->L_dn[i] - L) / MR, rate = A->droop_rate;
            Fb = rate * x / MR; kk -= rate / (MR * MR);
            pe += 0.5 * rate * x * x;
            if (!nodamp && Ld > 0) { double cw_ = 0.4 * sqrt(rate * mcx); Fb += cw_ * (-Ld / MR) / MR; cc -= cw_ / (MR * MR); }
        }
        m->Fs[i] = Fs; m->Fd[i] = Fd; m->Fb[i] = Fb * (L < m->L_up[i] ? m->MR_up[i] : m->MR_dn[i]);
        if (m->j_stf[i] >= 0) {
            const mb_joint* Sj = &W->Jt[m->j_strut[i]];
            v3 ax = mv(&W->B[m->kn[i]].R, Sj->aa), P_ = mb_point_world(&W->B[m->ch], Sj->pb);
            mb_joint* Sf = &W->Jt[m->j_stf[i]]; memset(Sf->J, 0, sizeof Sf->J);
            row_pt(W, m->kn[i], P_, ax, 1, Sf->J[0][0]); row_pt(W, m->ch, P_, ax, -1, Sf->J[0][1]);
            Sf->t[0] = 0; Sf->active = !nodamp; Sf->pre = M->strut_seal * h;
            /* top mount: axial load = spring + stop force (element value at the start of the step; elements are
               linearly implicit, so this normal lags by h k gdot at most) */
            mb_joint* Tf = &W->Jt[m->j_tmf[i]]; memset(Tf->J, 0, sizeof Tf->J);
            row_an(ax, 1, Tf->J[0][0]); row_an(ax, -1, Tf->J[0][1]); Tf->t[0] = 0;
            double cap = nodamp ? 0 : (M->strut_mount_torque + M->strut_mount_mu * M->strut_mount_radius * fabs(Fs + Fb)) * h;
            Tf->lo[0] = -cap; Tf->hi[0] = cap;
        }
        {   /* solid stop: rigid inelastic unilateral row, speculative (closing speed <= gap/h, never past it) */
            mb_joint* Hj = &W->Jt[m->j_hard[i]]; memset(Hj->J, 0, sizeof Hj->J);
            double gap = L - m->L_hard[i];
            if (m->L_hard[i] > 0 && gap < 0.03) {
                for (int q = 0; q < 6; q++) { Hj->J[0][0][q] = jj[0][q]; Hj->J[0][1][q] = jj[1][q]; }
                Hj->t[0] = gap > 0 ? -gap / h : 0; Hj->lo[0] = 0; Hj->hi[0] = 1e12;
            } else { Hj->lo[0] = Hj->hi[0] = 0; Hj->t[0] = 0; }
        }
        mb_elem_add(W, 2, bb, jj, Fs + Fd + Fb, kk, cc);
        c->pose[i].lo = mtv(&R, vsub(plo, Kp)); c->pose[i].Ls = L;
    }
    for (int a = 0; a < 2; a++) {
        if (!m->has_bar[a]) continue;
        int b0 = m->bar[a][0], b1 = m->bar[a][1];
        double g0 = rel_angle(W, m->ch, b0, m->qd[b0], EY), g1 = rel_angle(W, m->ch, b1, m->qd[b1], EY), gg = g0 - g1;
        int bb[2] = {b0, b1}; double jj[2][6]; row_an(yw, 1, jj[0]); row_an(yw, -1, jj[1]);
        mb_elem_add(W, 2, bb, jj, -m->k_bar[a] * gg, -m->k_bar[a], 0);
        pe += 0.5 * m->k_bar[a] * gg * gg;
    }
    m->tau_tb = 0; m->T_hands = 0; m->F_assist = 0; double F_act = 0;
    if (m->steer) {
        /* torsion bar (valve input shaft to pinion) */
        double ktb = P->assist ? P->assist_torsion_k : P->torsion_k;
        v3 ein = mv(&R, m->e_in);
        int bt[2] = {m->inp, m->pin}; double jt[2][6]; row_an(ein, 1, jt[0]); row_an(ein, -1, jt[1]);
        double tw = thi - thp, twd = mb_row_vel(W, 2, bt, jt);
        double tau = ktb * tw + (nodamp ? 0 : P->torsion_c * twd);
        mb_elem_add(W, 2, bt, jt, -tau, -ktb, nodamp ? 0 : -P->torsion_c);
        pe += 0.5 * ktb * tw * tw; m->tau_tb = tau;
        /* rack: hydraulic assist (open-centre valve law, pump flow limit, back-drive orifice damping), stops */
        int br[2] = {m->rack, m->hsg}; double jr[2][6];
        v3 xr = W->B[m->rack].x;
        jr[0][0] = yw.x; jr[0][1] = yw.y; jr[0][2] = yw.z; jr[0][3] = jr[0][4] = jr[0][5] = 0;
        row_pt(W, m->hsg, xr, yw, -1, jr[1]);
        double rd = mb_row_vel(W, 2, br, jr);
        double f = 0, kk = 0, cc = 0;
        if (P->assist && m->vmodel) {
            /* valve bridge with pressure states: backward-Euler pressures linearised in the (new) rack velocity, so the
               cylinder is a linearly implicit element between rack and housing; committed nonlinearly after the solve */
            double thv = (thi - thp) * sgn(c->r_p), Pl[3], dPdv[3];
            m->vQp = P->assist_flow * clampd(fabs(rpm) / P->assist_rpm_full, 0, 1);
            if (dc_valve_solve(&m->vlv, m->vp, thv, rd, r, m->vQp, h, Pl, dPdv) < 0) m->v_fail++;
            double F = P->assist_area * (Pl[0] - Pl[1]), cv = P->assist_area * (dPdv[0] - dPdv[1]);
            f += F; cc += cv;
            m->F_assist = F; m->v_th = thv; m->v_F0 = F; m->v_c = cv; m->v_rd0 = rd;
        } else if (P->assist) {
            double x = tau / P->assist_tref, x4 = x * x * x * x;
            double dP = P->assist_pmax * sgn(x) * x4 / (1 + x4);
            double F = P->assist_area * dP * sgn(c->r_p);
            double flow = P->assist_flow * clampd(fabs(rpm) / P->assist_rpm_full, 0, 1);
            double va = rd * sgn(F);
            if (va > 0) F *= clampd(1 - va * P->assist_area / fmax(flow, 1e-9), 0, 1);
            double cbd = 0;
            if (!nodamp && va < 0 && P->assist_backdrive > 0) {
                double Qe = fmax(flow, 0.1 * P->assist_flow);
                cbd = P->assist_backdrive * 2 * fabs(F) * P->assist_area / Qe;
                double room = fmax(1.25 * P->assist_pmax * P->assist_area - fabs(F), 0);
                if (cbd * fabs(rd) > room) cbd = room / fmax(fabs(rd), 1e-9);
            }
            double ch_ = nodamp ? 0 : P->assist_damping;
            f += F - ch_ * rd - cbd * rd; cc -= ch_ + cbd;
            m->F_assist = F - cbd * rd; F_act = F;   /* F: pump-powered (external energy), cbd/ch_: dissipative */
        }
        double ex = fabs(r) - c->rack_max;
        if (ex > 0) {
            f -= P->rack_stop_k * ex * sgn(r); kk -= P->rack_stop_k; pe += 0.5 * P->rack_stop_k * ex * ex;
            if (!nodamp && rd * r > 0) { f -= P->rack_stop_c * rd; cc -= P->rack_stop_c; }
        }
        if (f != 0 || kk != 0 || cc != 0) mb_elem_add(W, 2, br, jr, f, kk, cc);
        /* driver's hands on the rim (reaction through the seat = chassis) */
        if (!(c->dbg & DC_DBG_FREEZE_STEER)) {
            v3 er = mv(&R, m->e_rim);
            int bh[2] = {m->rim, m->ch}; double jh[2][6]; row_an(er, 1, jh[0]); row_an(er, -1, jh[1]);
            double thd = mb_row_vel(W, 2, bh, jh), Th, hk = 0, hc = 0;
            if (c->in.steer_mode == DC_STEER_ANGLE) {
                Th = c->in.hands_k * (c->in.steer_target - th) - c->in.hands_d * thd;
                if (c->in.hands_tmax > 0 && fabs(Th) > c->in.hands_tmax) Th = clampd(Th, -c->in.hands_tmax, c->in.hands_tmax);
                else { hk = -c->in.hands_k; hc = -c->in.hands_d; }
            } else Th = c->in.steer_torque;
            mb_elem_add(W, 2, bh, jh, Th, hk, hc);
            m->T_hands = Th;
        }
    }
    if (P->diff_viscous > 0 && !nodamp) {
        int bv[3] = {wl, wr, m->ch}; double jv[3][6];
        row_an(aL, 1, jv[0]); row_an(aR, -1, jv[1]); row_an(vsub(aR, aL), 1, jv[2]);
        mb_elem_add(W, 3, bv, jv, -P->diff_viscous * mb_row_vel(W, 3, bv, jv), 0, -P->diff_viscous);
    }
    /* tyre longitudinal force: linearly implicit in the coupled solve (dFx/dslip from the carcass/brush model) */
    double gd0[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; i++) {
        dc_tc* t = &m->tc[i]; m->el_tyre[i] = -1;
        if (!t->contact) continue;
        int bw[1] = {m->wh[i]}; double jw[1][6];
        row_pt(W, m->wh[i], t->p, t->xc, 1, jw[0]);
        double ce = (t->arm_x < -1e-3 && t->kx * t->D > 0) ? t->kx * t->D / t->arm_x : 0;
        m->el_tyre[i] = mb_elem_add(W, 1, bw, jw, t->Fx_star, 0, ce);
        gd0[i] = W->E[m->el_tyre[i]].gd;
    }
    /* ---- 6. solve (re-solve once if the linearised Fx overshoots the friction circle) */
    for (int b = 0; b < W->nb; b++) { m->v_save[b] = W->B[b].v; m->w_save[b] = W->B[b].w; }
    /* stored energy at the start of the step, every term at the same instant (energy audit) */
    if (m->nbush) for (int i = 0; i < 4; i++) for (int n = 0; n < m->narm[i]; n++) pe += mb_bushing_energy(W, m->j_arm_rev[i][n]);
    if (m->steer && m->j_rmount >= 0) pe += mb_bushing_energy(W, m->j_rmount);
    double E_start = pe;
    if (m->vmodel) { double eh = dc_valve_energy(&m->vlv, m->vp, r); E_start += eh; pe += eh; }
    for (int b = 0; b < W->nb; b++) {
        mb_body* B = &W->B[b];
        E_start += 0.5 * B->m * vdot(B->v, B->v) + 0.5 * vdot(B->w, mv(&B->Iw, B->w)) + B->m * g * B->x.z;
    }
    for (int i = 0; i < 4; i++) E_start += m->tc[i].pe + dc_tyre_shear_store(c, i, &m->tc[i]);   /* radial (phase A) + committed shear/twist */
    double dgd[4] = {0, 0, 0, 0};
    for (int pass = 0; pass < 2; pass++) {
        mb_solve(W);
        int redo = 0;
        for (int i = 0; i < 4; i++) {
            dc_tc* t = &m->tc[i]; if (m->el_tyre[i] < 0) { m->Fx_app[i] = 0; continue; }
            mb_elem* E = &W->E[m->el_tyre[i]];
            dgd[i] = mb_row_vel(W, E->nb, E->b, E->j) - gd0[i];
            double Fx = E->f + E->c * dgd[i];
            m->Fx_app[i] = Fx;
            double cap = t->mu_eff * t->Ntot, Fm = sqrt(Fx * Fx + t->Fy * t->Fy);
            if (pass == 0 && Fm > 1.02 * cap && cap > 0 && E->c != 0) {
                double s = cap / Fm;
                E->f = Fx * s; E->c = 0; redo = 1;
                mb_body* Wb = &W->B[m->wh[i]];
                v3 dF = vscl(t->yc, t->Fy * (s - 1));
                Wb->F = vadd(Wb->F, dF); Wb->T = vadd(Wb->T, vcross(vsub(t->p, Wb->x), dF));
                t->Fy *= s;
            }
        }
        if (!redo) break;
        m->resolves++;
        for (int b = 0; b < W->nb; b++) { W->B[b].v = m->v_save[b]; W->B[b].w = m->w_save[b]; }
    }
    /* tyre state commit (carcass deflections) with the solved slip */
    for (int i = 0; i < 4; i++) {
        dc_tc* t = &m->tc[i];
        if (!t->contact) { t->Fx = 0; t->F = V3(0, 0, 0); continue; }
        double dO = t->arm_x < -1e-3 ? dgd[i] / t->arm_x : 0;
        double Fx_app = m->Fx_app[i], Fy_app = t->Fy;
        dc_tyre_phaseB(c, i, cw[i], a_w[i], c->ty[i].omega + dO, h, t, 1);
        t->Fx = Fx_app; t->Fy = Fy_app;
        t->F = vadd(vscl(t->xc, Fx_app), vscl(t->yc, Fy_app));
        for (int q = 0; q < t->nrows; q++) if (t->N[q] > 0) t->F = vadd(t->F, vscl(t->nr[q], t->N[q]));
        Fext = vadd(Fext, t->F);
        Text = vadd(Text, vadd(vcross(t->p, t->F), m->T_tyre[i]));   /* contact force moment about the world origin + contact moments */
        /* no Win term: the road is stationary, so contact forces do no work on the vehicle+tyre system; tyre
           carcass deflection is stored energy (tc.pe) and slip/hysteresis shows up as dissipation */
        c->Fc[i] = t->F; c->Pc[i] = t->p;
    }
    /* external power sources (solved velocities): combustion torque, driver's hands, pump-powered assist,
       aero. Everything else (body contacts, dampers, Coulomb rows, orifices, LSD, integrator) is dissipation. */
    {
        v3 ax = mv(&R, EX);
        Win += h * Te * (vdot(W->B[m->eng].w, ax) - vdot(Ch->w, ax));
        Win += h * (vdot(F_chx, Ch->v) + vdot(T_chx, Ch->w));
        if (m->steer) {
            v3 er = mv(&R, m->e_rim);
            Win += h * m->T_hands * (vdot(W->B[m->rim].w, er) - vdot(Ch->w, er));
            v3 dvr = vsub(W->B[m->rack].v, mb_point_vel(&W->B[m->hsg], W->B[m->rack].x));
            Win += h * F_act * vdot(dvr, yw);
            if (m->vmodel) {   /* commit the pressure states with the solved rack velocity; the pump is the power source */
                double rdn = vdot(dvr, yw), Pn[3], d_[3];
                if (dc_valve_solve(&m->vlv, m->vp, m->v_th, rdn, r, m->vQp, h, Pn, d_) < 0) m->v_fail++;
                memcpy(m->vp, Pn, sizeof Pn);
                m->F_assist = m->v_F0 + m->v_c * (rdn - m->v_rd0);
                Win += h * m->vQp * Pn[2];
            }
        }
    }
    /* momentum before integration (for the Newton-3 audit) */
    v3 P0 = V3(0, 0, 0), L0 = V3(0, 0, 0);
    for (int b = 0; b < W->nb; b++) {
        mb_body* B = &W->B[b];
        P0 = vadd(P0, vscl(m->v_save[b], B->m));
        L0 = vadd(L0, vadd(vcross(B->x, vscl(m->v_save[b], B->m)), mv(&B->Iw, m->w_save[b])));
    }
    /* ---- 7. integrate + projection */
    mb_integrate(W, !(c->dbg & DC_DBG_NO_PROJECTION));
    m->lam_diff_prev = Gr != 0 ? W->Jt[m->j_diff].lam[0] : 0;
    /* ---- 8. write-back to the shared car state */
    Ch = &W->B[m->ch];
    m3 Rn = Ch->R;
    c->q = Ch->q;
    v3 rk = mv(&Rn, m->cgc);
    v3 vold = c->v;
    c->x = vsub(Ch->x, rk);
    c->v = vsub(Ch->v, vcross(Ch->w, rk));
    c->w = Ch->w;
    xw = mv(&Rn, EX); yw = mv(&Rn, EY);
    m3 RnT = mtrans(&Rn);
    for (int i = 0; i < 4; i++) {
        mb_body* Kb = &W->B[m->kn[i]];
        dc_pose* p = &c->pose[i];
        p->c = mtv(&Rn, vsub(Kb->x, c->x));
        p->R = mmul(&RnT, &Kb->R);
        double s_old = c->s[i];
        c->s[i] = p->c.z - c->K[i].hub0.z;
        v3 vrel = vsub(Kb->v, vadd(c->v, vcross(c->w, vsub(Kb->x, c->x))));
        c->sd[i] = vdot(vrel, mv(&Rn, EZ)); (void)s_old;
        mb_body* Wb = &W->B[m->wh[i]];
        v3 a = mv(&Wb->R, c->K[i].spin_axis_u);
        dc_tyre_state* ts = &c->ty[i];
        double om = vdot(Wb->w, a);
        ts->angle = fmod(ts->angle + h * (om - vdot(Kb->w, a)), 2 * M_PI);
        ts->omega = om;
        if (ts->leak_tau > 0) ts->pressure *= exp(-h / ts->leak_tau);
    }
    if (m->steer) {
        double th2, thp2, thi2, r2;
        steer_angles(c, &th2, &thp2, &thi2, &r2, 1);
        c->th = th2; c->r = r2;
        c->thd = vdot(vsub(W->B[m->rim].w, Ch->w), mv(&Rn, m->e_rim));
        v3 dv = vsub(W->B[m->rack].v, mb_point_vel(&W->B[m->hsg], W->B[m->rack].x));
        c->rd = vdot(dv, yw);
    }
    c->we = vdot(W->B[m->eng].w, xw) - vdot(Ch->w, xw);
    c->wg = vdot(W->B[m->gi].w, xw) - vdot(Ch->w, xw);
    /* ---- 9. audits: energy, momentum (Newton-3) */
    double ke = mb_kinetic(W), pg = 0;
    v3 P1 = V3(0, 0, 0), L1 = V3(0, 0, 0);
    for (int b = 0; b < W->nb; b++) {
        mb_body* B = &W->B[b];
        pg += B->m * g * B->x.z;
        P1 = vadd(P1, vscl(B->v, B->m));
        m3 t1 = mmul(&B->R, &B->Ib), Rt = mtrans(&B->R), Iw = mmul(&t1, &Rt);
        L1 = vadd(L1, vadd(vcross(B->x, vscl(B->v, B->m)), mv(&Iw, B->w)));
    }
    double tpe = 0; for (int i = 0; i < 4; i++) tpe += m->tc[i].pe;
    dc_telemetry* T = &c->tel;
    double Eprev = T->kinetic + T->potential;
    double Enew = ke + pg + pe + tpe;
    /* internal dissipation = external work in - stored energy change (dampers, friction, valve, tyre carcass
       damping; body-contact crush/scrape work is not in Win, so it shows up here too) */
    /* audit over the previous step: E(start k-1) + W_in(k-1) - E(start k) */
    if (m->audit_init) c->dissipated += m->E_audit - E_start + m->W_audit;
    m->E_audit = E_start; m->W_audit = Win; m->audit_init = 1; (void)Eprev; (void)Enew;
    (void)E0;
    T->kinetic = ke; T->potential = pg + pe + tpe;
    T->momentum[0] = P1.x; T->momentum[1] = P1.y; T->momentum[2] = P1.z;
    T->ang_momentum[0] = L1.x; T->ang_momentum[1] = L1.y; T->ang_momentum[2] = L1.z;
    T->ext_force[0] = Fext.x; T->ext_force[1] = Fext.y; T->ext_force[2] = Fext.z;
    T->ext_torque[0] = Text.x; T->ext_torque[1] = Text.y; T->ext_torque[2] = Text.z;
    v3 rf = vsub(vscl(vsub(P1, P0), 1 / h), Fext);
    v3 rt = vsub(vscl(vsub(L1, L0), 1 / h), Text);
    T->resid_force[0] = rf.x; T->resid_force[1] = rf.y; T->resid_force[2] = rf.z;
    T->resid_torque[0] = rt.x; T->resid_torque[1] = rt.y; T->resid_torque[2] = rt.z;
    /* ---- 10. telemetry */
    T->time = c->t; T->tick = c->tick;
    v3 acc = vscl(vsub(c->v, vold), 1.0 / h);
    T->vx = vdot(c->v, xw); T->vy = vdot(c->v, yw); T->speed = vlen(c->v);
    T->yaw_rate = vdot(c->w, mv(&Rn, EZ));
    T->ax = vdot(acc, xw); T->ay = vdot(acc, yw);
    T->sw_angle = c->th; T->sw_rate = c->thd; T->rack_pos = c->r; T->rack_vel = c->rd;
    T->column_torque = m->tau_tb; T->driver_torque = m->T_hands; T->assist_force = m->F_assist;
    double Frack = 0;
    for (int sd = 0; sd < 2 && m->steer; sd++) {
        int jl = m->j_link[sd][c->K[sd].rack_link]; if (jl < 0) continue;
        v3 F_, T_; mb_joint_wrench(W, jl, 1, &F_, &T_); Frack += vdot(F_, yw);
    }
    T->rack_force = Frack;
    T->engine_rpm = c->we * 30 / M_PI; T->clutch_slip = (c->we - c->wg) * 30 / M_PI; T->gear = c->in.gear;
    T->dissipated = c->dissipated; T->engine_running = c->engine_on; T->proj_work = 0;
    T->rack_fric_force = m->steer ? W->Jt[m->j_rfric].lam[0] / h : 0; T->col_fric_torque = m->steer ? W->Jt[m->j_cfric].lam[0] / h : 0;
    T->valve_p[0] = m->vp[0]; T->valve_p[1] = m->vp[1]; T->valve_p[2] = m->vp[2];
    T->pump_flow = m->vQp; T->hyd_energy = m->vmodel ? dc_valve_energy(&m->vlv, m->vp, c->r) : 0;
    T->valve_nfail = m->v_fail;
    T->pgs_iters = W->pgs_it; T->box_rows = W->nbox; T->box_dropped = W->box_dropped;
    for (int i = 0; i < 4; i++) {
        dc_wheel_telem* Wt = &T->w[i]; dc_tc* t = &m->tc[i]; const dc_kin* k = &c->K[i]; dc_pose* p = &c->pose[i];
        Wt->fz = t->Ntot; Wt->fx = t->Fx; Wt->fy = t->Fy; Wt->mz = t->Mz_brush + t->Mt - t->ey_new * t->Fx;
        Wt->force_world[0] = t->F.x; Wt->force_world[1] = t->F.y; Wt->force_world[2] = t->F.z;
        Wt->contact[0] = t->p.x; Wt->contact[1] = t->p.y; Wt->contact[2] = t->p.z;
        Wt->normal[0] = t->n.x; Wt->normal[1] = t->n.y; Wt->normal[2] = t->n.z;
        dc_tyre_telem_slip(c, i, t, h, &Wt->slip_angle, &Wt->slip_ratio);   /* relaxed kinematic slip */
        Wt->slip_angle_ss = t->slip_angle_ss; Wt->slip_ratio_ss = t->slip_ratio_ss;
        Wt->patch_half_len = t->a_eff; Wt->deflection = t->delta_eff; Wt->pressure = c->ty[i].pressure; Wt->rim_force = t->Fr;
        Wt->mu = t->mu_eff; Wt->sliding = fmin(t->lam, 1.0);
        Wt->travel = c->s[i]; Wt->travel_vel = c->sd[i];
        Wt->spring_force = m->Fs[i]; Wt->damper_force = m->Fd[i]; Wt->bumpstop_force = m->Fb[i] + W->Jt[m->j_hard[i]].lam[0] / h * m->MR_up[i];
        v3 ac = mv(&p->R, k->spin_axis_u);
        Wt->steer = atan2(-ac.x, ac.y); Wt->toe = Wt->steer;
        Wt->camber = t->contact ? ((i & 1) ? -t->camber_l : t->camber_l) : ((i & 1) ? asin(ac.z) : asin(-ac.z));
        Wt->omega = c->ty[i].omega;
        double bt = W->Jt[m->j_brake[i]].lam[0] / h;
        Wt->brake_torque = bt;
        double dt_ = 0;
        if (i == m->drv[0] || i == m->drv[1]) {
            int kk = i == m->drv[0] ? 1 : 2;
            v3 F_, T_; mb_joint_wrench(W, m->j_diff, kk, &F_, &T_); dt_ += vdot(T_, a_w[i]);
            mb_joint_wrench(W, m->j_lsd, kk - 1, &F_, &T_); dt_ += vdot(T_, a_w[i]);
        }
        Wt->drive_torque = dt_;
        int jl = k->rack_link >= 0 ? m->j_link[i][k->rack_link] : -1;
        Wt->tierod_force = jl >= 0 ? -W->Jt[jl].lam[0] / h : 0;
        for (int q = 0; q < DC_MAX_ROWS; q++) Wt->row_fz[q] = q < t->nrows ? t->N[q] : 0;
    }
    c->t += h; c->tick++;
}
