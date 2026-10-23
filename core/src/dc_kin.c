/* dc_kin.c - exact suspension kinematics from hardpoints.
 *
 * Unknown: upright pose (hub centre c, rotation R) in the chassis frame.  Six scalar constraints:
 *   rigid links      g_k = (|c + R u_k - (q_k + r*y*[rack])|^2 - L_k^2) / (2 L_k)
 *   strut (MacPh.)   g_j = (R e_j) . (T - c - R S)          j = 1,2  (top mount lies on the strut axis)
 *   travel           g_6 = c_z - c0_z - s                     (s = generalized "hub travel" coordinate)
 * Solved by Newton on (dc, dθ) with R <- exp(dθ) R.  Velocity Jacobian xi = -Cx^-1 Cs (exact),
 * acceleration bias Jdot*sdot by central difference of xi along the motion (O(eps^2)).
 */
#include "dc_internal.h"

static v3 mirror(v3 p, int right) { if (right) p.y = -p.y; return p; }
static v3 arr(const double* a) { return V3(a[0], a[1], a[2]); }

void dc_kin_setup(dc_car* c) {
    for (int i = 0; i < 4; i++) {
        const dc_axle_params* A = &c->P.axle[i / 2];
        dc_kin* k = &c->K[i];
        int right = i & 1;
        v3 off = V3(c->P.sprung_cg[0], c->P.sprung_cg[1], c->P.sprung_cg[2]);
        memset(k, 0, sizeof *k);
        k->type = A->type; k->nl = A->nlinks; k->rack_link = A->rack_link; k->right = right;
        k->steered = A->steered; k->arm_a = A->lower_arm_a; k->arm_b = A->lower_arm_b;
        k->hub0 = vsub(mirror(arr(A->hub), right), off);
        for (int l = 0; l < k->nl; l++) {
            k->lc[l] = vsub(mirror(arr(A->link_c[l]), right), off);
            k->lu0[l] = vsub(mirror(arr(A->link_u[l]), right), off);
            k->lu[l] = vsub(k->lu0[l], k->hub0);
            k->L[l] = vlen(vsub(k->lu0[l], k->lc[l]));
            if (l == k->rack_link) k->L[l] += A->toe_rod_delta;
        }
        if (k->type == DC_SUSP_STRUT) {
            k->strut_top = vsub(mirror(arr(A->strut_top), right), off);
            v3 lowp = vsub(mirror(arr(A->strut_low), right), off);
            k->strut_S = vsub(lowp, k->hub0);
            k->strut_d = vnorm(vsub(k->strut_top, lowp));
            v3 t = fabs(k->strut_d.x) < 0.9 ? V3(1, 0, 0) : V3(0, 1, 0);
            k->strut_e1 = vnorm(vcross(k->strut_d, t));
            k->strut_e2 = vcross(k->strut_d, k->strut_e1);
        }
        /* spindle axis (points to vehicle left on every wheel) with static camber/toe */
        double g = A->static_camber_deg * M_PI / 180.0, toe = A->static_toe_deg * M_PI / 180.0;
        v3 a = right ? V3(0, cos(g), sin(g)) : V3(0, cos(g), -sin(g));
        double yaw = right ? toe : -toe;
        k->spin_axis_u = V3(a.x * cos(yaw) - a.y * sin(yaw), a.x * sin(yaw) + a.y * cos(yaw), a.z);
        k->spr_on_upright = A->spring_on_upright;
        k->spr_lo = vsub(mirror(arr(A->spring_lo), right), off);
        k->spr_hi = vsub(mirror(arr(A->spring_hi), right), off);
        if (k->arm_a >= 0 && k->arm_b >= 0) {
            k->arm_p = k->lc[k->arm_a];
            k->arm_axis = vnorm(vsub(k->lc[k->arm_b], k->lc[k->arm_a]));
        } else k->spr_on_upright = 1;
    }
}

/* residuals and jacobian (6x6 wrt dc,dθ) and Cs (6x2 wrt s,r) */
static void constraints(const dc_kin* k, v3 c, const m3* R, double s, double r, double* g, double* Cx, double* Cs) {
    int row = 0;
    for (int l = 0; l < k->nl; l++, row++) {
        v3 rk = mv(R, k->lu[l]);
        v3 P = vadd(c, rk);
        v3 Q = k->lc[l]; if (l == k->rack_link && k->steered) Q.y += r;
        v3 d = vsub(P, Q); double L = k->L[l];
        g[row] = (vdot(d, d) - L * L) / (2 * L);
        if (Cx) {
            v3 dth = vcross(rk, d);
            Cx[row * 6 + 0] = d.x / L; Cx[row * 6 + 1] = d.y / L; Cx[row * 6 + 2] = d.z / L;
            Cx[row * 6 + 3] = dth.x / L; Cx[row * 6 + 4] = dth.y / L; Cx[row * 6 + 5] = dth.z / L;
            Cs[row * 2 + 0] = 0;
            Cs[row * 2 + 1] = (l == k->rack_link && k->steered) ? -d.y / L : 0;
        }
    }
    if (k->type == DC_SUSP_STRUT) {
        v3 rS = mv(R, k->strut_S); v3 P = vadd(c, rS); v3 D = vsub(k->strut_top, P);
        for (int j = 0; j < 2; j++, row++) {
            v3 E = mv(R, j == 0 ? k->strut_e1 : k->strut_e2);
            g[row] = vdot(E, D);
            if (Cx) {
                v3 dth = vsub(vcross(E, D), vcross(rS, E));
                Cx[row * 6 + 0] = -E.x; Cx[row * 6 + 1] = -E.y; Cx[row * 6 + 2] = -E.z;
                Cx[row * 6 + 3] = dth.x; Cx[row * 6 + 4] = dth.y; Cx[row * 6 + 5] = dth.z;
                Cs[row * 2 + 0] = 0; Cs[row * 2 + 1] = 0;
            }
        }
    }
    g[row] = c.z - k->hub0.z - s;
    if (Cx) {
        for (int j = 0; j < 6; j++) Cx[row * 6 + j] = 0;
        Cx[row * 6 + 2] = 1; Cs[row * 2 + 0] = -1; Cs[row * 2 + 1] = 0;
    }
}

static void spring_geom(const dc_kin* k, dc_pose* p, int need_jac) {
    v3 lo, dlo[2] = {{0, 0, 0}, {0, 0, 0}};
    if (k->spr_on_upright) {
        lo = vadd(p->c, mv(&p->R, vsub(k->spr_lo, k->hub0)));
        if (need_jac) for (int j = 0; j < 2; j++) dlo[j] = vadd(p->Jp[j], vcross(p->Jw[j], vsub(lo, p->c)));
    } else {
        v3 e = k->arm_axis, A = k->arm_p;
        v3 b0 = k->lu0[k->arm_a], b = vadd(p->c, mv(&p->R, k->lu[k->arm_a]));
        v3 r0 = vsub(b0, A), r1 = vsub(b, A);
        r0 = vsub(r0, vscl(e, vdot(e, r0))); r1 = vsub(r1, vscl(e, vdot(e, r1)));
        double phi = atan2(vdot(e, vcross(r0, r1)), vdot(r0, r1));
        v3 lo0 = vsub(k->spr_lo, A);
        m3 Rot = mexp(vscl(e, phi));
        lo = vadd(A, mv(&Rot, lo0));
        if (need_jac) {
            v3 exr = vcross(e, vsub(b, A)); double n2 = vdot(exr, exr);
            for (int j = 0; j < 2; j++) {
                v3 bd = vadd(p->Jp[j], vcross(p->Jw[j], vsub(b, p->c)));
                double dphi = vdot(exr, bd) / n2;
                dlo[j] = vscl(vcross(e, vsub(lo, A)), dphi);
            }
        }
    }
    p->lo = lo;
    v3 d = vsub(k->spr_hi, lo); p->Ls = vlen(d);
    if (need_jac) for (int j = 0; j < 2; j++) p->dL[j] = -vdot(d, dlo[j]) / p->Ls;
}

int dc_kin_solve(const dc_kin* k, double s, double r, dc_pose* p, int need_jac) {
    double g[6], Cx[36], Cs[12];
    int it, ok = 1;
    for (it = 0; it < 30; it++) {
        constraints(k, p->c, &p->R, s, r, g, Cx, Cs);
        double e = 0; for (int i = 0; i < 6; i++) e += g[i] * g[i];
        if (e < 1e-26) break;
        double b[6]; for (int i = 0; i < 6; i++) b[i] = -g[i];
        if (!dc_solve(6, Cx, b)) { ok = 0; break; }
        p->c = vadd(p->c, V3(b[0], b[1], b[2]));
        m3 dR = mexp(V3(b[3], b[4], b[5]));
        p->R = mmul(&dR, &p->R);
        if (it % 4 == 3) p->R = morthonorm(p->R);
    }
    if (need_jac) {
        constraints(k, p->c, &p->R, s, r, g, Cx, Cs);
        for (int j = 0; j < 2; j++) {
            double A[36], b[6]; memcpy(A, Cx, sizeof A);
            for (int i = 0; i < 6; i++) b[i] = -Cs[i * 2 + j];
            dc_solve(6, A, b);
            p->Jp[j] = V3(b[0], b[1], b[2]); p->Jw[j] = V3(b[3], b[4], b[5]);
        }
    }
    spring_geom(k, p, need_jac);
    return ok && it < 30;
}

void dc_kin_full(const dc_kin* k, double s, double r, double sd, double rd, dc_pose* p) {
    dc_kin_solve(k, s, r, p, 1);
    double sp = fabs(sd) + fabs(rd) * 3.0;
    p->bp = V3(0, 0, 0); p->bw = V3(0, 0, 0);
    if (sp < 1e-9) return;
    double eps = 1e-3 / (1.0 + sp * 10.0);   /* displacement eps*speed stays < 1e-4 m */
    dc_pose a = *p, b = *p;
    dc_kin_solve(k, s + eps * sd, r + eps * rd, &a, 1);
    dc_kin_solve(k, s - eps * sd, r - eps * rd, &b, 1);
    if (!k->steered) rd = 0;
    v3 xa_p = vadd(vscl(a.Jp[0], sd), vscl(a.Jp[1], rd)), xb_p = vadd(vscl(b.Jp[0], sd), vscl(b.Jp[1], rd));
    v3 xa_w = vadd(vscl(a.Jw[0], sd), vscl(a.Jw[1], rd)), xb_w = vadd(vscl(b.Jw[0], sd), vscl(b.Jw[1], rd));
    p->bp = vscl(vsub(xa_p, xb_p), 0.5 / eps);
    p->bw = vscl(vsub(xa_w, xb_w), 0.5 / eps);
}
