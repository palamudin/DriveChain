/* dc_tyre_tm.c - TARMAC tyre model (selectable: tyre_model = 1).
 *
 * C port of the tyre of "TARMAC Vehicle Dynamics Lab 0.5.0" (tyre.js contact geometry + the tyre-force part of
 * physics.js), written by the project owner and contributed to this repository; reference JavaScript kept in
 * core/ref/tarmac/. The port is faithful except where the host differs (z-up frame, our road grid, our wheel
 * bodies and implicit Fx row); every deviation is listed in docs/TYRE_TARMAC.md. Additions over the original
 * (TARMAC has no Mz): self-aligning moment and pneumatic trail from the footprint shear distribution, and a
 * tread-twist (parking / turn-slip) moment from the same footprint.
 *
 * Contact (tyre.js):
 *   61 radial rays in the wheel plane (±65°, 1.5° under the patch, 4° outside) repeated over nl lanes across the
 *   contact width Wcp = (1.03 - 0.004 AR%) W. Penetration of the undeformed tread circle pen = R0 - t.
 *   Penetrated volume per lane V = Σ pen_i w_i is mapped back to the flat-road deflection with the same volume
 *   (exact piecewise-linear inversion), so N = Kz δ_eff on a flat road and features shorter than the footprint are
 *   enveloped. Kz from Rhyne (2005): Kz[kgf/mm] = 0.00028 p[kPa] sqrt(Wcp[mm] D[mm]) + 3.45.
 *   Footprint by membrane equilibrium: the deepest overlap cells, down to a ring displacement chosen so that the
 *   contact area = Kz δ / (κ p); pressure in it ∝ overlap - ring. Load point Pc and normal n = pressure-weighted
 *   means of the hit points / surface normals; half length from the pressure second moment (parabola: Var = a²/5).
 *   N = max(0, Kz δ + cz δ') at Pc along n; rim pinch k_rim (δn,max - bottom) + c_rim δ' as a second row at the
 *   deepest flange point along its own surface normal (TARMAC: summed into N along n; see phaseA).
 * Tangential (physics.js):
 *   brush (carcass) deflection b, F = K b_new + c_d s_new, b_new = (b + h s)/(1 + h |vx| / σ),
 *   σ = σ_ref g, g = max(0.05, 2ρ/(1+ρ)), ρ = A a / (A a)_ref: tread brush in series with the carcass.
 *   s = slip velocity (Re Ω - vx, camber slip - vy), camber slip = |vx| a/(3 R0) sin γ.
 *   μ = μ0 grip (fref / max(0.15 fref, N))^ls, decayed toward floor·μ with sliding speed beyond the brush full-slide
 *   onset; |F| ≤ μ N; plastic return map of b onto |K b| ≤ μ N. Belt shift = b (moves the ray origins), carcass roll.
 */
#include "dc_tyre.h"
#include "dc_rgrid.h"

#define TM_FLANGE 0.0175       /* ETRTO J-flange height (TARMAC FLANGE) */
#define TM_EPS_RING 0.0004     /* m: cell ramp-in width of the footprint threshold (TARMAC) */
#define TM_NC (DC_TM_NR * DC_TM_NL)

static double tm_rhyne(const dc_tm_tab* B, double p_pa) {
    double pk = p_pa > 0 ? p_pa / 1000.0 : 0;
    return (0.00028 * pk * sqrt(B->tread * 1000.0 * 2 * B->R0 * 1000.0) + 3.45) * 9.81 * 1000.0;
}
static double tm_kz(const dc_tm_tab* B, const dc_tyre_params* T, double p) {
    return B->kz_mode > 0.5 ? dc_tyre_kz(T, p) : tm_rhyne(B, p);
}
static double tm_invert(const dc_tm_tab* B, double V) {
    if (V <= 0) return 0;
    int n = DC_TM_NR, lo = 0, hi = n - 1;
    if (V >= B->bpV[n - 1]) return (V + B->bpT[n - 1]) / B->bpS[n - 1];
    while (hi - lo > 1) { int mid = (lo + hi) >> 1; if (B->bpV[mid] <= V) lo = mid; else hi = mid; }
    return (V + B->bpT[lo]) / B->bpS[lo];
}

/* inverse of the circular-segment area (radius R) -> depth; Newton (same as dc_tyre.c invert_area) */
static double tm_seg_area(double d, double R) {
    if (d <= 0) return 0; if (d >= 2 * R) return M_PI * R * R;
    return R * R * acos(1 - d / R) - (R - d) * sqrt(2 * R * d - d * d);
}
static double tm_invert_area(double A, double R) {
    if (A <= 0) return 0;
    double d = pow(3.0 * A / (4.0 * sqrt(2.0 * R)), 2.0 / 3.0);
    for (int k = 0; k < 6; k++) {
        double f = tm_seg_area(d, R) - A, df = 2 * sqrt(fmax(2 * R * d - d * d, 1e-12));
        double st = f / df; d -= st; if (d < 0) d = 1e-9; if (d > 2 * R) d = 2 * R;
        if (fabs(st) < 1e-13) break;
    }
    return d;
}

typedef struct { double pen, area, len, lf, q; v3 P, n; double g; int lane; } tm_cell;

/* smoothed footprint area at ring threshold tau */
static double tm_area_at(const tm_cell* C, int nc, double tau) {
    double A = 0;
    for (int k = 0; k < nc; k++) { double x = (C[k].pen - tau) / TM_EPS_RING + 0.5; A += x >= 1 ? C[k].area : (x > 0 ? C[k].area * x : 0); }
    return A;
}
/* ring threshold for a target area (TARMAC: 34 bisection steps) */
static double tm_ring(const tm_cell* C, int nc, double dmax, double target) {
    if (!(tm_area_at(C, nc, 0) > target)) return 0;
    double lo = 0, hi = dmax + TM_EPS_RING;
    for (int it = 0; it < 34; it++) { double mid = 0.5 * (lo + hi); if (tm_area_at(C, nc, mid) > target) lo = mid; else hi = mid; }
    return 0.5 * (lo + hi);
}

/* flat-road footprint of an upright wheel at deflection delta (reference patch, tests) */
static void tm_flat(const dc_tm_tab* B, double delta, double Kz, double p, double* area, double* half) {
    tm_cell C[TM_NC]; int nc = 0; double dmax = 0;
    for (int j = 0; j < B->nl; j++) for (int i = 0; i < DC_TM_NR; i++) {
        double t = (B->R0 - delta) / B->cth[i], pen = B->R0 - t;
        if (pen <= 0) continue;
        tm_cell* c = &C[nc++]; c->pen = pen; c->area = B->w[i] * B->cth[i] * B->laneW; c->lf = t * B->sth[i]; c->len = c->area / B->laneW;
        if (pen > dmax) dmax = pen;
    }
    double target = p > 1000 ? Kz * delta / (B->kappa * p) : 1e30;
    double ring = tm_ring(C, nc, dmax, target), sw = 0, m1 = 0, m2 = 0;
    for (int k = 0; k < nc; k++) { double q = C[k].pen - ring; if (q <= 0) continue; double wt = q * C[k].area; sw += wt; m1 += wt * C[k].lf; m2 += wt * (C[k].lf * C[k].lf + C[k].len * C[k].len / 12); }
    *area = fmin(target, tm_area_at(C, nc, 0));
    *half = sw > 0 ? sqrt(fmax(0, 5 * (m2 / sw - (m1 / sw) * (m1 / sw)))) : 0;
}

void dc_tyre_tm_init(dc_car* c) {
    double cornerload = c->P.mass * c->P.gravity / 4;
    for (int ax = 0; ax < 2; ax++) {
        const dc_tyre_params* T = &c->P.axle[ax].tyre;
        dc_tm_tab* B = &c->tm[ax];
        memset(B, 0, sizeof *B);
        B->R0 = T->radius;
        /* rim_radius is the flange (rim outer) radius in this code base: bead seat = rim_radius - flange */
        B->H = T->radius - (T->rim_radius - TM_FLANGE);
        double W = T->width, AR = B->H / W;
        B->tread = (1.03 - 0.4 * AR) * W;               /* TARMAC contactWidth: (1.03 - 0.004 AR[%]) W */
        B->bottom = B->H - TM_FLANGE - (T->rim_pinch > 0 ? T->rim_pinch : 0.012);
        B->nl = T->tm_lanes > 0 ? (T->tm_lanes > DC_TM_NL ? DC_TM_NL : T->tm_lanes) : 7;
        B->laneW = B->tread / B->nl;
        for (int j = 0; j < B->nl; j++) B->lane[j] = (j + 0.5) * B->laneW - B->tread / 2;
        double th[DC_TM_NR]; int n = 0;
        for (int a = -65; a < -35; a += 4) th[n++] = a;
        for (int k = 0; k <= 44; k++) th[n++] = -33 + 1.5 * k;
        for (int a = 37; a <= 65; a += 4) th[n++] = a;
        for (int i = 0; i < n; i++) { th[i] *= M_PI / 180.0; B->cth[i] = cos(th[i]); B->sth[i] = sin(th[i]); }
        for (int i = 0; i < n; i++) {
            double ta = th[i > 0 ? i - 1 : 0], tb = th[i < n - 1 ? i + 1 : n - 1];
            B->w[i] = B->R0 * (tb - ta) / ((i == 0 || i == n - 1) ? 1 : 2);
        }
        /* flat-volume breakpoints, sorted by d (insertion sort; deterministic) */
        double d[DC_TM_NR], k_[DC_TM_NR];
        for (int i = 0; i < n; i++) { d[i] = B->R0 * (1 - B->cth[i]); k_[i] = B->w[i] / B->cth[i]; }
        for (int i = 1; i < n; i++) for (int j = i; j > 0 && d[j] < d[j - 1]; j--) {
            double x = d[j]; d[j] = d[j - 1]; d[j - 1] = x; x = k_[j]; k_[j] = k_[j - 1]; k_[j - 1] = x;
        }
        double S = 0, Tt = 0;
        for (int i = 0; i < n; i++) { S += k_[i]; Tt += k_[i] * d[i]; B->bpD[i] = d[i]; B->bpS[i] = S; B->bpT[i] = Tt; }
        for (int i = 0; i < n; i++) B->bpV[i] = i == 0 ? 0 : B->bpS[i - 1] * B->bpD[i] - B->bpT[i - 1];
        /* parameters (defaults documented in drivechain.h) */
        B->kz_mode = T->tm_kz_mode;
        B->shape = T->tm_shape;
        B->kappa = T->tm_contact_ratio > 0 ? T->tm_contact_ratio : 0.8;
        B->roll = T->tm_carcass_roll > 0 ? T->tm_carcass_roll : 0.6;
        B->cz = T->tm_cz > 0 ? T->tm_cz : 300;
        B->fref = T->tm_fz_ref > 0 ? T->tm_fz_ref : cornerload;
        B->kx = T->tm_kx > 0 ? T->tm_kx : T->carcass_kx;
        B->ky = T->tm_ky > 0 ? T->tm_ky : T->carcass_ky;
        double Ca0 = 3 * T->mu_y * T->fz0 / tan(T->peak_slip_deg * M_PI / 180.0);
        double Cref = Ca0 * pow(B->fref / T->fz0, T->ls_exp_y);   /* envelope brush cornering stiffness at fref (C ~ a^2 N^(ls-1), a^2 ~ N) */
        B->sy0 = T->tm_relax_y > 0 ? T->tm_relax_y : Cref / B->ky;
        B->sx0 = T->tm_relax_x > 0 ? T->tm_relax_x : T->cx_mult * Cref / B->kx;
        B->cd = T->tm_damping > 0 ? T->tm_damping : 420;
        B->ls = T->tm_load_sens > 0 ? T->tm_load_sens : 1 - T->ls_exp_y;
        B->mu0 = T->tm_mu > 0 ? T->tm_mu : T->mu_y * pow(B->fref / T->fz0, T->ls_exp_y - 1);
        B->floor_ = T->tm_slide_floor > 0 ? T->tm_slide_floor : 0.7;
        B->vslide = T->tm_slide_speed > 0 ? T->tm_slide_speed : 5.0;
        B->crr = T->tm_crr > 0 ? T->tm_crr : 0.012;
        B->krim = T->tm_k_rim > 0 ? T->tm_k_rim : 1.2e6;
        B->crim = T->tm_c_rim > 0 ? T->tm_c_rim : 1500;
        double Kz = tm_kz(B, T, T->pressure), area, half;
        tm_flat(B, B->fref / Kz, Kz, T->pressure, &area, &half);
        B->patch0 = area * half > 1e-12 ? area * half : 1e-6;
        /* tread brush stiffness per unit area: C_t = c_p A a (brush theory, C_t = 2 c_p w a^2), = 2 C_ref at fref
           (tread in series with an equal carcass term) -> c_p = 2 K_y σ_y / (A a)_ref */
        B->cp = 2 * B->ky * B->sy0 / B->patch0;
    }
}

/* First crossing of o + t d, t in [t0, tmax], with the bilinear road surface of the memo grid (exact per grid cell:
   the ray is walked cell by cell, f(t) = z(t) - H(t) is a quadratic inside a cell). Returns t or -1. */
static double tm_ray(rgrid* G, const dc_car* c, v3 o, v3 d, double t0, double tmax, v3* n, double* grip) {
    const double dw = G->dw, idw = G->idw;
    double t = t0;
    double px = o.x + t * d.x, py = o.y + t * d.y;
    int i = ifloor(px * idw), j = ifloor(py * idw);
    const int sx = d.x > 0 ? 1 : (d.x < 0 ? -1 : 0), sy = d.y > 0 ? 1 : (d.y < 0 ? -1 : 0);
    for (int guard = 0; guard < 256 && t <= tmax; guard++) {
        double tnx = sx ? (((sx > 0 ? i + 1 : i) * dw) - o.x) / d.x : 1e30;
        double tny = sy ? (((sy > 0 ? j + 1 : j) * dw) - o.y) / d.y : 1e30;
        double t1 = fmin(fmin(tnx, tny), tmax);
        if (t1 < t) t1 = t;
        double g00, g10, g01, g11;
        double h00 = grid_node(G, c, i, j, &g00), h10 = grid_node(G, c, i + 1, j, &g10);
        double h01 = grid_node(G, c, i, j + 1, &g01), h11 = grid_node(G, c, i + 1, j + 1, &g11);
        double A = h00, Bc = h10 - h00, Cc = h01 - h00, Dc = h11 - h10 - h01 + h00;
        double u0 = o.x * idw - i, v0 = o.y * idw - j, du = d.x * idw, dv = d.y * idw;
        double c0 = o.z - (A + Bc * u0 + Cc * v0 + Dc * u0 * v0);
        double c1 = d.z - (Bc * du + Cc * dv + Dc * (u0 * dv + v0 * du));
        double c2 = -Dc * du * dv;
#define FQ(tt) (c0 + (tt) * (c1 + (tt) * c2))
        double fa = FQ(t), fb = FQ(t1), ta = t, tb = t1;
        int hit = 0;
        if (fa <= 0) { tb = ta; hit = 1; }
        else if (fb <= 0) hit = 1;
        else if (c2 > 0) {                         /* convex: may dip below inside the cell */
            double tv = -c1 / (2 * c2);
            if (tv > ta && tv < tb && FQ(tv) <= 0) { tb = tv; hit = 1; }
        }
        if (hit) {
            double r = tb;
            if (tb > ta) {
                /* safeguarded Newton on the bracket [ta, tb] (f(ta) > 0 >= f(tb), single root) */
                double lo = ta, hi = tb; r = ta + (tb - ta) * fa / (fa - FQ(tb));
                for (int it = 0; it < 30; it++) {
                    double fr = FQ(r), dfr = c1 + 2 * c2 * r;
                    if (fr > 0) lo = r; else hi = r;
                    if (fabs(fr) < 1e-12 || hi - lo < 1e-12) break;
                    double rn = dfr != 0 ? r - fr / dfr : 0.5 * (lo + hi);
                    r = (rn > lo && rn < hi) ? rn : 0.5 * (lo + hi);
                }
            }
            double u = u0 + r * du, v = v0 + r * dv;
            double Hx = (Bc + Dc * v) * idw, Hy = (Cc + Dc * u) * idw;
            *n = vnorm(V3(-Hx, -Hy, 1));
            *grip = (1 - v) * (g00 + u * (g10 - g00)) + v * (g01 + u * (g11 - g01));
            return r;
        }
#undef FQ
        if (t1 >= tmax) break;
        t = t1;
        if (tnx <= tny) i += sx; else j += sy;
        if (tnx == tny && sx && sy) j += sy;   /* exact corner crossing */
    }
    return -1;
}

void dc_tyre_tm_phaseA(dc_car* c, int i, v3 cw, v3 a, v3 vc, v3 wc, double h, dc_tc* t) {
    const dc_tyre_params* T = &c->P.axle[i / 2].tyre;
    const dc_tm_tab* B = &c->tm[i / 2];
    dc_tyre_state* S = &c->ty[i];
    memset(t, 0, sizeof *t);
    const double R0 = B->R0, dw = c->P.road_resolve > 1e-4 ? c->P.road_resolve : 0.025;
    const int nl = B->nl, nodamp = c->dbg & DC_DBG_NO_DAMPING;
    const double p = S->pressure, Kz = tm_kz(B, T, p);
    v3 up = V3(0, 0, 1);
    v3 dn = vnorm(vadd(vneg(up), vscl(a, vdot(up, a))));
    v3 fw = vcross(dn, a);
    v3 wperp = vsub(wc, vscl(a, vdot(wc, a)));
    v3 sL = V3(a.x, a.y, 0); { double l = vlen(sL); sL = l > 1e-9 ? vscl(sL, 1 / l) : a; }   /* ground side (vehicle left) */
    t->nrows = 1; t->arm_x = -R0; t->Re = R0;
    /* belt shift (= brush deflection) and carcass roll: TARMAC uses s = driver-right, shiftY = b_right, so in our
       left-handed lateral state ey: origin offset = sL ey, roll = -carcassRoll ey / H, e = -(y (a.s_R)) roll */
    const double by = S->ey, bx = S->ex;
    const double roll = -B->roll * by / B->H, asR = -vdot(a, sL);
    rgrid GR; grid_init(&GR, cw.x, cw.y, dw);
    /* exact cull bound: highest grid node under the wheel's lower half */
    double hmax = -1e30;
    {
        double bx0 = 1e30, bx1 = -1e30, by0 = 1e30, by1 = -1e30, gd;
        double ext = R0 + fabs(bx) + 0.02, wy = 0.5 * B->tread + fabs(by) + 0.02;
        for (int cy = 0; cy < 2; cy++) for (int cx = 0; cx < 2; cx++) for (int cs = 0; cs < 2; cs++) {
            v3 pb = vadd(vmad(vmad(cw, a, cy ? wy : -wy), fw, cx ? ext : -ext), vscl(dn, cs ? R0 : 0.0));
            bx0 = fmin(bx0, pb.x); bx1 = fmax(bx1, pb.x); by0 = fmin(by0, pb.y); by1 = fmax(by1, pb.y);
        }
        int i0 = ifloor(bx0 / dw), i1 = ifloor(bx1 / dw) + 1, j0 = ifloor(by0 / dw), j1 = ifloor(by1 / dw) + 1;
        for (int ii = i0; ii <= i1; ii++) for (int jj = j0; jj <= j1; jj++) { double hn = grid_node(&GR, c, ii, jj, &gd); if (hn > hmax) hmax = hn; }
    }
    tm_cell C[TM_NC]; int nc = 0;
    double Vsum = 0, dmax = 0; int nfill = 0;
    v3 dir[DC_TM_NR];
    for (int k = 0; k < DC_TM_NR; k++) dir[k] = vadd(vscl(dn, B->cth[k]), vscl(fw, B->sth[k]));
    for (int j = 0; j < nl; j++) {
        double y = B->lane[j], e = -(y * asR) * roll;
        v3 o = vadd(vadd(vmad(cw, a, y), vscl(sL, by)), vadd(vscl(fw, bx), vscl(dn, e)));
        for (int k = 0; k < DC_TM_NR; k++) {
            v3 d = dir[k];
            if (d.z >= 0) continue;
            double zR = o.z + R0 * d.z;           /* lowest point of the ray inside the tread circle */
            if (zR >= hmax) continue;              /* every road node is below it: no hit with t < R0 */
            double ts = o.z > hmax ? (o.z - hmax) / (-d.z) : 0;
            v3 nh; double gg;
            double th = tm_ray(&GR, c, o, d, ts, R0, &nh, &gg);
            if (th < 0) continue;
            double pen = R0 - th;
            if (pen <= 0) continue;
            if (th <= 0 && ts <= 0) {
                /* ray origin inside the solid (a face taller than the wheel centre beside the lane: the lane is in a
                   wall). The ray reads its full length whatever the depth, so it has no depth gradient: it is no tread
                   contact and carries no load (TARMAC's lanes are tread columns; a sidewall jammed against a wall is
                   outside the model and left to the flange row of the lanes still outside and to body contact).
                   Kept in, its top-surface normal pointed a full-radius flange pinch straight up (1.3 m wall at 10 deg:
                   car launched, +155 % KE) and its volume read as a 0.25 m vertical squash along the face normal. */
                nfill++;
                continue;
            }
            tm_cell* cl = &C[nc++];
            cl->pen = pen; cl->area = B->w[k] * fabs(d.z) * B->laneW; cl->len = cl->area / B->laneW;
            cl->P = vmad(o, d, th); cl->n = nh; cl->g = gg; cl->lane = j;
            Vsum += pen * B->w[k];
            if (pen > dmax) dmax = pen;
        }
    }
    (void)nfill;
    if (nc == 0) goto airborne;
    double delta = tm_invert(B, Vsum / nl);
    double Ns = Kz * delta;
    double target = p > 1000 ? Ns / (B->kappa * p) : 1e30;
    double ring = tm_ring(C, nc, dmax, target);
    double sw = 0, m1 = 0, m2 = 0, gsum = 0; v3 Pc = V3(0, 0, 0), nsum = V3(0, 0, 0);
    double fs = vdot(fw, cw);
    for (int k = 0; k < nc; k++) {
        tm_cell* cl = &C[k]; double q = cl->pen - ring; cl->q = q > 0 ? q : 0;
        if (q <= 0) continue;
        double wt = q * cl->area; sw += wt;
        Pc = vmad(Pc, cl->P, wt); nsum = vmad(nsum, cl->n, wt); gsum += wt * cl->g;
        double lf = vdot(fw, cl->P) - fs;
        m1 += wt * lf; m2 += wt * (lf * lf + cl->len * cl->len / 12);
    }
    if (sw <= 0) goto airborne;
    Pc = vscl(Pc, 1 / sw);
    v3 n = vnorm(nsum);
    double half = sqrt(fmax(0, 5 * (m2 / sw - (m1 / sw) * (m1 / sw))));
    double areaS = fmin(target, tm_area_at(C, nc, 0));
    /* normal load (TARMAC substep): carcass + rim-flange pinch, along n at Pc */
    v3 vp = vadd(vc, vcross(wperp, vsub(Pc, cw)));
    double rate = -vdot(vp, n);
    /* rim-flange pinch. TARMAC: k_rim (deltaMax - bottom) + c_rim rate, summed into N along the footprint normal,
       i.e. the flange circle (radius Rf = R0 - bottom) bottoming through the crushed casing. Here the same circle is a
       conservative column potential, exactly the envelope tyre's rim construction (dc_tyre.c): per lane, vertical
       columns at wheel-fixed stations; overlap area Ar -> flange depth e = seg^-1(Ar, Rf); U = k_rim/nl e^2/2; force =
       its exact gradient (stiffness) + Hunt-Crossley damping along it, as a separate row. On a flat road e = deltaMax -
       bottom and the force is k_rim e along the normal: TARMAC's value. Elsewhere (step riser, wall) the deepest-ray
       form has no potential: its line of action and its slant depth switch with the ray (face -> top edge), which
       pumped energy into wall strikes (crash suite +155 % KE at 1.3 m, +68 % at 0.3 m). A face taller than the flange
       fills the chord and adds no more (column cap), as in the envelope. */
    double Frim = 0, pinch = 0; v3 Pr = Pc, nr = n;
    {
        const double Rf = R0 - B->bottom, dsr = fmin(0.004, dw / 6.0), e = 0.003;
        const int ns = 2 * (int)ceil(Rf / dsr), nsr = ns > 1024 ? 1024 : ns;
        const double ds = 2 * Rf / nsr;
        v3 Fs = V3(0, 0, 0), Ms = V3(0, 0, 0); double U = 0;
        hcell HC = {INT32_MIN, INT32_MIN, 0, 0, 0, 0, 0, 0, 0, 0};
        if (cw.z + Rf * dn.z < hmax)                      /* lowest flange point below the highest node, else no touch */
        for (int j = 0; j < nl; j++) {
            v3 o = vmad(cw, a, B->lane[j]);               /* the rim is rigid: no belt shift / roll */
            double Ar = 0; v3 Gr = V3(0, 0, 0), Mr = V3(0, 0, 0);
            for (int k = 0; k < nsr; k++) {
                double x = -Rf + (k + 0.5) * ds, Sr = sqrt(fmax(Rf * Rf - x * x, 0));
                v3 off = vadd(vscl(fw, x), vscl(dn, Sr)), pb = vadd(o, off);
                if (pb.z >= hmax) continue;
                double Hx, Hy, gg, H = grid_hc(&GR, &HC, c, pb.x, pb.y, &Hx, &Hy, &gg);
                double u = H - pb.z, umax = -2 * Sr * dn.z, ph, dph;
                if (u <= 0) continue;
                if (u >= umax) { u = umax; ph = umax < e ? umax * umax / (2 * e) : umax - e / 2; dph = 0; }
                else if (u < e) { ph = u * u / (2 * e); dph = u / e; } else { ph = u - e / 2; dph = 1; }
                v3 fi = vscl(V3(Hx, Hy, -1), -dph * ds);
                Ar += ph * ds; Gr = vadd(Gr, fi); Mr = vadd(Mr, vcross(off, fi));
            }
            if (Ar <= 0) continue;
            double el = tm_invert_area(Ar, Rf), chr = 2 * sqrt(fmax(2 * Rf * el - el * el, 1e-10));
            double sc = B->krim / nl * el / chr;
            Fs = vmad(Fs, Gr, sc);
            Ms = vadd(Ms, vadd(vcross(vsub(o, cw), vscl(Gr, sc)), vscl(Mr, sc)));
            U += 0.5 * B->krim / nl * el * el; if (el > pinch) pinch = el;
        }
        double Fl = vlen(Fs);
        if (Fl > 1e-9 && vdot(Fs, n) > 0) {
            nr = vscl(Fs, 1 / Fl);
            /* a point on the line of action: the stiffness force's moment about the wheel centre is Ms */
            Pr = vadd(cw, vscl(vcross(Fs, Ms), 1 / (Fl * Fl)));
            Pr = vmad(Pr, nr, vdot(vsub(Pc, Pr), nr));     /* slid along the line to the footprint's level */
            double rate_r = -vdot(vadd(vc, vcross(wperp, vsub(Pr, cw))), nr);
            Frim = fmax(0, Fl + (nodamp ? 0 : B->crim * fmin(pinch / 0.004, 1.0) * rate_r));
            t->pe += U;
        }
    }
    double Nel = fmax(0, Ns + (nodamp ? 0 : B->cz * rate));
    double N = Nel + Frim;
    t->pe += 0.5 * Kz * delta * delta;
    if (N <= 1e-9) goto airborne;
    t->contact = 1;
    t->p = Pc; t->n = n;
    t->xc = vnorm(vcross(a, n)); t->yc = vcross(n, t->xc);
    t->camber_l = -vdot(a, n);
    t->N[0] = Nel; t->nr[0] = n; t->orow[0] = Pc; t->pr[0] = Pc; t->Fz_t[0] = Nel; t->Frim[0] = 0;
    t->dl[0] = delta; t->dlr[0] = 0; t->a_r[0] = half;
    if (Frim > 0) {
        t->nrows = 2;
        t->N[1] = Frim; t->nr[1] = nr; t->orow[1] = Pr; t->pr[1] = Pr; t->Fz_t[1] = 0; t->Frim[1] = Frim;
        t->dl[1] = 0; t->dlr[1] = pinch; t->a_r[1] = half;
    }
    t->Ntot = N; t->Fz = Nel; t->Fr = Frim; t->delta_eff = delta; t->a_eff = half;
    t->grip = gsum / sw;
    if (c->dbg & DC_DBG_NO_FRICTION) t->grip = 0;
    S->delta_row[0] = delta;
    /* kinematics at Pc (non-spinning carrier) */
    t->Vcx = vdot(vp, t->xc); t->Vsy = vdot(vp, t->yc);
    t->Re = R0 - delta / 3.0;
    const double Om = S->omega, vx = t->Vcx, avx = fabs(vx);
    t->Vr = fabs(Om * t->Re);
    t->slip_ratio_ss = (Om * t->Re - vx) / fmax(avx, 0.5);
    t->slip_angle_ss = -atan2(t->Vsy, fmax(avx, 0.5));
    double csl = avx * half / (3 * R0) * t->camber_l;          /* camber slip velocity, toward +yc */
    double rho = areaS * half / B->patch0, g = fmax(0.05, 2 * rho / (1 + rho));
    double sigx = B->sx0 * g, sigy = B->sy0 * g;
    t->sig_x = sigx; t->sig_y = sigy; t->Ctot = B->ky * sigy; t->Cxtot = B->kx * sigx;
    double Dx = 1 + h * avx / sigx, Dy = 1 + h * avx / sigy;
    double mu = B->mu0 * t->grip * pow(B->fref / fmax(0.15 * B->fref, N), B->ls);
    double slx = t->Re * Om - vx, sly = csl - t->Vsy;
    {
        double Vs = hypot(slx, sly);
        if (Vs > 1e-6) {
            double N0 = mu * N;
            double fxo = fmax(0.05, avx * 3 * N0 / (B->kx * sigx)), fyo = fmax(0.05, avx * 3 * N0 / (B->ky * sigy));
            double sn = hypot(slx / fxo, sly / fyo), excess = sn > 1 ? Vs * (1 - 1 / sn) : 0;
            mu *= B->floor_ + (1 - B->floor_) * exp(-excess / B->vslide);
        }
    }
    t->mu_eff = mu;
    double cd = (t->grip > 0) ? B->cd : 0;
    /* brush: lateral explicit, longitudinal linearised in the wheel spin (implicit row in the solver) */
    double byn = (by + h * sly) / Dy;
    double ex_star = (bx + h * slx) / Dx;
    if (B->shape > 0.5 && t->grip > 0) {
        /* brush-saturation option (tyre_tm_shape = 1, not in TARMAC). TARMAC's deflection b obeys
           db/dt = s - |vx| b / sigma: the tread absorbs slip at the linear rate b/sigma = F/C. With a brush tread
           (Fiala 1954; Pacejka 2012 sec. 3.2) the tread slip that carries force F is F/C * phi(f), f = F/(mu N),
           phi = 3 theta / f, theta = 1 - (1 - f)^(1/3) (exact inverse of F = mu N (3 theta - 3 theta^2 + theta^3)):
           phi = 1 at small force, 3 at full sliding, so the curve rounds over and peaks at the brush full-slide slip
           3 mu N / C (the same point TARMAC's sliding-friction decay starts from). Fixed-point on phi (3 passes). */
        double cap0 = mu * N;
        for (int it = 0; it < 3; it++) {
            double f = cap0 > 0 ? hypot(B->kx * ex_star, B->ky * byn) / cap0 : 0;
            if (f > 0.999) f = 0.999;
            double phi = f > 1e-6 ? 3 * (1 - cbrt(1 - f)) / f : 1 + f / 3;
            Dx = 1 + h * avx * phi / sigx; Dy = 1 + h * avx * phi / sigy;
            byn = (by + h * sly) / Dy; ex_star = (bx + h * slx) / Dx;
        }
    }
    t->by_new = byn; t->ey_new = 0;   /* belt shift is in Pc already: no extra n x (ey Fx) moment */
    t->ex_star = ex_star;
    t->Fy = B->ky * byn + cd * sly;
    t->Fx_star = B->kx * ex_star + cd * slx;
    t->D = h * t->Re / Dx;
    t->kx = B->kx + cd * Dx / h;
    if (t->grip <= 0) { t->Fy = 0; t->Fx_star = 0; t->kx = 0; }
    double Fm = hypot(t->Fx_star, t->Fy), cap = mu * N;
    t->lam = cap > 0 ? Fm / cap : 0;
    /* ---- self-aligning moment and pneumatic trail from the footprint shear distribution (addition) ----
       Brush theory on the actual footprint: in each lane the tread deflection grows linearly from the leading edge,
       shear = min(c_p s ξ, μ p) per cell along the resultant's direction u; s is solved so that the shear integral
       equals the brush resultant |F| (the carcass deflection sets the force, the footprint sets where it acts).
       Mz = Σ (x τ_y - y τ_x) about Pc. Linear range: parabolic pressure -> trail a/3; full slide -> trail 0. */
    double Mz = 0, Mtmax = 0, Ip = 0;
    {
        double qsum = 0;
        for (int k = 0; k < nc; k++) qsum += C[k].q * C[k].area;
        double xl[DC_TM_NL], xr[DC_TM_NL];
        for (int j = 0; j < nl; j++) { xl[j] = -1e30; xr[j] = 1e30; }
        double xk[TM_NC], yk[TM_NC], mk[TM_NC], ak[TM_NC], xi[TM_NC];
        int m = 0;
        for (int k = 0; k < nc; k++) {
            if (C[k].q <= 0) continue;
            v3 r = vsub(C[k].P, Pc);
            xk[m] = vdot(r, t->xc); yk[m] = vdot(r, t->yc);
            mk[m] = cap * C[k].q * C[k].area / qsum; ak[m] = C[k].area;
            int j = C[k].lane;
            if (xk[m] + 0.5 * C[k].len > xl[j]) xl[j] = xk[m] + 0.5 * C[k].len;
            if (xk[m] - 0.5 * C[k].len < xr[j]) xr[j] = xk[m] - 0.5 * C[k].len;
            xi[m] = (double)j;   /* lane, resolved below */
            Ip += ak[m] * (xk[m] * xk[m] + yk[m] * yk[m]);
            Mtmax += mk[m] * sqrt(xk[m] * xk[m] + yk[m] * yk[m]);
            m++;
        }
        double wf = 0.5 * (1 + tanh(vx / 0.5));   /* leading edge: front when rolling forward, rear in reverse */
        for (int k = 0; k < m; k++) { int j = (int)xi[k]; xi[k] = wf * (xl[j] - xk[k]) + (1 - wf) * (xk[k] - xr[j]); if (xi[k] < 1e-6) xi[k] = 1e-6; }
        double Ft = fmin(Fm, cap);
        if (Ft > 1e-9 && m > 0) {
            double ux = t->Fx_star / Fm, uy = t->Fy / Fm;
            double shi = 0;
            for (int k = 0; k < m; k++) { double sk = mk[k] / (xi[k] * ak[k]); if (sk > shi) shi = sk; }
            double s = shi;
            if (Ft < cap * (1 - 1e-9)) {
                double lo = 0, hi = shi;
                for (int it = 0; it < 48; it++) {
                    double mid = 0.5 * (lo + hi), F = 0;
                    for (int k = 0; k < m; k++) F += fmin(mid * xi[k] * ak[k], mk[k]);
                    if (F < Ft) lo = mid; else hi = mid;
                }
                s = 0.5 * (lo + hi);
            }
            double F = 0;
            for (int k = 0; k < m; k++) { double tk = fmin(s * xi[k] * ak[k], mk[k]); F += tk; Mz += tk * (xk[k] * uy - yk[k] * ux); }
            if (F > 0) Mz *= Ft / F;   /* bisection residual */
        }
    }
    t->Mz_brush = Mz;
    if (fabs(t->Fy) > 1e-6) t->tp = -Mz / t->Fy;
    /* tread twist (parking / turn-slip moment, addition): torsional brush stiffness of the footprint, tread in series
       with an equal carcass share (as the translational split at fref); relaxes over the half length when rolling;
       bounded by the full-slide torsional friction moment, reduced by the translational usage */
    {
        double kt = 0.5 * B->cp * Ip;
        double wn = vdot(wc, n);
        t->phi_new = (S->phi - h * wn) / (1 + h * avx / fmax(half, 0.02));
        double Mmax = Mtmax * fmax(0.0, 1 - fmin(t->lam, 1.0)) + 1e-9;
        if (kt * fabs(t->phi_new) > Mmax) t->phi_new = sgn(t->phi_new) * Mmax / fmax(kt, 1e-9);
        if (t->grip <= 0) t->phi_new = 0;
        t->Mt = kt * t->phi_new;
        t->pe += 0.5 * kt * t->phi_new * t->phi_new;
    }
    /* rolling resistance (TARMAC: N crr Re as a resisting spin torque) */
    t->rr_e = nodamp ? 0 : B->crr * t->Re * tanh(Om * t->Re / 0.5);
    /* torques about the wheel centre except the Fx term. The normal load is a radial tread pressure resultant:
       it does not torque the spin axis (TARMAC applies it to the chassis/suspension only). */
    v3 tau = vcross(vsub(Pc, cw), vscl(n, Nel));
    if (Frim > 0) tau = vadd(tau, vcross(vsub(Pr, cw), vscl(nr, Frim)));
    tau = vsub(tau, vscl(a, vdot(tau, a)));   /* both are radial pressure resultants (tread / flange) */
    tau = vadd(tau, vcross(vsub(Pc, cw), vscl(t->yc, t->Fy)));
    tau = vadd(tau, vscl(n, t->Mz_brush + t->Mt));
    tau = vadd(tau, vscl(t->yc, -t->rr_e * N));
    t->tau = tau;
    t->Trest = vdot(tau, a);
    t->arm_x = vdot(a, vcross(vsub(Pc, cw), t->xc));
    return;
airborne:
    /* airborne: the free tread's shear deflection relaxes (hysteresis), as in the envelope model */
    t->contact = 0; t->pe = 0;
    S->ex /= (1 + h * 60); S->ey /= (1 + h * 60); S->phi /= (1 + h * 60);
    t->arm_x = -R0; t->Re = R0; t->kx = 0; t->D = 0;
}

void dc_tyre_tm_phaseB(dc_car* c, int i, v3 cw, v3 a, double omega_new, double h, dc_tc* t, int commit) {
    dc_tyre_state* S = &c->ty[i];
    const dc_tm_tab* B = &c->tm[i / 2];
    (void)h; (void)a;
    if (!t->contact) { t->F = V3(0, 0, 0); t->tau = V3(0, 0, 0); t->Fx = 0; return; }
    double dO = omega_new - S->omega;
    double ex = t->ex_star + t->D * dO;
    double Fx = t->Fx_star + t->kx * t->D * dO, Fy = t->Fy;
    double cap = t->mu_eff * t->Ntot, Fm = hypot(Fx, Fy);
    if (Fm > cap && Fm > 0) { Fx *= cap / Fm; Fy *= cap / Fm; }
    t->Fx = Fx;
    if (Fy != t->Fy) { t->tau = vadd(t->tau, vcross(vsub(t->p, cw), vscl(t->yc, Fy - t->Fy))); t->Fy = Fy; }
    t->F = vadd(vadd(vscl(t->xc, Fx), vscl(t->yc, Fy)), vscl(t->nr[0], t->N[0]));
    if (t->nrows > 1) t->F = vadd(t->F, vscl(t->nr[1], t->N[1]));
    t->tau = vadd(t->tau, vcross(vsub(t->p, cw), vscl(t->xc, Fx)));
    if (commit) {
        double ey = t->by_new;
        if (t->grip <= 0) { ex = 0; ey = 0; }
        /* plastic return map: no unlimited stored shear while sliding */
        double em = hypot(B->kx * ex, B->ky * ey);
        if (em > cap && em > 0) { ex *= cap / em; ey *= cap / em; }
        S->ex = ex; S->ey = ey; S->phi = t->phi_new;
        t->pe += 0.5 * B->kx * ex * ex + 0.5 * B->ky * ey * ey;
    }
}

/* test hook: flat-road reference footprint of axle ax at load Fz (area m^2, half length m, deflection m) */
void dc_tyre_tm_flat(const dc_car* c, int ax, double Fz, double* area, double* half, double* delta) {
    const dc_tm_tab* B = &c->tm[ax & 1];
    const dc_tyre_params* T = &c->P.axle[ax & 1].tyre;
    double Kz = tm_kz(B, T, T->pressure), d = Fz / Kz;
    tm_flat(B, d, Kz, T->pressure, area, half); *delta = d;
}
