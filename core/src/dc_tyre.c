#include <string.h>
#include <stdint.h>
/* dc_tyre.c - rubber-patch "wheel collider" and brush tyre.
 *
 * Envelope:  n_rows strips across the tread width, each a fan of n_rays rays (±span) in the wheel plane.
 *   Polar penetration area of a strip  A = Σ ½ (R² - t_j²)⁺ Δβ   (exact for the region between circle and road)
 *   Equivalent flat penetration δ from the circular-segment area  A(δ) = R² acos(1-δ/R) - (R-δ)√(2Rδ-δ²)
 *   → kerb edges grow the area continuously: no load spike when an edge enters the circle.
 *   Effective point/normal: penetration-weighted averages of ray hits / radial directions (−ray dir);
 *   on a plane this equals the plane normal, on a step face it tilts back (longitudinal impact force).
 *   Radial force  Fz = k(p)/n · δ (1 + prog (δ/δ_rim)²) + c/n · δ̇,   δ̇ = -(v_point · n)  (no finite differences)
 *   Rim:  same rays against radius R_rim + pinch, stiffness k_rim (bottoming), adds to friction normal load.
 * Brush (Fiala, parabolic pressure), per strip, combined slip:
 *   λ = |(Cx sx, Cy sy)| / (3 μ N);  F = μN[1-(1-λ)³] (λ<1) else μN;   t_p = a λ(1-λ)³ / (1-(1-λ)³)
 *   Cy(strip) = Cα0/n · (a/a_ref)² · (nN/Fz0)^(ls-1)     (a = patch half length = c_a √(2Rδ))
 * Transient: carcass deflection states (ex, ey) with relaxation by rolling speed Vr = |Ω Re|:
 *   e' = (e - h Vs) / (1 + h Vr/σ),  σ = C/k_carcass,  s = e/σ  (s → theoretical slip in steady state)
 *   Longitudinal update is implicit in the wheel spin Ω (linearised) - see dc_car.c.
 */
#include "dc_tyre.h"
#include "dc_rgrid.h"

#define HC_REF 0.004    /* m: indentation at which carcass damping reaches its full value */
#define COL_MAX 1024
#define COL_DS 0.004  /* m: wheel-fixed integration station pitch (upper bound; also <= road_resolve/6, see below) */
#define LANE_PITCH 0.02 /* m: lateral pitch of the contact lanes (normal-force integration across the width); upper bound, also <= road_resolve */
#define COL_EPS 0.003   /* m: C1 entry window of a column's penetration (no force kink at first touch) */

static double seg_area(double d, double R) {
    if (d <= 0) return 0; if (d >= 2 * R) return M_PI * R * R;
    return R * R * acos(1 - d / R) - (R - d) * sqrt(2 * R * d - d * d);
}
static double invert_area(double A, double R) {
    if (A <= 0) return 0;
    double d = pow(3.0 * A / (4.0 * sqrt(2.0 * R)), 2.0 / 3.0);
    for (int k = 0; k < 4; k++) {
        double f = seg_area(d, R) - A, df = 2 * sqrt(fmax(2 * R * d - d * d, 1e-12));
        double st = f / df; d -= st; if (d < 0) d = 1e-9; if (d > 2 * R) d = 2 * R;
        if (fabs(st) < 1e-13) break;      /* converged (quadratic): further steps change nothing at double precision */
    }
    return d;
}

/* ray (o + t d), t in [0,R]: first road crossing. returns t (R if none), normal and grip at hit */
static double ray_hit(const dc_car* c, v3 o, v3 d, double R, v3* n, double* grip) {
    double g;
    double h0 = dc_road_eval(c, o.x, o.y, n, &g);
    double f0 = o.z - h0;
    if (f0 <= 0) { *grip = g; return 0; }
    v3 pe = vmad(o, d, R);
    double f1 = pe.z - dc_road_eval(c, pe.x, pe.y, n, &g);
    if (f1 >= 0) { *grip = g; return R; }
    double a = 0, b = R, fa = f0, fb = f1; int side = 0; double t = R, ft = 0;
    for (int it = 0; it < 40; it++) {
        t = (a * fb - b * fa) / (fb - fa);
        v3 p = vmad(o, d, t);
        ft = p.z - dc_road_eval(c, p.x, p.y, n, &g);
        if (fabs(ft) < 1e-7 || b - a < 2e-6) break;
        if (ft * fb > 0) { b = t; fb = ft; if (side == -1) fa *= 0.5; side = -1; }
        else { a = t; fa = ft; if (side == 1) fb *= 0.5; side = 1; }
    }
    *grip = g;
    /* jump discontinuity in the heightfield (kerb/plank step face): the bracket collapses onto the jump
       while the residual stays finite (a genuine surface root drives it to ~0). There the surface is the
       vertical step face: use its normal, facing the ray. */
    if (fabs(ft) > 2e-4 && fmin(fabs(fa), fabs(fb)) > 1e-4) {
        double hl = sqrt(d.x * d.x + d.y * d.y);
        if (hl > 1e-3) *n = V3(-d.x / hl, -d.y / hl, 0);
    }
    return t;
}

typedef struct { double Cx, Cy, mu, N, a; } strip;

static void brush(const strip* s, int n, double sx, double sy, double* Fx, double* Fy, double* Mz, double* lam_max) {
    *Fx = *Fy = *Mz = 0; *lam_max = 0;
    for (int r = 0; r < n; r++) {
        if (s[r].N <= 0) continue;
        double tx = s[r].Cx * sx, ty = s[r].Cy * sy, th = sqrt(tx * tx + ty * ty);
        if (th < 1e-12) continue;
        double cap = 3 * s[r].mu * s[r].N, lam = th / cap, F, tp;
        if (lam < 1) {
            double m = 1 - lam, m3 = m * m * m;
            F = th * (1 - lam + lam * lam / 3.0);
            tp = lam > 1e-6 ? s[r].a * lam * m3 / (1 - m3) : s[r].a / 3.0;
        } else { F = s[r].mu * s[r].N; tp = 0; }
        if (lam > *lam_max) *lam_max = lam;
        double fx = F * tx / th, fy = F * ty / th;
        *Fx += fx; *Fy += fy; *Mz -= tp * fy;
    }
}

void dc_tyre_phaseA(dc_car* c, int i, v3 cw, v3 a, v3 vc, v3 wc, double h, dc_tc* t) {
    const dc_tyre_params* T = &c->P.axle[i / 2].tyre;
    if (T->model == DC_TYRE_TARMAC) { dc_tyre_tm_phaseA(c, i, cw, a, vc, wc, h, t); return; }
    dc_tyre_state* S = &c->ty[i];
    memset(t, 0, sizeof *t);
    const double R = T->radius, Rr = T->rim_radius + T->rim_pinch, w = T->width;
    const int nrows = T->n_rows < 1 ? 1 : (T->n_rows > DC_MAX_ROWS ? DC_MAX_ROWS : T->n_rows);
    const int single = T->n_rays == 1;   /* classic single-ray "WheelCollider" baseline (comparison only) */
    const double span = T->ray_span_deg * M_PI / 180.0, dw = c->P.road_resolve > 1e-4 ? c->P.road_resolve : 0.025;
    double p = S->pressure;
    double kz = dc_tyre_kz(T, p);
    const double dlim = fmax(R - Rr, 0.01);
    /* sidewall progression is pressure-borne: a flat tyre folds onto the rim instead of stiffening */
    const double prog = 0.5 * fmin(fmax(p / T->pressure_ref, 0.0), 1.5);
    const int nodamp = c->dbg & DC_DBG_NO_DAMPING;
    v3 up = V3(0, 0, 1);
    v3 dn = vnorm(vadd(vneg(up), vscl(a, vdot(up, a))));
    v3 fw = vcross(dn, a);
    v3 wperp = vsub(wc, vscl(a, vdot(wc, a)));
    double gsum = 0, gw = 0;
    rgrid GR; if (!single) grid_init(&GR, cw.x, cw.y, dw);
    t->nrows = nrows;
    v3 nsum = V3(0, 0, 0), psum = V3(0, 0, 0);
    const int nlane = single ? 1 : (int)ceil(w / nrows / fmin(LANE_PITCH, dw) - 1e-9);
    const double nl = (double)nrows * nlane;
    /* wheel-fixed integration stations (lane-independent tables: station x, disc and rim half-chords, offset of the
       lowest disc point from the lane origin) */
    const double xmax = R * sin(span);
    int ns = 2 * (int)ceil(xmax / fmin(COL_DS, dw / 6.0));   /* stations must resolve the grid's face ramp, else a step face aliases between stations */
    if (ns > COL_MAX) ns = COL_MAX;
    const double ds = 2 * xmax / ns;                      /* station pitch (wheel-fixed) */
    const double x0s = -xmax + 0.5 * ds;
    double st_x[COL_MAX], st_S[COL_MAX], st_Sr[COL_MAX]; v3 st_off[COL_MAX];
    if (!single) for (int i = 0; i < ns; i++) {
        double x = x0s + i * ds; st_x[i] = x; st_S[i] = sqrt(fmax(R * R - x * x, 0));
        st_Sr[i] = fabs(x) < Rr ? sqrt(Rr * Rr - x * x) : -1;
        st_off[i] = vadd(vscl(fw, x), vscl(dn, st_S[i]));
    }
    /* culling blocks of 4 stations: corner offsets (x at the block ends, S at its min/max) of each block's disc-bottom footprint */
    v3 bl_off[COL_MAX / 4 + 2][4];
    if (!single) for (int bk = 0; bk < (ns + 3) / 4; bk++) {
        double xa = x0s + 4 * bk * ds, xb = x0s + fmin(4 * bk + 3, ns - 1) * ds;
        double Sa = sqrt(fmax(R * R - xa * xa, 0)), Sb = sqrt(fmax(R * R - xb * xb, 0));
        double Smax = (xa <= 0 && xb >= 0) ? R : fmax(Sa, Sb), Smin = fmin(Sa, Sb);
        for (int k = 0; k < 4; k++) bl_off[bk][k] = vadd(vscl(fw, (k & 2) ? xb : xa), vscl(dn, (k & 1) ? Smax : Smin));
    }
    for (int r = 0; r < nrows; r++) {
        /* the strip's normal force is integrated over nlane lateral lanes (pitch <= LANE_PITCH), each its own exact
           column potential; the brush strip receives their resultant (force, point, moment). Coarse strips alone
           alias a laterally sliding step face (kerb struck at a shallow angle): a whole strip jumps onto the kerb at once. */
        v3 orw = vmad(cw, a, ((r + 0.5) / nrows - 0.5) * w), Fv = V3(0, 0, 0), Tq = V3(0, 0, 0), Pw = V3(0, 0, 0);
        double sN = 0, fzs = 0, frs = 0, dlw = 0, dlrm = 0;
        t->N[r] = 0; t->orow[r] = orw; t->Mst[r] = V3(0, 0, 0);
        /* Stations (wheel-fixed) and exact culling, shared by the row's lanes. A block of 4 stations can touch the road
           only if the highest road node of the grid cells under its footprint (the bilinear H never exceeds its cell's
           corner nodes) reaches the lowest disc point of the block. Exact bound: a culled station has u <= 0. */
        unsigned char act[COL_MAX / 4 + 2];
        if (!single) {
            const double y0 = ((double)r / nrows - 0.5) * w, y1 = ((r + 1.0) / nrows - 0.5) * w;
            const int nb = (ns + 3) / 4;
            /* row-wide bound first (same exactness argument, coarser footprint): a block whose lowest point is above every
               road node under the whole row cannot touch; only the rest need their own footprint scan */
            double hrow = -1e30;
            {
                double bx0 = 1e30, bx1 = -1e30, by0 = 1e30, by1 = -1e30, gdum;
                for (int cy = 0; cy < 2; cy++) for (int cx = 0; cx < 2; cx++) for (int cs = 0; cs < 2; cs++) {
                    v3 pb = vadd(vmad(vmad(cw, a, cy ? y1 : y0), fw, cx ? xmax : -xmax), vscl(dn, cs ? R : 0.0));
                    bx0 = fmin(bx0, pb.x); bx1 = fmax(bx1, pb.x); by0 = fmin(by0, pb.y); by1 = fmax(by1, pb.y);
                }
                int i0 = ifloor(bx0 / dw), i1 = ifloor(bx1 / dw) + 1, j0 = ifloor(by0 / dw), j1 = ifloor(by1 / dw) + 1;
                for (int ii = i0; ii <= i1; ii++) for (int jj = j0; jj <= j1; jj++) { double hn = grid_node(&GR, c, ii, jj, &gdum); if (hn > hrow) hrow = hn; }
            }
            const v3 rb[2] = {vmad(cw, a, y0), vmad(cw, a, y1)};
            for (int bk = 0; bk < nb; bk++) {
                double bx0 = 1e30, bx1 = -1e30, by0 = 1e30, by1 = -1e30, zlo = 1e30;
                for (int cy = 0; cy < 2; cy++) for (int k = 0; k < 4; k++) {
                    v3 pb = vadd(rb[cy], bl_off[bk][k]);
                    bx0 = fmin(bx0, pb.x); bx1 = fmax(bx1, pb.x); by0 = fmin(by0, pb.y); by1 = fmax(by1, pb.y);
                    if (k & 1) zlo = fmin(zlo, pb.z);
                }
                if (zlo >= hrow) { act[bk] = 0; continue; }
                int i0 = ifloor(bx0 / dw), i1 = ifloor(bx1 / dw) + 1, j0 = ifloor(by0 / dw), j1 = ifloor(by1 / dw) + 1;
                double hmax = -1e30, gdum;
                for (int ii = i0; ii <= i1 && hmax <= zlo; ii++) for (int jj = j0; jj <= j1; jj++) {
                    double hn = grid_node(&GR, c, ii, jj, &gdum); if (hn > hmax) { hmax = hn; if (hmax > zlo) break; }
                }
                act[bk] = hmax > zlo;
            }
        }
        for (int q = 0; q < nlane; q++) {
            double yoff = ((r * nlane + q + 0.5) / nl - 0.5) * w;
            v3 o = vmad(cw, a, yoff);
            double A = 0, Ar = 0, sw = 0; v3 P = V3(0, 0, 0), G = V3(0, 0, 0), Gr = V3(0, 0, 0), Mr = V3(0, 0, 0), Mrr = V3(0, 0, 0);
            double dl1 = -1, dlr1 = 0; v3 n1 = up;
            if (single) {       /* one ray straight down the wheel plane: penetration = R - t, force along the hit normal */
                v3 nh; double g; double th = ray_hit(c, o, dn, R, &nh, &g);
                if (th < R) {
                    dl1 = R - th; n1 = nh; P = vmad(o, dn, th); sw = 1; A = 1; G = nh; gsum += g; gw += 1;
                    if (th < Rr) { dlr1 = Rr - th; Ar = 1; Gr = nh; }
                }
            } else {
                /* Column envelope. The row's disc is cut into vertical strips at stations x_i fixed to the WHEEL
                   (pitch ~4 mm, symmetric). The road under them is H(x,y): the bilinear interpolant of road samples on a
                   WORLD-fixed grid of pitch road_resolve (25 mm). So:
                     - H is one fixed continuous world field -> the contact is a true potential: the station forces
                       f_i = -k' dphi/du grad(u_i) applied at the station points are its exact gradient (forces AND moments);
                     - flat road: integrand exactly wheel-relative -> no horizontal force ripple, momentum conserved;
                     - discontinuous road (kerb face, plank): the step becomes a 25 mm ramp fixed in the world that
                       slides continuously through the stations -> no force steps (a belt cannot wrap finer features).
                   A = sum phi(u_i) ds, u_i = H(b_i) - z(b_i), b_i the lowest disc point above station i. */
                hcell HC = {INT32_MIN, INT32_MIN, 0, 0, 0, 0, 0, 0, 0, 0};
                for (int i = 0; i < ns; i++) {
                    if (!act[i / 4]) { i |= 3; continue; }
                    const double x = st_x[i], S = st_S[i];
                    v3 pb = vadd(o, st_off[i]);
                    double Hx, Hy, gg;
                    double H = grid_hc(&GR, &HC, c, pb.x, pb.y, &Hx, &Hy, &gg);
                    double u = H - pb.z;
                    if (u <= 0) continue;
                    /* the column overlaps the disc only up to its top: a road higher than the disc at this station (a
                       face taller than the wheel) fills the chord, and further height adds neither area nor force.
                       Without this the potential kept growing with H and pushed the wheel UP a wall it can't climb. */
                    const double umax = -2 * S * dn.z;
                    double e = COL_EPS, ph, dph;
                    if (u >= umax) { u = umax; ph = umax < e ? umax * umax / (2 * e) : umax - e / 2; dph = 0; }
                    else if (u < e) { ph = u * u / (2 * e); dph = u / e; } else { ph = u - e / 2; dph = 1; }
                    v3 dudc = V3(Hx, Hy, -1);                     /* station rigidly attached to the wheel centre */
                    v3 fi = vscl(dudc, -dph * ds);
                    A += ph * ds; sw += ph; P = vadd(P, vscl(pb, ph));
                    G = vadd(G, fi); Mr = vadd(Mr, vcross(st_off[i], fi));
                    gsum += gg * ph; gw += ph;
                    if (st_Sr[i] > 0) {
                        double Sr = st_Sr[i];
                        double ur = H - (o.z + fw.z * x + dn.z * Sr), urmax = -2 * Sr * dn.z;
                        if (ur > 0) {
                            double phr, dphr;
                            if (ur >= urmax) { phr = urmax < e ? urmax * urmax / (2 * e) : urmax - e / 2; dphr = 0; }
                            else if (ur < e) { phr = ur * ur / (2 * e); dphr = ur / e; } else { phr = ur - e / 2; dphr = 1; }
                            v3 fr_i = vscl(dudc, -dphr * ds);
                            Ar += phr * ds; Gr = vadd(Gr, fr_i);
                            Mrr = vadd(Mrr, vcross(vadd(vscl(fw, x), vscl(dn, Sr)), fr_i));
                        }
                    }
                }
            }
            if (A <= 0 || sw <= 0) continue;
            double dl = single ? dl1 : invert_area(A, R);
            double chord = 2 * sqrt(fmax(2 * R * dl - dl * dl, 1e-10));
            double gl = vlen(G);
            v3 pr = vscl(P, 1.0 / sw), nrm = single ? n1 : (gl > 1e-12 ? vscl(G, 1 / gl) : up);
            double gain = single ? 1.0 : gl / chord;     /* =1 on a plane; >1 against step faces. Uncapped: the force must stay the exact gradient of t->pe or a face strike does net work */
            v3 Mq = V3(0, 0, 0);
            v3 vp = vadd(vc, vcross(wperp, vsub(pr, cw)));
            double dd = -vdot(vp, nrm);
            double fz = gain * kz / nl * dl * (1 + prog * (dl / dlim) * (dl / dlim)) + (nodamp ? 0 : T->cz / nl * fmin(dl / HC_REF, 1.0) * dd);   /* Hunt-Crossley: damping grows with indentation, no force step at first touch */
            if (fz < 0) fz = 0;
            if (!single && gl > 1e-12) {      /* moment of the station forces about o (stiffness part), minus the spin axis */
                double ks = gain * kz / nl * dl * (1 + prog * (dl / dlim) * (dl / dlim)) / gl;
                Mq = vscl(Mr, ks);
            }
            /* rim: same potential construction on the rim radius; its stiffness force is the exact gradient of
               0.5 k_rim dlr^2, i.e. along the rim's own area gradient Gr (a rim corner on a face pushes along Gr, not
               along the tyre's normal), with its own station moment; damping acts along the contact normal */
            double fr = 0, dlr = 0; v3 Frv = V3(0, 0, 0), Mqr = V3(0, 0, 0);
            if (Ar > 0) {
                dlr = single ? dlr1 : invert_area(Ar, Rr);
                double chr = 2 * sqrt(fmax(2 * Rr * dlr - dlr * dlr, 1e-10));
                double fks = T->k_rim / nl * dlr;
                double fcd = nodamp ? 0 : T->c_rim / nl * fmin(dlr / HC_REF, 1.0) * dd;
                if (single) { Frv = vscl(Gr, fks + fcd); }
                else {
                    double s_ = fks / chr;
                    Frv = vadd(vscl(Gr, s_), vscl(nrm, fcd)); Mqr = vscl(Mrr, s_);
                }
                if (vdot(Frv, nrm) < 0) { Frv = V3(0, 0, 0); Mqr = V3(0, 0, 0); }
                fr = vlen(Frv);
                t->pe += 0.5 * T->k_rim / nl * dlr * dlr;
            }
            t->pe += kz / nl * (0.5 * dl * dl + prog * dl * dl * dl * dl / (4 * dlim * dlim));
            double Nq = fz + fr;
            v3 Fq = vmad(Frv, nrm, fz);
            Fv = vadd(Fv, Fq); Tq = vadd(Tq, vadd(vcross(vsub(o, orw), Fq), vadd(Mq, Mqr)));
            Pw = vmad(Pw, pr, Nq); sN += Nq; fzs += fz; frs += fr; dlw += dl * Nq; if (dlr > dlrm) dlrm = dlr;
        }
        double Nr = vlen(Fv);
        if (sN <= 0 || Nr <= 1e-12) continue;
        v3 nrm = vscl(Fv, 1.0 / Nr), pr = vscl(Pw, 1.0 / sN);
        t->Mst[r] = vsub(Tq, vscl(a, vdot(Tq, a)));   /* lane levers along the axle and stiffness moments, spin component removed */
        t->pr[r] = pr; t->nr[r] = nrm; t->Fz_t[r] = fzs; t->Frim[r] = frs; t->N[r] = Nr;
        t->dl[r] = dlw / sN; t->dlr[r] = dlrm;
        nsum = vadd(nsum, Fv); psum = vadd(psum, vscl(pr, Nr));
        t->Ntot += Nr; t->Fz += fzs; t->Fr += frs;
        S->delta_row[r] = t->dl[r];
    }
    if (t->Ntot <= 1e-9) {
        t->contact = 0;
        /* airborne: carcass deflections relax (tyre hysteresis), twist too */
        S->ex /= (1 + h * 60); S->ey /= (1 + h * 60); S->phi /= (1 + h * 60);
        t->arm_x = -R; t->Re = R; t->kx = 0; t->D = 0;
        return;
    }
    t->contact = 1;
    t->grip = gw > 0 ? gsum / gw : 1;
    if (c->dbg & DC_DBG_NO_FRICTION) t->grip = 0;
    t->n = vnorm(nsum); t->p = vscl(psum, 1.0 / t->Ntot);
    t->xc = vnorm(vcross(a, t->n)); t->yc = vcross(t->n, t->xc);
    t->camber_l = -vdot(a, t->n);
    /* strip properties */
    double ca = T->patch_factor, a_ref = ca * sqrt(2 * R * T->fz0 / T->kz_ref);
    double Ca0 = 3 * T->mu_y * T->fz0 / tan(T->peak_slip_deg * M_PI / 180.0);
    double pr_ratio = p / T->pressure_ref; if (pr_ratio < 0) pr_ratio = 0;
    double carcass_scale = 0.35 + 0.65 * fmin(pr_ratio, 1.5);
    double g = fabs(t->camber_l);
    double cmb = clampd(1 + T->dcamber0 * g + T->dcamber1 * g * g, 0.5, 1.1);
    strip st[DC_MAX_ROWS];
    double mu_avg = 0;
    t->delta_eff = t->Fz / kz;
    for (int r = 0; r < nrows; r++) {
        double N = t->N[r];
        double dtot = t->dl[r];
        double a_r = ca * sqrt(2 * R * fmax(dtot, 0));
        double ratio = fmax(N * nrows / T->fz0, 0.05);
        st[r].N = N; st[r].a = a_r; t->a_r[r] = a_r;
        st[r].Cy = Ca0 / nrows * (a_r / a_ref) * (a_r / a_ref) * pow(ratio, T->ls_exp_y - 1);
        st[r].Cx = T->cx_mult * st[r].Cy;
        st[r].mu = T->mu_y * pow(ratio, T->ls_exp_y - 1) * t->grip * cmb;   /* lateral mu; x uses ratio below */
        t->Ctot += st[r].Cy; t->Cxtot += st[r].Cx;
        mu_avg += st[r].mu * N;
        t->a_eff += a_r * N;
    }
    mu_avg /= t->Ntot; t->a_eff /= t->Ntot;
    t->sig_y = fmax(t->Ctot / (T->carcass_ky * carcass_scale), 0.02);
    t->sig_x = fmax(t->Cxtot / (T->carcass_kx * carcass_scale), 0.02);
    /* kinematics at the patch (rigid motion of the non-spinning wheel) */
    v3 vp = vadd(vc, vcross(wperp, vsub(t->p, cw)));
    t->Vcx = vdot(vp, t->xc); t->Vsy = vdot(vp, t->yc);
    t->Re = R - fmin(t->delta_eff, dlim) / 3.0;
    double Om = S->omega;
    t->Vr = fabs(Om * t->Re);
    double Vsx = t->Vcx - Om * t->Re;
    double Vref = fmax(t->Vr, 1.0);
    t->slip_ratio_ss = -Vsx / fmax(fabs(t->Vcx), 0.5);
    t->slip_angle_ss = -atan2(t->Vsy, fmax(fabs(t->Vcx), 0.5));
    /* sliding friction falloff (from the kinematic theoretical slip) */
    double lss = sqrt(pow(t->Cxtot * Vsx / Vref, 2) + pow(t->Ctot * t->Vsy / Vref, 2)) / (3 * mu_avg * t->Ntot + 1e-9);
    double x = fmax(0, lss - 1) / fmax(T->falloff_speed, 1e-3);
    double mfac = T->falloff_level + (1 - T->falloff_level) / (1 + x * x);
    for (int r = 0; r < nrows; r++) {
        double ratio = fmax(st[r].N * nrows / T->fz0, 0.05);
        double muy = st[r].mu * mfac;
        double mux = T->mu_x * pow(ratio, T->ls_exp_x - 1) * t->grip * cmb * mfac;
        st[r].mu = sqrt(mux * muy);   /* single friction circle per strip (anisotropy via Cx/Cy) */
    }
    t->mu_eff = mu_avg * mfac;
    /* carcass states */
    t->ey_new = (S->ey - h * t->Vsy) / (1 + h * t->Vr / t->sig_y);
    t->ex_star = (S->ex - h * Vsx) / (1 + h * t->Vr / t->sig_x);
    t->D = h * t->Re / (1 + h * t->Vr / t->sig_x);
    double sy = t->ey_new / t->sig_y + T->camber_gain * t->camber_l;
    double sx = t->ex_star / t->sig_x;
    double Fx, Fy, Mz, lam;
    brush(st, nrows, sx, sy, &Fx, &Fy, &Mz, &lam);
    Mz *= T->trail_factor > 0 ? T->trail_factor : 1.0;   /* pressure-distribution trail correction (measured ~1/4 contact length) */
    double eps = 1e-6;
    double Fx2, Fy2, Mz2, l2;
    brush(st, nrows, sx + eps / t->sig_x, sy, &Fx2, &Fy2, &Mz2, &l2);
    double kb = (Fx2 - Fx) / eps;
    /* low-speed carcass damping (the relaxation term damps at speed) */
    double Vb = fmax(fabs(t->Vcx), t->Vr);
    double blend = nodamp ? 0 : 1.0 / (1 + (Vb / 2.0) * (Vb / 2.0));
    double mc = c->m_c[i];
    double cy = 2 * 0.5 * sqrt(T->carcass_ky * carcass_scale * mc) * blend;
    double cx = 2 * 0.5 * sqrt(T->carcass_kx * carcass_scale * mc) * blend;
    Fy += cy * (t->ey_new - S->ey) / h * (t->grip > 0);
    Fx += cx * (t->ex_star - S->ex) / h * (t->grip > 0);
    t->kx = kb + cx / h * (t->grip > 0);
    t->Fx_star = Fx; t->Fy = Fy; t->Mz_brush = Mz; t->lam = lam;
    /* tread twist (parking / turn-slip moment) */
    double wn = vdot(wc, t->n);   /* wheel yaw rate about the normal (spin axis ⟂ n) */
    t->phi_new = (S->phi - h * wn) / (1 + h * t->Vr / fmax(T->twist_relax, 1e-3));
    double Mmax = t->mu_eff * t->Ntot * (t->a_eff + 0.5 * w) / 3.0 * fmax(0.0, 1 - fmin(lam, 1.0)) + 1e-9;
    double kt = T->twist_k * t->Ntot / T->fz0;
    if (kt * fabs(t->phi_new) > Mmax) t->phi_new = sgn(t->phi_new) * Mmax / fmax(kt, 1e-9);
    t->Mt = kt * t->phi_new;
    t->rr_e = nodamp ? 0 : T->rr_factor * fmin(t->delta_eff, dlim) * tanh(Om * t->Re / 0.5);
    /* torques about the wheel centre, except the Fx term */
    v3 tau = V3(0, 0, 0);
    /* normal forces are the exact gradient of the strip potential: they act on the strip centre on the axle
       (moment arm yoff along a), so they never torque the spin axis and do no spurious work */
    for (int r = 0; r < nrows; r++) if (t->N[r] > 0) tau = vadd(tau, vcross(vsub(t->orow[r], cw), vscl(t->nr[r], t->N[r])));
    for (int r = 0; r < nrows; r++) if (t->N[r] > 0) tau = vadd(tau, t->Mst[r]);
    tau = vadd(tau, vcross(vsub(t->p, cw), vscl(t->yc, t->Fy)));
    tau = vadd(tau, vscl(t->n, t->Mz_brush + t->Mt));
    tau = vadd(tau, vscl(t->yc, -t->rr_e * t->Ntot));
    t->tau = tau;
    t->Trest = vdot(tau, a);
    t->arm_x = vdot(a, vcross(vsub(t->p, cw), t->xc)) - t->ey_new * vdot(a, t->n);
    if (t->Fy != 0) t->tp = -t->Mz_brush / t->Fy;
}

void dc_tyre_phaseB(dc_car* c, int i, v3 cw, v3 a, double omega_new, double h, dc_tc* t, int commit) {
    dc_tyre_state* S = &c->ty[i];
    const dc_tyre_params* T = &c->P.axle[i / 2].tyre;
    if (T->model == DC_TYRE_TARMAC) { dc_tyre_tm_phaseB(c, i, cw, a, omega_new, h, t, commit); return; }
    (void)h;
    if (!t->contact) { t->F = V3(0, 0, 0); t->tau = V3(0, 0, 0); t->Fx = 0; return; }
    double dO = omega_new - S->omega;
    double ex = t->ex_star + t->D * dO;
    double Fx = t->Fx_star + t->kx * t->D * dO;
    /* friction circle on the applied force (linearisation overshoot) */
    double cap = t->mu_eff * t->Ntot;
    double Fm = sqrt(Fx * Fx + t->Fy * t->Fy);
    double Fy = t->Fy;
    if (Fm > cap && Fm > 0) { Fx *= cap / Fm; Fy *= cap / Fm; }
    t->Fx = Fx;
    if (Fy != t->Fy) { t->tau = vadd(t->tau, vcross(vsub(t->p, cw), vscl(t->yc, Fy - t->Fy))); t->Fy = Fy; }
    t->F = vadd(vadd(vscl(t->xc, Fx), vscl(t->yc, Fy)), V3(0, 0, 0));
    for (int r = 0; r < t->nrows; r++) if (t->N[r] > 0) t->F = vadd(t->F, vscl(t->nr[r], t->N[r]));
    t->tau = vadd(t->tau, vcross(vsub(t->p, cw), vscl(t->xc, Fx)));
    t->tau = vadd(t->tau, vscl(t->n, -t->ey_new * Fx));
    if (commit) {
        /* clamp carcass deflection to the saturation boundary (excess is sliding) */
        double sx = ex / t->sig_x, sy = t->ey_new / t->sig_y;
        double lam = sqrt(pow(t->Cxtot * sx, 2) + pow(t->Ctot * sy, 2)) / (3 * cap + 1e-9);
        double ey = t->ey_new;
        if (lam > 1) { ex /= lam; ey /= lam; }
        S->ex = ex; S->ey = ey; S->phi = t->phi_new;
        double scale = 0.35 + 0.65 * fmin(S->pressure / T->pressure_ref, 1.5);
        t->pe += 0.5 * T->carcass_kx * scale * ex * ex + 0.5 * T->carcass_ky * scale * ey * ey;
    }
    (void)a;
}
