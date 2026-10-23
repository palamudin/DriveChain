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

static double seg_area(double d, double R) {
    if (d <= 0) return 0; if (d >= 2 * R) return M_PI * R * R;
    return R * R * acos(1 - d / R) - (R - d) * sqrt(2 * R * d - d * d);
}
static double invert_area(double A, double R) {
    if (A <= 0) return 0;
    double d = pow(3.0 * A / (4.0 * sqrt(2.0 * R)), 2.0 / 3.0);
    for (int k = 0; k < 4; k++) {
        double f = seg_area(d, R) - A, df = 2 * sqrt(fmax(2 * R * d - d * d, 1e-12));
        d -= f / df; if (d < 0) d = 1e-9; if (d > 2 * R) d = 2 * R;
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
    double a = 0, b = R, fa = f0, fb = f1; int side = 0; double t = R;
    for (int it = 0; it < 14; it++) {
        t = (a * fb - b * fa) / (fb - fa);
        v3 p = vmad(o, d, t);
        double ft = p.z - dc_road_eval(c, p.x, p.y, n, &g);
        if (fabs(ft) < 1e-7) break;
        if (ft * fb > 0) { b = t; fb = ft; if (side == -1) fa *= 0.5; side = -1; }
        else { a = t; fa = ft; if (side == 1) fb *= 0.5; side = 1; }
    }
    *grip = g;
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
    dc_tyre_state* S = &c->ty[i];
    memset(t, 0, sizeof *t);
    const double R = T->radius, Rr = T->rim_radius + T->rim_pinch, w = T->width;
    const int nrows = T->n_rows < 1 ? 1 : (T->n_rows > DC_MAX_ROWS ? DC_MAX_ROWS : T->n_rows);
    const int nr = T->n_rays < 3 ? 3 : (T->n_rays > DC_MAX_RAYS ? DC_MAX_RAYS : T->n_rays);
    const double span = T->ray_span_deg * M_PI / 180.0, dB = 2 * span / (nr - 1);
    double p = S->pressure;
    double kz = T->kz_ref + T->kz_per_pa * (p - T->pressure_ref); if (kz < 0.04 * T->kz_ref) kz = 0.04 * T->kz_ref;
    const double dlim = fmax(R - Rr, 0.01), prog = 0.5;
    const int nodamp = c->dbg & DC_DBG_NO_DAMPING;
    v3 up = V3(0, 0, 1);
    v3 dn = vnorm(vadd(vneg(up), vscl(a, vdot(up, a))));
    v3 fw = vcross(dn, a);
    v3 wperp = vsub(wc, vscl(a, vdot(wc, a)));
    double gsum = 0, gw = 0;
    t->nrows = nrows;
    v3 nsum = V3(0, 0, 0), psum = V3(0, 0, 0);
    for (int r = 0; r < nrows; r++) {
        double yoff = ((r + 0.5) / nrows - 0.5) * w;
        v3 o = vmad(cw, a, yoff);
        double A = 0, Ar = 0, sw = 0; v3 P = V3(0, 0, 0), Nn = V3(0, 0, 0), Nr = V3(0, 0, 0);
        for (int j = 0; j < nr; j++) {
            double b = -span + dB * j, wt = (j == 0 || j == nr - 1) ? 0.5 : 1.0;
            v3 d = vadd(vscl(dn, cos(b)), vscl(fw, sin(b)));
            v3 nh; double g;
            double th = ray_hit(c, o, d, R, &nh, &g);
            if (th < R) {
                double ww = R * R - th * th;
                A += 0.5 * ww * dB * wt; sw += ww * wt;
                P = vadd(P, vscl(vmad(o, d, th), ww * wt));
                /* reaction of a rigid circle on a penetrating obstacle is radial: the weighted mean radial
                   direction equals the road normal on any plane and tilts back on step/kerb faces */
                Nr = vadd(Nr, vscl(d, -ww * wt)); Nn = vadd(Nn, vscl(nh, ww * wt));
                gsum += g * ww; gw += ww;
                if (th < Rr) Ar += 0.5 * (Rr * Rr - th * th) * dB * wt;
            }
        }
        t->N[r] = 0;
        if (A <= 0 || sw <= 0) continue;
        double dl = invert_area(A, R);
        v3 pr = vscl(P, 1.0 / sw), nrm = vnorm(Nn);
        {   /* longitudinal tilt from the radial mean (step faces), lateral/vertical from road normals */
            v3 fw = vcross(a, nrm); double fl = vlen(fw);
            if (fl > 1e-6) { fw = vscl(fw, 1 / fl); v3 nr = vnorm(Nr);
                nrm = vnorm(vadd(vsub(nrm, vscl(fw, vdot(nrm, fw))), vscl(fw, vdot(nr, fw)))); }
        }
        v3 vp = vadd(vc, vcross(wperp, vsub(pr, cw)));
        double dd = -vdot(vp, nrm);
        double fz = kz / nrows * dl * (1 + prog * (dl / dlim) * (dl / dlim)) + (nodamp ? 0 : T->cz / nrows * dd);
        if (fz < 0) fz = 0;
        double fr = 0, dlr = 0;
        if (Ar > 0) {
            dlr = invert_area(Ar, Rr);
            fr = T->k_rim / nrows * dlr + (nodamp ? 0 : T->c_rim / nrows * dd);
            if (fr < 0) fr = 0;
            t->pe += 0.5 * T->k_rim / nrows * dlr * dlr;
        }
        t->pe += kz / nrows * (0.5 * dl * dl + prog * dl * dl * dl * dl / (4 * dlim * dlim));
        t->pr[r] = pr; t->nr[r] = nrm; t->Fz_t[r] = fz; t->Frim[r] = fr; t->N[r] = fz + fr;
        t->dl[r] = dl; t->dlr[r] = dlr;
        nsum = vadd(nsum, vscl(nrm, t->N[r])); psum = vadd(psum, vscl(pr, t->N[r]));
        t->Ntot += t->N[r]; t->Fz += fz; t->Fr += fr;
        S->delta_row[r] = dl;
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
    for (int r = 0; r < nrows; r++) if (t->N[r] > 0) tau = vadd(tau, vcross(vsub(t->pr[r], cw), vscl(t->nr[r], t->N[r])));
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
