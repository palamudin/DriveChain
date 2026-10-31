/* dc_tyre.h - envelope contact + brush tyre (internal). */
#ifndef DC_TYRE_H
#define DC_TYRE_H
#include "dc_internal.h"
/* plastic return of a carcass/brush deflection onto |K e| = G with isotropic Coulomb sliding (dc_tyre.c) */
void dc_tyre_carcass_return(double Kx, double Ky, double ax, double ay, double G, double* ex, double* ey);
typedef struct {
    int contact, nrows;
    v3 n, xc, yc, p;                 /* effective normal, contact frame, effective point (world) */
    v3 pr[DC_MAX_ROWS], nr[DC_MAX_ROWS], orow[DC_MAX_ROWS], Mst[DC_MAX_ROWS];
    double N[DC_MAX_ROWS], Fz_t[DC_MAX_ROWS], Frim[DC_MAX_ROWS], a_r[DC_MAX_ROWS], dl[DC_MAX_ROWS], dlr[DC_MAX_ROWS];
    double Ntot, Fz, Fr, delta_eff, a_eff;
    double Re, arm_x, Vcx, Vsy, Vr, sig_x, sig_y, Ctot, Cxtot;
    double ey_new, by_new, ex_star, D, Fx_star, kx, Fy, Mz_brush, mu_eff, lam;
    double phi_new, Mt, kt, grip, camber_l, rr_e, tp;   /* kt: tread-twist stiffness this step */
    double sCx[DC_MAX_ROWS], sCy[DC_MAX_ROWS], smu[DC_MAX_ROWS];   /* brush strips this step (carcass_store = 1) */
    double Fx;                       /* final */
    v3 F, tau;                       /* final world force and torque about wheel centre */
    double Trest;                    /* axial torque except Fx term */
    double slip_angle_ss, slip_ratio_ss;
    double pe;                       /* elastic energy stored in tyre (radial+shear) */
} dc_tc;
void dc_tyre_phaseA(dc_car* c, int i, v3 cw, v3 a, v3 vc, v3 wc, double h, dc_tc* t);
void dc_tyre_phaseA_all(dc_car* c, const v3* cw, const v3* a, const v3* vc, const v3* wc, double h, dc_tc* tc);   /* all 4, on the pool */
void dc_tyre_phaseB(dc_car* c, int i, v3 cw, v3 a, double omega_new, double h, dc_tc* t, int commit);
/* energy audit: shear (ex, ey) + tread-twist energy stored in the committed carcass state of wheel i, with this
   step's stiffnesses (call after phase A). t->pe holds only the radial store until phase B commits the new shear
   state, so a start-of-step energy is t->pe + dc_tyre_shear_store(). */
double dc_tyre_shear_store(const dc_car* c, int i, const dc_tc* t);
/* TARMAC tyre (tyre_model = 1): same interface, dispatched from dc_tyre_phaseA/B */
/* telemetry only (no effect on forces): advance the reported transient slip of wheel i one step h */
void dc_tyre_telem_slip(dc_car* c, int i, const dc_tc* t, double h, double* slip_angle, double* slip_ratio);
void dc_tyre_tm_init(dc_car* c);
void dc_tyre_tm_phaseA(dc_car* c, int i, v3 cw, v3 a, v3 vc, v3 wc, double h, dc_tc* t);
void dc_tyre_tm_phaseB(dc_car* c, int i, v3 cw, v3 a, double omega_new, double h, dc_tc* t, int commit);
void dc_tyre_tm_flat(const dc_car* c, int ax, double Fz, double* area, double* half, double* delta);   /* test hook */
/* vertical rate vs gauge pressure: linear (reference rate + pressure gain) around the reference,
   blended to the carcass-only rate kz_flat below half the reference pressure (a linear law extrapolated
   to 0 psi keeps a deflated tyre unphysically stiff), floored at 4% of kz_ref. */
static inline double dc_tyre_kz(const dc_tyre_params* T, double p) {
    double k = T->kz_ref + T->kz_per_pa * (p - T->pressure_ref);
    if (T->kz_flat > 0) {
        double plo = 0.5 * T->pressure_ref;
        if (p < plo) { double klo = T->kz_ref + T->kz_per_pa * (plo - T->pressure_ref), f = p > 0 ? p / plo : 0; k = T->kz_flat + f * (klo - T->kz_flat); }
    }
    return k < 0.04 * T->kz_ref ? 0.04 * T->kz_ref : k;
}
#endif
