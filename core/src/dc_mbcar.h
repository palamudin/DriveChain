/* dc_mbcar.h - full-vehicle maximal-coordinate model (solver = DC_SOLVER_MULTIBODY), internal. */
#ifndef DC_MBCAR_H
#define DC_MBCAR_H
#include "dc_internal.h"
#include "dc_mb.h"
#include "dc_tyre.h"

/* open-centre rotary valve + double-acting cylinder with oil compressibility (docs/VALVE.md) */
typedef struct {
    double a0, amin, thc, cd, rho, beta, Vc, Vs, Cs, Ap, half, prel, grel, ktb, tref, pmax, Qr, ptr;
    int edge;                                /* 0 = contoured to the design boost law, 1 = linear chamfer */
} dc_valve;
void dc_valve_setup(dc_valve* v, const dc_car* c);
double dc_valve_area(const dc_valve* v, double th);      /* metering edge that closes for th > 0 */
/* one backward-Euler step (h > 0) or the steady state (h = 0) of the bridge: P = {chamber A, chamber B, supply}.
   th: valve twist in the +rack sense (rad), rd: rack velocity, r: rack position, Qp: pump flow.
   dPdv: sensitivity of P to rd (linearly implicit coupling). Returns Newton iterations (< 0: not converged). */
int dc_valve_solve(const dc_valve* v, const double P0[3], double th, double rd, double r, double Qp, double h, double P[3], double dPdv[3]);
double dc_valve_energy(const dc_valve* v, const double P[3], double r);

#define DC_CTAB_N 128                        /* column brick: Hermite nodes over one rim turn */
typedef struct dc_mbcar {
    mb_world* W;
    int ch;                                  /* chassis body */
    int kn[4], wh[4];                        /* knuckle (upright + hub carrier), wheel (rim+tyre+hub+rotor) */
    int narm[4], arm[4][3], arm_l[4][3][2];  /* A-arm bodies built from link pairs sharing an upright point */
    int j_arm_rev[4][3], j_arm_bj[4][3];
    int nbush;                      /* arm pivots built as compliant bushings (0 = all rigid revolutes) */
    int j_link[4][DC_MAX_LINKS];             /* distance joint for unpaired links (tie rod/toe link/multilink rods), else -1 */
    int j_hub[4], j_strut[4];
    int lo_body[4];                          /* body carrying the spring lower seat (lower arm or knuckle) */
    v3 lo_local[4];                          /* spring lower seat in lo_body local frame */
    double L_up[4], L_dn[4], MR_up[4], MR_dn[4], L_hard[4]; int j_hard[4];
    int steer;                               /* steering chain present */
    int rack, pin, inp, ish, rim;
    int hsg, j_rmount;                       /* rack housing body (= ch when the mount is rigid) and its mount joint (-1) */
    int j_rack, j_pin, j_inp, j_rim, j_u1, j_u2, j_gear;
    int brick_col, j_ctab, col_axial;                   /* DC_BRICK_COLUMN: ishaft + cardans replaced by the table row j_ctab (ish = -1) */
    double ctab_g[DC_CTAB_N + 1], ctab_d[DC_CTAB_N + 1];
    double ctab_p[DC_CTAB_N + 1], ctab_pd[DC_CTAB_N + 1];  /* intermediate-shaft angle minus rim angle and its derivative (up-pass wrench) */
    v3 cu1, cu2, eau, ebu, eal, ebl;         /* cardan centres and design cross-pin arms (K frame) */   /* input angle minus rim angle (2 pi periodic) and its derivative at the nodes */
    v3 e_rim, e_is, e_in;                    /* shaft axes in the chassis (K) frame, pointing toward the driver */
    double th_prev_raw, th_acc, pin_prev_raw, pin_acc, inp_prev_raw, inp_acc;
    int has_bar[2], bar[2][2], j_bar[2][2], j_drop[2][2];
    double k_bar[2], G_bar[2];
    int eng, gi;
    int carrier, go, dcr, j_go, j_dcr, j_gbox, j_fd;   /* carrier-inertia driveline (docs/CARRIER.md): gearbox output + propshaft rotor, diff carrier rotor */
    int j_eng, j_gi, j_clutch, j_diff, j_lsd, j_brake[4], j_rfric, j_cfric, j_rstop;
    int j_pfric, j_bjf[4][3], j_stf[4], j_tmf[4];   /* load-dependent joint friction rows (-1 = none) */
    int drv[2];                              /* driven wheels */
    double lam_diff_prev;
    v3 cgc;                                  /* chassis body COM in the K frame */
    int nbody; v3 xd[MB_MAXB]; qt qd[MB_MAXB];
    v3 v_save[MB_MAXB], w_save[MB_MAXB];
    dc_tc tc[4];
    int el_tyre[4];
    double Fx_app[4];
    v3 T_tyre[4];
    double E_audit, W_audit, pe_body; int audit_init;   /* energy audit carried between steps */                 /* contact moments applied to each wheel this step (audit) */
    double diss;
    /* per step telemetry scratch */
    double Fs[4], Fd[4], Fb[4], tau_tb, T_hands, F_assist, F_rack;
    int vmodel; dc_valve vlv; double vp[3], vQp, v_th, v_F0, v_c, v_rd0; int v_fail;   /* valve pressure states (valve_model > 0) */
    int resolves;                            /* friction-circle re-solves (statistics) */
    int lowrank;    /* fast: pass-1 re-solve by the Woodbury low-rank update (mb_resolve_lowrank) */
    /* test hooks */
    v3 extF[MB_MAXB], extT[MB_MAXB];
} dc_mbcar;

dc_mbcar* dc_mbcar_create(dc_car* c);        /* NULL if the layout is unsupported (caller falls back to the reduced core) */
void dc_mbcar_destroy(dc_mbcar* m);
void dc_mbcar_reset(dc_car* c);
void dc_mbcar_step(dc_car* c);
#endif
