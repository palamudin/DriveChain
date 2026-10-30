/* dc_internal.h - private state of the DriveChain core. */
#ifndef DC_INTERNAL_H
#define DC_INTERNAL_H
#include "drivechain.h"
#include "dc_math.h"

#define NQ 12   /* generalized velocities: v(3) w(3) sdot(4) rdot sw_dot */
#define IV 0
#define IW 3
#define IS 6
#define IR 10
#define ISW 11

typedef struct {               /* kinematic description of one corner in the chassis frame (sprung-CG origin) */
    int type, nl, rack_link, arm_a, arm_b, right, steered;
    v3 hub0;                   /* design hub centre */
    v3 lc[DC_MAX_LINKS], lu0[DC_MAX_LINKS]; /* chassis pts, upright pts (design, chassis frame) */
    v3 lu[DC_MAX_LINKS];       /* upright pts in upright-local frame (relative to hub) */
    double L[DC_MAX_LINKS];
    v3 strut_top, strut_S, strut_d, strut_e1, strut_e2; /* S,d,e1,e2 upright-local */
    v3 spin_axis_u;            /* spindle axis in upright frame (points vehicle-left for every wheel) */
    v3 spr_lo, spr_hi;         /* spring mounts (lo on arm or upright) */
    int spr_on_upright;
    v3 arm_axis, arm_p;        /* lower-arm pivot axis (unit) and a point on it */
    double k_spring, L_free, MR0;
    double c_bump, c_fbump, v_bump, c_reb, c_freb, v_reb; /* along the damper axis */
} dc_kin;

typedef struct {               /* solved kinematic pose + jacobians wrt (s, r) */
    v3 c; m3 R;                /* hub position, upright rotation (chassis frame) */
    v3 Jp[2], Jw[2];           /* d c/d s_j, angular velocity per unit s_j (chassis frame) */
    v3 bp, bw;                 /* bias accel (Jdot sdot) */
    double Ls, dL[2];          /* spring length and gradient */
    v3 lo;                     /* spring lower mount current */
} dc_pose;

typedef struct {
    double ex, ey, phi;        /* carcass/tread deflection states (m, m, rad) */
    double pressure, leak_tau;
    double omega, angle;       /* absolute spin about spindle axis, render angle */
    double delta_row[DC_MAX_ROWS];
    double a_tr, k_tr;         /* TARMAC telemetry only: relaxation-length transient slip (rad, -), no effect on forces */
} dc_tyre_state;

/* TARMAC tyre: per-axle constant tables (ray fan, flat-volume inversion, derived parameters), built by dc_tyre_tm_init */
#define DC_TM_NR 61
#define DC_TM_NL 9
typedef struct {
    double R0, H, tread, laneW, bottom, lane[DC_TM_NL];
    int nl;
    double cth[DC_TM_NR], sth[DC_TM_NR], w[DC_TM_NR];
    double bpD[DC_TM_NR], bpS[DC_TM_NR], bpT[DC_TM_NR], bpV[DC_TM_NR];
    double kx, ky, sx0, sy0, cd, mu0, ls, fref, floor_, vslide, sexp, crr, krim, crim, cz, kappa, roll, kz_mode, shape, damp_mode;
    double patch0;           /* areaSmooth * halfLength on a flat road at fref (reference footprint) */
    double cp;               /* tread brush stiffness per unit area per unit deflection, N/m^3 (2 C_ref / patch0) */
} dc_tm_tab;

/* per-wheel persistent road-node cache (DC_ROAD_STATIC): toroidal, tagged by world lattice index */
#define RC_N 64
typedef struct { int i, j; double h, g; } dc_rc_ent;
typedef struct { double dw; dc_rc_ent e[RC_N * RC_N]; } dc_rcache;
void dc_rcache_clear(dc_car* c);
double dc_wall(void);

struct dc_car {
    dc_car_params P;
    dc_kin K[4];
    dc_road_fn road; void* road_user; int road_flags; dc_heightfield hf;
    dc_input in;
    int dbg;
    /* derived constants */
    double m_s, m_c[4]; v3 I_s_body, I_c_body[4];
    double s_dn[4], s_up[4], s_hard[4]; /* s_hard: bump stop solid (rigid unilateral limit), inside the linkage's valid range */
    double hard_f[4];              /* solid-stop reaction this step, wheel units (telemetry); s_dn/s_up: effective droop / bump-stop travel (pulled inside the linkage's valid range) */
    int kin_warn;                  /* bit i: corner i stops were pulled in (linkage toggles inside configured travel) */
    double r_p, rack_max, rack_hard, r_k_b[2];  /* pinion radius, rack travel, rigid end-stop (steel on steel), brush stiffness per axle */
    double rack_hard_f;                         /* rigid rack end-stop reaction this step, N (telemetry) */
    double static_fz[4];
    v3 col_axis;                     /* steering column axis (chassis frame) */
    double I_sw, I_pin, m_rack;
    /* state */
    v3 x; qt q; v3 v, w; double s[4], sd[4], r, rd, th, thd;
    int engine_on;
    int have_mt; double Pt[3], Lt[3], proj_work;   /* momentum projection targets */
    double we, wg;                   /* engine & gearbox spin (rad/s about chassis x) */
    double lam[10];                  /* driveline warm start */
    dc_tyre_state ty[4];
    dc_pose pose[4];
    double t; long long tick; double h; double acc_dt;
    int sub_idx, sub_n;              /* substep index within the current dc_step call and the count it will run (fast-model sweep budget) */
    double dissipated;
    double body_set[DC_MAX_BODYPTS], body_w, body_pe; int body_audit; /* permanent crush of each body contact point, m (elastic-plastic body contact) */
    /* per-step outputs */
    dc_telemetry tel;
    v3 Fc[4], Pc[4];                 /* contact force and application point (world) */
    v3 Fext_sum, Text_sum;
    double brake_imp[4], drive_imp[4];
    int shift_timer;
    double tel_udot[NQ];
    double link_f[4][DC_MAX_LINKS+1];  /* two-force member loads (+ = tension), last = spring/strut axial share */
    double link_resid[4];
    dc_tm_tab tm[2];                 /* TARMAC tyre tables per axle */
    dc_rcache* rc;                   /* [4] road-node caches, allocated at create, used only with DC_ROAD_STATIC */
    struct dc_pool* pool;            /* step worker pool (dc_set_threads), NULL = serial */
    dc_profile prof;
    struct dc_mbcar* mb;             /* multibody solver state (solver = DC_SOLVER_MULTIBODY), else NULL */
};

/* kinematics */
void dc_kin_setup(dc_car* c);
int  dc_kin_solve(const dc_kin* k, double s, double r, dc_pose* p, int need_jac);
void dc_kin_full(const dc_kin* k, double s, double r, double sd, double rd, dc_pose* p);
/* road */
double dc_road_eval(const dc_car* c, double x, double y, v3* n, double* grip);
double dc_road_height(const dc_car* c, double x, double y, double* grip);   /* height + grip only (no normal) */
/* body point vs road: minimum-translation depth (vertical-to-surface or horizontal-to-face, whichever is smaller);
   returns depth > 0 when inside, with *n the push-out direction */
double dc_body_depth(const dc_car* c, v3 X, v3* n, double* grip);
double dc_body_normal(dc_car* c, int k, double pen, double vn, int nodamp, double* pe, double* wdis, double* Nd);   /* wdis: crush work this step (J), Nd: damping part of N */
/* params */
void dc_params_finalize(dc_car_params* p);
#endif
