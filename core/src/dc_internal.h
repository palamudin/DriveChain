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
} dc_tyre_state;

struct dc_car {
    dc_car_params P;
    dc_kin K[4];
    dc_road_fn road; void* road_user; int road_flags; dc_heightfield hf;
    dc_input in;
    int dbg;
    /* derived constants */
    double m_s, m_c[4]; v3 I_s_body, I_c_body[4];
    double s_dn[4], s_up[4];       /* effective droop / bump-stop travel (stops pulled inside the linkage's valid range) */
    int kin_warn;                  /* bit i: corner i stops were pulled in (linkage toggles inside configured travel) */
    double r_p, rack_max, r_k_b[2];  /* pinion radius, rack travel, brush stiffness per axle */
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
    double dissipated;
    /* per-step outputs */
    dc_telemetry tel;
    v3 Fc[4], Pc[4];                 /* contact force and application point (world) */
    v3 Fext_sum, Text_sum;
    double brake_imp[4], drive_imp[4];
    int shift_timer;
    double tel_udot[NQ];
    double link_f[4][DC_MAX_LINKS+1];  /* two-force member loads (+ = tension), last = spring/strut axial share */
    double link_resid[4];
    struct dc_mbcar* mb;             /* multibody solver state (solver = DC_SOLVER_MULTIBODY), else NULL */
};

/* kinematics */
void dc_kin_setup(dc_car* c);
int  dc_kin_solve(const dc_kin* k, double s, double r, dc_pose* p, int need_jac);
void dc_kin_full(const dc_kin* k, double s, double r, double sd, double rd, dc_pose* p);
/* road */
double dc_road_eval(const dc_car* c, double x, double y, v3* n, double* grip);
double dc_road_height(const dc_car* c, double x, double y, double* grip);   /* height + grip only (no normal) */
/* params */
void dc_params_finalize(dc_car_params* p);
#endif
