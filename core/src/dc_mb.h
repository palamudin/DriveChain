/* dc_mb.h - maximal-coordinate multibody library (internal).
 *
 * Bodies: 6 DOF each (world COM velocity v, world angular velocity w), full body-frame inertia tensor.
 * Joints: rows J (up to MB_JB bodies per joint) solved with the bodies in ONE symmetric quasi-definite system
 *     [ A   J^T ] [ V+ ]   [ r ]        A = M + h D + h^2 K   (rigid-body mass + linearised force elements)
 *     [ J   -E  ] [ mu ] = [ t ]        r = M V + h f_ext + gyro impulse + element terms,  lambda = -mu (impulse)
 *   factorised by block LDL^T with a static elimination order (minimum degree, bodies before their joints,
 *   designated "root" bodies last) computed once at setup: no allocation per step.
 * Box rows (Coulomb friction, brakes, clutch, ratio rows that switch on/off) are kept out of the factorisation and
 *   solved exactly on their Schur complement W = J_B K^-1 J_B^T (small dense box LCP), then one more solve.
 * Force elements: scalar generalised forces f(g, gdot) along a Jacobian j over up to 3 bodies; linearly implicit.
 * Drift: velocity rows target J V+ = t exactly (no Baumgarte energy); positions are post-stabilised by one
 *   projection solve with the same factorisation (dx = -K^-1 J^T ... phi).
 */
#ifndef DC_MB_H
#define DC_MB_H
#include "dc_math.h"

#define MB_MAXB 48
#define MB_MAXJ 120
#define MB_JB 4
#define MB_MAXCPL 160
#define MB_MAXE 160
#define MB_MAXN (MB_MAXB + MB_MAXJ)
#define MB_MAXBOX 112                /* box (Schur) rows per step */
#define MB_FSRC 4                    /* normal-reaction sources per friction joint */

enum { MBJ_SPHERICAL, MBJ_REVOLUTE, MBJ_UJOINT, MBJ_UJOINT_SLIP, MBJ_DISTANCE, MBJ_PRISMATIC, MBJ_WELD, MBJ_POINTLINE, MBJ_CUSTOM };

typedef struct {
    double m; m3 Ib, Ibinv;          /* mass, body-frame inertia about COM */
    v3 x; qt q; v3 v, w;             /* state (world) */
    m3 R, Iw;                        /* per step */
    v3 F, T;                         /* applied external force / torque (world, about COM) for this step */
    int root;                        /* eliminated last (chassis) */
    char name[16];
} mb_body;

typedef struct {
    int type, nb, m;                 /* bodies (-1 = ground), rows */
    int b[MB_JB];
    v3 pa, pb;                       /* anchors, body-local (relative to COM); ground: world */
    v3 aa, ab;                       /* axes, body-local (revolute/prismatic/U shaft axis/point-line line) */
    v3 ea, eb;                       /* U-joint cross arms, body-local */
    qt q0;                           /* design relative orientation conj(q_b) q_a (weld, prismatic) */
    double L;                        /* distance */
    int box;                         /* 1: Schur box row(s) (not in the factorisation) */
    int active;                      /* box rows only */
    int proj;                        /* include in position projection */
    double compl;                    /* regularisation / compliance (velocity per impulse) */
    /* load-dependent Coulomb bound (box joints with nsrc > 0): every row r obeys |lam_r| <= B, or the row vector
       |lam| <= B when cone = 1, with  B = pre + sum_k c_k * max(0, |N_k| - d_k)
       N_k = impulse vector of rows r0..r0+nr-1 of factorised joint j (minus its component along ax when nr = 3 and
       ax != 0). N_k is evaluated INSIDE the box LCP from the coupled solution (the factorised multipliers are affine in
       the box impulses), so friction and the normal reaction it depends on are one solve, not a lagged estimate.
       pre, d_k are impulses (force * h); c_k is dimensionless or a lever arm (torque rows from force normals). */
    int nsrc, cone;
    struct { int j, r0, nr; double c, d; v3 ax; } src[MB_FSRC];
    double pre;
    double bound;                    /* B at the solution (impulse), telemetry */
    /* per step */
    double J[6][MB_JB][6];
    double phi[6], t[6], lo[6], hi[6];
    double lam[6];                   /* impulse of each row this step (world-row sense: body gets J^T lam) */
    char name[16];
} mb_joint;

typedef struct {                     /* force element, filled each step */
    int nb, b[3];
    double j[3][6];
    double f, k, c, gd;              /* force along +j, df/dg (<=0 stable), df/dgdot (<=0), current gdot */
} mb_elem;

typedef struct {
    int nb, nj; mb_body B[MB_MAXB]; mb_joint Jt[MB_MAXJ];
    int elem_uncoupled;              /* count of element body pairs with no declared coupling (bug indicator) */
    int ncpl, cpl[MB_MAXCPL][2];     /* declared body-body couplings (force elements) */
    double h;
    /* elements this step */
    int ne; mb_elem E[MB_MAXE];
    /* symbolic */
    int n, nfj;                       /* nodes, factorised joints */
    int node_of_joint[MB_MAXJ], joint_of_node[MB_MAXN];
    int dim[MB_MAXN], off[MB_MAXN], N;
    int order[MB_MAXN], rank[MB_MAXN];
    int nbr_start[MB_MAXN + 1], *nbr;  /* later neighbours of each node (in elimination order) */
    int *nbr_blk;                     /* block offset for (nbr, node) */
    int *blkpos;                      /* n*n -> block offset or -1 */
    int nblk_d;                       /* doubles in block pool */
    double *blk, *L;                  /* block pool (rows = later node, cols = earlier node), L factors */
    double *D, *Dinv;                 /* diag blocks 6x6 per node (36 each) */
    double *x, *r, *z;                /* solve vectors */
    int nbox; int box_j[MB_MAXBOX], box_r[MB_MAXBOX];
    double *zb;                       /* nbox * N */
    double W[MB_MAXBOX * MB_MAXBOX], ub[MB_MAXBOX], lb_[MB_MAXBOX], hb[MB_MAXBOX];
    int box_of[MB_MAXJ];              /* first box slot of a box joint this step (-1 none) */
    int box_dropped;                  /* box rows dropped because MB_MAXBOX was exceeded (must stay 0) */
    double pgs_vtol, pgs_vres;        /* PGS stops when the velocity complementarity residual < pgs_vtol (m/s | rad/s) */
    int ready;
    int shake;                        /* 1: projection also corrects velocity (SHAKE/RATTLE-like, energy-consistent) */
    void (*phi_cb)(void* user);       /* called after the position update, before projection: refresh CUSTOM rows' phi */
    void* cb_user;
    int pgs_iters;                    /* box-LCP Gauss-Seidel iteration cap (default 400) */
    /* stats */
    double pgs_res; int fact_fail; int pgs_it;
} mb_world;

mb_world* mb_create(void);
void mb_destroy(mb_world* W);
int  mb_add_body(mb_world* W, const char* name, double m, m3 Ib, v3 x, qt q);
int  mb_add_joint(mb_world* W, int type, const char* name, int a, int b);
void mb_couple(mb_world* W, int a, int b);
double mb_debug_residual(mb_world* W);
/* convenience joint constructors from WORLD points/axes at the current (design) pose */
int  mb_spherical(mb_world* W, const char* nm, int a, int b, v3 p);
int  mb_revolute(mb_world* W, const char* nm, int a, int b, v3 p, v3 axis);
int  mb_distance(mb_world* W, const char* nm, int a, v3 pa, int b, v3 pb);
int  mb_prismatic(mb_world* W, const char* nm, int a, int b, v3 axis);
int  mb_pointline(mb_world* W, const char* nm, int a, v3 pa, v3 axis, int b, v3 pb);
int  mb_weld(mb_world* W, const char* nm, int a, int b);
int  mb_ujoint(mb_world* W, const char* nm, int a, int b, v3 p, v3 shaft_a, v3 shaft_b, int slip);
int  mb_custom(mb_world* W, const char* nm, int m, int box, int nb, const int* bodies);
/* add a normal-reaction source to box joint fj (load-dependent friction bound, see mb_joint.src) */
int  mb_fric_source(mb_world* W, int fj, int j, int r0, int nr, double c, v3 ax);
int  mb_finalize(mb_world* W);       /* symbolic analysis; returns 0 on failure */
/* per step */
void mb_begin(mb_world* W, double h); /* R, Iw; clears F/T and elements; evaluates standard joints */
void mb_eval_joint(mb_world* W, int j);
int  mb_elem_add(mb_world* W, int nb, const int* b, double j[][6], double f, double k, double c);
void mb_solve(mb_world* W);          /* V+ (velocities only) */
void mb_integrate(mb_world* W, int project);
/* helpers */
void mb_row_point(const mb_body* B, v3 p, v3 u, double out[6]);  /* force u at world point p -> 6-row */
void mb_row_ang(v3 u, double out[6]);
double mb_row_vel(const mb_world* W, int nb, const int* b, double j[][6]);
v3 mb_point_world(const mb_body* B, v3 local);
v3 mb_point_vel(const mb_body* B, v3 pw);
double mb_kinetic(const mb_world* W);
void mb_joint_wrench(const mb_world* W, int j, int k, v3* F, v3* T);   /* reaction force/torque on body k of joint j (world), last step */
#endif
