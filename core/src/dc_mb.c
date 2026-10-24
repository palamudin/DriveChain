/* dc_mb.c - maximal-coordinate multibody library: joints, force elements, block-sparse LDL^T, Schur box LCP.
 * See dc_mb.h for the formulation. Allocation happens only in mb_create / mb_finalize. */
#include "dc_mb.h"
#include <stdlib.h>
#include <stdio.h>

#define REG 1e-11      /* default joint regularisation (velocity per impulse): ~1e-11 m/s per N s */

static const v3 AX[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

mb_world* mb_create(void) { mb_world* W = (mb_world*)calloc(1, sizeof(mb_world)); if (W) W->shake = 1; return W; }
void mb_destroy(mb_world* W) {
    if (!W) return;
    free(W->nbr); free(W->nbr_blk); free(W->blkpos); free(W->blk); free(W->L); free(W->D); free(W->Dinv);
    free(W->x); free(W->r); free(W->z); free(W->zb); free(W);
}

static m3 m3inv(const m3* A) {
    double a = A->m[0][0], b = A->m[0][1], c = A->m[0][2], d = A->m[1][0], e = A->m[1][1], f = A->m[1][2], g = A->m[2][0], h = A->m[2][1], i = A->m[2][2];
    double det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    m3 r; double s = 1.0 / det;
    r.m[0][0] = (e * i - f * h) * s; r.m[0][1] = (c * h - b * i) * s; r.m[0][2] = (b * f - c * e) * s;
    r.m[1][0] = (f * g - d * i) * s; r.m[1][1] = (a * i - c * g) * s; r.m[1][2] = (c * d - a * f) * s;
    r.m[2][0] = (d * h - e * g) * s; r.m[2][1] = (b * g - a * h) * s; r.m[2][2] = (a * e - b * d) * s;
    return r;
}
static qt qconj(qt q) { q.x = -q.x; q.y = -q.y; q.z = -q.z; return q; }

int mb_add_body(mb_world* W, const char* name, double m, m3 Ib, v3 x, qt q) {
    if (W->nb >= MB_MAXB) return -1;
    mb_body* B = &W->B[W->nb];
    memset(B, 0, sizeof *B);
    B->m = m; B->Ib = Ib; B->Ibinv = m3inv(&Ib); B->x = x; B->q = qnorm(q); B->R = qtom(B->q);
    snprintf(B->name, sizeof B->name, "%s", name ? name : "");
    return W->nb++;
}
int mb_add_joint(mb_world* W, int type, const char* name, int a, int b) {
    if (W->nj >= MB_MAXJ) return -1;
    mb_joint* J = &W->Jt[W->nj];
    memset(J, 0, sizeof *J);
    J->type = type; J->nb = 2; J->b[0] = a; J->b[1] = b; J->b[2] = J->b[3] = -1;
    J->compl = REG; J->proj = 1; J->active = 1; J->q0.w = 1;
    snprintf(J->name, sizeof J->name, "%s", name ? name : "");
    static const int rows[] = {3, 5, 4, 3, 1, 5, 6, 2, 1};
    J->m = rows[type];
    return W->nj++;
}
void mb_couple(mb_world* W, int a, int b) {
    if (a < 0 || b < 0 || a == b) return;
    for (int i = 0; i < W->ncpl; i++) if ((W->cpl[i][0] == a && W->cpl[i][1] == b) || (W->cpl[i][0] == b && W->cpl[i][1] == a)) return;
    if (W->ncpl < MB_MAXCPL) { W->cpl[W->ncpl][0] = a; W->cpl[W->ncpl][1] = b; W->ncpl++; }
}
v3 mb_point_world(const mb_body* B, v3 l) { return vadd(B->x, mv(&B->R, l)); }
v3 mb_point_vel(const mb_body* B, v3 pw) { return vadd(B->v, vcross(B->w, vsub(pw, B->x))); }
static v3 to_local(const mb_world* W, int b, v3 pw) { if (b < 0) return pw; const mb_body* B = &W->B[b]; return mtv(&B->R, vsub(pw, B->x)); }
static v3 dir_local(const mb_world* W, int b, v3 d) { if (b < 0) return d; return mtv(&W->B[b].R, d); }
static v3 pw_of(const mb_world* W, int b, v3 l) { return b < 0 ? l : mb_point_world(&W->B[b], l); }
static v3 dw_of(const mb_world* W, int b, v3 l) { return b < 0 ? l : mv(&W->B[b].R, l); }
static qt q_of(const mb_world* W, int b) { if (b < 0) { qt q = {1, 0, 0, 0}; return q; } return W->B[b].q; }

static void perp2(v3 n, v3* t1, v3* t2) {
    v3 a = fabs(n.x) < 0.6 ? V3(1, 0, 0) : (fabs(n.y) < 0.6 ? V3(0, 1, 0) : V3(0, 0, 1));
    *t1 = vnorm(vcross(n, a)); *t2 = vcross(n, *t1);
}

int mb_spherical(mb_world* W, const char* nm, int a, int b, v3 p) {
    int j = mb_add_joint(W, MBJ_SPHERICAL, nm, a, b); if (j < 0) return j;
    W->Jt[j].pa = to_local(W, a, p); W->Jt[j].pb = to_local(W, b, p); return j;
}
int mb_revolute(mb_world* W, const char* nm, int a, int b, v3 p, v3 axis) {
    int j = mb_add_joint(W, MBJ_REVOLUTE, nm, a, b); if (j < 0) return j;
    axis = vnorm(axis);
    W->Jt[j].pa = to_local(W, a, p); W->Jt[j].pb = to_local(W, b, p);
    W->Jt[j].aa = dir_local(W, a, axis); W->Jt[j].ab = dir_local(W, b, axis); return j;
}
int mb_distance(mb_world* W, const char* nm, int a, v3 pa, int b, v3 pb) {
    int j = mb_add_joint(W, MBJ_DISTANCE, nm, a, b); if (j < 0) return j;
    W->Jt[j].pa = to_local(W, a, pa); W->Jt[j].pb = to_local(W, b, pb); W->Jt[j].L = vlen(vsub(pa, pb)); return j;
}
int mb_prismatic(mb_world* W, const char* nm, int a, int b, v3 axis) {   /* a slides along axis fixed in b */
    int j = mb_add_joint(W, MBJ_PRISMATIC, nm, a, b); if (j < 0) return j;
    v3 pa = a >= 0 ? W->B[a].x : V3(0, 0, 0);
    W->Jt[j].pa = to_local(W, a, pa); W->Jt[j].pb = to_local(W, b, pa);
    W->Jt[j].ab = dir_local(W, b, vnorm(axis));
    W->Jt[j].q0 = qmul(qconj(q_of(W, b)), q_of(W, a)); return j;
}
int mb_weld(mb_world* W, const char* nm, int a, int b) {
    int j = mb_add_joint(W, MBJ_WELD, nm, a, b); if (j < 0) return j;
    v3 pa = a >= 0 ? W->B[a].x : V3(0, 0, 0);
    W->Jt[j].pa = to_local(W, a, pa); W->Jt[j].pb = to_local(W, b, pa);
    W->Jt[j].q0 = qmul(qconj(q_of(W, b)), q_of(W, a)); return j;
}
/* universal joint: centre p, cross arm ea on a perpendicular to shaft_a and to shaft_b, eb on b perpendicular to shaft_b and ea.
   slip: the centre may slide along shaft_a (spline), i.e. only the 2 translations perpendicular to shaft_a are locked. */
int mb_ujoint(mb_world* W, const char* nm, int a, int b, v3 p, v3 sa, v3 sb, int slip) {
    int j = mb_add_joint(W, slip ? MBJ_UJOINT_SLIP : MBJ_UJOINT, nm, a, b); if (j < 0) return j;
    sa = vnorm(sa); sb = vnorm(sb);
    v3 ea = vcross(sa, sb);
    if (vlen(ea) < 1e-6) { v3 t2; perp2(sa, &ea, &t2); } else ea = vnorm(ea);
    v3 eb = vnorm(vcross(sb, ea));
    mb_joint* J = &W->Jt[j];
    J->pa = to_local(W, a, p); J->pb = to_local(W, b, p);
    J->aa = dir_local(W, a, sa); J->ab = dir_local(W, b, sb);
    J->ea = dir_local(W, a, ea); J->eb = dir_local(W, b, eb);
    return j;
}
int mb_custom(mb_world* W, const char* nm, int m, int box, int nb, const int* bodies) {
    int j = mb_add_joint(W, MBJ_CUSTOM, nm, bodies[0], nb > 1 ? bodies[1] : -1); if (j < 0) return j;
    mb_joint* J = &W->Jt[j];
    J->m = m; J->box = box; J->nb = nb; for (int k = 0; k < nb; k++) J->b[k] = bodies[k];
    J->proj = 0;
    for (int r = 0; r < 6; r++) { J->lo[r] = -1e30; J->hi[r] = 1e30; }
    return j;
}

void mb_row_point(const mb_body* B, v3 p, v3 u, double o[6]) {
    v3 m = vcross(vsub(p, B->x), u);
    o[0] = u.x; o[1] = u.y; o[2] = u.z; o[3] = m.x; o[4] = m.y; o[5] = m.z;
}
void mb_row_ang(v3 u, double o[6]) { o[0] = o[1] = o[2] = 0; o[3] = u.x; o[4] = u.y; o[5] = u.z; }
static void rowp(const mb_world* W, int b, v3 p, v3 u, double s, double o[6]) {
    if (b < 0) { memset(o, 0, 6 * sizeof(double)); return; }
    mb_row_point(&W->B[b], p, u, o); for (int i = 0; i < 6; i++) o[i] *= s;
}
static void rowa(int b, v3 u, double s, double o[6]) {
    if (b < 0) { memset(o, 0, 6 * sizeof(double)); return; }
    mb_row_ang(vscl(u, s), o);
}
static v3 rot_err(qt qa, qt qb, qt q0) {   /* world rotation taking (qb q0) to qa, small-angle vector */
    qt e = qmul(qa, qconj(qmul(qb, q0)));
    if (e.w < 0) { e.w = -e.w; e.x = -e.x; e.y = -e.y; e.z = -e.z; }
    return V3(2 * e.x, 2 * e.y, 2 * e.z);
}

/* evaluate rows (J) and position error (phi) of a standard joint at the current configuration */
static void eval_std(const mb_world* W, const mb_joint* J, double Jo[6][MB_JB][6], double* phi) {
    int a = J->b[0], b = J->b[1];
    v3 pa = pw_of(W, a, J->pa), pb = pw_of(W, b, J->pb);
    int r = 0;
    switch (J->type) {
    case MBJ_SPHERICAL: case MBJ_REVOLUTE: case MBJ_UJOINT: case MBJ_WELD:
        for (int k = 0; k < 3; k++, r++) {
            rowp(W, a, pa, AX[k], 1, Jo[r][0]); rowp(W, b, pb, AX[k], -1, Jo[r][1]);
            phi[r] = vget(vsub(pa, pb), k);
        }
        if (J->type == MBJ_REVOLUTE) {
            v3 na = dw_of(W, a, J->aa), nb = dw_of(W, b, J->ab), t1, t2; perp2(na, &t1, &t2);
            v3 t[2] = {t1, t2}; v3 e = vcross(nb, na);
            for (int k = 0; k < 2; k++, r++) { rowa(a, t[k], 1, Jo[r][0]); rowa(b, t[k], -1, Jo[r][1]); phi[r] = vdot(e, t[k]); }
        } else if (J->type == MBJ_UJOINT) {
            v3 ea = dw_of(W, a, J->ea), eb = dw_of(W, b, J->eb), c = vcross(ea, eb);
            rowa(a, c, 1, Jo[r][0]); rowa(b, c, -1, Jo[r][1]); phi[r] = vdot(ea, eb); r++;
        } else if (J->type == MBJ_WELD) {
            v3 e = rot_err(q_of(W, a), q_of(W, b), J->q0);
            for (int k = 0; k < 3; k++, r++) { rowa(a, AX[k], 1, Jo[r][0]); rowa(b, AX[k], -1, Jo[r][1]); phi[r] = vget(e, k); }
        }
        break;
    case MBJ_UJOINT_SLIP: case MBJ_POINTLINE: {
        /* a carries the line (anchor pa, axis aa); b's point pb stays on it */
        v3 na = dw_of(W, a, J->aa), t1, t2; perp2(na, &t1, &t2); v3 t[2] = {t1, t2};
        for (int k = 0; k < 2; k++, r++) { rowp(W, a, pb, t[k], 1, Jo[r][0]); rowp(W, b, pb, t[k], -1, Jo[r][1]); phi[r] = vdot(vsub(pa, pb), t[k]); }
        if (J->type == MBJ_UJOINT_SLIP) {
            v3 ea = dw_of(W, a, J->ea), eb = dw_of(W, b, J->eb), c = vcross(ea, eb);
            rowa(a, c, 1, Jo[r][0]); rowa(b, c, -1, Jo[r][1]); phi[r] = vdot(ea, eb); r++;
        }
        break; }
    case MBJ_DISTANCE: {
        v3 d = vsub(pa, pb); double l = vlen(d); v3 u = l > 1e-12 ? vscl(d, 1 / l) : V3(1, 0, 0);
        rowp(W, a, pa, u, 1, Jo[0][0]); rowp(W, b, pb, u, -1, Jo[0][1]); phi[0] = l - J->L; r = 1;
        break; }
    case MBJ_PRISMATIC: {
        /* a slides along axis ab fixed in b; translation rows at a's anchor point (material point of b there) */
        v3 nb = dw_of(W, b, J->ab), t1, t2; perp2(nb, &t1, &t2); v3 t[2] = {t1, t2};
        for (int k = 0; k < 2; k++, r++) { rowp(W, a, pa, t[k], 1, Jo[r][0]); rowp(W, b, pa, t[k], -1, Jo[r][1]); phi[r] = vdot(vsub(pa, pb), t[k]); }
        v3 e = rot_err(q_of(W, a), q_of(W, b), J->q0);
        for (int k = 0; k < 3; k++, r++) { rowa(a, AX[k], 1, Jo[r][0]); rowa(b, AX[k], -1, Jo[r][1]); phi[r] = vget(e, k); }
        break; }
    default: break;
    }
}
void mb_eval_joint(mb_world* W, int j) {
    mb_joint* J = &W->Jt[j];
    if (J->type == MBJ_CUSTOM) return;
    memset(J->J, 0, sizeof J->J);
    eval_std(W, J, J->J, J->phi);
}

/* ------------------------------------------------------------------ symbolic analysis */
static int blk_of(const mb_world* W, int i, int j) { return W->blkpos[i * W->n + j]; }

int mb_finalize(mb_world* W) {
    int nb = W->nb, n = nb, nfj = 0;
    for (int j = 0; j < W->nj; j++) { W->node_of_joint[j] = -1; if (!W->Jt[j].box) { W->node_of_joint[j] = n; W->joint_of_node[n] = j; n++; nfj++; } }
    W->n = n; W->nfj = nfj;
    int N = 0;
    for (int i = 0; i < n; i++) { W->dim[i] = i < nb ? 6 : W->Jt[W->joint_of_node[i]].m; W->off[i] = N; N += W->dim[i]; }
    W->N = N;
    unsigned char* adj = (unsigned char*)calloc((size_t)n * n, 1);
    #define ADJ(i, j) adj[(i) * n + (j)]
    for (int j = 0; j < W->nj; j++) {
        int nd = W->node_of_joint[j]; if (nd < 0) continue;
        for (int k = 0; k < W->Jt[j].nb; k++) { int b = W->Jt[j].b[k]; if (b >= 0) { ADJ(nd, b) = ADJ(b, nd) = 1; } }
    }
    for (int c = 0; c < W->ncpl; c++) { int a = W->cpl[c][0], b = W->cpl[c][1]; ADJ(a, b) = ADJ(b, a) = 1; }
    int* elim = (int*)calloc(n, sizeof(int));
    unsigned char* orig = (unsigned char*)malloc((size_t)n * n); memcpy(orig, adj, (size_t)n * n);
    for (int t = 0; t < n; t++) {
        int best = -1, bdeg = 1 << 30;
        for (int pass = 0; pass < 2 && best < 0; pass++)
        for (int i = 0; i < n; i++) {
            if (elim[i]) continue;
            int ok;
            if (i < nb) ok = pass == 1 || !W->B[i].root;
            else {   /* a joint is eligible once one of its bodies is eliminated (its pivot is then -J A^-1 J^T) */
                ok = 0; const mb_joint* J = &W->Jt[W->joint_of_node[i]];
                for (int k = 0; k < J->nb; k++) if (J->b[k] >= 0 && elim[J->b[k]]) ok = 1;
                for (int k = 0; k < J->nb; k++) if (J->b[k] < 0 && J->nb == 2) ok = ok || 0;
            }
            if (!ok) continue;
            int deg = 0; for (int j = 0; j < n; j++) if (!elim[j] && j != i && ADJ(i, j)) deg++;
            if (deg < bdeg) { bdeg = deg; best = i; }
        }
        if (best < 0) {   /* only joints between un-eliminated roots / ground remain */
            for (int i = 0; i < n; i++) if (!elim[i]) { best = i; break; }
        }
        W->order[t] = best; W->rank[best] = t; elim[best] = 1;
        int nbrs[MB_MAXN], cnt = 0;
        for (int j = 0; j < n; j++) if (!elim[j] && ADJ(best, j)) nbrs[cnt++] = j;
        for (int a = 0; a < cnt; a++) for (int b = a + 1; b < cnt; b++) ADJ(nbrs[a], nbrs[b]) = ADJ(nbrs[b], nbrs[a]) = 1;
    }
    /* blocks */
    W->blkpos = (int*)malloc(sizeof(int) * n * n);
    for (int i = 0; i < n * n; i++) W->blkpos[i] = -1;
    int pool = 0, nn = 0;
    for (int i = 0; i < n; i++) for (int j = i + 1; j < n; j++) if (ADJ(i, j)) {
        W->blkpos[i * n + j] = W->blkpos[j * n + i] = pool; pool += W->dim[i] * W->dim[j]; nn++;
    }
    W->nblk_d = pool;
    W->blk = (double*)calloc(pool + 1, sizeof(double)); W->L = (double*)calloc(pool + 1, sizeof(double));
    W->D = (double*)calloc((size_t)n * 36, sizeof(double)); W->Dinv = (double*)calloc((size_t)n * 36, sizeof(double));
    W->nbr = (int*)malloc(sizeof(int) * (2 * nn + 1)); W->nbr_blk = (int*)malloc(sizeof(int) * (2 * nn + 1));
    int c = 0;
    for (int i = 0; i < n; i++) {
        W->nbr_start[i] = c;
        for (int j = 0; j < n; j++) if (ADJ(i, j) && W->rank[j] > W->rank[i]) { W->nbr[c] = j; W->nbr_blk[c] = W->blkpos[i * n + j]; c++; }
    }
    W->nbr_start[n] = c;
    W->x = (double*)calloc(N, sizeof(double)); W->r = (double*)calloc(N, sizeof(double)); W->z = (double*)calloc(N, sizeof(double));
    W->zb = (double*)calloc((size_t)64 * N, sizeof(double));
    (void)orig; free(orig); free(adj); free(elim);
    #undef ADJ
    W->ready = 1;
    return 1;
}

/* add M (di x dj, row-major, rows = node i) into K(i,j) */
static void addblk(mb_world* W, int i, int j, const double* M) {
    int di = W->dim[i], dj = W->dim[j];
    if (i == j) { double* D = W->D + 36 * i; for (int a = 0; a < di; a++) for (int b = 0; b < dj; b++) D[a * di + b] += M[a * dj + b]; return; }
    int p = blk_of(W, i, j); if (p < 0) return;
    double* B = W->blk + p;
    if (W->rank[i] > W->rank[j]) { for (int a = 0; a < di * dj; a++) B[a] += M[a]; }
    else { for (int a = 0; a < di; a++) for (int b = 0; b < dj; b++) B[b * di + a] += M[a * dj + b]; }
}

static int inv_small(int n, const double* A, double* Ai) {
    double M[36], I[36]; memcpy(M, A, sizeof(double) * n * n);
    for (int i = 0; i < n * n; i++) I[i] = 0; for (int i = 0; i < n; i++) I[i * n + i] = 1;
    int ok = 1;
    for (int k = 0; k < n; k++) {
        int p = k; double mx = fabs(M[k * n + k]);
        for (int i = k + 1; i < n; i++) if (fabs(M[i * n + k]) > mx) { mx = fabs(M[i * n + k]); p = i; }
        if (mx < 1e-300) { ok = 0; M[k * n + k] = 1e-300; p = k; }
        if (p != k) for (int j = 0; j < n; j++) { double t = M[k * n + j]; M[k * n + j] = M[p * n + j]; M[p * n + j] = t; t = I[k * n + j]; I[k * n + j] = I[p * n + j]; I[p * n + j] = t; }
        double iv = 1.0 / M[k * n + k];
        for (int j = 0; j < n; j++) { M[k * n + j] *= iv; I[k * n + j] *= iv; }
        for (int i = 0; i < n; i++) if (i != k) {
            double f = M[i * n + k]; if (f == 0) continue;
            for (int j = 0; j < n; j++) { M[i * n + j] -= f * M[k * n + j]; I[i * n + j] -= f * I[k * n + j]; }
        }
    }
    memcpy(Ai, I, sizeof(double) * n * n);
    return ok;
}

static void factor(mb_world* W) {
    int n = W->n;
    for (int t = 0; t < n; t++) {
        int k = W->order[t], dk = W->dim[k];
        if (!inv_small(dk, W->D + 36 * k, W->Dinv + 36 * k)) W->fact_fail++;
        const double* Di = W->Dinv + 36 * k;
        for (int e = W->nbr_start[k]; e < W->nbr_start[k + 1]; e++) {
            int i = W->nbr[e], di = W->dim[i];
            const double* B = W->blk + W->nbr_blk[e]; double* L = W->L + W->nbr_blk[e];
            for (int a = 0; a < di; a++) for (int b = 0; b < dk; b++) {
                double s = 0; for (int q = 0; q < dk; q++) s += B[a * dk + q] * Di[q * dk + b];
                L[a * dk + b] = s;
            }
        }
        for (int e = W->nbr_start[k]; e < W->nbr_start[k + 1]; e++) {
            int i = W->nbr[e], di = W->dim[i]; const double* Li = W->L + W->nbr_blk[e];
            for (int f = W->nbr_start[k]; f < W->nbr_start[k + 1]; f++) {
                int j = W->nbr[f]; if (W->rank[j] > W->rank[i]) continue;
                int dj = W->dim[j]; const double* Bj = W->blk + W->nbr_blk[f];
                double* T;
                if (i == j) T = W->D + 36 * i;
                else T = W->blk + blk_of(W, i, j);       /* rows i (later), cols j */
                for (int a = 0; a < di; a++) for (int b = 0; b < dj; b++) {
                    double s = 0; for (int q = 0; q < dk; q++) s += Li[a * dk + q] * Bj[b * dk + q];
                    T[a * dj + b] -= s;
                }
            }
        }
    }
}
static void ksolve(const mb_world* W, double* x) {
    int n = W->n;
    for (int t = 0; t < n; t++) {
        int k = W->order[t], dk = W->dim[k]; const double* xk = x + W->off[k];
        for (int e = W->nbr_start[k]; e < W->nbr_start[k + 1]; e++) {
            int i = W->nbr[e], di = W->dim[i]; const double* L = W->L + W->nbr_blk[e]; double* xi = x + W->off[i];
            for (int a = 0; a < di; a++) { double s = 0; for (int q = 0; q < dk; q++) s += L[a * dk + q] * xk[q]; xi[a] -= s; }
        }
    }
    for (int k = 0; k < n; k++) {
        int dk = W->dim[k]; double* xk = x + W->off[k], tmp[6]; const double* Di = W->Dinv + 36 * k;
        for (int a = 0; a < dk; a++) { double s = 0; for (int q = 0; q < dk; q++) s += Di[a * dk + q] * xk[q]; tmp[a] = s; }
        memcpy(xk, tmp, sizeof(double) * dk);
    }
    for (int t = n - 1; t >= 0; t--) {
        int k = W->order[t], dk = W->dim[k]; double* xk = x + W->off[k];
        for (int e = W->nbr_start[k]; e < W->nbr_start[k + 1]; e++) {
            int i = W->nbr[e], di = W->dim[i]; const double* L = W->L + W->nbr_blk[e]; const double* xi = x + W->off[i];
            for (int q = 0; q < dk; q++) { double s = 0; for (int a = 0; a < di; a++) s += L[a * dk + q] * xi[a]; xk[q] -= s; }
        }
    }
}

/* ------------------------------------------------------------------ per step */
void mb_begin(mb_world* W, double h) {
    W->h = h; W->ne = 0;
    for (int i = 0; i < W->nb; i++) {
        mb_body* B = &W->B[i];
        B->R = qtom(B->q);
        m3 t = mmul(&B->R, &B->Ib), Rt = mtrans(&B->R); B->Iw = mmul(&t, &Rt);
        B->F = V3(0, 0, 0); B->T = V3(0, 0, 0);
    }
    for (int j = 0; j < W->nj; j++) mb_eval_joint(W, j);
}
int mb_elem_add(mb_world* W, int nb, const int* b, double j[][6], double f, double k, double c) {
    if (W->ne >= MB_MAXE) return -1;
    mb_elem* E = &W->E[W->ne++];
    E->nb = nb; for (int i = 0; i < nb; i++) { E->b[i] = b[i]; memcpy(E->j[i], j[i], sizeof E->j[i]); }
    E->f = f; E->k = k; E->c = c;
    E->gd = mb_row_vel(W, nb, b, j);
    return W->ne - 1;
}
double mb_row_vel(const mb_world* W, int nb, const int* b, double j[][6]) {
    double s = 0;
    for (int i = 0; i < nb; i++) {
        if (b[i] < 0) continue; const mb_body* B = &W->B[b[i]];
        s += j[i][0] * B->v.x + j[i][1] * B->v.y + j[i][2] * B->v.z + j[i][3] * B->w.x + j[i][4] * B->w.y + j[i][5] * B->w.z;
    }
    return s;
}
static v3 gyro_impulse(const mb_body* B, double h) {
    v3 wb = mtv(&B->R, B->w), Iw = mv(&B->Ib, wb);
    v3 f = vscl(vcross(wb, Iw), h);
    if (vlen(f) < 1e-300) return V3(0, 0, 0);
    /* Jacobian of I(w - w1) + h w x I w at w1: I + h ([w]x I - [I w]x) */
    double Jm[9];
    for (int c = 0; c < 3; c++) {
        v3 e = AX[c], Ie = mv(&B->Ib, e);
        v3 col = vadd(Ie, vscl(vsub(vcross(wb, Ie), vcross(Iw, e)), h));
        Jm[0 * 3 + c] = col.x; Jm[1 * 3 + c] = col.y; Jm[2 * 3 + c] = col.z;
    }
    double rhs[3] = {-f.x, -f.y, -f.z};
    if (!dc_solve(3, Jm, rhs)) return V3(0, 0, 0);
    v3 dw = V3(rhs[0], rhs[1], rhs[2]);
    return mv(&B->R, mv(&B->Ib, dw));
}

static void assemble(mb_world* W) {
    int n = W->n, nb = W->nb; double h = W->h;
    memset(W->D, 0, sizeof(double) * 36 * n);
    memset(W->blk, 0, sizeof(double) * (W->nblk_d + 1));
    memset(W->r, 0, sizeof(double) * W->N);
    for (int i = 0; i < nb; i++) {
        mb_body* B = &W->B[i]; double* D = W->D + 36 * i; double* r = W->r + W->off[i];
        D[0] = D[7] = D[14] = B->m;
        for (int a = 0; a < 3; a++) for (int b = 0; b < 3; b++) D[(3 + a) * 6 + 3 + b] = B->Iw.m[a][b];
        v3 L = mv(&B->Iw, B->w), g = gyro_impulse(B, h);
        r[0] = B->m * B->v.x + h * B->F.x; r[1] = B->m * B->v.y + h * B->F.y; r[2] = B->m * B->v.z + h * B->F.z;
        r[3] = L.x + h * B->T.x + g.x; r[4] = L.y + h * B->T.y + g.y; r[5] = L.z + h * B->T.z + g.z;
    }
    for (int e = 0; e < W->ne; e++) {
        mb_elem* E = &W->E[e];
        double s = -(h * E->c + h * h * E->k); if (s < 0) s = 0;
        double ff = h * (E->f - E->c * E->gd);
        for (int a = 0; a < E->nb; a++) {
            int ba = E->b[a]; if (ba < 0) continue;
            double* r = W->r + W->off[ba];
            for (int q = 0; q < 6; q++) r[q] += E->j[a][q] * ff;
            if (s == 0) continue;
            for (int b = a; b < E->nb; b++) {      /* off-diagonal pair blocks are stored once (symmetric) */
                int bb = E->b[b]; if (bb < 0) continue;
                double M[36]; for (int p = 0; p < 6; p++) for (int q = 0; q < 6; q++) M[p * 6 + q] = s * E->j[a][p] * E->j[b][q];
                if (ba == bb || blk_of(W, ba, bb) >= 0) addblk(W, ba, bb, M);
            }
        }
    }
    for (int j = 0; j < W->nj; j++) {
        int nd = W->node_of_joint[j]; if (nd < 0) continue;
        mb_joint* J = &W->Jt[j]; int m = J->m;
        double* D = W->D + 36 * nd; double* r = W->r + W->off[nd];
        for (int a = 0; a < m; a++) { D[a * m + a] = -J->compl; r[a] = J->t[a]; }
        for (int k = 0; k < J->nb; k++) {
            int b = J->b[k]; if (b < 0) continue;
            double M[36]; for (int a = 0; a < m; a++) for (int q = 0; q < 6; q++) M[a * 6 + q] = J->J[a][k][q];
            addblk(W, nd, b, M);
        }
    }
}

void mb_solve(mb_world* W) {
    assemble(W);
    factor(W);
    int N = W->N;
    /* free solution */
    memcpy(W->x, W->r, sizeof(double) * N);
    ksolve(W, W->x);
    /* box rows */
    int nbx = 0;
    for (int j = 0; j < W->nj; j++) {
        mb_joint* J = &W->Jt[j]; if (!J->box) continue;
        for (int r = 0; r < J->m; r++) {
            if (!J->active || (J->lo[r] == 0 && J->hi[r] == 0)) { J->lam[r] = 0; continue; }
            if (nbx >= 64) break;
            W->box_j[nbx] = j; W->box_r[nbx] = r; nbx++;
        }
    }
    W->nbox = nbx;
    if (nbx > 0) {
        for (int s = 0; s < nbx; s++) {
            mb_joint* J = &W->Jt[W->box_j[s]]; int r = W->box_r[s];
            double* z = W->zb + (size_t)s * N; memset(z, 0, sizeof(double) * N);
            for (int k = 0; k < J->nb; k++) { int b = J->b[k]; if (b < 0) continue; for (int q = 0; q < 6; q++) z[W->off[b] + q] += J->J[r][k][q]; }
            ksolve(W, z);
        }
        for (int s = 0; s < nbx; s++) {
            mb_joint* J = &W->Jt[W->box_j[s]]; int r = W->box_r[s];
            double u0 = 0;
            for (int k = 0; k < J->nb; k++) { int b = J->b[k]; if (b < 0) continue; for (int q = 0; q < 6; q++) u0 += J->J[r][k][q] * W->x[W->off[b] + q]; }
            W->ub[s] = u0;
            for (int t2 = 0; t2 < nbx; t2++) {
                const double* z = W->zb + (size_t)t2 * N; double w = 0;
                for (int k = 0; k < J->nb; k++) { int b = J->b[k]; if (b < 0) continue; for (int q = 0; q < 6; q++) w += J->J[r][k][q] * z[W->off[b] + q]; }
                W->W[s * 64 + t2] = w;
            }
            W->hb[s] = J->lam[r] < J->lo[r] ? J->lo[r] : (J->lam[r] > J->hi[r] ? J->hi[r] : J->lam[r]);   /* warm start */
        }
        /* projected Gauss-Seidel on the small dense Schur complement (exact box LCP at convergence) */
        double res = 0;
        for (int it = 0; it < 200; it++) {
            res = 0;
            for (int s = 0; s < nbx; s++) {
                mb_joint* J = &W->Jt[W->box_j[s]]; int r = W->box_r[s];
                double u = W->ub[s]; for (int t2 = 0; t2 < nbx; t2++) u += W->W[s * 64 + t2] * W->hb[t2];
                double d = W->W[s * 64 + s]; if (d <= 1e-300) continue;
                double nl = W->hb[s] - (u - J->t[r]) / d;
                nl = nl < J->lo[r] ? J->lo[r] : (nl > J->hi[r] ? J->hi[r] : nl);
                double dd = fabs(nl - W->hb[s]) * sqrt(d); if (dd > res) res = dd;
                W->hb[s] = nl;
            }
            if (res < 1e-13) break;
        }
        W->pgs_res = res;
        memcpy(W->x, W->r, sizeof(double) * N);
        for (int s = 0; s < nbx; s++) {
            mb_joint* J = &W->Jt[W->box_j[s]]; int r = W->box_r[s];
            J->lam[r] = W->hb[s];
            for (int k = 0; k < J->nb; k++) { int b = J->b[k]; if (b < 0) continue; for (int q = 0; q < 6; q++) W->x[W->off[b] + q] += J->J[r][k][q] * W->hb[s]; }
        }
        ksolve(W, W->x);
    }
    for (int i = 0; i < W->nb; i++) {
        const double* x = W->x + W->off[i];
        W->B[i].v = V3(x[0], x[1], x[2]); W->B[i].w = V3(x[3], x[4], x[5]);
    }
    for (int j = 0; j < W->nj; j++) {
        int nd = W->node_of_joint[j]; if (nd < 0) continue;
        for (int a = 0; a < W->Jt[j].m; a++) W->Jt[j].lam[a] = -W->x[W->off[nd] + a];
    }
}

void mb_integrate(mb_world* W, int project) {
    double h = W->h;
    for (int i = 0; i < W->nb; i++) {
        mb_body* B = &W->B[i];
        B->x = vmad(B->x, B->v, h);
        B->q = qnorm(qmul(qexp(vscl(B->w, h)), B->q));
        B->R = qtom(B->q);
    }
    if (!project) return;
    /* post-stabilisation: one Gauss-Newton projection with the factorised system:  [A J^T; J -E][dx; nu] = [0; -phi] */
    double* x = W->z; memset(x, 0, sizeof(double) * W->N);
    double Jt[6][MB_JB][6], phi[6];
    for (int j = 0; j < W->nj; j++) {
        int nd = W->node_of_joint[j]; if (nd < 0) continue;
        mb_joint* J = &W->Jt[j]; if (!J->proj) continue;
        if (J->type == MBJ_CUSTOM) { for (int a = 0; a < J->m; a++) x[W->off[nd] + a] = -J->phi[a]; continue; }
        eval_std(W, J, Jt, phi);
        for (int a = 0; a < J->m; a++) x[W->off[nd] + a] = -phi[a];
    }
    ksolve(W, x);
    /* SHAKE-style: the position correction is also a velocity correction dx/h (it is the centripetal/curvature part of
       the constraint impulse that the velocity-level rows at q_n cannot see); its multiplier is added to the reaction. */
    double ih = 1.0 / h;
    for (int j = 0; j < W->nj; j++) {
        int nd = W->node_of_joint[j]; if (nd < 0 || !W->Jt[j].proj) continue;
        for (int a = 0; a < W->Jt[j].m; a++) W->Jt[j].lam[a] -= x[W->off[nd] + a] * ih;
    }
    for (int i = 0; i < W->nb; i++) {
        mb_body* B = &W->B[i]; const double* d = x + W->off[i];
        if (W->shake) { B->v = vadd(B->v, vscl(V3(d[0], d[1], d[2]), ih)); B->w = vadd(B->w, vscl(V3(d[3], d[4], d[5]), ih)); }
        B->x = vadd(B->x, V3(d[0], d[1], d[2]));
        B->q = qnorm(qmul(qexp(V3(d[3], d[4], d[5])), B->q));
        B->R = qtom(B->q);
    }
}

double mb_kinetic(const mb_world* W) {
    double ke = 0;
    for (int i = 0; i < W->nb; i++) {
        const mb_body* B = &W->B[i];
        m3 t = mmul(&B->R, &B->Ib), Rt = mtrans(&B->R), Iw = mmul(&t, &Rt);
        ke += 0.5 * B->m * vdot(B->v, B->v) + 0.5 * vdot(B->w, mv(&Iw, B->w));
    }
    return ke;
}
void mb_joint_wrench(const mb_world* W, int j, int k, v3* F, v3* T) {
    const mb_joint* J = &W->Jt[j]; double h = W->h;
    v3 f = V3(0, 0, 0), t = V3(0, 0, 0);
    for (int r = 0; r < J->m; r++) {
        f = vadd(f, vscl(V3(J->J[r][k][0], J->J[r][k][1], J->J[r][k][2]), J->lam[r] / h));
        t = vadd(t, vscl(V3(J->J[r][k][3], J->J[r][k][4], J->J[r][k][5]), J->lam[r] / h));
    }
    *F = f; *T = t;
}
