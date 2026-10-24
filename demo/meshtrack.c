/* meshtrack.c - see meshtrack.h */
#include "meshtrack.h"
#include "track.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Peak dry friction relative to dry dense asphalt (=1). Sources: Wong, Theory of Ground Vehicles 4th ed., Table 1.4
   (asphalt/concrete dry 0.8-0.9, gravel 0.6, earth/grass ~0.55); Bosch Automotive Handbook (tyre-road friction
   table: concrete ~ asphalt dry); painted kerb / road marking skid resistance lower than bare asphalt (EN 1436
   SRT classes S1..S5 vs. typical asphalt SRT 55-65).  Setts/cobbles: polished stone, lower macrotexture.
   These are the same multipliers the procedural maps use for the same surfaces. */
double mt_default_grip(int s) {
    switch (s) {
    case SURF_TARMAC: case SURF_PAD: case SURF_LINE: return 1.0;
    case SURF_CONCRETE: return 1.0;
    case SURF_KERB: return 0.92;
    case SURF_SAUSAGE: return 0.85;
    case SURF_PAVE: return 0.85;
    case SURF_ROUGH: return 0.75;
    case SURF_GRAVEL: return 0.70;
    case SURF_GRASS: return 0.62;
    default: return 1.0;
    }
}

int mt_read(const char* path, int* nv, int* nt, double** v, int** t, unsigned char** s, char* err, int errn) {
    FILE* f = fopen(path, "rb"); if (!f) { snprintf(err, errn, "cannot open %s", path); return 0; }
    char mg[8]; int h[3];
    if (fread(mg, 1, 8, f) != 8 || memcmp(mg, "DCMESH01", 8) || fread(h, 4, 3, f) != 3 || h[0] <= 0 || h[1] <= 0) { fclose(f); snprintf(err, errn, "%s: not a DCMESH01 file", path); return 0; }
    *nv = h[0]; *nt = h[1];
    *v = (double*)malloc(sizeof(double) * 3 * h[0]); *t = (int*)malloc(sizeof(int) * 3 * h[1]); *s = (unsigned char*)malloc(h[1]);
    int ok = fread(*v, sizeof(double), 3 * (size_t)h[0], f) == 3 * (size_t)h[0] && fread(*t, sizeof(int), 3 * (size_t)h[1], f) == 3 * (size_t)h[1]
          && fread(*s, 1, h[1], f) == (size_t)h[1];
    fclose(f);
    if (ok) for (int i = 0; i < 3 * h[1]; i++) if ((*t)[i] < 0 || (*t)[i] >= h[0]) { ok = 0; break; }
    if (!ok) { free(*v); free(*t); free(*s); snprintf(err, errn, "%s: truncated or bad indices", path); return 0; }
    return 1;
}

static int is_ground(const meshtrk_t* M, int k) { return M->s[k] != SURF_WALL; }

int mt_build(meshtrk_t* M, const char* collider, const char* scenery, char* err, int errn) {
    memset(M, 0, sizeof *M);
    if (!mt_read(collider, &M->nv, &M->nt, &M->v, &M->t, &M->s, err, errn)) return 0;
    if (scenery && *scenery) { char e2[256]; if (!mt_read(scenery, &M->snv, &M->snt, &M->sv, &M->st, &M->ss, e2, sizeof e2)) { fprintf(stderr, "[meshtrack] scenery: %s (continuing without)\n", e2); M->snv = M->snt = 0; } }
    for (int i = 0; i < 16; i++) M->grip[i] = mt_default_grip(i);
    for (int k = 0; k < 3; k++) { M->bmin[k] = 1e30; M->bmax[k] = -1e30; }
    for (int i = 0; i < M->nv; i++) for (int k = 0; k < 3; k++) { double c = M->v[3 * i + k]; if (c < M->bmin[k]) M->bmin[k] = c; if (c > M->bmax[k]) M->bmax[k] = c; }
    M->n = (double*)malloc(sizeof(double) * 3 * M->nt);
    for (int k = 0; k < M->nt; k++) {
        const double *a = &M->v[3 * M->t[3 * k]], *b = &M->v[3 * M->t[3 * k + 1]], *c = &M->v[3 * M->t[3 * k + 2]];
        double e1[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]}, e2[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
        double n[3] = {e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]};
        double l = sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        if (l < 1e-12) { M->n_degenerate++; n[0] = n[1] = 0; n[2] = 1; l = 1; }
        for (int j = 0; j < 3; j++) n[j] /= l;
        if (M->s[k] == SURF_WALL) M->n_wall++;
        else { if (n[2] < 0) { M->n_flipped++; for (int j = 0; j < 3; j++) n[j] = -n[j]; } if (n[2] < 0.05) M->n_vertical++; }
        memcpy(&M->n[3 * k], n, sizeof n);
    }
    /* grid over the ground triangles' XY bounding boxes */
    M->gcell = 1.0; M->gx0 = M->bmin[0] - 1; M->gy0 = M->bmin[1] - 1;
    M->gnx = (int)((M->bmax[0] - M->gx0) / M->gcell) + 2; M->gny = (int)((M->bmax[1] - M->gy0) / M->gcell) + 2;
    size_t nc = (size_t)M->gnx * M->gny;
    M->cstart = (int*)calloc(nc + 1, sizeof(int));
    for (int pass = 0; pass < 2; pass++) {
        int* fill = pass ? (int*)calloc(nc, sizeof(int)) : NULL;
        for (int k = 0; k < M->nt; k++) {
            if (!is_ground(M, k)) continue;
            double x0 = 1e30, x1 = -1e30, y0 = 1e30, y1 = -1e30;
            for (int j = 0; j < 3; j++) { const double* p = &M->v[3 * M->t[3 * k + j]]; x0 = fmin(x0, p[0]); x1 = fmax(x1, p[0]); y0 = fmin(y0, p[1]); y1 = fmax(y1, p[1]); }
            /* widen by the seam tolerance so the crack fallback finds neighbours in the same cell */
            int i0 = (int)floor((x0 - MT_SEAM_TOL - M->gx0) / M->gcell), i1 = (int)floor((x1 + MT_SEAM_TOL - M->gx0) / M->gcell);
            int j0 = (int)floor((y0 - MT_SEAM_TOL - M->gy0) / M->gcell), j1 = (int)floor((y1 + MT_SEAM_TOL - M->gy0) / M->gcell);
            if (i0 < 0) i0 = 0; if (j0 < 0) j0 = 0; if (i1 >= M->gnx) i1 = M->gnx - 1; if (j1 >= M->gny) j1 = M->gny - 1;
            for (int j = j0; j <= j1; j++) for (int i = i0; i <= i1; i++) {
                size_t c = (size_t)j * M->gnx + i;
                if (!pass) M->cstart[c + 1]++; else M->cidx[M->cstart[c] + fill[c]++] = k;
            }
        }
        if (!pass) { for (size_t c = 0; c < nc; c++) M->cstart[c + 1] += M->cstart[c]; M->cidx = (int*)malloc(sizeof(int) * (M->cstart[nc] + 1)); }
        else free(fill);
    }
    return 1;
}

/* closest point of triangle abc to p, in the XY plane; returns squared distance and the z there */
static double closest_xy(const double* a, const double* b, const double* c, double px, double py, double* z) {
    /* barycentric of the XY projection, then clamp to the triangle by testing the three edges */
    double best = 1e30;
    const double* P[3] = {a, b, c};
    for (int e = 0; e < 3; e++) {
        const double *u = P[e], *w = P[(e + 1) % 3];
        double dx = w[0] - u[0], dy = w[1] - u[1], L2 = dx * dx + dy * dy;
        double t = L2 > 0 ? ((px - u[0]) * dx + (py - u[1]) * dy) / L2 : 0; t = t < 0 ? 0 : (t > 1 ? 1 : t);
        double qx = u[0] + t * dx - px, qy = u[1] + t * dy - py, d2 = qx * qx + qy * qy;
        if (d2 < best) { best = d2; *z = u[2] + t * (w[2] - u[2]); }
    }
    return best;
}

double mt_height(const meshtrk_t* M, double x, double y, double* n, int* surf, double* grip, int* tri) {
    int i = (int)floor((x - M->gx0) / M->gcell), j = (int)floor((y - M->gy0) / M->gcell);
    int bk = -1; double bh = -1e30;
    if (i >= 0 && j >= 0 && i < M->gnx && j < M->gny) {
        size_t c = (size_t)j * M->gnx + i;
        for (int q = M->cstart[c]; q < M->cstart[c + 1]; q++) {
            int k = M->cidx[q];
            const double *a = &M->v[3 * M->t[3 * k]], *b = &M->v[3 * M->t[3 * k + 1]], *cc = &M->v[3 * M->t[3 * k + 2]];
            double e1x = b[0] - a[0], e1y = b[1] - a[1], e2x = cc[0] - a[0], e2y = cc[1] - a[1];
            double det = e1x * e2y - e1y * e2x;
            if (fabs(det) < 1e-12) continue;
            double rx = x - a[0], ry = y - a[1];
            double u = (rx * e2y - ry * e2x) / det, v = (e1x * ry - e1y * rx) / det;
            const double eps = 1e-9;
            if (u < -eps || v < -eps || u + v > 1 + eps) continue;
            double h = a[2] + u * (b[2] - a[2]) + v * (cc[2] - a[2]);
            if (h > bh) { bh = h; bk = k; }
        }
        if (bk < 0) {   /* crack between triangles: nearest triangle within the seam tolerance */
            double bd = MT_SEAM_TOL * MT_SEAM_TOL;
            for (int q = M->cstart[c]; q < M->cstart[c + 1]; q++) {
                int k = M->cidx[q]; double z = 0;
                double d2 = closest_xy(&M->v[3 * M->t[3 * k]], &M->v[3 * M->t[3 * k + 1]], &M->v[3 * M->t[3 * k + 2]], x, y, &z);
                if (d2 < bd || (d2 == bd && z > bh)) { bd = d2; bh = z; bk = k; }
            }
        }
    }
    if (bk < 0) {
        ((meshtrk_t*)M)->misses++;
        if (n) { n[0] = n[1] = 0; n[2] = 1; } if (surf) *surf = SURF_OTHER; if (grip) *grip = 1.0; if (tri) *tri = -1;
        return M->bmin[2] - 20.0;
    }
    if (n) memcpy(n, &M->n[3 * bk], 3 * sizeof(double));
    if (surf) *surf = M->s[bk]; if (grip) *grip = M->grip[M->s[bk] & 15]; if (tri) *tri = bk;
    return bh;
}

int mt_layers(const meshtrk_t* M, double x, double y, double* z, int* tri, int max) {
    int i = (int)floor((x - M->gx0) / M->gcell), j = (int)floor((y - M->gy0) / M->gcell), n = 0;
    if (i < 0 || j < 0 || i >= M->gnx || j >= M->gny) return 0;
    size_t c = (size_t)j * M->gnx + i;
    for (int q = M->cstart[c]; q < M->cstart[c + 1]; q++) {
        int k = M->cidx[q];
        const double *a = &M->v[3 * M->t[3 * k]], *b = &M->v[3 * M->t[3 * k + 1]], *cc = &M->v[3 * M->t[3 * k + 2]];
        double e1x = b[0] - a[0], e1y = b[1] - a[1], e2x = cc[0] - a[0], e2y = cc[1] - a[1], det = e1x * e2y - e1y * e2x;
        if (fabs(det) < 1e-12) continue;
        double rx = x - a[0], ry = y - a[1], u = (rx * e2y - ry * e2x) / det, v = (e1x * ry - e1y * rx) / det;
        if (u < -1e-9 || v < -1e-9 || u + v > 1 + 1e-9) continue;
        if (n < max) { z[n] = a[2] + u * (b[2] - a[2]) + v * (cc[2] - a[2]); if (tri) tri[n] = k; }
        n++;
    }
    return n;
}

void mt_free_scenery(meshtrk_t* M) { free(M->sv); free(M->st); free(M->ss); M->sv = NULL; M->st = NULL; M->ss = NULL; M->snv = M->snt = 0; }
void mt_free(meshtrk_t* M) {
    if (!M) return;
    mt_free_scenery(M); free(M->v); free(M->t); free(M->s); free(M->n); free(M->cstart); free(M->cidx);
    memset(M, 0, sizeof *M);
}
