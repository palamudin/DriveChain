/* dc_rgrid.h - memoised world-grid road sampler shared by the tyre contact models (internal).
 * Bilinear H on a world-fixed lattice of pitch road_resolve, sampled lazily around one wheel. */
#ifndef DC_RGRID_H
#define DC_RGRID_H
#include <string.h>
#include <stdint.h>
#include <limits.h>
#include "dc_internal.h"
#define GRID_N 48
typedef struct { dc_rcache* P; double dw, idw; int i0, j0; unsigned char have[GRID_N][GRID_N]; double h[GRID_N][GRID_N], g[GRID_N][GRID_N]; } rgrid;
/* exact floor for |v| < 2^31 without a libm call (baseline x86-64 has no SSE4.1 roundsd) */
static inline int ifloor(double v) { int i = (int)v; return i - (v < (double)i); }
static inline void grid_init(rgrid *G, double cx, double cy, double dw) {
    G->dw = dw; G->idw = 1.0 / dw; G->i0 = (int)floor(cx / dw) - GRID_N / 2; G->j0 = (int)floor(cy / dw) - GRID_N / 2;
    memset(G->have, 0, sizeof G->have); G->P = NULL;
}
/* grid_init plus the wheel's persistent node cache when the host declared the road static (DC_ROAD_STATIC) */
static inline void grid_init_w(rgrid *G, dc_car *c, int wheel, double cx, double cy, double dw) {
    grid_init(G, cx, cy, dw);
    if ((c->road_flags & DC_ROAD_STATIC) && c->rc) {
        dc_rcache *P = &c->rc[wheel];
        if (P->dw != dw) { for (int k = 0; k < RC_N * RC_N; k++) P->e[k].i = INT_MIN; P->dw = dw; }
        G->P = P;
    }
}
/* road node (i,j) of the world lattice, through the persistent cache if any (bit-identical: same call, same arguments) */
static inline double grid_fetch(rgrid *G, const dc_car *c, int i, int j, double *g) {
    dc_rcache *P = G->P;
    if (P) {
        dc_rc_ent *e = &P->e[(j & (RC_N - 1)) * RC_N + (i & (RC_N - 1))];
        if (e->i == i && e->j == j) { *g = e->g; return e->h; }
        double h = dc_road_height(c, i * G->dw, j * G->dw, g);
        e->i = i; e->j = j; e->h = h; e->g = *g; return h;
    }
    return dc_road_height(c, i * G->dw, j * G->dw, g);
}
static inline double grid_node(rgrid *G, const dc_car *c, int i, int j, double *g) {
    int li = i - G->i0, lj = j - G->j0;
    if (li >= 0 && lj >= 0 && li < GRID_N && lj < GRID_N) {
        if (!G->have[li][lj]) { G->h[li][lj] = grid_fetch(G, c, i, j, &G->g[li][lj]); G->have[li][lj] = 1; }
        *g = G->g[li][lj]; return G->h[li][lj];
    }
    return grid_fetch(G, c, i, j, g);
}
/* bilinear road height H(x,y) on the memo grid, with a one-cell cache: consecutive stations of a lane mostly fall in the same road cell (station pitch ~dw/6) */
typedef struct { int i, j; double h00, h10, h01, h11, g00, g10, g01, g11; } hcell;
static inline double grid_hc(rgrid *G, hcell *C, const dc_car *c, double x, double y, double *Hx, double *Hy, double *grip) {
    double fx = x * G->idw, fy = y * G->idw;
    int i = ifloor(fx), j = ifloor(fy); fx -= i; fy -= j;
    if (i != C->i || j != C->j) {
        int li = i - G->i0, lj = j - G->j0;
        if (li >= 0 && lj >= 0 && li < GRID_N - 1 && lj < GRID_N - 1) {
            for (int a = 0; a < 2; a++) for (int b = 0; b < 2; b++)
                if (!G->have[li + a][lj + b]) { G->h[li + a][lj + b] = grid_fetch(G, c, i + a, j + b, &G->g[li + a][lj + b]); G->have[li + a][lj + b] = 1; }
            C->h00 = G->h[li][lj]; C->h10 = G->h[li + 1][lj]; C->h01 = G->h[li][lj + 1]; C->h11 = G->h[li + 1][lj + 1];
            C->g00 = G->g[li][lj]; C->g10 = G->g[li + 1][lj]; C->g01 = G->g[li][lj + 1]; C->g11 = G->g[li + 1][lj + 1];
        } else {
            C->h00 = grid_node(G, c, i, j, &C->g00); C->h10 = grid_node(G, c, i + 1, j, &C->g10);
            C->h01 = grid_node(G, c, i, j + 1, &C->g01); C->h11 = grid_node(G, c, i + 1, j + 1, &C->g11);
        }
        C->i = i; C->j = j;
    }
    double d0 = C->h10 - C->h00, d1 = C->h11 - C->h01;
    double hy0 = C->h00 + fx * d0, hy1 = C->h01 + fx * d1;
    *Hx = ((1 - fy) * d0 + fy * d1) * G->idw;
    *Hy = (hy1 - hy0) * G->idw;
    *grip = (1 - fy) * (C->g00 + fx * (C->g10 - C->g00)) + fy * (C->g01 + fx * (C->g11 - C->g01));
    return hy0 + fy * (hy1 - hy0);
}
#endif
