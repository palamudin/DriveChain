/* roadbake.c - prewarm-baked road lattice; see roadbake.h */
#include "roadbake.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#define TN 64                       /* nodes per tile side (1.6 m at 25 mm) */
typedef struct { int ti, tj; double base, q; void* h; unsigned char* g; } rb_tile;
struct roadbake {
    const track_t* T; double dw; int bits;
    rb_tile* tile; int nt; int* hash; int hmask;
    double pal[256]; int npal;
    unsigned char* pool;
    roadbake_stats st;
};
static inline unsigned hkey(int ti, int tj) { return (unsigned)ti * 73856093u ^ (unsigned)tj * 19349663u; }
static inline int fdiv(int a, int b) { return a >= 0 ? a / b : -((-a + b - 1) / b); }
static int hfind(const roadbake* B, int ti, int tj) {
    for (unsigned k = hkey(ti, tj) & B->hmask;; k = (k + 1) & B->hmask) {
        int t = B->hash[k]; if (t < 0) return -1;
        if (B->tile[t].ti == ti && B->tile[t].tj == tj) return t;
    }
}
static double now_s(void) { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return ts.tv_sec + 1e-9 * ts.tv_nsec; }
static double live(const track_t* T, double x, double y, double* g) { *g = 1; return track_road_fn((void*)T, x, y, NULL, NULL, NULL, g); }

roadbake* roadbake_build(const track_t* T, double dw, int bits, double band, char* err, int errn) {
    double t0 = now_s();
    if (bits != 16 && bits != 32) { snprintf(err, errn, "bake bits %d (16 or 32)", bits); return NULL; }
    if (T->n < 2 || T->length <= 0) { snprintf(err, errn, "track has no centreline"); return NULL; }
    roadbake* B = (roadbake*)calloc(1, sizeof *B); B->T = T; B->dw = dw; B->bits = bits;
    /* mark: tiles touched by the band (0.25 m probes, then one tile of margin so a wheel at the band edge stays inside) */
    int cap = 1 << 12; B->hmask = 2 * cap - 1; B->hash = (int*)malloc(sizeof(int) * 2 * cap); memset(B->hash, -1, sizeof(int) * 2 * cap);
    B->tile = (rb_tile*)malloc(sizeof(rb_tile) * cap);
    const double tw = TN * dw;
    for (double s = 0; s < T->length; s += 0.25) for (double d = -band; d <= band + 1e-9; d += 0.25) {
        double x, y; track_point(T, s, d, &x, &y, NULL);
        int ti0 = (int)floor(x / tw), tj0 = (int)floor(y / tw);
        for (int a = -1; a <= 1; a++) for (int b = -1; b <= 1; b++) {
            int ti = ti0 + a, tj = tj0 + b;
            if (hfind(B, ti, tj) >= 0) continue;
            if (B->nt == cap) {   /* grow and rehash */
                cap *= 2; B->tile = (rb_tile*)realloc(B->tile, sizeof(rb_tile) * cap);
                free(B->hash); B->hmask = 2 * cap - 1; B->hash = (int*)malloc(sizeof(int) * 2 * cap); memset(B->hash, -1, sizeof(int) * 2 * cap);
                for (int t = 0; t < B->nt; t++) { unsigned k = hkey(B->tile[t].ti, B->tile[t].tj) & B->hmask; while (B->hash[k] >= 0) k = (k + 1) & B->hmask; B->hash[k] = t; }
            }
            unsigned k = hkey(ti, tj) & B->hmask; while (B->hash[k] >= 0) k = (k + 1) & B->hmask;
            B->hash[k] = B->nt; B->tile[B->nt].ti = ti; B->tile[B->nt].tj = tj; B->nt++;
        }
    }
    const size_t hb = (size_t)TN * TN * (bits / 8), gb = (size_t)TN * TN;
    B->pool = (unsigned char*)malloc((hb + gb) * B->nt);
    if (!B->pool) { snprintf(err, errn, "bake: out of memory (%d tiles)", B->nt); roadbake_free(B); return NULL; }
    static double hv[TN * TN];
    double se = 0;
    for (int t = 0; t < B->nt; t++) {
        rb_tile* P = &B->tile[t]; P->h = B->pool + (hb + gb) * t; P->g = B->pool + (hb + gb) * t + hb;
        double mn = 1e30, mx = -1e30;
        for (int lj = 0; lj < TN; lj++) for (int li = 0; li < TN; li++) {
            int i = P->ti * TN + li, j = P->tj * TN + lj; double g;
            double h = live(T, i * dw, j * dw, &g); hv[lj * TN + li] = h; mn = fmin(mn, h); mx = fmax(mx, h);
            int p = 0; while (p < B->npal && B->pal[p] != g) p++;
            if (p == B->npal) { if (B->npal == 256) { snprintf(err, errn, "bake: more than 256 grip values"); roadbake_free(B); return NULL; } B->pal[B->npal++] = g; }
            P->g[lj * TN + li] = (unsigned char)p;
        }
        if (bits == 16) {   /* step: the smallest power of two that spans the tile in 65535 steps; base on that step's grid */
            int e; frexp(fmax(mx - mn, 1e-12) / 65535.0, &e); P->q = ldexp(1.0, e); if (P->q < 0x1p-30) P->q = 0x1p-30;
            P->base = floor(mn / P->q) * P->q;
            if ((mx - P->base) / P->q > 65535) P->q *= 2, P->base = floor(mn / P->q) * P->q;
        } else { P->q = 0x1p-10; P->base = floor(mn / P->q) * P->q; }   /* float32 offsets from a 2^-10 m aligned base */
        for (int k = 0; k < TN * TN; k++) {
            double dec;
            if (bits == 16) { uint16_t u = (uint16_t)lrint((hv[k] - P->base) / P->q); ((uint16_t*)P->h)[k] = u; dec = P->base + u * P->q; }
            else { float f = (float)(hv[k] - P->base); ((float*)P->h)[k] = f; dec = P->base + (double)f; }
            double e = fabs(dec - hv[k]); if (e > B->st.max_err) B->st.max_err = e; se += e * e;
            if (hv[k] == 0) { B->st.zeros++; if (dec != 0) { snprintf(err, errn, "bake: zero not exact"); roadbake_free(B); return NULL; } }
        }
    }
    B->st.bits = bits; B->st.tiles = B->nt; B->st.nodes = (long long)B->nt * TN * TN; B->st.grip_vals = B->npal;
    B->st.rms_err = sqrt(se / (double)(B->st.nodes ? B->st.nodes : 1));
    B->st.bytes = (long long)(hb + gb) * B->nt + (long long)sizeof(rb_tile) * B->nt + (long long)sizeof(int) * (B->hmask + 1);
    B->st.bake_s = now_s() - t0;
    return B;
}
void roadbake_free(roadbake* B) { if (!B) return; free(B->pool); free(B->tile); free(B->hash); free(B); }
void roadbake_get_stats(const roadbake* B, roadbake_stats* s) { if (B) *s = B->st; else memset(s, 0, sizeof *s); }
double roadbake_road_fn(void* user, double x, double y, double* nx, double* ny, double* nz, double* grip) {
    roadbake* B = (roadbake*)user;
    if (!nx) {
        const double idw = 1.0 / B->dw;
        int i = (int)lrint(x * idw), j = (int)lrint(y * idw);
        if (i * B->dw == x && j * B->dw == y) {   /* a lattice node, exactly as the tyre model forms it (i * dw) */
            int ti = fdiv(i, TN), tj = fdiv(j, TN), t = hfind(B, ti, tj);
            if (t >= 0) {
                const rb_tile* P = &B->tile[t]; const int k = (j - tj * TN) * TN + (i - ti * TN);
                if (grip) *grip = B->pal[P->g[k]];
                return B->bits == 16 ? P->base + ((const uint16_t*)P->h)[k] * P->q : P->base + (double)((const float*)P->h)[k];
            }
        }
    }
    return track_road_fn((void*)B->T, x, y, nx, ny, nz, grip);
}
