/* meshdiag - quality report for a mesh map (demo/meshtrack): open edges, flipped / vertical / degenerate faces,
   holes and seam steps along the driven band, stacked layers.  Usage: meshdiag <map dir> [out.csv]
   The CSV has one row per band sample with a problem (hole, crack fallback, step, layer). */
#include "../demo/track.h"
#include "../demo/meshtrack.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
static track_t TRK;
typedef struct { int a, b, t; } edge_t;
static int ecmp(const void* x, const void* y) { const edge_t *p = x, *q = y; return p->a != q->a ? (p->a < q->a ? -1 : 1) : (p->b != q->b ? (p->b < q->b ? -1 : 1) : 0); }
static const char* SN[] = {"tarmac", "kerb", "grass", "rough", "sausage", "line", "pave", "pad", "gravel", "wall", "other", "concrete"};
int main(int argc, char** argv) {
    if (argc < 2) { fprintf(stderr, "usage: meshdiag <map dir> [out.csv]\n"); return 2; }
    char err[256];
    if (!track_load_mesh(&TRK, argv[1], err, sizeof err)) { fprintf(stderr, "%s\n", err); return 1; }
    meshtrk_t* M = TRK.mesh;
    printf("map '%s': %d verts, %d tris, bbox x[%.1f %.1f] y[%.1f %.1f] z[%.2f %.2f]; centreline %.1f m\n", TRK.name, M->nv, M->nt,
           M->bmin[0], M->bmax[0], M->bmin[1], M->bmax[1], M->bmin[2], M->bmax[2], TRK.length);
    int cnt[16] = {0}; for (int k = 0; k < M->nt; k++) cnt[M->s[k] & 15]++;
    for (int i = 0; i < 12; i++) if (cnt[i]) printf("  %-9s %7d tris  grip %.2f\n", SN[i], cnt[i], M->grip[i]);
    printf("faces: %d degenerate, %d ground faces wound downward (flipped; normal used upward), %d ground faces steeper than 87 deg, %d wall\n",
           M->n_degenerate, M->n_flipped, M->n_vertical, M->n_wall);
    /* open edges of the ground surface (by vertex index; a seam between separately exported pieces shows up here
       even when it is geometrically closed, so also test by position) */
    edge_t* E = malloc(sizeof(edge_t) * 3 * (size_t)M->nt); int ne = 0;
    /* weld by position (1 mm) */
    int* weld = malloc(sizeof(int) * M->nv);
    { typedef struct { long long x, y, z; int i; } key_t; key_t* K = malloc(sizeof(key_t) * M->nv);
      for (int i = 0; i < M->nv; i++) { K[i].x = llround(M->v[3 * i] * 1000); K[i].y = llround(M->v[3 * i + 1] * 1000); K[i].z = llround(M->v[3 * i + 2] * 1000); K[i].i = i; }
      int kc(const void* a, const void* b) { const key_t *p = a, *q = b; if (p->x != q->x) return p->x < q->x ? -1 : 1; if (p->y != q->y) return p->y < q->y ? -1 : 1; if (p->z != q->z) return p->z < q->z ? -1 : 1; return 0; }
      qsort(K, M->nv, sizeof(key_t), kc);
      for (int i = 0; i < M->nv; i++) weld[K[i].i] = (i > 0 && !kc(&K[i], &K[i - 1])) ? weld[K[i - 1].i] : K[i].i;
      free(K); }
    for (int k = 0; k < M->nt; k++) { if (M->s[k] == SURF_WALL) continue;
        for (int e = 0; e < 3; e++) { int a = weld[M->t[3 * k + e]], b = weld[M->t[3 * k + (e + 1) % 3]]; if (a == b) continue; E[ne].a = a < b ? a : b; E[ne].b = a < b ? b : a; E[ne].t = k; ne++; } }
    qsort(E, ne, sizeof(edge_t), ecmp);
    int nopen = 0, nnm = 0, nopen_band = 0; double lopen = 0, lopen_band = 0;
    for (int i = 0; i < ne;) { int j = i; while (j < ne && !ecmp(&E[i], &E[j])) j++;
        if (j - i == 1) { const double *a = &M->v[3 * E[i].a], *b = &M->v[3 * E[i].b]; double L = sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
            nopen++; lopen += L; double d, s; track_height(&TRK, 0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1]), NULL, NULL, &d, &s);
            if (fabs(d) < TRK.half_w + 2) { nopen_band++; lopen_band += L; } }
        else if (j - i > 2) nnm++;
        i = j; }
    printf("edges (welded 1 mm): %d open edges (%.0f m), %d of them within the driven band |d| < %.1f m (%.1f m); %d non-manifold edges\n",
           nopen, lopen, nopen_band, TRK.half_w + 2, lopen_band, nnm);
    /* band scan: every 5 cm along and across the centreline, |d| <= half_w + 3 m */
    FILE* o = argc > 2 ? fopen(argv[2], "w") : NULL; if (o) fprintf(o, "s,d,x,y,kind,value,surf\n");
    double ds = 0.05, W = TRK.half_w + 3; long ns = 0, nhole = 0, ncrack = 0, nstep = 0, nlayer = 0; double maxstep = 0, maxstep_s = 0, maxstep_d = 0;
    long scount[16] = {0};
    int nd = (int)(2 * W / ds) + 1; double* prev = malloc(sizeof(double) * nd); int* prevs = malloc(sizeof(int) * nd);
    for (double s = 0; s < TRK.length; s += ds) {
        double hrow_prev = 0; int srow_prev = -1;
        for (int i = 0; i < nd; i++) {
            double d = -W + i * ds, x, y; track_point(&TRK, s, d, &x, &y, NULL);
            double z[16]; int tr[16]; int nl = mt_layers(M, x, y, z, tr, 16); int sf; double h = mt_height(M, x, y, NULL, &sf, NULL, NULL);
            ns++; if (fabs(d) <= TRK.half_w) scount[sf & 15]++;
            if (h < M->bmin[2] - 1) { nhole++; if (o) fprintf(o, "%.2f,%.2f,%.3f,%.3f,hole,0,%d\n", s, d, x, y, sf); }
            else if (nl == 0) { ncrack++; if (o) fprintf(o, "%.2f,%.2f,%.3f,%.3f,crack,0,%d\n", s, d, x, y, sf); }
            if (nl > 1) { double mx = -1e9, mn = 1e9; for (int q = 0; q < nl && q < 16; q++) { mx = fmax(mx, z[q]); mn = fmin(mn, z[q]); }
                if (mx - mn > 0.3) { nlayer++; if (o) fprintf(o, "%.2f,%.2f,%.3f,%.3f,layer,%.3f,%d\n", s, d, x, y, mx - mn, sf); } }
            /* steps: height jump between neighbouring samples (5 cm) across or along, on road surfaces only */
            int road = sf == SURF_TARMAC || sf == SURF_KERB || sf == SURF_CONCRETE || sf == SURF_PAVE;
            if (h > M->bmin[2] - 1 && road && fabs(d) <= TRK.half_w) {
                double st = 0; if (i > 0 && srow_prev >= 0) st = fabs(h - hrow_prev); if (s > 0 && prevs[i] >= 0) st = fmax(st, fabs(h - prev[i]));
                if (st > 0.015) { nstep++; if (o) fprintf(o, "%.2f,%.2f,%.3f,%.3f,step,%.4f,%d\n", s, d, x, y, st, sf); }
                if (st > maxstep) { maxstep = st; maxstep_s = s; maxstep_d = d; }
            }
            prev[i] = h; prevs[i] = road ? 1 : -1; hrow_prev = h; srow_prev = road ? 1 : -1;
        }
    }
    printf("band scan (%.0f cm grid, |d| <= %.1f m, %ld samples): %ld holes (> %.0f mm from any triangle), %ld cracks bridged (<= %.0f mm), %ld stacked-layer samples (> 0.3 m apart)\n",
           ds * 100, W, ns, nhole, MT_SEAM_TOL * 1e3, ncrack, MT_SEAM_TOL * 1e3, nlayer);
    printf("road steps > 15 mm between 5 cm neighbours inside |d| <= %.1f m: %ld (max %.1f mm at s %.1f d %+.2f)\n", TRK.half_w, nstep, maxstep * 1e3, maxstep_s, maxstep_d);
    long tot = 0; for (int i = 0; i < 16; i++) tot += scount[i];
    printf("surface under the band |d| <= %.1f m:", TRK.half_w); for (int i = 0; i < 12; i++) if (scount[i]) printf(" %s %.1f%%", SN[i], 100.0 * scount[i] / tot); printf("\n");
    if (o) fclose(o);
    return 0;
}
