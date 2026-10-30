/* dc_lane.h - compute-offload lane (internal). A stage packs its inputs into batched SoA buffers (the "pack" step,
   done by the caller), the lane runs the batch on the stage's backend (cpu: scalar float32, cpu_simd: the same
   arithmetic 4-wide SSE2 or 8-wide AVX2 by runtime dispatch, gpu: OpenCL compute, loaded at run time) and the caller
   maps the results back ("unpack"). Backends are bit-identical by construction (same IEEE single operations in the
   same order, no contraction); "auto" picks the fastest backend that reproduces the cpu result bit for bit on a
   self-test batch, so the choice never changes a result. First client: the TARMAC patch rays (dc_tyre_tm.c). */
#ifndef DC_LANE_H
#define DC_LANE_H
#include "drivechain.h"
enum { DC_LANE_PATCH, DC_LANE_NSTAGE };
#define DC_PT_MAXN 4096          /* patch-table nodes per wheel (64 x 64: 1.6 m square at 25 mm) */
#define DC_RB_MAX 480            /* rays per batch (>= DC_TM_NR * DC_TM_NL), multiple of 8 */
#define DC_LANE_GUARD 64         /* cell steps per ray before it is handed back (st = -1) */
/* patch table: the road nodes the footprint can touch, float32 heights relative to the wheel centre and grips,
   node (a, b) at [b * ni + a]; x, y in cells of the road lattice. gax is the reserved grip-axis slot (wet grass /
   aquaplaning-to-drag-strip range), not built yet: NULL. */
typedef struct { int ni, nj; float idw; const float* gax; float h[DC_PT_MAXN], g[DC_PT_MAXN]; } dc_ptab;
/* ray batch: origin (u0, v0) in cells, z0 in m relative to the table, direction (du, dv) in cells per m, dz; ray
   parameter t in m from ts to tmax. Out: st 1 hit (th, slopes hx hy per m, grip gr), 0 miss, -1 left the table */
typedef struct {
    int n; float tmax;
    float u0[DC_RB_MAX], v0[DC_RB_MAX], z0[DC_RB_MAX], du[DC_RB_MAX], dv[DC_RB_MAX], dz[DC_RB_MAX], ts[DC_RB_MAX];
    float th[DC_RB_MAX], hx[DC_RB_MAX], hy[DC_RB_MAX], gr[DC_RB_MAX]; int st[DC_RB_MAX];
} dc_rayb;
/* per-wheel scratch of the patch stage (allocated with the lane, outside the step) */
typedef struct dc_lanew { dc_ptab T; dc_rayb R; unsigned short jk[DC_RB_MAX]; } dc_lanew;
typedef struct dc_lane dc_lane;
dc_lane* dc_lane_create(void);
void     dc_lane_destroy(dc_lane* L);
int      dc_lane_set(dc_lane* L, int stage, int backend);   /* DC_OFFLOAD_*; returns the backend in use */
int      dc_lane_get(const dc_lane* L, int stage);
/* run one batch of the patch stage on wheel w's backend context (thread-safe across distinct w) */
void     dc_lane_patch(dc_lane* L, int w, const dc_ptab* T, dc_rayb* R);
void     dc_lane_stats(const dc_lane* L, int stage, dc_offload_stats* out);
const char* dc_lane_backend_name(int backend);
#endif
