/* lane_tests.c - the compute-offload lane's backends give identical bits (cpu, cpu_simd 4- and 8-wide, gpu if an
   OpenCL device is present), on random rough patches and ray fans. */
#include "../core/src/dc_lane.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
static unsigned S = 1;
static float rnd(void) { S = S * 1664525u + 1013904223u; return (float)(S >> 8) / 16777216.0f; }
int main(void) {
    static dc_ptab T; static dc_rayb R0, R[4];
    dc_lane* L = dc_lane_create(); int bad = 0, hits = 0, fb = 0, tot = 0;
    int have_gpu = dc_lane_set(L, DC_LANE_PATCH, DC_OFFLOAD_GPU) == DC_OFFLOAD_GPU;
    dc_offload_stats st; dc_lane_stats(L, 0, &st);
    printf("gpu: %s\n", have_gpu ? st.device : "none");
    for (int trial = 0; trial < 400; trial++) {
        T.ni = 20 + trial % 17; T.nj = 18 + trial % 23; T.idw = 40; T.gax = NULL;
        float amp = trial % 4 == 0 ? 0.0f : trial % 4 == 1 ? 0.002f : trial % 4 == 2 ? 0.02f : 0.08f;
        for (int k = 0; k < T.ni * T.nj; k++) { T.h[k] = -0.28f + amp * rnd() + (trial % 5 == 0 ? 0.001f * (k % T.ni) : 0); T.g[k] = 0.8f + 0.2f * rnd(); }
        R0.tmax = 0.3f; R0.n = 0;
        for (int r = 0; r < 300 + trial % 100; r++) {
            float a = (rnd() - 0.5f) * 2.3f, b = (rnd() - 0.5f) * 0.4f;
            R0.u0[r] = T.ni * (0.3f + 0.4f * rnd()); R0.v0[r] = T.nj * (0.3f + 0.4f * rnd()); R0.z0[r] = 0.01f * (rnd() - 0.5f);
            R0.du[r] = 40 * sinf(a) * cosf(b); R0.dv[r] = trial % 7 == 0 ? 0 : 40 * sinf(b); R0.dz[r] = -cosf(a) * cosf(b);
            if (trial % 11 == 0) R0.du[r] = 0;
            R0.ts[r] = 0.2f * rnd(); R0.n = r + 1;
        }
        int bks[4] = {DC_OFFLOAD_CPU, DC_OFFLOAD_SIMD, DC_OFFLOAD_SIMD, DC_OFFLOAD_GPU};
        for (int q = 0; q < 4; q++) {
            if (q == 3 && !have_gpu) { R[3] = R[0]; continue; }
            if (q == 2) setenv("DC_LANE_SIMDW", "4", 1); else unsetenv("DC_LANE_SIMDW");
            dc_lane* M = q == 3 ? L : dc_lane_create();
            dc_lane_set(M, DC_LANE_PATCH, bks[q]);
            R[q] = R0; dc_lane_patch(M, 0, &T, &R[q]);
            if (q != 3) dc_lane_destroy(M);
        }
        for (int q = 1; q < 4; q++) {
            int n = R0.n;
            if (memcmp(R[q].th, R[0].th, 4 * n) || memcmp(R[q].hx, R[0].hx, 4 * n) || memcmp(R[q].hy, R[0].hy, 4 * n) || memcmp(R[q].gr, R[0].gr, 4 * n) || memcmp(R[q].st, R[0].st, 4 * n)) {
                int r = 0; while (r < n && R[q].th[r] == R[0].th[r] && R[q].st[r] == R[0].st[r] && R[q].hx[r] == R[0].hx[r] && R[q].gr[r] == R[0].gr[r]) r++;
                if (bad < 10) printf("trial %d backend %d differs at ray %d: th %.9g vs %.9g st %d vs %d\n", trial, q, r, R[q].th[r], R[0].th[r], R[q].st[r], R[0].st[r]);
                bad++;
            }
        }
        for (int r = 0; r < R0.n; r++) { hits += R[0].st[r] > 0; fb += R[0].st[r] < 0; tot++; }
    }
    printf("rays %d hits %d handed back %d\n", tot, hits, fb);
    printf("lane_tests: %s (%d mismatching batches)\n", bad ? "FAIL" : "PASS", bad);
    dc_lane_destroy(L);
    return bad != 0;
}
