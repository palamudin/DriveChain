#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200112L
#endif
/* dc_lane.c - see dc_lane.h. */
#include "dc_lane.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif
double dc_wall(void);

/* ---- cpu: one ray, scalar float32. The vector kernels (dc_lane_kern.h) perform these operations element-wise, in
   this order; keep the two in step. */
static void ray1(const dc_ptab* T, dc_rayb* R, int r) {
    const int ni = T->ni, nj = T->nj; const float tmax = R->tmax, idw = T->idw;
    const float u0 = R->u0[r], v0 = R->v0[r], z0 = R->z0[r], du = R->du[r], dv = R->dv[r], dz = R->dz[r];
    const int sx = du > 0 ? 1 : (du < 0 ? -1 : 0), sy = dv > 0 ? 1 : (dv < 0 ? -1 : 0);
    float t = R->ts[r], pu = u0 + t * du, pv = v0 + t * dv;
    int i = (int)pu, j = (int)pv; i -= pu < (float)i; j -= pv < (float)j;
    R->st[r] = 0; R->th[r] = R->hx[r] = R->hy[r] = R->gr[r] = 0;
    for (int g = 0; g < DC_LANE_GUARD; g++) {
        if (!(t <= tmax)) return;
        if (i < 0 || j < 0 || i >= ni - 1 || j >= nj - 1) { R->st[r] = -1; return; }
        const float fi = (float)i, fj = (float)j;
        float tnx = sx > 0 ? (fi + 1.0f - u0) / du : (sx < 0 ? (fi - u0) / du : 1e30f);
        float tny = sy > 0 ? (fj + 1.0f - v0) / dv : (sy < 0 ? (fj - v0) / dv : 1e30f);
        float tm = tnx < tny ? tnx : tny, t1 = tm < tmax ? tm : tmax; if (t1 < t) t1 = t;
        const int q = j * ni + i;
        float h00 = T->h[q], h10 = T->h[q + 1], h01 = T->h[q + ni], h11 = T->h[q + ni + 1];
        float A = h00, Bc = h10 - h00, Cc = h01 - h00, Dc = h11 - h10 - h01 + h00;
        float uu = u0 - fi, vv = v0 - fj;
        float c0 = z0 - (A + Bc * uu + Cc * vv + Dc * uu * vv);
        float c1 = dz - (Bc * du + Cc * dv + Dc * (uu * dv + vv * du));
        float c2 = (0.0f - Dc) * du * dv;
        float fa = c0 + t * (c1 + t * c2), fb = c0 + t1 * (c1 + t1 * c2);
        int h1 = fa <= 0, h2 = !h1 && fb <= 0, h3 = 0; float tb = t1, ftb = fb;
        if (!h1 && !h2 && c2 > 0) {
            float tv = (0.0f - c1) / (2.0f * c2), fv = c0 + tv * (c1 + tv * c2);
            if (tv > t && tv < t1 && fv <= 0) { h3 = 1; tb = tv; ftb = fv; }
        }
        if (h1 || h2 || h3) {
            float rr = t;
            if (!h1) {
                float lo = t, hi = tb; rr = t + (tb - t) * fa / (fa - ftb);
                for (int it = 0; it < 3; it++) {
                    float fr = c0 + rr * (c1 + rr * c2), dfr = c1 + 2.0f * c2 * rr;
                    if (fr > 0) lo = rr; else if (fr < 0) hi = rr; else { lo = rr; hi = rr; }
                    float mid = 0.5f * (lo + hi), rn = dfr != 0 ? rr - fr / dfr : mid;
                    rr = (rn > lo && rn < hi) ? rn : mid;
                }
            }
            float u = uu + rr * du, v = vv + rr * dv;
            float g00 = T->g[q], g10 = T->g[q + 1], g01 = T->g[q + ni], g11 = T->g[q + ni + 1];
            R->th[r] = rr; R->hx[r] = (Bc + Dc * v) * idw; R->hy[r] = (Cc + Dc * u) * idw;
            R->gr[r] = (1.0f - v) * (g00 + u * (g10 - g00)) + v * (g01 + u * (g11 - g01));
            R->st[r] = 1; return;
        }
        if (t1 >= tmax) return;
        t = t1;
        if (tnx <= tny) i += sx; else j += sy;
        if (tnx == tny && sx && sy) j += sy;
    }
    R->st[r] = -1;
}
static void patch_cpu(const dc_ptab* T, dc_rayb* R) { for (int r = 0; r < R->n; r++) ray1(T, R, r); }

#define LK_CAT2(a, b) a##b
#define LK_CAT(a, b) LK_CAT2(a, b)
#define LK_W 4
#define LK_NAME patch_v4
#define LK_ATTR
#include "dc_lane_kern.h"
#undef LK_W
#undef LK_NAME
#undef LK_ATTR
#if defined(__x86_64__) || defined(__i386__)
#define LK_W 8
#define LK_NAME patch_v8
#define LK_ATTR __attribute__((target("avx2")))
#include "dc_lane_kern.h"
#undef LK_W
#undef LK_NAME
#undef LK_ATTR
#define HAVE_V8 1
#endif

/* ---- gpu: OpenCL 1.2, loaded at run time (no SDK or import library needed to build; absent runtime = no gpu) */
typedef int cl_int; typedef unsigned cl_uint; typedef unsigned long long cl_ulong; typedef cl_ulong cl_bitfield;
typedef void *cl_platform_id, *cl_device_id, *cl_context, *cl_command_queue, *cl_mem, *cl_program, *cl_kernel, *cl_event;
#define CL_DEVICE_TYPE_GPU (1 << 2)
#define CL_DEVICE_TYPE_ALL 0xFFFFFFFF
#define CL_DEVICE_NAME 0x102B
#define CL_MEM_READ_ONLY (1 << 2)
#define CL_MEM_WRITE_ONLY (1 << 1)
#define CL_PROGRAM_BUILD_LOG 0x1183
typedef struct {
    cl_int (*GetPlatformIDs)(cl_uint, cl_platform_id*, cl_uint*);
    cl_int (*GetDeviceIDs)(cl_platform_id, cl_bitfield, cl_uint, cl_device_id*, cl_uint*);
    cl_int (*GetDeviceInfo)(cl_device_id, cl_uint, size_t, void*, size_t*);
    cl_context (*CreateContext)(const void*, cl_uint, const cl_device_id*, void*, void*, cl_int*);
    cl_command_queue (*CreateCommandQueue)(cl_context, cl_device_id, cl_bitfield, cl_int*);
    cl_program (*CreateProgramWithSource)(cl_context, cl_uint, const char**, const size_t*, cl_int*);
    cl_int (*BuildProgram)(cl_program, cl_uint, const cl_device_id*, const char*, void*, void*);
    cl_int (*GetProgramBuildInfo)(cl_program, cl_device_id, cl_uint, size_t, void*, size_t*);
    cl_kernel (*CreateKernel)(cl_program, const char*, cl_int*);
    cl_mem (*CreateBuffer)(cl_context, cl_bitfield, size_t, void*, cl_int*);
    cl_int (*SetKernelArg)(cl_kernel, cl_uint, size_t, const void*);
    cl_int (*EnqueueWriteBuffer)(cl_command_queue, cl_mem, cl_uint, size_t, size_t, const void*, cl_uint, const cl_event*, cl_event*);
    cl_int (*EnqueueReadBuffer)(cl_command_queue, cl_mem, cl_uint, size_t, size_t, void*, cl_uint, const cl_event*, cl_event*);
    cl_int (*EnqueueNDRangeKernel)(cl_command_queue, cl_kernel, cl_uint, const size_t*, const size_t*, const size_t*, cl_uint, const cl_event*, cl_event*);
    cl_int (*Finish)(cl_command_queue);
    cl_int (*ReleaseMemObject)(cl_mem); cl_int (*ReleaseKernel)(cl_kernel); cl_int (*ReleaseProgram)(cl_program);
    cl_int (*ReleaseCommandQueue)(cl_command_queue); cl_int (*ReleaseContext)(cl_context);
} clfn;
/* the kernel: ray1() in OpenCL C, contraction off, correctly rounded division (requested at build) */
static const char* CL_SRC =
"#pragma OPENCL FP_CONTRACT OFF\n"
"__kernel void patch(__global const float* tab, __global const float* in, __global float* out, int ni, int nj, int n, float tmax, float idw) {\n"
" int r = get_global_id(0); if (r >= n) return;\n"
" __global const float* H = tab; __global const float* G = tab + ni * nj;\n"
" float u0 = in[r], v0 = in[n + r], z0 = in[2*n + r], du = in[3*n + r], dv = in[4*n + r], dz = in[5*n + r], t = in[6*n + r];\n"
" int sx = du > 0 ? 1 : (du < 0 ? -1 : 0), sy = dv > 0 ? 1 : (dv < 0 ? -1 : 0);\n"
" float pu = u0 + t * du, pv = v0 + t * dv; int i = (int)pu, j = (int)pv; i -= pu < (float)i; j -= pv < (float)j;\n"
" float st = 0, oth = 0, ohx = 0, ohy = 0, ogr = 0; int g;\n"
" for (g = 0; g < 64; g++) {\n"
"  if (!(t <= tmax)) break;\n"
"  if (i < 0 || j < 0 || i >= ni - 1 || j >= nj - 1) { st = -1; break; }\n"
"  float fi = (float)i, fj = (float)j;\n"
"  float tnx = sx > 0 ? (fi + 1.0f - u0) / du : (sx < 0 ? (fi - u0) / du : 1e30f);\n"
"  float tny = sy > 0 ? (fj + 1.0f - v0) / dv : (sy < 0 ? (fj - v0) / dv : 1e30f);\n"
"  float tm = tnx < tny ? tnx : tny, t1 = tm < tmax ? tm : tmax; if (t1 < t) t1 = t;\n"
"  int q = j * ni + i;\n"
"  float h00 = H[q], h10 = H[q + 1], h01 = H[q + ni], h11 = H[q + ni + 1];\n"
"  float A = h00, Bc = h10 - h00, Cc = h01 - h00, Dc = h11 - h10 - h01 + h00;\n"
"  float uu = u0 - fi, vv = v0 - fj;\n"
"  float c0 = z0 - (A + Bc * uu + Cc * vv + Dc * uu * vv);\n"
"  float c1 = dz - (Bc * du + Cc * dv + Dc * (uu * dv + vv * du));\n"
"  float c2 = (0.0f - Dc) * du * dv;\n"
"  float fa = c0 + t * (c1 + t * c2), fb = c0 + t1 * (c1 + t1 * c2);\n"
"  int h1 = fa <= 0, h2 = !h1 && fb <= 0, h3 = 0; float tb = t1, ftb = fb;\n"
"  if (!h1 && !h2 && c2 > 0) { float tv = (0.0f - c1) / (2.0f * c2), fv = c0 + tv * (c1 + tv * c2); if (tv > t && tv < t1 && fv <= 0) { h3 = 1; tb = tv; ftb = fv; } }\n"
"  if (h1 || h2 || h3) {\n"
"   float rr = t;\n"
"   if (!h1) { float lo = t, hi = tb; rr = t + (tb - t) * fa / (fa - ftb);\n"
"    for (int it = 0; it < 3; it++) { float fr = c0 + rr * (c1 + rr * c2), dfr = c1 + 2.0f * c2 * rr;\n"
"     if (fr > 0) lo = rr; else if (fr < 0) hi = rr; else { lo = rr; hi = rr; }\n"
"     float mid = 0.5f * (lo + hi), rn = dfr != 0 ? rr - fr / dfr : mid; rr = (rn > lo && rn < hi) ? rn : mid; } }\n"
"   float u = uu + rr * du, v = vv + rr * dv;\n"
"   float g00 = G[q], g10 = G[q + 1], g01 = G[q + ni], g11 = G[q + ni + 1];\n"
"   oth = rr; ohx = (Bc + Dc * v) * idw; ohy = (Cc + Dc * u) * idw;\n"
"   ogr = (1.0f - v) * (g00 + u * (g10 - g00)) + v * (g01 + u * (g11 - g01)); st = 1; break; }\n"
"  if (t1 >= tmax) break;\n"
"  t = t1; if (tnx <= tny) i += sx; else j += sy; if (tnx == tny && sx && sy) j += sy;\n"
" }\n"
" if (g == 64) st = -1;\n"
" out[r] = oth; out[n + r] = ohx; out[2*n + r] = ohy; out[3*n + r] = ogr; out[4*n + r] = st;\n"
"}\n";
typedef struct { cl_command_queue q; cl_kernel k; cl_mem tab, in, out; } clw;   /* per car and wheel */
/* process-wide: runtime, context and program are opened once and kept to exit (cars are created and destroyed often) */
typedef struct { int ok, tried; void* lib; clfn f; cl_context ctx; cl_device_id dev; cl_program prog; char name[96]; } clstate;
static clstate CLG;

static void* sym(void* lib, const char* s) {
#ifdef _WIN32
    return (void*)GetProcAddress((HMODULE)lib, s);
#else
    return dlsym(lib, s);
#endif
}
static void clw_close(clw* W) {
    if (W->tab) CLG.f.ReleaseMemObject(W->tab);
    if (W->in) CLG.f.ReleaseMemObject(W->in);
    if (W->out) CLG.f.ReleaseMemObject(W->out);
    if (W->k) CLG.f.ReleaseKernel(W->k);
    if (W->q) CLG.f.ReleaseCommandQueue(W->q);
    memset(W, 0, sizeof *W);
}
static int clw_open(clw* W) {
    cl_int e;
    W->q = CLG.f.CreateCommandQueue(CLG.ctx, CLG.dev, 0, &e);
    W->k = CLG.f.CreateKernel(CLG.prog, "patch", &e);
    W->tab = CLG.f.CreateBuffer(CLG.ctx, CL_MEM_READ_ONLY, sizeof(float) * 2 * DC_PT_MAXN, NULL, &e);
    W->in = CLG.f.CreateBuffer(CLG.ctx, CL_MEM_READ_ONLY, sizeof(float) * 7 * DC_RB_MAX, NULL, &e);
    W->out = CLG.f.CreateBuffer(CLG.ctx, CL_MEM_WRITE_ONLY, sizeof(float) * 5 * DC_RB_MAX, NULL, &e);
    if (!W->q || !W->k || !W->tab || !W->in || !W->out) { clw_close(W); return 0; }
    return 1;
}
static int cl_open(void) {
    clstate* G = &CLG;
    if (G->tried) return G->ok;
    G->tried = 1;
    if (getenv("DC_LANE_NOGPU")) return 0;
#ifdef _WIN32
    G->lib = (void*)LoadLibraryA("OpenCL.dll");
#else
    G->lib = dlopen("libOpenCL.so.1", RTLD_NOW); if (!G->lib) G->lib = dlopen("libOpenCL.so", RTLD_NOW);
#endif
    if (!G->lib) return 0;
    void** fp = (void**)&G->f;
    static const char* names[] = { "clGetPlatformIDs", "clGetDeviceIDs", "clGetDeviceInfo", "clCreateContext", "clCreateCommandQueue",
        "clCreateProgramWithSource", "clBuildProgram", "clGetProgramBuildInfo", "clCreateKernel", "clCreateBuffer", "clSetKernelArg",
        "clEnqueueWriteBuffer", "clEnqueueReadBuffer", "clEnqueueNDRangeKernel", "clFinish", "clReleaseMemObject", "clReleaseKernel",
        "clReleaseProgram", "clReleaseCommandQueue", "clReleaseContext" };
    for (size_t k = 0; k < sizeof names / sizeof *names; k++) { fp[k] = sym(G->lib, names[k]); if (!fp[k]) return 0; }
    cl_platform_id pl[8]; cl_uint np = 0; cl_int e;
    if (G->f.GetPlatformIDs(8, pl, &np) != 0 || np == 0) return 0;
    /* prefer a GPU on any platform; else any device (a CPU OpenCL runtime then exercises the path) */
    int found = 0;
    for (int pass = 0; pass < 2 && !found; pass++)
        for (cl_uint p = 0; p < np && p < 8 && !found; p++) {
            cl_uint nd = 0;
            if (G->f.GetDeviceIDs(pl[p], pass ? CL_DEVICE_TYPE_ALL : CL_DEVICE_TYPE_GPU, 1, &G->dev, &nd) == 0 && nd > 0) found = 1;
        }
    if (!found) return 0;
    G->f.GetDeviceInfo(G->dev, CL_DEVICE_NAME, sizeof G->name - 1, G->name, NULL);
    G->ctx = G->f.CreateContext(NULL, 1, &G->dev, NULL, NULL, &e); if (!G->ctx) return 0;
    G->prog = G->f.CreateProgramWithSource(G->ctx, 1, &CL_SRC, NULL, &e); if (!G->prog) return 0;
    if (G->f.BuildProgram(G->prog, 1, &G->dev, "-cl-fp32-correctly-rounded-divide-sqrt", NULL, NULL) != 0
        && G->f.BuildProgram(G->prog, 1, &G->dev, "", NULL, NULL) != 0) {
        char log[2048] = {0}; G->f.GetProgramBuildInfo(G->prog, G->dev, CL_PROGRAM_BUILD_LOG, sizeof log - 1, log, NULL);
        fprintf(stderr, "[lane] OpenCL build failed: %s\n", log); return 0;
    }
    G->ok = 1; return 1;
}
/* pack -> one write of the table, one of the rays, one launch, one blocking read -> unpack */
/* tph (optional): wall seconds of the three phases, accumulated: [0] pack + upload enqueue (table, rays),
   [1] launch + clFinish (the uploads and the kernel run on the device), [2] blocking readback + unpack */
static int patch_gpu(clw* W, const dc_ptab* T, dc_rayb* R, double* tph) {
    double w0 = tph ? dc_wall() : 0, w1 = 0, w2 = 0;
    clstate* G = &CLG; const int n = R->n, nn = T->ni * T->nj;
    float in[7 * DC_RB_MAX], out[5 * DC_RB_MAX];
    memcpy(in, R->u0, 4 * n); memcpy(in + n, R->v0, 4 * n); memcpy(in + 2 * n, R->z0, 4 * n); memcpy(in + 3 * n, R->du, 4 * n);
    memcpy(in + 4 * n, R->dv, 4 * n); memcpy(in + 5 * n, R->dz, 4 * n); memcpy(in + 6 * n, R->ts, 4 * n);
    cl_int e = 0;
    e |= G->f.EnqueueWriteBuffer(W->q, W->tab, 0, 0, 4 * nn, T->h, 0, NULL, NULL);
    e |= G->f.EnqueueWriteBuffer(W->q, W->tab, 0, 4 * nn, 4 * nn, T->g, 0, NULL, NULL);
    e |= G->f.EnqueueWriteBuffer(W->q, W->in, 0, 0, 4 * 7 * n, in, 0, NULL, NULL);
    if (tph) w1 = dc_wall();
    int ni = T->ni, nj = T->nj; float tmax = R->tmax, idw = T->idw;
    e |= G->f.SetKernelArg(W->k, 0, sizeof(cl_mem), &W->tab); e |= G->f.SetKernelArg(W->k, 1, sizeof(cl_mem), &W->in);
    e |= G->f.SetKernelArg(W->k, 2, sizeof(cl_mem), &W->out); e |= G->f.SetKernelArg(W->k, 3, sizeof ni, &ni);
    e |= G->f.SetKernelArg(W->k, 4, sizeof nj, &nj); e |= G->f.SetKernelArg(W->k, 5, sizeof n, &n);
    e |= G->f.SetKernelArg(W->k, 6, sizeof tmax, &tmax); e |= G->f.SetKernelArg(W->k, 7, sizeof idw, &idw);
    size_t gs = (size_t)((n + 63) / 64 * 64), ls = 64;
    e |= G->f.EnqueueNDRangeKernel(W->q, W->k, 1, NULL, &gs, &ls, 0, NULL, NULL);
    if (tph) { e |= G->f.Finish(W->q); w2 = dc_wall(); }   /* splits device time from the readback (the read below blocks anyway) */
    e |= G->f.EnqueueReadBuffer(W->q, W->out, 1, 0, 4 * 5 * n, out, 0, NULL, NULL);
    if (e) return 0;
    memcpy(R->th, out, 4 * n); memcpy(R->hx, out + n, 4 * n); memcpy(R->hy, out + 2 * n, 4 * n); memcpy(R->gr, out + 3 * n, 4 * n);
    for (int r = 0; r < n; r++) R->st[r] = (int)out[4 * n + r];
    if (tph) { double w3 = dc_wall(); tph[0] += w1 - w0; tph[1] += w2 - w1; tph[2] += w3 - w2; }
    return 1;
}

/* ---- the lane */
struct dc_lane {
    int bk[DC_LANE_NSTAGE], simdw;
    clw gw[4]; int gpu_ok;
    struct { long long calls, rays, fb; double s, mx; } st[DC_LANE_NSTAGE][4];   /* per stage, per wheel (one writer each) */
    double cal_us[3]; int cal_match[3];
    /* auto: the self-test batch only screens the backends (bit match, gross cost); the pick is made on the live
       batches. Each wheel cycles the candidates in blocks of AUTO_BLK calls for AUTO_N calls, then keeps the one
       with the lowest mean. Every candidate reproduces the cpu result bit for bit, so switching never changes a
       result and the run stays deterministic. */
    int autom[DC_LANE_NSTAGE], ncand[DC_LANE_NSTAGE], cand[DC_LANE_NSTAGE][3];
    struct { int b; long long n[3]; double s[3]; } au[DC_LANE_NSTAGE][4];
    /* instrumentation, per wheel (one writer each): live cost per backend actually run, gpu phase split and
       failed launches, batches skipped because the patch was planar (closed form, no rays marched) */
    struct { long long n[3]; double s[3]; } lv[4];
    struct { long long n, fail; double ph[3]; } gt[4];
    long long skip[4];
    /* same-tick cross-check: every xc_every-th batch of a non-cpu backend is re-run on the scalar cpu backend
       from the same inputs and compared (results stay the backend's own); scratch allocated with the lane */
    int xc_every;
    struct { long long n, flips; double dth, dsl; } xc[4];
    dc_rayb xr[4];
};
#define AUTO_BLK 32
#define AUTO_N (AUTO_BLK * 12)
const char* dc_lane_backend_name(int b) {
    return b == DC_OFFLOAD_REF ? "ref" : b == DC_OFFLOAD_CPU ? "cpu" : b == DC_OFFLOAD_SIMD ? "cpu_simd" : b == DC_OFFLOAD_GPU ? "gpu" : "auto";
}
dc_lane* dc_lane_create(void) {
    dc_lane* L = (dc_lane*)calloc(1, sizeof *L); if (!L) return NULL;
    for (int s = 0; s < DC_LANE_NSTAGE; s++) L->bk[s] = DC_OFFLOAD_REF;
    for (int b = 0; b < 3; b++) L->cal_us[b] = -1;   /* -1 = self-test not run (fixed backends skip it) */
#ifdef HAVE_V8
    __builtin_cpu_init(); L->simdw = __builtin_cpu_supports("avx2") ? 8 : 4;
#else
    L->simdw = 4;
#endif
    { const char* e = getenv("DC_LANE_SIMDW"); if (e && atoi(e) == 4) L->simdw = 4; }
    return L;
}
void dc_lane_destroy(dc_lane* L) { if (!L) return; if (L->gpu_ok) for (int w = 0; w < 4; w++) clw_close(&L->gw[w]); free(L); }
static int gpu_up(dc_lane* L) {
    if (!L->gpu_ok && cl_open()) {
        int ok = 1; for (int w = 0; w < 4; w++) ok &= clw_open(&L->gw[w]);
        if (!ok) for (int w = 0; w < 4; w++) clw_close(&L->gw[w]);
        L->gpu_ok = ok;
    }
    return L->gpu_ok;
}
static void run_patch(dc_lane* L, int bk, int w, const dc_ptab* T, dc_rayb* R) {
    if (bk == DC_OFFLOAD_GPU && L->gpu_ok) {
        if (patch_gpu(&L->gw[w], T, R, L->gt[w].ph)) { L->gt[w].n++; return; }
        L->gt[w].fail++;   /* launch or transfer error: this batch runs on cpu_simd (same bits) */
    }
    if (bk == DC_OFFLOAD_SIMD || bk == DC_OFFLOAD_GPU) {
        int n = R->n, np = (n + 7) & ~7;   /* pad to the vector width: padded rays start past tmax (st 0) */
        for (int r = n; r < np; r++) { R->u0[r] = R->v0[r] = R->z0[r] = R->du[r] = R->dv[r] = R->dz[r] = 0; R->ts[r] = R->tmax + 1; }
        R->n = np;
#ifdef HAVE_V8
        if (L->simdw == 8) patch_v8(T, R); else
#endif
        patch_v4(T, R);
        R->n = n; return;
    }
    patch_cpu(T, R);
}
/* synthetic self-test batch: a rough 30 x 30 patch and a fan of rays (deterministic) */
static void cal_batch(dc_ptab* T, dc_rayb* R) {
    T->ni = T->nj = 30; T->idw = 40; T->gax = NULL;
    unsigned s = 12345u;
    for (int k = 0; k < 900; k++) { s = s * 1664525u + 1013904223u; T->h[k] = -0.29f + 0.004f * (float)(s >> 8) / 16777216.0f; T->g[k] = 1.0f - 0.1f * (float)(k % 7) / 7; }
    R->tmax = 0.3f; R->n = 0;
    for (int l = 0; l < 7; l++) for (int k = 0; k < 66; k++) {
        int r = R->n++; float a = (-65 + 2 * k) * 0.01745f;
        R->u0[r] = 15 + 0.3f * l; R->v0[r] = 15.2f; R->z0[r] = 0;
        R->du[r] = 40 * __builtin_sinf(a); R->dv[r] = 0.3f; R->dz[r] = -__builtin_cosf(a); R->ts[r] = 0.25f;
    }
}
static double CAL_US[3]; static int CAL_MATCH[3], CAL_DONE;   /* process-wide: measured once */
static void calibrate(dc_lane* L, int stage) {
    if (CAL_DONE) { for (int b = 0; b < 3; b++) { L->cal_us[b] = CAL_US[b]; L->cal_match[b] = CAL_MATCH[b]; } if (CAL_US[2] > 0) gpu_up(L); return; }
    static dc_ptab T; static dc_rayb R, ref, X;   /* runs once, from the configuring thread */
    cal_batch(&T, &R); ref = R; patch_cpu(&T, &ref);
    for (int b = 0; b < 3; b++) {
        L->cal_us[b] = -1; L->cal_match[b] = 0;
        if (b == DC_OFFLOAD_GPU && !gpu_up(L)) continue;
        X = R; run_patch(L, b, 0, &T, &X);   /* warm up (gpu: first launch compiles/uploads) */
        int same = memcmp(X.th, ref.th, 4 * R.n) == 0 && memcmp(X.hx, ref.hx, 4 * R.n) == 0 && memcmp(X.hy, ref.hy, 4 * R.n) == 0
                && memcmp(X.gr, ref.gr, 4 * R.n) == 0 && memcmp(X.st, ref.st, sizeof(int) * R.n) == 0;
        double best = 1e30;
        for (int rep = 0; rep < 9; rep++) {
            double t0 = dc_wall();
            for (int k = 0; k < 8; k++) run_patch(L, b, 0, &T, &X);
            double dt = (dc_wall() - t0) / 8; if (dt < best) best = dt;
        }
        L->cal_us[b] = best * 1e6; L->cal_match[b] = same;
    }
    for (int b = 0; b < 3; b++) { CAL_US[b] = L->cal_us[b]; CAL_MATCH[b] = L->cal_match[b]; }
    memset(L->gt, 0, sizeof L->gt);   /* self-test launches are not live batches */
    CAL_DONE = 1;
    if (getenv("DC_LANE_VERBOSE")) fprintf(stderr, "[lane] patch calibration us/batch: cpu %.1f simd(w%d) %.1f%s gpu %.1f%s %s\n", CAL_US[0], L->simdw,
        CAL_US[1], CAL_MATCH[1] ? "" : " (MISMATCH)", CAL_US[2], CAL_US[2] > 0 && !CAL_MATCH[2] ? " (MISMATCH)" : "", CLG.name);
    (void)stage;
}
int dc_lane_set(dc_lane* L, int stage, int b) {
    if (!L || stage < 0 || stage >= DC_LANE_NSTAGE) return DC_OFFLOAD_REF;
    if (b == DC_OFFLOAD_AUTO) {
        calibrate(L, stage);
        int best = DC_OFFLOAD_CPU;
        for (int k = 1; k < 3; k++) if (L->cal_us[k] > 0 && L->cal_match[k] && L->cal_us[k] < L->cal_us[best]) best = k;
        b = best;
        /* live candidates: bit-matching backends within 3x of the best self-test cost (a far slower backend,
           e.g. a gpu whose launch latency dwarfs the batch, is not probed: it would spike the first ticks) */
        L->ncand[stage] = 0;
        for (int k = 0; k < 3; k++) if (L->cal_us[k] > 0 && (k == DC_OFFLOAD_CPU || L->cal_match[k]) && L->cal_us[k] <= 3 * L->cal_us[best]) L->cand[stage][L->ncand[stage]++] = k;
        L->autom[stage] = L->ncand[stage] > 1;
        memset(L->au[stage], 0, sizeof L->au[stage]); for (int w = 0; w < 4; w++) L->au[stage][w].b = -1;
    } else { L->autom[stage] = 0; if (b == DC_OFFLOAD_GPU && !gpu_up(L)) b = DC_OFFLOAD_SIMD; }   /* no OpenCL runtime/device: closest cpu backend */
    if (b < DC_OFFLOAD_REF || b > DC_OFFLOAD_GPU) b = DC_OFFLOAD_REF;
    L->bk[stage] = b; return b;
}
int dc_lane_get(const dc_lane* L, int stage) { return L ? L->bk[stage] : DC_OFFLOAD_REF; }
void dc_lane_patch(dc_lane* L, int w, const dc_ptab* T, dc_rayb* R) {
    int b = L->bk[DC_LANE_PATCH];
    const int sg = DC_LANE_PATCH;
    if (L->autom[sg]) {
        if (L->au[sg][w].b >= 0) b = L->au[sg][w].b;
        else b = L->cand[sg][(L->st[sg][w].calls / AUTO_BLK) % L->ncand[sg]];
    }
    double t0 = dc_wall();
    run_patch(L, b, w, T, R);
    double dt = dc_wall() - t0;
    if (L->autom[sg] && L->au[sg][w].b < 0) {
        L->au[sg][w].n[b]++; L->au[sg][w].s[b] += dt;
        if (L->st[sg][w].calls + 1 >= AUTO_N) {
            int bb = -1; double bm = 1e30;
            for (int k = 0; k < 3; k++) if (L->au[sg][w].n[k] > 0 && L->au[sg][w].s[k] / L->au[sg][w].n[k] < bm) { bm = L->au[sg][w].s[k] / L->au[sg][w].n[k]; bb = k; }
            L->au[sg][w].b = bb;
        }
    }
    L->lv[w].n[b]++; L->lv[w].s[b] += dt;
    if (L->xc_every > 0 && b != DC_OFFLOAD_CPU && L->st[sg][w].calls % L->xc_every == 0) {
        dc_rayb* X = &L->xr[w]; const int n = R->n;
        X->n = n; X->tmax = R->tmax;
        memcpy(X->u0, R->u0, 4 * n); memcpy(X->v0, R->v0, 4 * n); memcpy(X->z0, R->z0, 4 * n);
        memcpy(X->du, R->du, 4 * n); memcpy(X->dv, R->dv, 4 * n); memcpy(X->dz, R->dz, 4 * n); memcpy(X->ts, R->ts, 4 * n);
        patch_cpu(T, X);
        long long fl = 0; double dth = 0, dsl = 0;
        for (int r = 0; r < n; r++) {
            if (X->st[r] != R->st[r]) { fl++; continue; }
            if (R->st[r] != 1) continue;
            dth = fmax(dth, fabs((double)X->th[r] - R->th[r]));
            dsl = fmax(dsl, fmax(fabs((double)X->hx[r] - R->hx[r]), fmax(fabs((double)X->hy[r] - R->hy[r]), fabs((double)X->gr[r] - R->gr[r]))));
        }
        L->xc[w].n++; L->xc[w].flips += fl; L->xc[w].dth = fmax(L->xc[w].dth, dth); L->xc[w].dsl = fmax(L->xc[w].dsl, dsl);
    }
    int fb = 0; for (int r = 0; r < R->n; r++) fb += R->st[r] < 0;
    L->st[DC_LANE_PATCH][w].calls++; L->st[DC_LANE_PATCH][w].rays += R->n; L->st[DC_LANE_PATCH][w].fb += fb;
    L->st[DC_LANE_PATCH][w].s += dt; if (dt > L->st[DC_LANE_PATCH][w].mx) L->st[DC_LANE_PATCH][w].mx = dt;
}
void dc_lane_stats(const dc_lane* L, int stage, dc_offload_stats* o) {
    memset(o, 0, sizeof *o);
    if (!L) { o->backend = DC_OFFLOAD_REF; return; }
    o->backend = L->bk[stage]; o->simd_width = L->simdw; o->auto_sel = L->autom[stage] ? 1 : 0;
    if (L->autom[stage]) {   /* report the live pick (wheel 0; -1 while still probing) */
        o->backend = L->au[stage][0].b >= 0 ? L->au[stage][0].b : L->bk[stage]; if (L->au[stage][0].b < 0) o->auto_sel = 2;
    }
    for (int k = 0; k < 3; k++) {   /* live mean over all wheels of what actually ran (auto: probe batches included) */
        long long n = 0; double sm = 0; for (int w = 0; w < 4; w++) { n += L->lv[w].n[k]; sm += L->lv[w].s[k]; }
        o->live_us[k] = n ? 1e6 * sm / n : -1;
    }
    { long long gn = 0; double ph[3] = {0, 0, 0};
      for (int w = 0; w < 4; w++) { gn += L->gt[w].n; o->gpu_fail += L->gt[w].fail; for (int k = 0; k < 3; k++) ph[k] += L->gt[w].ph[k]; o->skips += L->skip[w]; }
      o->gpu_calls = gn; for (int k = 0; k < 3; k++) o->gpu_phase_us[k] = gn ? 1e6 * ph[k] / gn : -1; }
    o->xcheck = L->xc_every;
    for (int w = 0; w < 4; w++) { o->xc_calls += L->xc[w].n; o->xc_flips += L->xc[w].flips; o->xc_max_dth = fmax(o->xc_max_dth, L->xc[w].dth); o->xc_max_dother = fmax(o->xc_max_dother, L->xc[w].dsl); }
    for (int w = 0; w < 4; w++) {
        o->calls += L->st[stage][w].calls; o->rays += L->st[stage][w].rays; o->fallback += L->st[stage][w].fb;
        o->seconds += L->st[stage][w].s; if (L->st[stage][w].mx > o->max_s) o->max_s = L->st[stage][w].mx;
    }
    for (int b = 0; b < 3; b++) { o->cal_us[b] = L->cal_us[b]; o->cal_match[b] = L->cal_match[b]; }
    o->gpu_ok = L->gpu_ok;
    snprintf(o->device, sizeof o->device, "%s", L->gpu_ok ? CLG.name : "");
}
void dc_lane_note_skip(dc_lane* L, int w) { if (L) L->skip[w]++; }
void dc_lane_set_xcheck(dc_lane* L, int every) { if (L) L->xc_every = every > 0 ? every : 0; }
