/* main.c - DriveChain raylib 3D test drive.
   Physics on its own thread (1 kHz ticks, 2 kHz core substeps), rendering at whatever the GPU does (vsync optional),
   SDL2 wheel/pedals/H-shifter + constant-force FFB, procedural audio, debug overlays, headless screenshot mode. */
#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"
#include "sim.h"
#include "drvproto.h"
#include "objload.h"
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#ifdef _WIN32
/* windows.h clashes with raylib (CloseWindow, DrawText, Rectangle...): declare just what we need from winmm */
__declspec(dllimport) unsigned int __stdcall timeBeginPeriod(unsigned int uPeriod);
__declspec(dllimport) unsigned int __stdcall timeEndPeriod(unsigned int uPeriod);
#endif

static cfg_t CFG; static track_t TRK; static audio_t AUD; static sim_t SIM;
static pthread_mutex_t MTX = PTHREAD_MUTEX_INITIALIZER;
static sim_snap VIEW;              /* render copy */
static volatile int RUN = 1;
static int g_threaded = 1;

/* ---------------- frames: physics (x fwd, y left, z up) -> raylib (X, Y up, Z) = (x, z, -y) ---------------- */
static Vector3 R3(double x, double y, double z) { return (Vector3){(float)x, (float)z, (float)-y}; }
static Vector3 RV(const double* p) { return R3(p[0], p[1], p[2]); }
typedef struct { double m[3][3]; double t[3]; } xf_t;
static xf_t xf_from(const dc_xform* x) {
    double w = x->quat[0], a = x->quat[1], b = x->quat[2], c = x->quat[3]; xf_t r;
    r.m[0][0] = 1 - 2 * (b * b + c * c); r.m[0][1] = 2 * (a * b - w * c); r.m[0][2] = 2 * (a * c + w * b);
    r.m[1][0] = 2 * (a * b + w * c); r.m[1][1] = 1 - 2 * (a * a + c * c); r.m[1][2] = 2 * (b * c - w * a);
    r.m[2][0] = 2 * (a * c - w * b); r.m[2][1] = 2 * (b * c + w * a); r.m[2][2] = 1 - 2 * (a * a + b * b);
    for (int i = 0; i < 3; i++) r.t[i] = x->pos[i];
    return r;
}
static xf_t xf_mul(const xf_t* A, const xf_t* B) { xf_t r;
    for (int i = 0; i < 3; i++) { for (int j = 0; j < 3; j++) r.m[i][j] = A->m[i][0] * B->m[0][j] + A->m[i][1] * B->m[1][j] + A->m[i][2] * B->m[2][j];
        r.t[i] = A->m[i][0] * B->t[0] + A->m[i][1] * B->t[1] + A->m[i][2] * B->t[2] + A->t[i]; }
    return r; }
static xf_t xf_trans(double x, double y, double z) { xf_t r; memset(&r, 0, sizeof r); r.m[0][0] = r.m[1][1] = r.m[2][2] = 1; r.t[0] = x; r.t[1] = y; r.t[2] = z; return r; }
static xf_t xf_axis(const double* ax, double ang) {  /* rotation about unit axis */
    xf_t r; memset(&r, 0, sizeof r); double c = cos(ang), s = sin(ang), C = 1 - c, x = ax[0], y = ax[1], z = ax[2];
    r.m[0][0] = c + x * x * C; r.m[0][1] = x * y * C - z * s; r.m[0][2] = x * z * C + y * s;
    r.m[1][0] = y * x * C + z * s; r.m[1][1] = c + y * y * C; r.m[1][2] = y * z * C - x * s;
    r.m[2][0] = z * x * C - y * s; r.m[2][1] = z * y * C + x * s; r.m[2][2] = c + z * z * C; return r; }
static xf_t xf_apply_pt(const xf_t* A, const double* p, double* o) { for (int i = 0; i < 3; i++) o[i] = A->m[i][0] * p[0] + A->m[i][1] * p[1] + A->m[i][2] * p[2] + A->t[i]; return *A; }
/* raylib matrix = B * A where B maps physics->render */
static Matrix to_rl(const xf_t* A) {
    static const double B[3][3] = {{1, 0, 0}, {0, 0, 1}, {0, -1, 0}};
    double M[3][3], t[3];
    for (int i = 0; i < 3; i++) { for (int j = 0; j < 3; j++) M[i][j] = B[i][0] * A->m[0][j] + B[i][1] * A->m[1][j] + B[i][2] * A->m[2][j];
        t[i] = B[i][0] * A->t[0] + B[i][1] * A->t[1] + B[i][2] * A->t[2]; }
    /* local mesh coordinates are physics-frame too: right-multiply by B^-1 is NOT wanted (meshes are authored in physics frame) */
    Matrix r = {0};
    r.m0 = (float)M[0][0]; r.m4 = (float)M[0][1]; r.m8 = (float)M[0][2]; r.m12 = (float)t[0];
    r.m1 = (float)M[1][0]; r.m5 = (float)M[1][1]; r.m9 = (float)M[1][2]; r.m13 = (float)t[1];
    r.m2 = (float)M[2][0]; r.m6 = (float)M[2][1]; r.m10 = (float)M[2][2]; r.m14 = (float)t[2];
    r.m15 = 1; return r;
}

/* ---------------- lit shader ---------------- */
static const char* VS =
"#version 330\n in vec3 vertexPosition; in vec2 vertexTexCoord; in vec3 vertexNormal; in vec4 vertexColor;\n"
"uniform mat4 mvp; uniform mat4 matModel; uniform mat4 matNormal;\n"
"out vec3 fragPos; out vec2 fragTexCoord; out vec4 fragColor; out vec3 fragNormal;\n"
"void main(){ fragPos = vec3(matModel*vec4(vertexPosition,1.0)); fragTexCoord = vertexTexCoord; fragColor = vertexColor;\n"
"  fragNormal = normalize(vec3(matNormal*vec4(vertexNormal,0.0))); gl_Position = mvp*vec4(vertexPosition,1.0); }\n";
static const char* FS =
"#version 330\n in vec3 fragPos; in vec2 fragTexCoord; in vec4 fragColor; in vec3 fragNormal;\n"
"uniform sampler2D texture0; uniform vec4 colDiffuse; uniform vec3 viewPos; uniform vec3 sunDir; uniform float fogDen;\n"
"out vec4 finalColor;\n"
"void main(){ vec4 tc = texture(texture0, fragTexCoord)*colDiffuse*fragColor;\n"
"  vec3 n = normalize(fragNormal); vec3 v = normalize(viewPos - fragPos); if (dot(n, v) < 0.0) n = -n;\n"
"  float d = max(dot(n, -sunDir), 0.0); float sky = 0.5 + 0.5*n.y;\n"
"  vec3 h = normalize(-sunDir + v); float sp = pow(max(dot(n,h),0.0), 40.0)*0.25;\n"
"  vec3 c = tc.rgb*(0.30*vec3(0.75,0.85,1.0)*sky + 0.85*vec3(1.0,0.96,0.88)*d) + sp*d;\n"
"  float dist = length(viewPos - fragPos); float f = 1.0 - exp(-fogDen*dist);\n"
"  finalColor = vec4(mix(c, vec3(0.70,0.78,0.88), f), 1.0); }\n";
static Shader LIT; static int LOC_VIEW, LOC_SUN, LOC_FOG;

/* ---------------- world meshes ---------------- */
#define MAXCH 2048
static Model CH[MAXCH]; static Vector3 CHC[MAXCH]; static int NCH;
static Color surf_col(int sf, double x, double y, double s, double h) {
    double n = 0.5 + 0.5 * sin(x * 12.9898 + y * 78.233) * sin(x * 3.1 - y * 1.7);
    switch (sf) {
    case SURF_TARMAC: { int g = (int)(58 + 10 * n); return (Color){g, g, (unsigned char)(g + 4), 255}; }
    case SURF_LINE: return (Color){225, 225, 220, 255};
    case SURF_PAD: { int g = (int)(72 + 8 * n); return (Color){g, g, (unsigned char)(g + 3), 255}; }
    case SURF_PAVE: { int g = (int)(70 + 900 * h); g = g < 25 ? 25 : (g > 140 ? 140 : g); return (Color){(unsigned char)(g + 8), (unsigned char)(g + 4), g, 255}; }
    case SURF_KERB: return ((int)floor(s / 1.5)) & 1 ? (Color){205, 30, 25, 255} : (Color){235, 235, 230, 255};
    case SURF_SAUSAGE: return ((int)floor(s / 0.6)) & 1 ? (Color){230, 190, 20, 255} : (Color){25, 25, 25, 255};
    case SURF_ROUGH: { int g = (int)(90 + 30 * n); return (Color){(unsigned char)(g + 25), (unsigned char)g, (unsigned char)(g - 30), 255}; }
    default: { int g = (int)(95 + 35 * n + 300 * h); g = g < 40 ? 40 : (g > 200 ? 200 : g); return (Color){(unsigned char)(g * 0.45), (unsigned char)g, (unsigned char)(g * 0.35), 255}; }
    }
}
/* generic grid mesh: positions from a callback over (u,v) */
typedef void (*gridfn)(void* u, int i, int j, double* x, double* y, double* z, Color* c);
static Mesh grid_mesh(int nu, int nv, gridfn f, void* user) {
    Mesh m = {0};
    m.vertexCount = nu * nv; m.triangleCount = 2 * (nu - 1) * (nv - 1);
    m.vertices = (float*)MemAlloc(m.vertexCount * 3 * sizeof(float));
    m.normals = (float*)MemAlloc(m.vertexCount * 3 * sizeof(float));
    m.texcoords = (float*)MemAlloc(m.vertexCount * 2 * sizeof(float));
    m.colors = (unsigned char*)MemAlloc(m.vertexCount * 4);
    m.indices = (unsigned short*)MemAlloc(m.triangleCount * 3 * sizeof(unsigned short));
    double* P = (double*)malloc(sizeof(double) * 3 * nu * nv);
    for (int j = 0; j < nv; j++) for (int i = 0; i < nu; i++) {
        int k = j * nu + i; double x, y, z; Color c; f(user, i, j, &x, &y, &z, &c);
        P[3 * k] = x; P[3 * k + 1] = y; P[3 * k + 2] = z;
        Vector3 r = R3(x, y, z); m.vertices[3 * k] = r.x; m.vertices[3 * k + 1] = r.y; m.vertices[3 * k + 2] = r.z;
        m.colors[4 * k] = c.r; m.colors[4 * k + 1] = c.g; m.colors[4 * k + 2] = c.b; m.colors[4 * k + 3] = 255;
        m.texcoords[2 * k] = 0; m.texcoords[2 * k + 1] = 0;
    }
    for (int j = 0; j < nv; j++) for (int i = 0; i < nu; i++) {
        int k = j * nu + i, i0 = i > 0 ? i - 1 : i, i1 = i < nu - 1 ? i + 1 : i, j0 = j > 0 ? j - 1 : j, j1 = j < nv - 1 ? j + 1 : j;
        double* a = &P[3 * (j * nu + i1)]; double* b = &P[3 * (j * nu + i0)]; double* c = &P[3 * (j1 * nu + i)]; double* d = &P[3 * (j0 * nu + i)];
        double du[3] = {a[0] - b[0], a[1] - b[1], a[2] - b[2]}, dv[3] = {c[0] - d[0], c[1] - d[1], c[2] - d[2]};
        double n[3] = {du[1] * dv[2] - du[2] * dv[1], du[2] * dv[0] - du[0] * dv[2], du[0] * dv[1] - du[1] * dv[0]};
        if (n[2] < 0) { n[0] = -n[0]; n[1] = -n[1]; n[2] = -n[2]; }
        double l = sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]) + 1e-12;
        Vector3 r = R3(n[0] / l, n[1] / l, n[2] / l); m.normals[3 * k] = r.x; m.normals[3 * k + 1] = r.y; m.normals[3 * k + 2] = r.z;
    }
    int t = 0;
    for (int j = 0; j < nv - 1; j++) for (int i = 0; i < nu - 1; i++) {
        unsigned short a = j * nu + i, b = a + 1, c = a + nu, d = c + 1;
        /* CCW seen from above in render space */
        m.indices[t++] = a; m.indices[t++] = c; m.indices[t++] = b;
        m.indices[t++] = b; m.indices[t++] = c; m.indices[t++] = d;
    }
    free(P);
    UploadMesh(&m, false);
    return m;
}
typedef struct { double x0, y0, step; } terr_u;
static void terr_fn(void* u, int i, int j, double* x, double* y, double* z, Color* c) {
    terr_u* T = (terr_u*)u; *x = T->x0 + i * T->step; *y = T->y0 + j * T->step;
    int sf; double d, s; double h = track_height(&TRK, *x, *y, &sf, NULL, &d, &s);
    int band = fabs(d) < TRK.half_w + TRK.kerb_w + 1.6;
    if (band) h -= 0.06;   /* the ribbon mesh covers the road band */
    *z = h; *c = surf_col(band && (sf == SURF_TARMAC || sf == SURF_LINE || sf == SURF_KERB || sf == SURF_SAUSAGE || sf == SURF_PAVE) ? SURF_GRASS : sf, *x, *y, s, h);
}
typedef struct { int k0, sub; double wmax, dd; } rib_u;
static void rib_fn(void* u, int i, int j, double* x, double* y, double* z, Color* c) {
    rib_u* R = (rib_u*)u; double s = (R->k0 + (double)j / R->sub) * TRK.ds, d = -R->wmax + i * R->dd;
    track_point(&TRK, s, d, x, y, NULL);
    int sf; double dd, ss; double h = track_height(&TRK, *x, *y, &sf, NULL, &dd, &ss);
    *z = h + 0.004; *c = surf_col(sf, *x, *y, ss, h);
}
static Material MAT_V; static int FAR_CH = -1;
static void far_fn(void* u, int i, int j, double* x, double* y, double* z, Color* c) {
    terr_u* T = (terr_u*)u; *x = T->x0 + i * T->step; *y = T->y0 + j * T->step; *z = -0.35; *c = (Color){52, 98, 40, 255};
}
static void build_world(void) {
    MAT_V = LoadMaterialDefault(); MAT_V.shader = LIT;
    /* track ribbon chunks: 40 samples (20 m) x across 0.125 m */
    double wmax = TRK.half_w + TRK.kerb_w + 1.5; int nu = (int)(2 * wmax / 0.125) + 1;
    for (int k0 = 0; k0 < TRK.n && NCH < MAXCH; k0 += 40) {
        /* finer along-track rows where the surface has short features (proving-ground pave / single-wheel events) */
        double sa = k0 * TRK.ds, sb = (k0 + 40) * TRK.ds; int sub = 1;
        if (TRK.layout == TRACK_PROVING) {
            if (sb > 100 && sa < 300) sub = 8;
            else if ((sb > 340 && sa < 540) || (sb > TRK.s_str2 + 100 && sa < TRK.s_str2 + 530) || (sb > TRK.s_arc2 + 30 && sa < TRK.s_arc2 + 410)) sub = 4;
        }
        rib_u R = {k0, sub, wmax, 2 * wmax / (nu - 1)};
        int nv = (k0 + 40 < TRK.n ? 40 : TRK.n - k0) * sub + 1;
        Mesh m = grid_mesh(nu, nv, rib_fn, &R);
        CH[NCH] = LoadModelFromMesh(m); CH[NCH].materials[0].shader = LIT;
        double x, y; track_point(&TRK, (k0 + 20) * TRK.ds, 0, &x, &y, NULL); CHC[NCH] = R3(x, y, 0); NCH++;
    }
    /* far ground: big plane slightly below the terrain so the horizon has no gap */
    { terr_u T = {TRK.xmin - 2500, TRK.ymin - 2500, (TRK.xmax - TRK.xmin + 5000) / 15.0};
      Mesh m = grid_mesh(16, 16, far_fn, &T); CH[NCH] = LoadModelFromMesh(m); CH[NCH].materials[0].shader = LIT; CHC[NCH] = R3(0, 0, 0); FAR_CH = NCH++; }
    /* terrain chunks: 32 m; 0.5 m step near the track / in the rough zone, else 2 m */
    for (double y0 = TRK.ymin; y0 < TRK.ymax; y0 += 32)
        for (double x0 = TRK.xmin; x0 < TRK.xmax && NCH < MAXCH; x0 += 32) {
            double cx = x0 + 16, cy = y0 + 16, d, s; track_height(&TRK, cx, cy, NULL, NULL, &d, &s);
            int fine = fabs(d) < 45 || hypot(cx - TRK.rough_cx, cy - TRK.rough_cy) < TRK.rough_r + 25;
            double st = fine ? 0.5 : 2.0;
            if (TRK.layout == TRACK_PROVING) {   /* painted skidpad rings / slalom line need a fine grid; the rest of the pad is flat */
                double rr = hypot(cx - TRK.skid_cx, cy - TRK.skid_cy);
                if (fabs(rr - TRK.skid_r) < 26 || (fabs(cy - TRK.cone_y[0]) < 24 && cx > TRK.cone_x[0] - 50 && cx < TRK.cone_x[TRK.ncone - 1] + 50)) st = 0.25;
                else if (cx > TRK.pad_x0 + 20 && cx < TRK.pad_x1 - 20 && cy > TRK.pad_y0 + 20 && cy < TRK.pad_y1 - 20) st = 2.0;
            }
            int n = (int)(32 / st) + 1;
            terr_u T = {x0, y0, st};
            Mesh m = grid_mesh(n, n, terr_fn, &T);
            CH[NCH] = LoadModelFromMesh(m); CH[NCH].materials[0].shader = LIT; CHC[NCH] = R3(cx, cy, 0); NCH++;
        }
}

/* ---------------- car mesh ---------------- */
typedef struct { int ok; Model body, wheel[4], cal[4], steer; double wc[4][3], sc[3], sax[3]; } carmesh_t;
static carmesh_t CM;
static void model_shader(Model* m) { for (int i = 0; i < m->materialCount; i++) m->materials[i].shader = LIT; }
static int load_carmesh(const char* dir) {
    char p[512]; static const char* cn[4] = {"fl", "fr", "rl", "rr"};
    snprintf(p, sizeof p, "%s/mesh.cfg", dir);
    cfg_t mc; memset(&mc, 0, sizeof mc);
    if (!*dir || !cfg_load(&mc, p)) return 0;
    for (int i = 0; i < 4; i++) { char k[32]; snprintf(k, sizeof k, "wheel_%s", cn[i]); sscanf(cfg_str(&mc, k, "0 0 0"), "%lf %lf %lf", &CM.wc[i][0], &CM.wc[i][1], &CM.wc[i][2]); }
    sscanf(cfg_str(&mc, "steer_center", "0 0 0"), "%lf %lf %lf", &CM.sc[0], &CM.sc[1], &CM.sc[2]);
    sscanf(cfg_str(&mc, "steer_axis", "-1 0 0"), "%lf %lf %lf", &CM.sax[0], &CM.sax[1], &CM.sax[2]);
    snprintf(p, sizeof p, "%s/body.obj", dir); if (!obj_load(p, &CM.body)) return 0; model_shader(&CM.body);
    for (int i = 0; i < 4; i++) {
        snprintf(p, sizeof p, "%s/wheel_%s.obj", dir, cn[i]); obj_load(p, &CM.wheel[i]); model_shader(&CM.wheel[i]);
        snprintf(p, sizeof p, "%s/caliper_%s.obj", dir, cn[i]); obj_load(p, &CM.cal[i]); model_shader(&CM.cal[i]);
    }
    snprintf(p, sizeof p, "%s/steer.obj", dir); obj_load(p, &CM.steer); model_shader(&CM.steer);
    CM.ok = 1; return 1;
}
static void draw_model_xf(Model* m, const xf_t* X) {
    Matrix M = to_rl(X);
    for (int i = 0; i < m->meshCount; i++) DrawMesh(m->meshes[i], m->materials[m->meshMaterial[i]], M);
}

/* ---------------- drawing helpers ---------------- */
static void seg3(const double* a, const double* b, float r, Color c) { DrawCylinderEx(RV(a), RV(b), r, r, 6, c); }
static void draw_spring(const double* a, const double* b, float rad, Color c) {
    double ax[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]}, L = sqrt(ax[0] * ax[0] + ax[1] * ax[1] + ax[2] * ax[2]);
    if (L < 1e-4) return; for (int k = 0; k < 3; k++) ax[k] /= L;
    double t[3] = {0, 0, 1}; if (fabs(ax[2]) > 0.9) { t[0] = 1; t[2] = 0; }
    double u[3] = {ax[1] * t[2] - ax[2] * t[1], ax[2] * t[0] - ax[0] * t[2], ax[0] * t[1] - ax[1] * t[0]}, ul = sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
    for (int k = 0; k < 3; k++) u[k] /= ul;
    double w[3] = {ax[1] * u[2] - ax[2] * u[1], ax[2] * u[0] - ax[0] * u[2], ax[0] * u[1] - ax[1] * u[0]};
    int N = 96; double prev[3] = {0, 0, 0};
    for (int i = 0; i <= N; i++) {
        double f = (double)i / N, ph = f * 2 * M_PI * 7, p[3] = {0, 0, 0};
        for (int k = 0; k < 3; k++) p[k] = a[k] + ax[k] * L * f + rad * (cos(ph) * u[k] + sin(ph) * w[k]);
        if (i) DrawCylinderEx(RV(prev), RV(p), 0.006f, 0.006f, 4, c);
        memcpy(prev, p, sizeof p);
    }
}
static void draw_placeholder_car(const sim_snap* V, const dc_car_params* P) {
    xf_t ch = xf_from(&V->parts[DC_PART_CHASSIS]);
    /* chassis: floor pan + cabin boxes (wire) so the linkages stay visible */
    double hx = P->axle[0].hub[0] + 0.75, rx = P->axle[1].hub[0] - 0.65, hw = 0.82, scg[3] = {P->sprung_cg[0], P->sprung_cg[1], P->sprung_cg[2]};
    double c8[8][3] = {{hx, hw, -0.12}, {hx, -hw, -0.12}, {rx, -hw, -0.12}, {rx, hw, -0.12}, {hx, hw, 0.25}, {hx, -hw, 0.25}, {rx, -hw, 0.25}, {rx, hw, 0.25}};
    double w[8][3];
    for (int i = 0; i < 8; i++) { double l[3] = {c8[i][0] - scg[0], c8[i][1] - scg[1], c8[i][2] - scg[2]}; xf_apply_pt(&ch, l, w[i]); }
    static const int E[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    for (int e = 0; e < 12; e++) DrawLine3D(RV(w[E[e][0]]), RV(w[E[e][1]]), (Color){200, 40, 40, 255});
    for (int i = 0; i < 4; i++) {
        xf_t wx = xf_from(&V->parts[DC_PART_WHEEL0 + i]);
        const dc_tyre_params* T = &P->axle[i / 2].tyre;
        double a[3] = {0, -T->width / 2, 0}, b[3] = {0, T->width / 2, 0}, A[3], B[3];
        xf_apply_pt(&wx, a, A); xf_apply_pt(&wx, b, B);
        DrawCylinderEx(RV(A), RV(B), (float)T->radius, (float)T->radius, 24, (Color){30, 30, 32, 255});
        for (int s = 0; s < 5; s++) {   /* spokes, so wheel spin is visible */
            double ang = s * 2 * M_PI / 5, o[3] = {0, (i & 1 ? -1 : 1) * (T->width / 2 + 0.005), 0}, e2[3] = {T->rim_radius * cos(ang), o[1], T->rim_radius * sin(ang)}, O[3], E2[3];
            xf_apply_pt(&wx, o, O); xf_apply_pt(&wx, e2, E2); DrawCylinderEx(RV(O), RV(E2), 0.012f, 0.012f, 4, LIGHTGRAY);
        }
    }
}
static void draw_links(const sim_snap* V) {
    for (int k = 0; k < V->nseg; k++) {
        const dc_seg* s = &V->seg[k];
        switch (s->type) {
        case DC_SEG_SPRING: draw_spring(s->a, s->b, 0.035f, (Color){60, 200, 80, 255}); break;
        case DC_SEG_TIEROD: seg3(s->a, s->b, 0.010f, (Color){240, 200, 40, 255}); break;
        case DC_SEG_STRUT: seg3(s->a, s->b, 0.022f, (Color){200, 80, 220, 255}); break;
        default: seg3(s->a, s->b, 0.011f, s->corner < 0 ? (Color){240, 140, 40, 255} : (Color){90, 150, 240, 255});
        }
    }
    for (int i = 0; i < 4; i++) {   /* upright as a small box at the hub */
        Vector3 p = RV(V->parts[DC_PART_UPRIGHT0 + i].pos);
        DrawCube(p, 0.05f, 0.05f, 0.05f, (Color){160, 160, 170, 255});
    }
}
static void draw_forces(const sim_snap* V, const dc_car_params* P) {
    for (int i = 0; i < 4; i++) {
        const dc_wheel_telem* w = &V->t.w[i];
        if (w->fz <= 1) continue;
        double c[3] = {w->contact[0], w->contact[1], w->contact[2] + 0.01}, n[3] = {w->normal[0], w->normal[1], w->normal[2]};
        double fn = w->force_world[0] * n[0] + w->force_world[1] * n[1] + w->force_world[2] * n[2];
        double ft[3] = {w->force_world[0] - fn * n[0], w->force_world[1] - fn * n[1], w->force_world[2] - fn * n[2]};
        double sc = 1.0 / 6000;
        double a[3] = {c[0] + n[0] * fn * sc, c[1] + n[1] * fn * sc, c[2] + n[2] * fn * sc};
        double b[3] = {c[0] + ft[0] * sc, c[1] + ft[1] * sc, c[2] + ft[2] * sc};
        seg3(c, a, 0.012f, (Color){60, 230, 90, 255});
        seg3(c, b, 0.014f, (Color){(unsigned char)(80 + 175 * fmin(w->sliding, 1)), (unsigned char)(200 - 150 * fmin(w->sliding, 1)), 40, 255});
        /* contact patch: rectangle on the effective road plane, oriented along the wheel heading */
        xf_t wx = xf_from(&V->parts[DC_PART_WHEEL0 + i]);
        double ax[3] = {wx.m[0][1], wx.m[1][1], wx.m[2][1]};            /* spin axis */
        double fw[3] = {ax[1] * n[2] - ax[2] * n[1], ax[2] * n[0] - ax[0] * n[2], ax[0] * n[1] - ax[1] * n[0]};
        double fl = sqrt(fw[0] * fw[0] + fw[1] * fw[1] + fw[2] * fw[2]) + 1e-9; for (int k = 0; k < 3; k++) fw[k] /= fl;
        double lt[3] = {n[1] * fw[2] - n[2] * fw[1], n[2] * fw[0] - n[0] * fw[2], n[0] * fw[1] - n[1] * fw[0]};
        double hl = w->patch_half_len, hw = P->axle[i / 2].tyre.width * 0.45, q[4][3];
        for (int k = 0; k < 3; k++) {
            q[0][k] = c[k] + fw[k] * hl + lt[k] * hw; q[1][k] = c[k] - fw[k] * hl + lt[k] * hw;
            q[2][k] = c[k] - fw[k] * hl - lt[k] * hw; q[3][k] = c[k] + fw[k] * hl - lt[k] * hw;
        }
        Color pc = w->rim_force > 1 ? (Color){255, 0, 255, 255} : (Color){(unsigned char)(255 * fmin(w->sliding, 1)), (unsigned char)(255 * (1 - fmin(w->sliding, 1))), 60, 255};
        for (int e = 0; e < 4; e++) DrawLine3D(RV(q[e]), RV(q[(e + 1) % 4]), pc);
    }
}
static void draw_car(const sim_snap* V, const dc_car_params* P, int xray) {
    xf_t ch = xf_from(&V->parts[DC_PART_CHASSIS]);
    const double* scg = P->sprung_cg;
    if (CM.ok) {
        rlDisableBackfaceCulling();
        xf_t off = xf_trans(-scg[0], -scg[1], -scg[2]), B = xf_mul(&ch, &off);
        if (!xray) draw_model_xf(&CM.body, &B);
        for (int i = 0; i < 4; i++) {
            xf_t W = xf_from(&V->parts[DC_PART_WHEEL0 + i]);       /* spins + steers + camber from the physics */
            draw_model_xf(&CM.wheel[i], &W);
            xf_t U = xf_from(&V->parts[DC_PART_UPRIGHT0 + i]);     /* caliper rides on the upright */
            draw_model_xf(&CM.cal[i], &U);
        }
        /* steering wheel: rotates about its own column axis by the PHYSICAL column angle */
        xf_t T1 = xf_trans(CM.sc[0], CM.sc[1], CM.sc[2]), Rk = xf_axis(CM.sax, V->t.sw_angle), L = xf_mul(&T1, &Rk), S2 = xf_mul(&B, &L);
        draw_model_xf(&CM.steer, &S2);
        rlEnableBackfaceCulling();
    } else draw_placeholder_car(V, P);
}

/* ---------------- HUD ---------------- */
#define HIST 600
static float H_ffb[HIST], H_sw[HIST], H_ay[HIST]; static int H_n;
static void bar(int x, int y, int w, int h, double v, Color c, const char* lab) {
    DrawRectangle(x, y, w, h, (Color){0, 0, 0, 120}); DrawRectangle(x, y + (int)(h * (1 - fmin(fmax(v, 0), 1))), w, (int)(h * fmin(fmax(v, 0), 1)), c);
    DrawText(lab, x + w / 2 - MeasureText(lab, 12) / 2, y + h + 3, 12, RAYWHITE);
}
static const char* surf_name(int s) { static const char* n[] = {"tarmac", "kerb", "grass", "rough", "sausage", "paint", "pave", "pad"}; return s >= 0 && s < 8 ? n[s] : "?"; }
static void draw_hud(const sim_snap* V, const dc_car_params* P, int sw_, int sh_, double fps, const char* cam, int full) {
    const dc_telemetry* t = &V->t; char b[512]; int x = 12, y = 10;
    DrawRectangle(0, 0, 410, full ? 405 : 120, (Color){0, 0, 0, 140});
    snprintf(b, sizeof b, "%5.1f km/h   gear %s   %4.0f rpm%s", t->speed * 3.6, t->gear < 0 ? "R" : (t->gear == 0 ? "N" : (char[]){(char)('0' + t->gear), 0}), t->engine_rpm, t->engine_running ? "" : "  (stalled - clutch in to crank)");
    DrawText(b, x, y, 20, RAYWHITE); y += 26;
    snprintf(b, sizeof b, "steer %+6.1f deg   column (FFB) %+5.2f N m   FFB out %+4.2f", t->sw_angle * 180 / M_PI, t->column_torque, V->ffb_level); DrawText(b, x, y, 14, RAYWHITE); y += 18;
    snprintf(b, sizeof b, "rack %+6.1f mm  F %+6.0f N  assist %s %+6.0f N", t->rack_pos * 1e3, t->rack_force, V->assist ? "HYD" : "off", t->assist_force); DrawText(b, x, y, 14, RAYWHITE); y += 18;
    snprintf(b, sizeof b, "ay %+5.2f g  ax %+5.2f g  yaw %+5.1f deg/s  beta %+5.1f deg", t->ay / 9.81, t->ax / 9.81, t->yaw_rate * 180 / M_PI, V->beta * 180 / M_PI); DrawText(b, x, y, 14, RAYWHITE); y += 18;
    snprintf(b, sizeof b, "physics %.1f us/substep @%.0f Hz (%.1fx RT)  render %.0f fps  cam %s", V->us_per_substep, P->substep_hz, V->rt_factor, fps, cam); DrawText(b, x, y, 14, (Color){180, 220, 255, 255}); y += 20;
    if (!full) return;
    DrawText("      Fz     Fx     Fy    Mz   alpha  kappa slide trav  psi   rim  surf", x, y, 13, (Color){255, 220, 120, 255}); y += 16;
    static const char* cn[4] = {"FL", "FR", "RL", "RR"};
    for (int i = 0; i < 4; i++) {
        const dc_wheel_telem* w = &t->w[i];
        snprintf(b, sizeof b, "%s %6.0f %6.0f %6.0f %5.0f %6.2f %6.3f %4.2f %5.1f %4.1f %5.0f %s", cn[i], w->fz, w->fx, w->fy, w->mz, w->slip_angle * 180 / M_PI, w->slip_ratio, w->sliding,
                 w->travel * 1e3, w->pressure / 6894.757, w->rim_force, surf_name(V->surf[i]));
        DrawText(b, x, y, 13, w->rim_force > 1 ? MAGENTA : RAYWHITE); y += 16;
    }
    snprintf(b, sizeof b, "caster %.1f  KPI %.1f  trail %.0f mm  scrub %.0f mm (FL)", t->w[0].caster * 180 / M_PI, t->w[0].kpi * 180 / M_PI, t->w[0].trail * 1e3, t->w[0].scrub * 1e3); DrawText(b, x, y, 13, RAYWHITE); y += 16;
    snprintf(b, sizeof b, "KE %.0f J  PE %.0f J  dissipated %.0f J  N3 resid %.1e N", t->kinetic, t->potential, t->dissipated,
             sqrt(t->resid_force[0] * t->resid_force[0] + t->resid_force[1] * t->resid_force[1] + t->resid_force[2] * t->resid_force[2])); DrawText(b, x, y, 13, RAYWHITE); y += 18;
    snprintf(b, sizeof b, "input: %s%s%s", V->device_kind == 1 ? "wheel " : (V->device_kind == 2 ? "gamepad " : "keyboard"), V->device, V->has_ffb ? " [FFB]" : ""); DrawText(b, x, y, 13, RAYWHITE); y += 18;
    if (V->drv_on || V->drv_takeover || V->drv_clients) {
        static const char* MN[] = {"off", "hands-off", "torque", "angle", "path", "locked"};
        snprintf(b, sizeof b, "DRIVER %s  steer %s  clients %d  lat %+.2f m  aim %+.0f deg%s", V->drv_on ? "ENGAGED" : "released", MN[(V->drv_mode >= 0 && V->drv_mode <= 5) ? V->drv_mode : 0], V->drv_clients,
                 V->drv_st.lat_err, V->drv_st.rim_cmd * 180 / M_PI, V->drv_takeover ? "   TAKEOVER (human) - G re-engages" : "");
        DrawText(b, x, y, 14, V->drv_takeover ? ORANGE : (Color){120, 255, 160, 255}); y += 18;
    }
    if (V->device_kind == 1) {   /* raw device state, for mapping axes / H-shifter buttons in demo.ini */
        int n = snprintf(b, sizeof b, "axes"); for (int a = 0; a < V->raw_naxes && a < 8; a++) n += snprintf(b + n, sizeof b - n, " %d:%+.2f", a, V->raw_axis[a]);
        n += snprintf(b + n, sizeof b - n, "  buttons"); for (int k = 0; k < 32; k++) if (V->raw_buttons & (1 << k)) n += snprintf(b + n, sizeof b - n, " %d", k);
        snprintf(b + n, sizeof b - n, "  H %s  ffb %+.2f", V->hgear == -99 ? "off" : (V->hgear == -1 ? "R" : (V->hgear == 0 ? "N" : TextFormat("%d", V->hgear))), V->ffb_level);
        DrawText(b, x, y, 12, (Color){200, 220, 255, 255}); y += 16;
    }
    /* pedals */
    bar(x, y, 16, 60, V->in.throttle, GREEN, "T"); bar(x + 24, y, 16, 60, V->in.brake, RED, "B"); bar(x + 48, y, 16, 60, V->in.clutch, SKYBLUE, "C"); bar(x + 72, y, 16, 60, V->in.handbrake, ORANGE, "H");
    /* friction circles */
    for (int i = 0; i < 4; i++) {
        int cx = x + 130 + (i & 1) * 62, cy = y + 18 + (i / 2) * 44; const dc_wheel_telem* w = &t->w[i];
        DrawCircleLines(cx, cy, 18, GRAY);
        double mz = fmax(w->mu * w->fz, 1);
        DrawCircle(cx - (int)(18 * w->fy / mz), cy - (int)(18 * w->fx / mz), 4, w->sliding > 0.6 ? RED : (w->sliding > 0.25 ? YELLOW : GREEN));
    }
    /* FFB / steer history */
    int gx = x + 270, gy = y, gw = 120, gh = 70;
    DrawRectangleLines(gx, gy, gw, gh, GRAY); DrawLine(gx, gy + gh / 2, gx + gw, gy + gh / 2, DARKGRAY);
    for (int k = 1; k < HIST && k < H_n; k++) {
        int i0 = (H_n - k) % HIST, i1 = (H_n - k - 1 + HIST) % HIST;
        int x0 = gx + gw - (k - 1) * gw / HIST, x1 = gx + gw - k * gw / HIST;
        DrawLine(x0, gy + gh / 2 - (int)(H_ffb[i0] * gh / 2 / 8), x1, gy + gh / 2 - (int)(H_ffb[i1] * gh / 2 / 8), YELLOW);
        DrawLine(x0, gy + gh / 2 - (int)(H_sw[i0] * gh / 2 / 6.0), x1, gy + gh / 2 - (int)(H_sw[i1] * gh / 2 / 6.0), SKYBLUE);
    }
    DrawText("FFB N m (y) / steer (b)", gx, gy + gh + 2, 10, RAYWHITE);
    DrawText("keys: 1-9,0 spots  F1 spot list  arrows/WASD drive  A/Z or Q/E shift  C cam  F2 x-ray  F3 hud  F4 forces  H assist  P puncture  R reset  O autopilot  G plug-in driver  Space handbrake  LShift clutch",
             10, sh_ - 18, 12, (Color){230, 230, 230, 200});
    (void)sw_;
}

static const char* spot_label(int i);
/* spot list overlay: shown at start for ~10 s (fades), F1 toggles */
static int HELP_ON = 1, HELP_STICKY = 0; static double HELP_T0 = 0;
static void draw_spot_help(int sw_, int sh_, float a) {
    int w = 360, h = 30 + 22 * TRK_NSPOT + 26, x = sw_ - w - 16, y = 70; (void)sh_;
    DrawRectangle(x, y, w, h, (Color){10, 20, 30, (unsigned char)(200 * a)});
    DrawText("PROVING GROUND  -  teleport keys", x + 12, y + 8, 18, (Color){255, 220, 120, (unsigned char)(255 * a)});
    for (int i = 0; i < TRK_NSPOT; i++) {
        double px, py, pyaw; if (!track_spot(&TRK, i, &px, &py, &pyaw)) continue;
        char b[96]; snprintf(b, sizeof b, "%d   %s", i < 9 ? i + 1 : 0, spot_label(i));
        DrawText(b, x + 16, y + 34 + 22 * i, 18, (Color){240, 240, 240, (unsigned char)(255 * a)});
    }
    DrawText("F1 show/hide   R reset   G plug-in driver", x + 12, y + h - 24, 14, (Color){180, 220, 255, (unsigned char)(255 * a)});
}

/* ---------------- physics thread ---------------- */
static double wall(void) { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return ts.tv_sec + ts.tv_nsec * 1e-9; }
static void* phys_thread(void* arg) {
    (void)arg;
#ifdef _WIN32
    timeBeginPeriod(1);
#endif
    double t0 = wall(), sim = 0; const double dt = 0.001;
    while (RUN) {
        double now = wall() - t0; int n = 0;
        while (sim < now && n < 50) {
            pthread_mutex_lock(&MTX);
            sim_tick(&SIM, dt);
            pthread_mutex_unlock(&MTX);
            sim += dt; n++;
        }
        if (n >= 50) sim = now;     /* fell behind (debugger, heavy load): drop time instead of spiralling */
        struct timespec ts = {0, 300000}; nanosleep(&ts, NULL);
    }
    return NULL;
}
static void audio_cb(void* buf, unsigned int frames) { audio_render(&AUD, (float*)buf, (int)frames); }

/* ---------------- cameras ---------------- */
enum { CAM_CHASE, CAM_COCKPIT, CAM_WHEEL, CAM_ORBIT, CAM_TV, CAM_N };
static const char* CAMN[] = {"chase", "cockpit", "wheel", "orbit", "tv"};
static double ORB_Y = 0.6, ORB_P = 0.35, ORB_D = 6; static Vector3 CHASE_P; static int CHASE_INIT;
static Camera3D make_cam(int mode, const sim_snap* V, const dc_car_params* P, double fovv, double yaw_off, double dt) {
    Camera3D cam = {0}; cam.fovy = (float)fovv; cam.projection = CAMERA_PERSPECTIVE; cam.up = (Vector3){0, 1, 0};
    xf_t ch = xf_from(&V->parts[DC_PART_CHASSIS]);
    const double* scg = P->sprung_cg;
    double fwd[3] = {ch.m[0][0], ch.m[1][0], ch.m[2][0]}, hy = atan2(fwd[1], fwd[0]);
    if (mode == CAM_COCKPIT) {
        double base[3]; if (CM.ok) memcpy(base, CM.sc, sizeof base); else { base[0] = -0.20; base[1] = 0.37; base[2] = 0.30; }
        double eye_l[3] = {base[0] + cfg_num(&CFG, "camera.cockpit_dx", -0.50) - scg[0], base[1] + cfg_num(&CFG, "camera.cockpit_dy", 0.0) - scg[1], base[2] + cfg_num(&CFG, "camera.cockpit_dz", 0.26) - scg[2]};
        double pitch = cfg_num(&CFG, "camera.cockpit_pitch_deg", -9) * M_PI / 180;
        double look_l[3] = {eye_l[0] + 10 * cos(pitch) * cos(yaw_off), eye_l[1] + 10 * cos(pitch) * sin(yaw_off), eye_l[2] + 10 * sin(pitch)};
        double up_l[3] = {eye_l[0], eye_l[1], eye_l[2] + 1};
        double e[3], l[3], u[3]; xf_apply_pt(&ch, eye_l, e); xf_apply_pt(&ch, look_l, l); xf_apply_pt(&ch, up_l, u);
        cam.position = RV(e); cam.target = RV(l); Vector3 U = RV(u); cam.up = Vector3Normalize(Vector3Subtract(U, cam.position));
        return cam;
    }
    Vector3 tgt = RV(ch.t);
    if (mode == CAM_CHASE) {
        double d = cfg_num(&CFG, "camera.chase_dist", 6.0), h = cfg_num(&CFG, "camera.chase_height", 1.8);
        double yaw = hy + yaw_off;
        Vector3 want = R3(ch.t[0] - d * cos(yaw), ch.t[1] - d * sin(yaw), ch.t[2] + h);
        if (!CHASE_INIT || dt <= 0) { CHASE_P = want; CHASE_INIT = 1; }
        else { float a = (float)(1 - exp(-dt * 6)); CHASE_P = Vector3Lerp(CHASE_P, want, a); }
        cam.position = CHASE_P; cam.target = Vector3Add(tgt, (Vector3){0, 0.5f, 0});
        if (yaw_off != 0) { double yy = hy + yaw_off; cam.target = Vector3Add(CHASE_P, R3(10 * cos(yy), 10 * sin(yy), -1.2)); }
    } else if (mode == CAM_WHEEL) {   /* low, outside the front-left wheel, looking at the suspension */
        double l[3] = {P->axle[0].hub[0] - scg[0] + 0.9, P->axle[0].hub[1] - scg[1] + 1.3, -0.05 - scg[2]}, t2[3] = {P->axle[0].hub[0] - scg[0] - 0.1, P->axle[0].hub[1] - scg[1] - 0.2, P->axle[0].hub[2] - scg[2] + 0.02}, e[3], tt[3];
        xf_apply_pt(&ch, l, e); xf_apply_pt(&ch, t2, tt);
        cam.position = RV(e); cam.target = RV(tt);
    } else if (mode == CAM_ORBIT) {
        double yaw = hy + ORB_Y;
        cam.position = Vector3Add(tgt, R3(ORB_D * cos(ORB_P) * cos(yaw), ORB_D * cos(ORB_P) * sin(yaw), ORB_D * sin(ORB_P)));
        cam.target = tgt;
    } else {   /* tv: fixed points every 80 m along the outside */
        double d, s; track_height(&TRK, ch.t[0], ch.t[1], NULL, NULL, &d, &s);
        double s0 = floor(s / 80) * 80 + 55, x, y; track_point(&TRK, s0, (TRK.curv[(int)(s0 / TRK.ds) % TRK.n] > 0 ? -1 : 1) * 16, &x, &y, NULL);
        cam.position = R3(x, y, track_height(&TRK, x, y, NULL, NULL, NULL, NULL) + 3.5); cam.target = tgt; cam.fovy = (float)fmax(8, fmin(60, 300 / fmax(Vector3Distance(cam.position, tgt), 1) * 4));
    }
    return cam;
}

/* ---------------- proving-ground signage: a numbered board per teleport spot (key 1..9, 0) ---------------- */
static RenderTexture2D SIGN_RT[TRK_NSPOT]; static double SIGN_P[TRK_NSPOT][3]; static int NSIGN = 0;
static const char* spot_label(int i) {
    static const char* L[TRK_NSPOT] = {"START HUB", "START STRAIGHT", "SKIDPAD R40", "PAVE", "UNDULATIONS", "BANKED OVAL", "SLALOM", "BUMPS / POTHOLES", "RUTS", "BUMPY CORNER"};
    return L[i];
}
static void build_signs(void) {
    if (TRK.layout != TRACK_PROVING) return;
    for (int i = 0; i < TRK_NSPOT; i++) {
        double x, y, yaw; if (!track_spot(&TRK, i, &x, &y, &yaw)) continue;
        double off = i == 0 ? 7.0 : (i == 2 || i == 6 ? 6.0 : TRK.half_w + TRK.kerb_w + 3.0), ahead = i == 0 ? 14.0 : 12.0;
        double sx = x + cos(yaw) * ahead + sin(yaw) * off, sy = y + sin(yaw) * ahead - cos(yaw) * off;   /* right of the spot, ahead */
        SIGN_P[i][0] = sx; SIGN_P[i][1] = sy; SIGN_P[i][2] = track_height(&TRK, sx, sy, NULL, NULL, NULL, NULL);
        SIGN_RT[i] = LoadRenderTexture(512, 128);
        BeginTextureMode(SIGN_RT[i]);
        ClearBackground(i == 0 ? (Color){20, 90, 40, 255} : (Color){20, 50, 110, 255});
        DrawRectangleLinesEx((Rectangle){4, 4, 504, 120}, 6, RAYWHITE);
        char b[64]; snprintf(b, sizeof b, "%d", i < 9 ? i + 1 : 0);
        DrawText(b, 24, 22, 84, YELLOW);
        DrawText(spot_label(i), 100, i == 0 ? 18 : 40, 44, RAYWHITE);
        if (i == 0) DrawText("PROVING GROUND   keys 1-9,0   F1 list", 100, 76, 26, (Color){220, 255, 220, 255});
        EndTextureMode();
        NSIGN = i + 1;
    }
}
static void draw_signs(Camera3D cam) {
    for (int i = 0; i < NSIGN; i++) {
        const double* p = SIGN_P[i]; float h = i == 0 ? 3.6f : 3.0f, w = i == 0 ? 6.0f : 4.4f;
        DrawCylinder(R3(p[0], p[1], p[2]), 0.06f, 0.06f, h - w / 8, 8, (Color){200, 200, 200, 255});
        DrawBillboardRec(cam, SIGN_RT[i].texture, (Rectangle){0, 0, 512, -128}, R3(p[0], p[1], p[2] + h), (Vector2){w, w / 4}, WHITE);
    }
    if (TRK.layout == TRACK_PROVING) {   /* painted start box + corner poles at the hub */
        double x, y, yaw; track_spot(&TRK, 0, &x, &y, &yaw);
        double z = track_height(&TRK, x, y, NULL, NULL, NULL, NULL) + 0.01;
        Color wc = (Color){235, 235, 235, 255};
        DrawCube(R3(x - 4, y, z), 0.15f, 0.01f, 5.0f, wc); DrawCube(R3(x + 5, y, z), 0.15f, 0.01f, 5.0f, wc);
        DrawCube(R3(x + 0.5, y + 2.5, z), 9.0f, 0.01f, 0.15f, wc); DrawCube(R3(x + 0.5, y - 2.5, z), 9.0f, 0.01f, 0.15f, wc);
        for (int k = 0; k < 4; k++) { double px = x + (k & 1 ? 5 : -4), py = y + (k & 2 ? 2.5 : -2.5);
            DrawCylinder(R3(px, py, z), 0.05f, 0.05f, 1.2f, 8, k < 2 ? (Color){255, 200, 0, 255} : (Color){0, 200, 80, 255}); }
    }
}

static void draw_scene(Camera3D cam, const sim_snap* V, const dc_car_params* P, int xray, int forces, int camode) {
    float vp[3] = {cam.position.x, cam.position.y, cam.position.z}; SetShaderValue(LIT, LOC_VIEW, vp, SHADER_UNIFORM_VEC3);
    BeginMode3D(cam); rlDisableBackfaceCulling();   /* raylib re-enables culling in some draw calls; terrain winding must not matter */
    for (int i = 0; i < NCH; i++) {
        float dx = CHC[i].x - cam.position.x, dz = CHC[i].z - cam.position.z;
        if (i != FAR_CH && dx * dx + dz * dz > 420.0f * 420.0f) continue;
        DrawMesh(CH[i].meshes[0], CH[i].materials[0], MatrixIdentity());
    }
    for (int i = 0; i < TRK.ncone; i++) {   /* slalom cones (visual only: 0.7 m traffic cones) */
        double z = track_height(&TRK, TRK.cone_x[i], TRK.cone_y[i], NULL, NULL, NULL, NULL);
        DrawCube(R3(TRK.cone_x[i], TRK.cone_y[i], z + 0.015), 0.38f, 0.03f, 0.38f, (Color){30, 30, 30, 255});
        DrawCylinder(R3(TRK.cone_x[i], TRK.cone_y[i], z + 0.03), 0.025f, 0.17f, 0.68f, 10, (Color){245, 110, 20, 255});
        DrawCylinder(R3(TRK.cone_x[i], TRK.cone_y[i], z + 0.36), 0.09f, 0.11f, 0.12f, 10, (Color){240, 240, 240, 255});
    }
    draw_signs(cam);
    draw_car(V, P, xray);
    if (xray || !CM.ok || camode == CAM_WHEEL) { if (xray) rlDisableDepthTest(); draw_links(V); rlEnableDepthTest(); }
    if (forces) { rlDisableDepthTest(); draw_forces(V, P); rlEnableDepthTest(); }
    EndMode3D();
}

int main(int argc, char** argv) {
    const char* cfgpath = "demo.ini";
    const char* shots = NULL; const char* wav = NULL; double shot_dt = 1.0 / 60; double max_time = 0;
    for (int i = 1; i < argc; i++) if (!strcmp(argv[i], "--config") && i + 1 < argc) cfgpath = argv[++i];
    if (!cfg_load(&CFG, cfgpath)) printf("[demo] no %s - using defaults\n", cfgpath);
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--set") && i + 1 < argc) { char kv[256]; snprintf(kv, sizeof kv, "%s", argv[++i]); char* e = strchr(kv, '='); if (e) { *e = 0; cfg_set(&CFG, kv, e + 1); } }
        else if (!strcmp(argv[i], "--shots") && i + 1 < argc) shots = argv[++i];       /* "t:cam:file,t:cam:file" (sim seconds) */
        else if (!strcmp(argv[i], "--audio-wav") && i + 1 < argc) wav = argv[++i];
        else if (!strcmp(argv[i], "--time") && i + 1 < argc) max_time = atof(argv[++i]);
    }
    int headless = shots != NULL || (wav && max_time > 0);
    { const char* lay = cfg_str(&CFG, "track.layout", "proving");
      track_build(&TRK, cfg_num(&CFG, "track.rough_scale", 1.0), !strcmp(lay, "circuit") ? TRACK_CIRCUIT : TRACK_PROVING); }
    char err[256] = "";
    audio_init(&AUD, 44100, 1000);
    AUD.master = (float)cfg_num(&CFG, "audio.master", 0.8); AUD.engine_gain = (float)cfg_num(&CFG, "audio.engine", 1.0);
    AUD.tyre_gain = (float)cfg_num(&CFG, "audio.tyres", 1.0); AUD.road_gain = (float)cfg_num(&CFG, "audio.road", 1.0); AUD.wind_gain = (float)cfg_num(&CFG, "audio.wind", 1.0);
    if (headless && (getenv("DEMO_SPOT") || !cfg_str(&CFG, "start.spot", NULL))) cfg_set(&CFG, "start.spot", getenv("DEMO_SPOT") ? getenv("DEMO_SPOT") : "start");   /* autopilot shots start on the straight */
    if (headless) { cfg_set(&CFG, "input.device", "keyboard"); if (!getenv("DEMO_MANUAL")) cfg_set(&CFG, "autopilot.enabled", "1"); }
    if (!sim_init(&SIM, &CFG, &TRK, &AUD, err, sizeof err)) { fprintf(stderr, "car init failed: %s\n", err); return 1; }
    const dc_car_params* P = dc_params(SIM.car);
    /* plug-in driver control port (JSON lines over TCP, see docs/DRIVER_INTERFACE.md): --drive-port N or driver.port */
    static proto_t PROTO;
    { int port = cfg_int(&CFG, "driver.port", 0);
      for (int i = 1; i < argc; i++) if (!strcmp(argv[i], "--drive-port") && i + 1 < argc) port = atoi(argv[i + 1]);
      if (port > 0) { proto_init(&PROTO, &SIM, 0);
          if (proto_listen(&PROTO, cfg_str(&CFG, "driver.bind", "127.0.0.1"), port)) printf("[demo] driver control port %s:%d (JSON lines; tools/dcdrive.py --tcp)\n", cfg_str(&CFG, "driver.bind", "127.0.0.1"), port);
          else fprintf(stderr, "[demo] could not listen on driver port %d\n", port); } }
    printf("[demo] track %.0f m, car %s (%s, %d substep Hz), mesh %s\n", TRK.length, cfg_str(&CFG, "car.ini", "(built-in default)"), P->assist ? "hydraulic assist" : "manual rack", (int)P->substep_hz, cfg_str(&CFG, "car.mesh_dir", "(placeholder)"));

    int W = cfg_int(&CFG, "display.width", 1600), Hh = cfg_int(&CFG, "display.height", 900);
    unsigned flags = FLAG_WINDOW_RESIZABLE;
    if (cfg_int(&CFG, "display.vsync", 0)) flags |= FLAG_VSYNC_HINT;
    if (cfg_int(&CFG, "display.msaa", 1) && !headless) flags |= FLAG_MSAA_4X_HINT;
    if (cfg_int(&CFG, "display.fullscreen", 0)) flags |= FLAG_FULLSCREEN_MODE;
    SetConfigFlags(flags); SetTraceLogLevel(LOG_WARNING);
    printf("[demo] init window\n"); fflush(stdout);
    InitWindow(W, Hh, "DriveChain test drive");
    printf("[demo] window ok\n"); fflush(stdout);
    SetTargetFPS(cfg_int(&CFG, "display.fps_cap", 0));
    SetExitKey(KEY_ESCAPE);
    LIT = LoadShaderFromMemory(VS, FS);
    LOC_VIEW = GetShaderLocation(LIT, "viewPos"); LOC_SUN = GetShaderLocation(LIT, "sunDir"); LOC_FOG = GetShaderLocation(LIT, "fogDen");
    Vector3 sun = Vector3Normalize((Vector3){-0.45f, -0.8f, -0.35f}); SetShaderValue(LIT, LOC_SUN, &sun, SHADER_UNIFORM_VEC3);
    float fog = 0.0022f; SetShaderValue(LIT, LOC_FOG, &fog, SHADER_UNIFORM_FLOAT);
    double tw0 = GetTime(); build_world(); build_signs(); HELP_T0 = GetTime(); for (int i = 0; i < TRK_NSPOT; i++) { double a_, b_, c_; if (track_spot(&TRK, i, &a_, &b_, &c_)) printf("[demo] key %d -> %s\n", i < 9 ? i + 1 : 0, spot_label(i)); } printf("[demo] world: %d chunks in %.1f s\n", NCH, GetTime() - tw0); fflush(stdout);
    if (load_carmesh(cfg_str(&CFG, "car.mesh_dir", ""))) printf("[demo] car mesh loaded (%d body meshes)\n", CM.body.meshCount);
    else printf("[demo] no car mesh -> placeholder components\n");

    AudioStream st = {0}; int audio_on = 0;
    if (!headless && cfg_int(&CFG, "audio.enabled", 1)) {
        InitAudioDevice();
        if (IsAudioDeviceReady()) { SetAudioStreamBufferSizeDefault(512); st = LoadAudioStream(44100, 32, 2); SetAudioStreamCallback(st, audio_cb); PlayAudioStream(st); audio_on = 1; }
    }
    int camode = CAM_CHASE; for (int i = 0; i < CAM_N; i++) if (!strcmp(cfg_str(&CFG, "camera.mode", "chase"), CAMN[i])) camode = i;
    rlDisableBackfaceCulling();
    int xray = cfg_int(&CFG, "display.xray", 0), hud = 1, forces = cfg_int(&CFG, "display.force_arrows", 1);
    double fovv = cfg_num(&CFG, "display.fov_v_deg", 50);
    int triple = cfg_int(&CFG, "display.triple", 0);
    pthread_t th;
    g_threaded = !headless;
    if (g_threaded) pthread_create(&th, NULL, phys_thread, NULL);

    /* headless shot list */
    typedef struct { double t; int cam; char file[256]; int done; char cond; } shot_t; shot_t SH[16]; int nsh = 0;
    if (shots) {
        char buf[2048]; snprintf(buf, sizeof buf, "%s", shots);
        for (char* tok = strtok(buf, ","); tok && nsh < 16; tok = strtok(NULL, ",")) {
            char cn[32] = "chase"; SH[nsh].done = 0;
            SH[nsh].cond = 0; if (*tok == 'k' || *tok == 's') SH[nsh].cond = *tok++;
            if (sscanf(tok, "%lf:%31[^:]:%255s", &SH[nsh].t, cn, SH[nsh].file) == 3) {
                SH[nsh].cam = 0; for (int i = 0; i < CAM_N; i++) if (!strcmp(cn, CAMN[i])) SH[nsh].cam = i;
                if (SH[nsh].t + (SH[nsh].cond ? 60 : 0) > max_time) max_time = SH[nsh].t + (SH[nsh].cond ? 60 : 0); nsh++;
            }
        }
    }
    float* wavbuf = NULL; int wavn = 0, wavcap = 0;
    if (wav && headless) { wavcap = (int)((max_time + 1) * 44100); wavbuf = (float*)malloc(sizeof(float) * 2 * wavcap); }
    double last = GetTime();
    while (!WindowShouldClose()) {
        double now = GetTime(), dt = now - last; last = now;
        /* ---- keyboard -> physics control ---- */
        pthread_mutex_lock(&MTX);
        sim_ctl* C = &SIM.ctl;
        C->kb_left = IsKeyDown(KEY_LEFT) || IsKeyDown(KEY_A); C->kb_right = IsKeyDown(KEY_RIGHT) || IsKeyDown(KEY_D);
        C->kb_up = IsKeyDown(KEY_UP) || IsKeyDown(KEY_W); C->kb_down = IsKeyDown(KEY_DOWN) || IsKeyDown(KEY_S);
        C->kb_clutch = IsKeyDown(KEY_LEFT_SHIFT); C->kb_hand = IsKeyDown(KEY_SPACE);
        if (IsKeyPressed(KEY_E) || IsKeyPressed(KEY_PAGE_UP)) C->shift_up++;
        if (IsKeyPressed(KEY_Q) || IsKeyPressed(KEY_PAGE_DOWN) || IsKeyPressed(KEY_Z)) C->shift_down++;
        if (IsKeyPressed(KEY_R)) C->reset++;
        for (int k = 0; k < TRK_NSPOT; k++) if (IsKeyPressed(k < 9 ? KEY_ONE + k : KEY_ZERO)) { C->spot = k; C->spot_req++; }
        if (IsKeyPressed(KEY_P)) C->puncture++;
        if (IsKeyPressed(KEY_H)) C->toggle_assist++;
        if (IsKeyPressed(KEY_O)) C->autopilot++;
        if (IsKeyPressed(KEY_G)) C->drv_toggle++;
        if (IsKeyPressed(KEY_PAUSE) || IsKeyPressed(KEY_F5)) C->pause = !C->pause;
        if (headless) {   /* synchronous fixed-step physics for reproducible shots */
            int n = (int)(shot_dt / 0.001 + 0.5);
            for (int k = 0; k < n; k++) sim_tick(&SIM, 0.001);
            if (wavbuf) { int fr = (int)(shot_dt * 44100 + 0.5); if (wavn + fr <= wavcap) { audio_render(&AUD, wavbuf + 2 * wavn, fr); wavn += fr; } }
            dt = shot_dt;
        }
        VIEW = SIM.snap;
        if (getenv("DEMO_TRACE") && headless && ((int)(VIEW.sim_time * 60 + 0.5)) % 60 == 0) { double d_, s_; track_height(&TRK, VIEW.parts[0].pos[0], VIEW.parts[0].pos[1], NULL, NULL, &d_, &s_);
            printf("[trace] t %.1f s %.0f d %+.1f v %.1f gear %d rpm %.0f ay %+.2f sw %+.0f\n", VIEW.sim_time, s_, d_, VIEW.t.speed * 3.6, VIEW.t.gear, VIEW.t.engine_rpm, VIEW.t.ay / 9.81, VIEW.t.sw_angle * 57.3); }
        pthread_mutex_unlock(&MTX);
        if (IsKeyPressed(KEY_C)) { camode = (camode + 1) % CAM_N; CHASE_INIT = 0; }
        if (IsKeyPressed(KEY_F1)) { int vis = HELP_ON && (HELP_STICKY || GetTime() < HELP_T0 + 12.0); if (vis) HELP_ON = 0; else { HELP_ON = 1; HELP_STICKY = 1; } }
        if (IsKeyPressed(KEY_F2)) xray = !xray;
        if (IsKeyPressed(KEY_F3)) hud = (hud + 1) % 3;
        if (IsKeyPressed(KEY_F4)) forces = !forces;
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) { Vector2 d = GetMouseDelta(); ORB_Y -= d.x * 0.01; ORB_P = fmin(1.4, fmax(-0.1, ORB_P + d.y * 0.01)); }
        ORB_D = fmax(2, ORB_D - GetMouseWheelMove() * 0.5);
        H_ffb[H_n % HIST] = (float)VIEW.t.column_torque; H_sw[H_n % HIST] = (float)VIEW.t.sw_angle; H_ay[H_n % HIST] = (float)VIEW.t.ay; H_n++;

        int cur_cam = camode; shot_t* take = NULL;
        if (headless) for (int i = 0; i < nsh; i++) if (!SH[i].done && VIEW.sim_time >= SH[i].t - 1e-9) {
            int ok = 1;
            if (SH[i].cond == 'k') { ok = 0; for (int w = 0; w < 4; w++) if ((VIEW.surf[w] == SURF_KERB || VIEW.surf[w] == SURF_SAUSAGE) && VIEW.t.w[w].fz > 500) ok = 1; }
            if (SH[i].cond == 's') { ok = 0; for (int w = 0; w < 4; w++) if (VIEW.t.w[w].sliding > 0.5) ok = 1; }
            if (ok) { take = &SH[i]; cur_cam = SH[i].cam; } break; }

        BeginDrawing();
        ClearBackground((Color){150, 185, 225, 255});
        int sw = GetScreenWidth(), sh = GetScreenHeight();
        if (triple && cur_cam == CAM_COCKPIT) {
            /* three panels, each its own frustum yawed by the panel's horizontal FOV (+ bezel) */
            double aspect = (sw / 3.0) / sh, hf = 2 * atan(tan(fovv * M_PI / 360) * aspect);
            double side = cfg_num(&CFG, "display.triple_side_deg", 0) > 0 ? cfg_num(&CFG, "display.triple_side_deg", 0) * M_PI / 180 : hf + cfg_num(&CFG, "display.bezel_deg", 1.5) * M_PI / 180;
            for (int p = 0; p < 3; p++) {
                rlViewport(p * sw / 3, 0, sw / 3, sh);
                Camera3D cam = make_cam(cur_cam, &VIEW, P, fovv, (1 - p) * side, dt);
                rlMatrixMode(RL_PROJECTION); rlLoadIdentity();
                double top = 0.05 * tan(fovv * M_PI / 360), right = top * aspect;
                rlFrustum(-right, right, -top, top, 0.05, 2000);
                rlMatrixMode(RL_MODELVIEW); rlLoadIdentity();
                Matrix view = MatrixLookAt(cam.position, cam.target, cam.up); rlMultMatrixf(MatrixToFloat(view));
                rlEnableDepthTest(); rlDisableBackfaceCulling();
                float vp[3] = {cam.position.x, cam.position.y, cam.position.z}; SetShaderValue(LIT, LOC_VIEW, vp, SHADER_UNIFORM_VEC3);
                for (int i = 0; i < NCH; i++) DrawMesh(CH[i].meshes[0], CH[i].materials[0], MatrixIdentity());
                draw_car(&VIEW, P, xray);
                rlDrawRenderBatchActive();
            }
            rlViewport(0, 0, sw, sh); rlMatrixMode(RL_PROJECTION); rlLoadIdentity(); rlOrtho(0, sw, sh, 0, 0, 1); rlMatrixMode(RL_MODELVIEW); rlLoadIdentity();
            rlDisableDepthTest();
        } else {
            Camera3D cam = make_cam(cur_cam, &VIEW, P, fovv, 0, dt);
            draw_scene(cam, &VIEW, P, xray, forces, cur_cam);
        }
        if (hud) draw_hud(&VIEW, P, sw, sh, GetFPS(), CAMN[cur_cam], hud == 1);
        { double now_ = GetTime(), a = HELP_ON ? (HELP_STICKY ? 1.0 : fmin(1.0, fmax(0.0, (HELP_T0 + 12.0 - now_) / 2.0))) : 0.0;
          if (a > 0 && (!headless || getenv("DEMO_HELP"))) draw_spot_help(sw, sh, (float)a); }
        EndDrawing();
        if (take) {
            Image im = LoadImageFromScreen(); ExportImage(im, take->file); UnloadImage(im);
            printf("[shot] t=%.2f cam=%s -> %s  (v %.1f km/h, ay %.2f g, sw %.0f deg, surf %s/%s/%s/%s)\n", VIEW.sim_time, CAMN[cur_cam], take->file, VIEW.t.speed * 3.6, VIEW.t.ay / 9.81,
                   VIEW.t.sw_angle * 180 / M_PI, surf_name(VIEW.surf[0]), surf_name(VIEW.surf[1]), surf_name(VIEW.surf[2]), surf_name(VIEW.surf[3]));
            take->done = 1;
        }
        if (headless) { int all = 1; for (int i = 0; i < nsh; i++) if (!SH[i].done) all = 0; if (all || VIEW.sim_time >= max_time) break; }
    }
    RUN = 0;
    if (g_threaded) pthread_join(th, NULL);
    if (wavbuf) { audio_write_wav(wav, wavbuf, wavn, 44100); printf("[audio] wrote %s (%.1f s)\n", wav, wavn / 44100.0); free(wavbuf); }
    if (audio_on) { UnloadAudioStream(st); CloseAudioDevice(); }
    CloseWindow();
    sim_shutdown(&SIM);
    return 0;
}
