/* dc_math.h - small double-precision vector / matrix helpers (header-only, no allocation). */
#ifndef DC_MATH_H
#define DC_MATH_H
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <string.h>

typedef struct { double x, y, z; } v3;
typedef struct { double m[3][3]; } m3;          /* row-major */
typedef struct { double w, x, y, z; } qt;

static inline v3 V3(double x, double y, double z) { v3 r = {x, y, z}; return r; }
static inline v3 vadd(v3 a, v3 b) { return V3(a.x + b.x, a.y + b.y, a.z + b.z); }
static inline v3 vsub(v3 a, v3 b) { return V3(a.x - b.x, a.y - b.y, a.z - b.z); }
static inline v3 vscl(v3 a, double s) { return V3(a.x * s, a.y * s, a.z * s); }
static inline v3 vmad(v3 a, v3 b, double s) { return V3(a.x + b.x * s, a.y + b.y * s, a.z + b.z * s); }
static inline double vdot(v3 a, v3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline v3 vcross(v3 a, v3 b) { return V3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }
static inline double vlen(v3 a) { return sqrt(vdot(a, a)); }
static inline v3 vnorm(v3 a) { double l = vlen(a); return l > 1e-300 ? vscl(a, 1.0 / l) : V3(0, 0, 0); }
static inline v3 vneg(v3 a) { return V3(-a.x, -a.y, -a.z); }
static inline double vget(v3 a, int i) { return i == 0 ? a.x : (i == 1 ? a.y : a.z); }

static inline m3 m3id(void) { m3 r; memset(&r, 0, sizeof r); r.m[0][0] = r.m[1][1] = r.m[2][2] = 1; return r; }
static inline v3 mv(const m3* A, v3 v) {
    return V3(A->m[0][0] * v.x + A->m[0][1] * v.y + A->m[0][2] * v.z,
              A->m[1][0] * v.x + A->m[1][1] * v.y + A->m[1][2] * v.z,
              A->m[2][0] * v.x + A->m[2][1] * v.y + A->m[2][2] * v.z);
}
static inline v3 mtv(const m3* A, v3 v) { /* A^T v */
    return V3(A->m[0][0] * v.x + A->m[1][0] * v.y + A->m[2][0] * v.z,
              A->m[0][1] * v.x + A->m[1][1] * v.y + A->m[2][1] * v.z,
              A->m[0][2] * v.x + A->m[1][2] * v.y + A->m[2][2] * v.z);
}
static inline m3 mmul(const m3* A, const m3* B) {
    m3 r; for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++)
        r.m[i][j] = A->m[i][0] * B->m[0][j] + A->m[i][1] * B->m[1][j] + A->m[i][2] * B->m[2][j];
    return r;
}
static inline m3 mtrans(const m3* A) { m3 r; for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) r.m[i][j] = A->m[j][i]; return r; }
static inline v3 mcol(const m3* A, int j) { return V3(A->m[0][j], A->m[1][j], A->m[2][j]); }

/* Rodrigues: rotation matrix for rotation vector w */
static inline m3 mexp(v3 w) {
    double th = vlen(w); m3 R = m3id();
    double a, b;
    if (th < 1e-8) { a = 1.0 - th * th / 6.0; b = 0.5 - th * th / 24.0; }
    else { a = sin(th) / th; b = (1.0 - cos(th)) / (th * th); }
    double K[3][3] = {{0, -w.z, w.y}, {w.z, 0, -w.x}, {-w.y, w.x, 0}};
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
        double kk = 0; for (int k = 0; k < 3; k++) kk += K[i][k] * K[k][j];
        R.m[i][j] += a * K[i][j] + b * kk;
    }
    return R;
}
static inline m3 morthonorm(m3 R) { /* Gram-Schmidt on columns */
    v3 c0 = vnorm(mcol(&R, 0)); v3 c1 = mcol(&R, 1);
    c1 = vnorm(vsub(c1, vscl(c0, vdot(c0, c1)))); v3 c2 = vcross(c0, c1);
    m3 r; r.m[0][0] = c0.x; r.m[1][0] = c0.y; r.m[2][0] = c0.z;
    r.m[0][1] = c1.x; r.m[1][1] = c1.y; r.m[2][1] = c1.z;
    r.m[0][2] = c2.x; r.m[1][2] = c2.y; r.m[2][2] = c2.z; return r;
}
static inline qt qmul(qt a, qt b) {
    qt r = {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
    return r;
}
static inline qt qnorm(qt q) { double l = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z); q.w /= l; q.x /= l; q.y /= l; q.z /= l; return q; }
static inline qt qexp(v3 w) { /* quaternion for rotation vector w */
    double th = vlen(w); qt q;
    if (th < 1e-12) { q.w = 1; q.x = w.x * 0.5; q.y = w.y * 0.5; q.z = w.z * 0.5; return qnorm(q); }
    double s = sin(0.5 * th) / th; q.w = cos(0.5 * th); q.x = w.x * s; q.y = w.y * s; q.z = w.z * s; return q;
}
static inline m3 qtom(qt q) {
    m3 R; double w = q.w, x = q.x, y = q.y, z = q.z;
    R.m[0][0] = 1 - 2 * (y * y + z * z); R.m[0][1] = 2 * (x * y - w * z); R.m[0][2] = 2 * (x * z + w * y);
    R.m[1][0] = 2 * (x * y + w * z); R.m[1][1] = 1 - 2 * (x * x + z * z); R.m[1][2] = 2 * (y * z - w * x);
    R.m[2][0] = 2 * (x * z - w * y); R.m[2][1] = 2 * (y * z + w * x); R.m[2][2] = 1 - 2 * (x * x + y * y);
    return R;
}
static inline qt mtoq(const m3* R) {
    qt q; double t = R->m[0][0] + R->m[1][1] + R->m[2][2];
    if (t > 0) { double s = sqrt(t + 1.0) * 2; q.w = 0.25 * s; q.x = (R->m[2][1] - R->m[1][2]) / s; q.y = (R->m[0][2] - R->m[2][0]) / s; q.z = (R->m[1][0] - R->m[0][1]) / s; }
    else if (R->m[0][0] > R->m[1][1] && R->m[0][0] > R->m[2][2]) { double s = sqrt(1.0 + R->m[0][0] - R->m[1][1] - R->m[2][2]) * 2; q.w = (R->m[2][1] - R->m[1][2]) / s; q.x = 0.25 * s; q.y = (R->m[0][1] + R->m[1][0]) / s; q.z = (R->m[0][2] + R->m[2][0]) / s; }
    else if (R->m[1][1] > R->m[2][2]) { double s = sqrt(1.0 + R->m[1][1] - R->m[0][0] - R->m[2][2]) * 2; q.w = (R->m[0][2] - R->m[2][0]) / s; q.x = (R->m[0][1] + R->m[1][0]) / s; q.y = 0.25 * s; q.z = (R->m[1][2] + R->m[2][1]) / s; }
    else { double s = sqrt(1.0 + R->m[2][2] - R->m[0][0] - R->m[1][1]) * 2; q.w = (R->m[1][0] - R->m[0][1]) / s; q.x = (R->m[0][2] + R->m[2][0]) / s; q.y = (R->m[1][2] + R->m[2][1]) / s; q.z = 0.25 * s; }
    return qnorm(q);
}
/* R * diag(I) * R^T applied to w */
static inline v3 inertia_world(const m3* R, v3 Ibody, v3 w) {
    v3 wl = mtv(R, w); wl.x *= Ibody.x; wl.y *= Ibody.y; wl.z *= Ibody.z; return mv(R, wl);
}
static inline double clampd(double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); }
static inline double sgn(double x) { return x > 0 ? 1.0 : (x < 0 ? -1.0 : 0.0); }

/* Dense solve A x = b (n<=16) by Gaussian elimination with partial pivoting. A, b overwritten. Returns 0 if singular. */
static inline int dc_solve(int n, double* A, double* b) {
    for (int k = 0; k < n; k++) {
        int p = k; double mx = fabs(A[k * n + k]);
        for (int i = k + 1; i < n; i++) if (fabs(A[i * n + k]) > mx) { mx = fabs(A[i * n + k]); p = i; }
        if (mx < 1e-300) return 0;
        if (p != k) { for (int j = 0; j < n; j++) { double t = A[k * n + j]; A[k * n + j] = A[p * n + j]; A[p * n + j] = t; } double t = b[k]; b[k] = b[p]; b[p] = t; }
        double inv = 1.0 / A[k * n + k];
        for (int i = k + 1; i < n; i++) {
            double f = A[i * n + k] * inv; if (f == 0) continue;
            for (int j = k; j < n; j++) A[i * n + j] -= f * A[k * n + j];
            b[i] -= f * b[k];
        }
    }
    for (int i = n - 1; i >= 0; i--) { double s = b[i]; for (int j = i + 1; j < n; j++) s -= A[i * n + j] * b[j]; b[i] = s / A[i * n + i]; }
    return 1;
}
/* Cholesky solve for SPD A (n<=16): in place. */
static inline int dc_chol_solve(int n, double* A, double* b) {
    for (int j = 0; j < n; j++) {
        double s = A[j * n + j]; for (int k = 0; k < j; k++) s -= A[j * n + k] * A[j * n + k];
        if (s <= 0) return 0; double d = sqrt(s); A[j * n + j] = d;
        for (int i = j + 1; i < n; i++) { double t = A[i * n + j]; for (int k = 0; k < j; k++) t -= A[i * n + k] * A[j * n + k]; A[i * n + j] = t / d; }
    }
    for (int i = 0; i < n; i++) { double s = b[i]; for (int k = 0; k < i; k++) s -= A[i * n + k] * b[k]; b[i] = s / A[i * n + i]; }
    for (int i = n - 1; i >= 0; i--) { double s = b[i]; for (int k = i + 1; k < n; k++) s -= A[k * n + i] * b[k]; b[i] = s / A[i * n + i]; }
    return 1;
}
#endif
