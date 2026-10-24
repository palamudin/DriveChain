#include "track.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* ---------- deterministic value noise ---------- */
static double hash2(int x, int y, int seed) {
    unsigned h = (unsigned)x * 374761393u + (unsigned)y * 668265263u + (unsigned)seed * 2246822519u;
    h = (h ^ (h >> 13)) * 1274126177u; h ^= h >> 16;
    return (h & 0xffffff) / (double)0xffffff * 2.0 - 1.0;
}
static double vnoise(double x, double y, int seed) {
    int ix = (int)floor(x), iy = (int)floor(y); double fx = x - ix, fy = y - iy;
    double ux = fx * fx * fx * (fx * (fx * 6 - 15) + 10), uy = fy * fy * fy * (fy * (fy * 6 - 15) + 10); /* C2 quintic */
    double a = hash2(ix, iy, seed), b = hash2(ix + 1, iy, seed), c = hash2(ix, iy + 1, seed), d = hash2(ix + 1, iy + 1, seed);
    return a + (b - a) * ux + (c - a) * uy + (a - b - c + d) * ux * uy;
}
static double smooth01(double t) { t = t < 0 ? 0 : (t > 1 ? 1 : t); return t * t * (3 - 2 * t); }

/* ---------- centreline: closed centripetal Catmull-Rom through control points ---------- */
static const double CP[][2] = {
    {-150, 0}, {0, 0}, {200, 0}, {300, 4}, {362, 38}, {378, 100}, {350, 150}, {290, 165}, {245, 140},
    {205, 150}, {160, 135}, {118, 175}, {60, 205}, {-40, 200}, {-120, 172}, {-200, 125}, {-235, 62}, {-215, 14},
};
#define NCP ((int)(sizeof CP / sizeof CP[0]))
static void cr(double t, const double* p0, const double* p1, const double* p2, const double* p3, double* o) {
    double t2 = t * t, t3 = t2 * t;
    for (int k = 0; k < 2; k++)
        o[k] = 0.5 * ((2 * p1[k]) + (-p0[k] + p2[k]) * t + (2 * p0[k] - 5 * p1[k] + 4 * p2[k] - p3[k]) * t2 + (-p0[k] + 3 * p1[k] - 3 * p2[k] + p3[k]) * t3);
}

void track_build(track_t* T, double rough_scale, int layout) {
    memset(T, 0, sizeof *T);
    T->half_w = 6.0; T->kerb_w = 1.2; T->rough_scale = rough_scale; T->layout = layout;
    /* dense polyline */
    static double dx[200000], dy[200000]; int nd = 0;
    if (layout == TRACK_PROVING) {
        /* proving-ground loop ("oval"): 600 m straights at y = 0 (eastbound) and y = 300 (westbound), R 150 m banked ends.
           s = 0 at (0,0). Straight 1 = pave + undulations, straight 2 = single-wheel bumps / potholes / camber change / ruts,
           west turn = bumpy corner. Inside: flat dynamics pad with a R 40 m skidpad and an 18 m slalom. */
        T->half_w = 5.0; T->kerb_w = 1.0; T->bank_deg = 6.0;
        double R = 150;
        for (double x = 0; x < 600; x += 0.05) { dx[nd] = x; dy[nd] = 0; nd++; }
        for (double a = -M_PI / 2; a < M_PI / 2; a += 0.05 / R) { dx[nd] = 600 + R * cos(a); dy[nd] = 150 + R * sin(a); nd++; }
        for (double x = 600; x > 0; x -= 0.05) { dx[nd] = x; dy[nd] = 300; nd++; }
        for (double a = M_PI / 2; a < 1.5 * M_PI; a += 0.05 / R) { dx[nd] = R * cos(a); dy[nd] = 150 + R * sin(a); nd++; }
        T->s_arc1 = 600; T->s_str2 = 600 + M_PI * R; T->s_arc2 = 1200 + M_PI * R;
        T->pad_x0 = 70; T->pad_x1 = 530; T->pad_y0 = 45; T->pad_y1 = 255;
        T->skid_cx = 170; T->skid_cy = 150; T->skid_r = 40;
        T->ncone = 12; for (int i = 0; i < T->ncone; i++) { T->cone_x[i] = 290 + 18.0 * i; T->cone_y[i] = 150; }
    } else
    for (int i = 0; i < NCP; i++) {
        const double *p0 = CP[(i + NCP - 1) % NCP], *p1 = CP[i], *p2 = CP[(i + 1) % NCP], *p3 = CP[(i + 2) % NCP];
        for (int j = 0; j < 400; j++) { double o[2]; cr(j / 400.0, p0, p1, p2, p3, o); dx[nd] = o[0]; dy[nd] = o[1]; nd++; }
    }
    double L = 0; for (int i = 0; i < nd; i++) { int j = (i + 1) % nd; L += hypot(dx[j] - dx[i], dy[j] - dy[i]); }
    T->n = (int)(L / 0.5); if (T->n > TRK_MAXS) T->n = TRK_MAXS;
    T->ds = L / T->n; T->length = L;
    /* resample at uniform arc length */
    double acc = 0; int seg = 0; double segl = hypot(dx[1] - dx[0], dy[1] - dy[0]);
    for (int k = 0; k < T->n; k++) {
        double target = k * T->ds;
        while (acc + segl < target && seg < nd - 1) { acc += segl; seg++; int j = (seg + 1) % nd; segl = hypot(dx[j] - dx[seg], dy[j] - dy[seg]); }
        int j = (seg + 1) % nd; double f = segl > 0 ? (target - acc) / segl : 0;
        T->px[k] = dx[seg] + (dx[j] - dx[seg]) * f; T->py[k] = dy[seg] + (dy[j] - dy[seg]) * f; T->s[k] = target;
    }
    for (int k = 0; k < T->n; k++) {
        int a = (k + T->n - 1) % T->n, b = (k + 1) % T->n;
        double tx = T->px[b] - T->px[a], ty = T->py[b] - T->py[a], l = hypot(tx, ty);
        T->tx[k] = tx / l; T->ty[k] = ty / l;
    }
    for (int k = 0; k < T->n; k++) {
        int a = (k + T->n - 4) % T->n, b = (k + 4) % T->n;
        double h1 = atan2(T->ty[a], T->tx[a]), h2 = atan2(T->ty[b], T->tx[b]), dh = h2 - h1;
        while (dh > M_PI) dh -= 2 * M_PI; while (dh < -M_PI) dh += 2 * M_PI;
        T->curv[k] = dh / (8 * T->ds);
    }
    /* kerbs where the corner is tighter than R 120 m (both edges, widened 8 m either side) */
    for (int k = 0; k < T->n; k++) {
        if (fabs(T->curv[k]) < 1.0 / 120) continue;
        for (int m = -16; m <= 16; m++) { int q = (k + m + T->n) % T->n; T->kerb_l[q] = T->kerb_r[q] = 1; }
    }
    /* features (arc length along the start straight: start at s=40; bumps 120..190; plank at 215) */
    T->start_s = 40; T->bump_s0 = 110; T->bump_s1 = 185; T->plank_s = 210;
    if (layout == TRACK_PROVING) { memset(T->kerb_l, 0, sizeof T->kerb_l); memset(T->kerb_r, 0, sizeof T->kerb_r); T->start_s = 20; T->bump_s0 = T->bump_s1 = 0; T->plank_s = -1e6; }
    /* sausage kerb: at the chicane apex (find tightest point between CP 8 and 10) */
    double best = 0; int kb = 0;
    for (int k = 0; k < T->n; k++) {
        double x = T->px[k], y = T->py[k];
        if (x > 190 && x < 260 && y > 130 && fabs(T->curv[k]) > best) { best = fabs(T->curv[k]); kb = k; }
    }
    T->sausage_s0 = T->s[kb] - 6; T->sausage_s1 = T->s[kb] + 6;
    T->rough_cx = 95; T->rough_cy = 85; T->rough_r = 42;
    if (layout == TRACK_PROVING) { T->sausage_s0 = T->sausage_s1 = 0; T->rough_r = 0; }
    /* bounds + nearest grid */
    T->xmin = T->ymin = 1e9; T->xmax = T->ymax = -1e9;
    for (int k = 0; k < T->n; k++) {
        if (T->px[k] < T->xmin) T->xmin = T->px[k]; if (T->px[k] > T->xmax) T->xmax = T->px[k];
        if (T->py[k] < T->ymin) T->ymin = T->py[k]; if (T->py[k] > T->ymax) T->ymax = T->py[k];
    }
    T->xmin -= 120; T->ymin -= 120; T->xmax += 120; T->ymax += 120;
    T->gcell = 4.0; T->gnx = (int)((T->xmax - T->xmin) / T->gcell) + 1; T->gny = (int)((T->ymax - T->ymin) / T->gcell) + 1;
    T->grid = (int*)malloc(sizeof(int) * T->gnx * T->gny);
    for (int j = 0; j < T->gny; j++)
        for (int i = 0; i < T->gnx; i++) {
            double x = T->xmin + (i + 0.5) * T->gcell, y = T->ymin + (j + 0.5) * T->gcell, bd = 1e18; int bk = 0;
            for (int k = 0; k < T->n; k += 4) { double d = (T->px[k] - x) * (T->px[k] - x) + (T->py[k] - y) * (T->py[k] - y); if (d < bd) { bd = d; bk = k; } }
            T->grid[j * T->gnx + i] = bk;
        }
}

static void nearest(const track_t* T, double x, double y, double* d, double* s, int* kk) {
    int i = (int)((x - T->xmin) / T->gcell), j = (int)((y - T->ymin) / T->gcell);
    if (i < 0) i = 0; if (j < 0) j = 0; if (i >= T->gnx) i = T->gnx - 1; if (j >= T->gny) j = T->gny - 1;
    int k0 = T->grid[j * T->gnx + i], bk = k0; double bd = 1e18;
    for (int m = -14; m <= 14; m++) {
        int k = (k0 + m + T->n) % T->n;
        double dd = (T->px[k] - x) * (T->px[k] - x) + (T->py[k] - y) * (T->py[k] - y);
        if (dd < bd) { bd = dd; bk = k; }
    }
    /* project on the two adjacent segments for a continuous (s, d) */
    double rx = x - T->px[bk], ry = y - T->py[bk];
    double along = rx * T->tx[bk] + ry * T->ty[bk];
    *d = -rx * T->ty[bk] + ry * T->tx[bk];
    double sv = T->s[bk] + along; if (sv < 0) sv += T->length; if (sv >= T->length) sv -= T->length;
    *s = sv; *kk = bk;
}

/* signed distance along s from a to b in (-L/2, L/2] */
static double sdiff(const track_t* T, double a, double b) { double v = b - a; while (v > T->length / 2) v -= T->length; while (v <= -T->length / 2) v += T->length; return v; }


/* ---------------- proving ground surface (layout 1) ----------------
   Straight 1 (s 0..600, eastbound):  pave 100..300 | long-wave undulations 340..440 (25 m, +-90 mm) | 450..510 (9 m, +-35 mm)
                                      | 515..560 short warp (3.2 m + diagonal 7 m twist)
   East turn (600..1071): banked 6 deg.
   Straight 2 (u = s - s_str2, westbound): left-only bump u 60, right-only bump 110 (50 mm x 0.7 m half-sine),
                                      left pothole 160, right pothole 210 (45 mm x 0.9 m), camber change 260..340 (+-3 %),
                                      tramline ruts 380..500 (18 mm deep, wandering)
   West turn (s_arc2..): banked 6 deg, bumpy (value-noise +-30 mm, 2..5 m waves) from +40 m to +400 m.
   Pad: flat tarmac x 70..530, y 45..255; skidpad circle R 40 at (170,150), lanes marked at R 38 / 42; slalom cones y = 150. */
static double win(double u, double a, double b, double r) { return smooth01((u - a) / r) * smooth01((b - u) / r); }
static double pave_h(double u, double d) {
    /* granite setts ~16 x 12 cm, staggered rows, individual set height +-14 mm, 3 mm domed tops, 8 mm joints 18 mm deep,
       plus the long-wave unevenness of an old cobbled road (+-25 mm over 1.5..4 m) and a strong 4 % crown */
    const double L = 0.16, W = 0.12;
    int r = (int)floor(u / L); double fu = u / L - r;
    double dd = d / W + ((r & 1) ? 0.5 : 0.0); int c = (int)floor(dd); double fd = dd - c;
    double hs = 0.010 * hash2(r, c, 41) + 0.004 * hash2(r, c, 43);
    double dome = 0.003 * sin(M_PI * fu) * sin(M_PI * fd);
    double e = fmin(fmin(fu, 1 - fu) * L, fmin(fd, 1 - fd) * W);
    double edge = smooth01((e - 0.004) / 0.012);
    double top = hs + dome;
    return -0.018 + (0.018 + top) * edge + 0.017 * vnoise(u / 4.0, d / 4.0, 47) + 0.008 * vnoise(u / 1.5, d / 1.5, 53);
}
static double pg_road(const track_t* T, double s, double d, double x, double y, int* sf, double* g) {
    double h = 0;
    if (s >= 100 && s <= 300) {
        double w = win(s, 100, 300, 1.0);
        h += w * (pave_h(s, d) - 0.04 * (d / T->half_w) * (d / T->half_w));
        if (w > 0.5) { *sf = SURF_PAVE; *g = 0.85; }
    }
    if (s > 340 && s < 440) h += win(s, 340, 440, 12) * 0.09 * sin(2 * M_PI * (s - 340) / 25.0);
    if (s > 450 && s < 510) h += win(s, 450, 510, 6) * 0.035 * sin(2 * M_PI * (s - 450) / 9.0);
    if (s > 515 && s < 560) h += win(s, 515, 560, 4) * (0.015 * sin(2 * M_PI * (s - 515) / 3.2) + 0.02 * sin(2 * M_PI * (s - 515) / 7.0) * d / T->half_w);
    double u = s - T->s_str2;
    if (u > 0 && u < 600) {
        /* single-wheel bumps (car centred: wheels at d = +-0.7) */
        static const double bu[2] = {60, 110}, bs[2] = {1, -1};
        for (int i = 0; i < 2; i++) if (u > bu[i] && u < bu[i] + 0.7) {
            double lat = smooth01((bs[i] * d - 0.15) / 0.1) * smooth01((1.7 - bs[i] * d) / 0.1);
            h += 0.05 * sin(M_PI * (u - bu[i]) / 0.7) * lat;
        }
        static const double pu[2] = {160, 210}, ps[2] = {1, -1};
        for (int i = 0; i < 2; i++) if (u > pu[i] - 0.1 && u < pu[i] + 1.0) {
            double lat = smooth01((ps[i] * d - 0.3) / 0.04) * smooth01((1.2 - ps[i] * d) / 0.04);
            h -= 0.045 * smooth01((u - pu[i]) / 0.04) * smooth01((pu[i] + 0.9 - u) / 0.04) * lat;
        }
        if (u > 260 && u < 340) h += 0.03 * sin(2 * M_PI * (u - 260) / 80.0) * d;   /* cross-slope +3 % -> -3 % */
        if (u > 380 && u < 500) {
            double w = win(u, 380, 500, 8), c0 = 0.72 + 0.12 * sin(2 * M_PI * (u - 380) / 35.0) + 0.0 * d;
            for (int sd = -1; sd <= 1; sd += 2) { double q = (d - sd * c0) / 0.22; if (fabs(q) < 1) h -= w * 0.018 * 0.5 * (1 + cos(M_PI * q)); }
        }
    }
    double ua = s - T->s_arc2;
    if (ua > 40 && ua < 400) h += win(ua, 40, 400, 10) * (0.022 * vnoise(x / 4.0, y / 4.0, 61) + 0.010 * vnoise(x / 1.8, y / 1.8, 67));
    return h;
}
/* banking: outside of the bends raised, ramps in over 40 m; returns height at lateral offset d (clamped to the road band) */
static double pg_bank(const track_t* T, double s, double d) {
    double w = 0;
    if (s > T->s_arc1 - 40 && s < T->s_str2 + 40) w = win(s, T->s_arc1 - 40, T->s_str2 + 40, 40);
    else if (s > T->s_arc2 - 40 || s < 40) { double ss = s < 40 ? s + T->length : s; w = win(ss, T->s_arc2 - 40, T->length + 40, 40); }
    double dc = fmax(-T->half_w, fmin(T->half_w, d));
    return -w * tan(T->bank_deg * M_PI / 180) * dc;   /* CCW loop: inside of the bends is +d, outer edge (-d) raised */
}
static double pg_height(const track_t* T, double x, double y, double d, double s, int* surf, double* grip) {
    double ad = fabs(d), hw = T->half_w;
    int sf = SURF_GRASS; double g = 0.62, h;
    double bank = pg_bank(T, s, d);
    if (ad <= hw) {
        sf = SURF_TARMAC; g = 1.0;
        h = -0.015 * ad * ad / (2 * hw) + bank + pg_road(T, s, d, x, y, &sf, &g);
        if (ad > hw - 0.35 && ad < hw - 0.2) sf = SURF_LINE;
    } else {
        double e = ad - hw;
        double edge = -0.015 * hw / 2 + bank;
        h = edge * (1 - smooth01(e / 10.0)) - 0.03 * smooth01(e / 0.5) + smooth01((e - 0.5) / 2.0) * (0.010 * vnoise(x * 2.3, y * 2.3, 7) + 0.020 * vnoise(x * 0.7, y * 0.7, 3));
        if (x > T->pad_x0 && x < T->pad_x1 && y > T->pad_y0 && y < T->pad_y1) {
            double ep = fmin(fmin(x - T->pad_x0, T->pad_x1 - x), fmin(y - T->pad_y0, T->pad_y1 - y));
            double w = smooth01(ep / 1.0);
            h = h * (1 - w);
            sf = SURF_PAD; g = 1.0;
            double r = hypot(x - T->skid_cx, y - T->skid_cy);
            if (fabs(r - (T->skid_r - 2)) < 0.12 || fabs(r - (T->skid_r + 2)) < 0.12 || r < 0.4) sf = SURF_LINE;
            if (fabs(y - T->cone_y[0]) < 0.08 && x > T->cone_x[0] - 30 && x < T->cone_x[T->ncone - 1] + 30) sf = SURF_LINE;
        }
    }
    if (surf) *surf = sf; if (grip) *grip = g;
    return h;
}
double track_height(const track_t* T, double x, double y, int* surf, double* grip, double* d_out, double* s_out) {
    double d, s; int k; nearest(T, x, y, &d, &s, &k);
    if (d_out) *d_out = d; if (s_out) *s_out = s;
    if (T->layout == TRACK_PROVING) return pg_height(T, x, y, d, s, surf, grip);
    double ad = fabs(d), hw = T->half_w, kw = T->kerb_w;
    int kerb = d > 0 ? T->kerb_l[k] : T->kerb_r[k];
    int sf = SURF_TARMAC; double g = 1.0, h = 0;
    /* tarmac: slight crown (1.5 % fall to each edge) + bump section + plank */
    double ac = fmin(ad, hw + kw);   /* crown only across the road band (was unbounded -> terrain sank far from the track) */
    double tar = -0.015 * ac * ac / (2 * hw);
    {
        double u = sdiff(T, T->bump_s0, s), len = T->bump_s1 - T->bump_s0;
        if (u > 0 && u < len) {
            double env = sin(M_PI * u / len);
            tar += env * (0.022 * sin(2 * M_PI * u / 6.5) + 0.012 * sin(2 * M_PI * (u + 0.6 * d) / 2.3) + 0.010 * sin(2 * M_PI * (u - 0.9 * d) / 4.1));
        }
        double p = sdiff(T, T->plank_s, s);   /* 2 cm x 0.30 m plank across the road, slightly rounded edges */
        if (fabs(p) < 0.15 + 0.02) tar += 0.020 * smooth01((0.15 + 0.02 - fabs(p)) / 0.04);
    }
    if (ad <= hw) {
        h = tar;
        if (ad > hw - 0.35 && ad < hw - 0.2) sf = SURF_LINE;
        double u = sdiff(T, T->start_s, s); if (u > -0.5 && u < 0.5) sf = SURF_LINE;
    } else if (kerb && ad <= hw + kw) {
        /* rumble kerb: 25 mm crowned body, 8 mm ridges every 0.5 m, chamfered 0.25 m leading edge */
        double e = ad - hw, base = tar - 0.015 * (e) / 1.0;
        double prof = 0.025 * smooth01(e / 0.25) * (1 - smooth01((e - (kw - 0.25)) / 0.25));
        double ridge = 0.008 * fmax(0.0, sin(2 * M_PI * s / 0.5)) * smooth01(e / 0.3) * (1 - smooth01((e - kw + 0.3) / 0.3));
        h = base + prof + ridge; sf = SURF_KERB; g = 0.92;
    } else {
        /* verge: drops 3 cm over 0.5 m then grass with noise; rough zone and far terrain get more */
        double e = ad - hw - (kerb ? kw : 0);
        double edge = tar - 0.015 * (kerb ? kw : 0);
        double nfine = 0.010 * vnoise(x * 2.3, y * 2.3, 7) + 0.020 * vnoise(x * 0.7, y * 0.7, 3);
        double nbig = 0.25 * vnoise(x / 18.0, y / 18.0, 11) + 0.10 * vnoise(x / 7.0, y / 7.0, 5);
        double fade = smooth01((e - 0.5) / 2.0), far = smooth01((e - 8) / 25.0);
        h = edge - 0.03 * smooth01(e / 0.5) + fade * nfine * T->rough_scale + far * nbig;
        sf = SURF_GRASS; g = 0.62;
        double rr = hypot(x - T->rough_cx, y - T->rough_cy);
        if (rr < T->rough_r) {
            double w = smooth01((T->rough_r - rr) / 8.0);
            double rough = 0.09 * vnoise(x / 2.2, y / 2.2, 21) + 0.05 * vnoise(x / 0.9, y / 0.9, 23) + 0.015 * vnoise(x * 3.1, y * 3.1, 29)
                         + 0.6 * vnoise(x / 12.0, y / 12.0, 31);
            h += w * rough * T->rough_scale; if (w > 0.3) { sf = SURF_ROUGH; g = 0.75; }
        }
    }
    /* sausage kerb on the chicane apex inside edge: 60 mm tall, 0.35 m wide, rounded */
    {
        double u = sdiff(T, T->sausage_s0, s), len = T->sausage_s1 - T->sausage_s0;
        if (u > 0 && u < len && T->curv[k] != 0) {
            double side = T->curv[k] > 0 ? 1 : -1;   /* inside of the bend */
            double e = side * d - (hw - 0.6);
            if (fabs(e) < 0.25) {
                double t = 1 - (e / 0.25) * (e / 0.25);
                h += 0.06 * t * smooth01(u / 0.6) * smooth01((len - u) / 0.6); sf = SURF_SAUSAGE; g = 0.85;
            }
        }
    }
    if (surf) *surf = sf; if (grip) *grip = g;
    return h;
}

double track_road_fn(void* user, double x, double y, double* nx, double* ny, double* nz, double* grip) {
    const track_t* T = (const track_t*)user;
    const double e = 0.02;
    double h = track_height(T, x, y, NULL, grip, NULL, NULL);
    double hx = (track_height(T, x + e, y, NULL, NULL, NULL, NULL) - track_height(T, x - e, y, NULL, NULL, NULL, NULL)) / (2 * e);
    double hy = (track_height(T, x, y + e, NULL, NULL, NULL, NULL) - track_height(T, x, y - e, NULL, NULL, NULL, NULL)) / (2 * e);
    double l = sqrt(hx * hx + hy * hy + 1);
    *nx = -hx / l; *ny = -hy / l; *nz = 1 / l;
    return h;
}

void track_point(const track_t* T, double s, double d, double* x, double* y, double* yaw) {
    while (s < 0) s += T->length; while (s >= T->length) s -= T->length;
    int k = (int)(s / T->ds); if (k >= T->n) k = T->n - 1; int k2 = (k + 1) % T->n; double f = s / T->ds - k;
    double px = T->px[k] + (T->px[k2] - T->px[k]) * f, py = T->py[k] + (T->py[k2] - T->py[k]) * f;
    double tx = T->tx[k] + (T->tx[k2] - T->tx[k]) * f, ty = T->ty[k] + (T->ty[k2] - T->ty[k]) * f, l = hypot(tx, ty);
    tx /= l; ty /= l;
    *x = px - ty * d; *y = py + tx * d; if (yaw) *yaw = atan2(ty, tx);
}

static const char* SPOTN[TRK_NSPOT] = {"start", "skidpad", "pave", "undulations", "oval", "slalom", "bumps", "ruts", "bumpy_corner"};
const char* track_spot(const track_t* T, int i, double* x, double* y, double* yaw) {
    if (i < 0 || i >= TRK_NSPOT) return NULL;
    if (T->layout != TRACK_PROVING) { if (i) return NULL; track_point(T, T->start_s, 0, x, y, yaw); return SPOTN[0]; }
    switch (i) {
    case 1: *x = T->skid_cx; *y = T->skid_cy - T->skid_r; *yaw = 0; break;   /* on the ring, driving CCW */
    case 5: *x = T->cone_x[0] - 60; *y = T->cone_y[0]; *yaw = 0; break;
    default: {
        static const double ss[TRK_NSPOT] = {20, 0, 60, 300, 560, 0, 0, 0, 0};
        double s = ss[i];
        if (i == 6) s = T->s_str2 + 20; if (i == 7) s = T->s_str2 + 350; if (i == 8) s = T->s_arc2 + 10;
        track_point(T, s, 0, x, y, yaw);
    } }
    return SPOTN[i];
}
int track_spot_by_name(const track_t* T, const char* name) {
    for (int i = 0; i < TRK_NSPOT; i++) if (!strcmp(name, SPOTN[i])) return i;
    return -1;
}
