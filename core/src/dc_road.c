/* dc_road.c - road query: user callback, baked heightfield (bilinear), or flat ground. */
#include "dc_internal.h"

static double hf_h(const dc_heightfield* f, int i, int j) {
    if (i < 0) i = 0; if (j < 0) j = 0; if (i >= f->nx) i = f->nx - 1; if (j >= f->ny) j = f->ny - 1;
    return f->h[j * f->nx + i];
}
double dc_road_eval(const dc_car* c, double x, double y, v3* n, double* grip) {
    if (c->road) {
        double nx = 0, ny = 0, nz = 1, g = 1;
        double h = c->road(c->road_user, x, y, &nx, &ny, &nz, &g);
        *n = V3(nx, ny, nz); *grip = g; return h;
    }
    if (c->hf.h) {
        const dc_heightfield* f = &c->hf;
        double u = (x - f->x0) / f->cell, v = (y - f->y0) / f->cell;
        int i = (int)floor(u), j = (int)floor(v); double fu = u - i, fv = v - j;
        double h00 = hf_h(f, i, j), h10 = hf_h(f, i + 1, j), h01 = hf_h(f, i, j + 1), h11 = hf_h(f, i + 1, j + 1);
        double h = h00 * (1 - fu) * (1 - fv) + h10 * fu * (1 - fv) + h01 * (1 - fu) * fv + h11 * fu * fv;
        double dhx = ((h10 - h00) * (1 - fv) + (h11 - h01) * fv) / f->cell;
        double dhy = ((h01 - h00) * (1 - fu) + (h11 - h10) * fu) / f->cell;
        *n = vnorm(V3(-dhx, -dhy, 1));
        *grip = 1;
        if (f->grip) {
            int ii = (int)floor(u + 0.5), jj = (int)floor(v + 0.5);
            if (ii < 0) ii = 0; if (jj < 0) jj = 0; if (ii >= f->nx) ii = f->nx - 1; if (jj >= f->ny) jj = f->ny - 1;
            *grip = f->grip[jj * f->nx + ii];
        }
        return h;
    }
    *n = V3(0, 0, 1); *grip = 1; return 0;
}

/* Height-only query for the tyre's world-lattice sampler, which builds its own slopes from the lattice. */
double dc_road_height(const dc_car* c, double x, double y, double* grip) {
    if (c->road && (c->road_flags & DC_ROAD_HEIGHT_ONLY)) { double g = 1; double h = c->road(c->road_user, x, y, NULL, NULL, NULL, &g); *grip = g; return h; }
    v3 n; return dc_road_eval(c, x, y, &n, grip);
}

/* Body (chassis) point contact. A point below a height field is pushed out along the SHORTEST escape: vertically onto
   the surface (depth (H - z) n_z, the distance to the local tangent plane) or horizontally out through a step face.
   The vertical-only rule is wrong at a face higher than the body clearance (wall, ledge, tall kerb): a point that
   crosses the face is suddenly "deep" (up to the step height) and is shot upward - a non-conservative jump that
   launched the car off a 1 m ledge face. With the shortest escape the depth is continuous as the point crosses the
   face (0 at the face, growing with horizontal intrusion) and the reaction is horizontal, as from a wall.
   Horizontal search: 16 world-fixed directions (worst-case direction error 11.25 deg, depth error 2 %), marched at
   BODY_STEP to BODY_REACH, the first crossing refined by bisection. Run only when the point is inside the road. */
#define BODY_NDIR 16
#define BODY_STEP 0.05
#define BODY_REACH 0.6
double dc_body_depth(const dc_car* c, v3 X, v3* n, double* grip) {
    v3 nv; double g;
    double H = dc_road_eval(c, X.x, X.y, &nv, &g);
    double dv = (H - X.z) * nv.z;
    if (dv <= 0) { *n = nv; *grip = g; return 0; }
    double best = dv; v3 bn = nv;
    for (int k = 0; k < BODY_NDIR; k++) {
        double an = 2 * M_PI * k / BODY_NDIR, ex = cos(an), ey = sin(an);
        double s_in = 0, s_out = -1;
        for (double s = BODY_STEP; s <= BODY_REACH + 1e-9 && s < best; s += BODY_STEP) {
            double gg; if (dc_road_height(c, X.x + s * ex, X.y + s * ey, &gg) < X.z) { s_out = s; break; }
            s_in = s;
        }
        if (s_out < 0) continue;
        for (int it = 0; it < 10; it++) {          /* bisection to BODY_STEP / 1024 ~ 0.05 mm */
            double sm = 0.5 * (s_in + s_out), gg;
            if (dc_road_height(c, X.x + sm * ex, X.y + sm * ey, &gg) < X.z) s_out = sm; else s_in = sm;
        }
        double d = 0.5 * (s_in + s_out);
        if (d < best) { best = d; bn = V3(ex, ey, 0); }
    }
    *n = bn; *grip = g; return best;
}

/* Body (structure) contact normal force for point k at penetration pen, closing speed -vn.
   Elastic-plastic with linear isotropic hardening and a permanent set (1-D return mapping, committed once per
   substep: both cores evaluate the body contacts exactly once per step):
     elastic   N = k (pen - p)                  while k (pen - p) <= Fy + H p
     plastic   p = (k pen - Fy) / (k + H)       (loading tangent k H / (k + H), the crush stiffness)
   Unloading is elastic with the stiff k, so a crash returns only the small elastic energy (restitution ~0.1-0.3,
   in line with full-scale barrier tests) and the crush work is dissipated -- an elastic penalty spring stored the
   whole impact energy and gave it back, which launched the car off walls.  Crush parameters follow the Campbell
   linear crush model (force per width A + B C, SAE 740565; NHTSA small-car A ~35-60 kN/m, B ~0.5-1.2 MN/m^2):
   per point (two points share ~1.3 m of front width) Fy ~ 40 kN, H ~ 0.6 MN/m.  Light scrapes stay elastic. */
double dc_body_normal(dc_car* c, int k, double pen, double vn, int nodamp, double* pe, double* wdis, double* Nd) {
    const dc_car_params* P = &c->P;
    double p = c->body_set[k], e = pen - p;
    *pe = 0; *wdis = 0; *Nd = 0;
    if (e <= 0) return 0;
    if (P->body_fy > 0 && P->body_k * e > P->body_fy + P->body_kh * p) {
        double pn = (P->body_k * pen - P->body_fy) / (P->body_k + P->body_kh);
        /* crush work this step: the yield force (mean over the set increment) times the new permanent set; it leaves
           the elastic store for good (the hardening part is locked into the deformed structure, not recoverable) */
        *wdis = (P->body_fy + P->body_kh * 0.5 * (p + pn)) * (pn - p);
        p = pn; c->body_set[k] = p; e = pen - p;
    }
    *pe = 0.5 * P->body_k * e * e;
    double Ne = P->body_k * e, N = Ne - (nodamp ? 0 : P->body_c * vn);
    if (N < 0) N = 0;
    *Nd = N - Ne;          /* damping part actually applied (clipped with the total) */
    return N;
}
