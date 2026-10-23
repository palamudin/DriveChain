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
