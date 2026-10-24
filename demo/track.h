/* track.h - procedural test circuit: tarmac ribbon with painted lines, rumble kerbs, a sausage kerb, a bump
   section and a plank, grass verges, and an off-road "rough" zone. One analytic height/grip field serves
   both the physics (dc_road_fn) and the render meshes, so what you see is what the tyres touch. */
#ifndef DEMO_TRACK_H
#define DEMO_TRACK_H
#define TRK_MAXS 8192
enum { SURF_TARMAC = 0, SURF_KERB = 1, SURF_GRASS = 2, SURF_ROUGH = 3, SURF_SAUSAGE = 4, SURF_LINE = 5 };
typedef struct {
    int n; double ds, length;
    double px[TRK_MAXS], py[TRK_MAXS], tx[TRK_MAXS], ty[TRK_MAXS], curv[TRK_MAXS], s[TRK_MAXS];
    unsigned char kerb_l[TRK_MAXS], kerb_r[TRK_MAXS];
    double half_w, kerb_w;
    double xmin, xmax, ymin, ymax;
    int gnx, gny; double gcell; int* grid;           /* nearest-sample acceleration grid */
    double rough_cx, rough_cy, rough_r;              /* off-road zone */
    double bump_s0, bump_s1, plank_s, sausage_s0, sausage_s1, start_s;
    double rough_scale;                              /* multiplier on rough-terrain amplitude (config) */
} track_t;
void track_build(track_t* T, double rough_scale);
/* height + surface at (x,y); d_out/s_out: signed lateral offset (+ left) and arc length of the nearest centreline point */
double track_height(const track_t* T, double x, double y, int* surf, double* grip, double* d_out, double* s_out);
double track_road_fn(void* user, double x, double y, double* nx, double* ny, double* nz, double* grip); /* dc_road_fn */
void track_point(const track_t* T, double s, double d, double* x, double* y, double* yaw); /* centreline frame */
#endif
