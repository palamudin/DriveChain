/* track.h - procedural test circuit: tarmac ribbon with painted lines, rumble kerbs, a sausage kerb, a bump
   section and a plank, grass verges, and an off-road "rough" zone. One analytic height/grip field serves
   both the physics (dc_road_fn) and the render meshes, so what you see is what the tyres touch. */
#ifndef DEMO_TRACK_H
#define DEMO_TRACK_H
#define TRK_MAXS 8192
enum { SURF_TARMAC = 0, SURF_KERB = 1, SURF_GRASS = 2, SURF_ROUGH = 3, SURF_SAUSAGE = 4, SURF_LINE = 5, SURF_PAVE = 6, SURF_PAD = 7 };
enum { TRACK_CIRCUIT = 0, TRACK_PROVING = 1 };
#define TRK_MAXCONE 64
#define TRK_NSPOT 9
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
    int layout;                                      /* TRACK_CIRCUIT | TRACK_PROVING */
    /* proving ground (layout 1): see track.c for the full map */
    double bank_deg, s_arc1, s_str2, s_arc2;
    double pad_x0, pad_x1, pad_y0, pad_y1, skid_cx, skid_cy, skid_r;
    int ncone; double cone_x[TRK_MAXCONE], cone_y[TRK_MAXCONE];
} track_t;
void track_build(track_t* T, double rough_scale, int layout);
/* named spawn points (keys 1..9 in the demo): returns name or NULL */
const char* track_spot(const track_t* T, int i, double* x, double* y, double* yaw);
int track_spot_by_name(const track_t* T, const char* name);
/* height + surface at (x,y); d_out/s_out: signed lateral offset (+ left) and arc length of the nearest centreline point */
double track_height(const track_t* T, double x, double y, int* surf, double* grip, double* d_out, double* s_out);
/* proving-ground feature ids for the test-only isolation hook (drivetest kickback): -1 = all features */
enum { PGF_PAVE, PGF_UNDUL25, PGF_UNDUL9, PGF_TWIST, PGF_BUMP_L, PGF_BUMP_R, PGF_POT_L, PGF_POT_R, PGF_CROSSFALL, PGF_RUTS, PGF_BUMPYARC };
extern int track_isolate_feature;
double track_road_fn(void* user, double x, double y, double* nx, double* ny, double* nz, double* grip); /* dc_road_fn */
void track_point(const track_t* T, double s, double d, double* x, double* y, double* yaw); /* centreline frame */
#endif
