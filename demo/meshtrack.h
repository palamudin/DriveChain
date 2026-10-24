/* meshtrack.h - triangle-mesh drivable map: a collider loaded as is (no smoothing or remeshing) and queried as a
   2.5-D height field for the tyres, plus optional render-only scenery.

   File format "DCMESH01" (little endian):
     char magic[8]; int32 nv, nt, reserved; double v[nv][3] (x fwd/east, y left/north, z up, metres);
     int32 tri[nt][3]; uint8 surface[nt]  (SURF_* ids from track.h)

   Height query: the highest non-wall triangle whose XY projection contains the point (exact barycentric
   plane height, exact face normal).  Points that fall in a crack between triangles (seam / T-junction up to
   MT_SEAM_TOL) take the closest point of the nearest triangle, so tiny gaps never open a hole; anything further
   from every triangle is "off the map" (very low height, counted in MeshTrack.misses).
   Walls (SURF_WALL) are not ground: the core has no body/wall contact, so they are render-only for now. */
#ifndef DEMO_MESHTRACK_H
#define DEMO_MESHTRACK_H
#define MT_SEAM_TOL 0.05
typedef struct meshtrk_s {
    int nv, nt; double* v; int* t; unsigned char* s;    /* collider */
    double* n;                                         /* per-triangle unit normal (z >= 0 for ground) */
    double bmin[3], bmax[3];
    double gx0, gy0, gcell; int gnx, gny; int* cstart; int* cidx;   /* XY grid of ground triangles */
    int snv, snt; double* sv; int* st; unsigned char* ss;           /* scenery (render only), may be empty */
    double grip[16];                                   /* per surface */
    int n_flipped, n_vertical, n_wall, n_degenerate;   /* load diagnostics */
    volatile long misses;                              /* off-map queries (diagnostic, not deterministic across threads) */
} meshtrk_t;
int mt_read(const char* path, int* nv, int* nt, double** v, int** t, unsigned char** s, char* err, int errn);
int mt_build(meshtrk_t* M, const char* collider, const char* scenery, char* err, int errn);
/* height at (x,y); n (3 doubles, may be NULL) = face normal; surf/grip/tri may be NULL. Returns zmin - 20 m off map. */
double mt_height(const meshtrk_t* M, double x, double y, double* n, int* surf, double* grip, int* tri);
/* all ground triangles whose XY projection contains (x,y): heights into z[] (up to max), returns the count */
int mt_layers(const meshtrk_t* M, double x, double y, double* z, int* tri, int max);
/* scenery.bin colour classes (render only; the tyre never sees scenery): ids >= 32, anything else uses the surface colours */
enum { MESH_SC_FOLIAGE = 32, MESH_SC_TYRES = 33, MESH_SC_FENCE = 34 };
void mt_free(meshtrk_t* M);
void mt_free_scenery(meshtrk_t* M);   /* render-only data, once uploaded */
/* default dry-friction multiplier relative to dry asphalt for each SURF_* id (see meshtrack.c for sources) */
double mt_default_grip(int surf);
#endif
