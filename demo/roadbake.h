/* roadbake.h - prewarm-baked road lattice (demo host side).
 * At car build the road under the drivable band (centreline +- band) is sampled once on the tyre model's world
 * lattice (pitch road_resolve) into 64x64-node tiles: heights as float32 or 16-bit offsets from a per-tile base,
 * grip as an 8-bit index into an exact palette. In the step, a lattice-node query that falls in a baked tile is a
 * hash lookup and a decode; any other query (off the band, off the lattice, normals) goes to the live road. The
 * bake allocates once; lookups allocate nothing and only read, so worker threads may share it.
 * Exact zeros stay exact: every tile base and step is a power-of-two multiple, so a node at height 0 decodes to 0. */
#ifndef ROADBAKE_H
#define ROADBAKE_H
#include "track.h"
typedef struct roadbake roadbake;
typedef struct {
    int bits;              /* 32 or 16 */
    long long tiles, nodes, zeros;      /* tiles baked, nodes, nodes exactly 0 (all decode to 0) */
    long long bytes;       /* tile payload + index */
    double bake_s;         /* wall time of the bake */
    double max_err, rms_err;            /* decoded vs live height over every baked node (m) */
    int grip_vals;         /* palette size */
} roadbake_stats;
/* bits 32 or 16; band = half-width of the baked strip around the centreline (m). NULL on failure (err set). */
roadbake* roadbake_build(const track_t* T, double dw, int bits, double band, char* err, int errn);
void roadbake_free(roadbake* B);
void roadbake_get_stats(const roadbake* B, roadbake_stats* s);
double roadbake_road_fn(void* user, double x, double y, double* nx, double* ny, double* nz, double* grip);   /* dc_road_fn, user = roadbake* */
#endif
