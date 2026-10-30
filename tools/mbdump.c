/* mbdump: print the multibody structure of a car (bodies, joints, row counts, factor size) for brick planning. */
#include "../demo/sim.h"
#include "../core/src/dc_mbcar.h"
#include <stdio.h>
#include <string.h>
static const char* TN[] = {"spherical","revolute","ujoint","ujoint_slip","distance","prismatic","weld","pointline","custom","bush","beam"};
int main(int argc, char** argv) {
    static cfg_t cfg; static sim_t S; static track_t T; char err[256];
    cfg_set(&cfg, "car.setup", "default"); cfg_set(&cfg, "physics.tick_hz", "240"); cfg_set(&cfg, "physics.substeps", "4");
    if (argc > 1 && *argv[1]) cfg_set(&cfg, "car.ini", argv[1]);
    for (int i = 2; i < argc; i++) { char b[256]; snprintf(b, sizeof b, "%s", argv[i]); char* e = strchr(b, '='); if (e) { *e = 0; cfg_set(&cfg, b, e + 1); } }
    if (!track_select(&T, "proving", 1.0, err, sizeof err)) { fprintf(stderr, "%s\n", err); return 1; }
    if (!sim_init(&S, &cfg, &T, NULL, err, sizeof err)) { fprintf(stderr, "%s\n", err); return 1; }
    for (int i = 0; i < 20; i++) sim_tick(&S, S.tick_dt);
    mb_world* W = S.car->mb->W;
    int rows_f = 0, rows_b = 0;
    printf("bodies %d\n", W->nb);
    for (int b = 0; b < W->nb; b++) printf("  B%-2d %-14s m %.3f\n", b, W->B[b].name, W->B[b].m);
    printf("joints %d\n", W->nj);
    for (int j = 0; j < W->nj; j++) {
        mb_joint* J = &W->Jt[j]; if (J->box) rows_b += J->m; else rows_f += J->m;
        printf("  J%-3d %-14s %-11s rows %d %s bodies", j, J->name, TN[J->type], J->m, J->box ? "box " : "fact");
        for (int k = 0; k < J->nb; k++) printf(" %s", J->b[k] >= 0 ? W->B[J->b[k]].name : "ground"); printf("\n");
    }
    printf("summary: bodies %d (%d dof), factorised rows %d, box rows %d, nodes %d, N %d, L-block doubles %d, nbox(last) %d, elems %d, couplings %d\n",
           W->nb, 6 * W->nb, rows_f, rows_b, W->n, W->N, W->nblk_d, W->nbox, W->ne, W->ncpl);
    return 0;
}
