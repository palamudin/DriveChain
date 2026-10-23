#include "drivechain.h"
#include <stdio.h>
#include <string.h>
int main(int argc, char** argv) {
    dc_car_params p; dc_default_params(&p);
    if (argc > 1) { char e[256]; if (!dc_load_params(argv[1], &p, e, 256)) { printf("load: %s\n", e); return 1; } }
    dc_car* c = dc_create(&p);
    dc_input in; memset(&in, 0, sizeof in); in.gear = 0; in.steer_mode = DC_STEER_ANGLE; in.hands_k = 20; in.hands_d = 1; in.hands_tmax = 15;
    dc_set_input(c, &in);
    dc_telemetry t;
    for (int k = 0; k <= 400; k++) {
        dc_step(c, 1.0 / 100);
        dc_get_telemetry(c, &t);
        if (k % 50 == 0) {
            dc_xform x[11]; dc_get_parts(c, x, 11);
            printf("t=%.2f z=%.4f fz=%.0f %.0f %.0f %.0f s=%.4f %.4f %.4f %.4f KE=%.3f\n", t.time, x[0].pos[2],
                t.w[0].fz, t.w[1].fz, t.w[2].fz, t.w[3].fz, t.w[0].travel, t.w[1].travel, t.w[2].travel, t.w[3].travel, t.kinetic);
        }
    }
    for (int i = 0; i < 4; i++) printf("w%d camber %.2f toe %.3f caster %.2f kpi %.2f scrub %.1fmm trail %.1fmm tierod %.0f\n", i,
        t.w[i].camber * 57.3, t.w[i].toe * 57.3, t.w[i].caster * 57.3, t.w[i].kpi * 57.3, t.w[i].scrub * 1000, t.w[i].trail * 1000, t.w[i].tierod_force);
    dc_destroy(c);
    return 0;
}
