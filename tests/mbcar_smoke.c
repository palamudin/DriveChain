/* quick static/rolling smoke of the multibody vehicle vs the reduced core */
#include "drivechain.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
int main(int argc, char** argv) {
    const char* ini = argc > 1 ? argv[1] : "cars/mx5_na.ini";
    double T = argc > 2 ? atof(argv[2]) : 3.0, v0 = argc > 3 ? atof(argv[3]) : 0;
    for (int s = 0; s < 2; s++) {
        dc_car_params p; char err[256];
        dc_default_params(&p);
        if (!dc_load_params(ini, &p, err, sizeof err)) { printf("load: %s\n", err); return 1; }
        p.solver = s;
        dc_car* c = dc_create(&p);
        double z[3] = {0, 0, 0}; dc_reset(c, z, 0, v0);
        dc_input in; memset(&in, 0, sizeof in); in.gear = v0 > 0 ? 3 : 0; in.clutch = 0;
        dc_set_input(c, &in);
        dc_telemetry t; int n = (int)(T / 0.01);
        for (int k = 0; k < n; k++) {
            dc_step(c, 0.01); dc_get_telemetry(c, &t);
            if (k % (n / 6 > 0 ? n / 6 : 1) == 0 || k == n - 1)
                printf("[%s] t=%.2f v=%.3f vy=%.4f r=%.4f fz=%.0f %.0f %.0f %.0f trav=%.4f %.4f %.4f %.4f sw=%.4f E=%.1f diss=%.1f rf=%.2e rt=%.2e\n",
                       dc_active_solver(c) ? "MB " : "OLD", t.time, t.vx, t.vy, t.yaw_rate, t.w[0].fz, t.w[1].fz, t.w[2].fz, t.w[3].fz,
                       t.w[0].travel, t.w[1].travel, t.w[2].travel, t.w[3].travel, t.sw_angle, t.kinetic + t.potential, t.dissipated,
                       sqrt(t.resid_force[0]*t.resid_force[0]+t.resid_force[1]*t.resid_force[1]+t.resid_force[2]*t.resid_force[2]),
                       sqrt(t.resid_torque[0]*t.resid_torque[0]+t.resid_torque[1]*t.resid_torque[1]+t.resid_torque[2]*t.resid_torque[2]));
            if (t.vx != t.vx) { printf("NaN\n"); break; }
        }
        printf("  mass*g=%.1f sumfz=%.1f\n", p.mass * p.gravity, t.w[0].fz + t.w[1].fz + t.w[2].fz + t.w[3].fz);
        dc_destroy(c);
    }
    return 0;
}
