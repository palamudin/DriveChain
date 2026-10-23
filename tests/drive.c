#include "drivechain.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
int main(int argc, char** argv) {
    dc_car_params p; dc_default_params(&p);
    dc_car* c = dc_create(&p);
    dc_input in; memset(&in, 0, sizeof in); in.steer_mode = DC_STEER_ANGLE; in.hands_k = 20; in.hands_d = 1; in.hands_tmax = 15;
    dc_telemetry t;
    clock_t t0 = clock();
    for (int k = 0; k <= 2000; k++) {
        double tm = k * 0.01;
        in.gear = tm < 0.5 ? 0 : (tm < 4 ? 1 : (tm < 8 ? 2 : 3));
        in.clutch = tm < 0.5 ? 1 : (tm < 1.5 ? 1.5 - tm : 0);
        if ((tm > 3.9 && tm < 4.1) || (tm > 7.9 && tm < 8.1)) in.clutch = 1;
        in.throttle = (tm > 0.5 && tm < 12) ? 0.6 : 0;
        in.brake = tm > 16 ? 0.6 : 0;
        in.steer_target = (tm > 12 && tm < 16) ? 1.5 : 0;
        dc_set_input(c, &in);
        dc_step(c, 0.01);
        dc_get_telemetry(c, &t);
        if (k % 50 == 0)
            printf("t=%5.2f v=%6.2f vy=%5.2f r=%5.2f ay=%5.2f rpm=%5.0f g=%d fx=%6.0f %6.0f fy=%6.0f %6.0f %6.0f %6.0f sw=%5.2f col=%5.2f sl=%.3f\n", t.time, t.vx, t.vy, t.yaw_rate, t.ay,
                t.engine_rpm, t.gear, t.w[2].fx, t.w[3].fx, t.w[0].fy, t.w[1].fy, t.w[2].fy, t.w[3].fy, t.sw_angle, t.column_torque, t.w[2].slip_ratio);
    }
    printf("cpu %.2f us/substep\n", (double)(clock() - t0) / CLOCKS_PER_SEC * 1e6 / (2000 * 20));
    return 0;
}
