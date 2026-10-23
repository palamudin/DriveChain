/* gen_cars.c - writes the public example car files from code defaults:
 *   cars/mx5_na.ini      RWD double-wishbone roadster (= dc_default_params, public-spec MX-5 NA-like numbers)
 *   cars/hatch_fwd.ini   FWD MacPherson-strut front / multilink rear hatchback (generic, public-spec-like) */
#include "drivechain.h"
#include <stdio.h>
#include <string.h>
static void s3(double* d, double x, double y, double z) { d[0] = x; d[1] = y; d[2] = z; }
int main(void) {
    dc_car_params p; dc_default_params(&p);
    if (!dc_save_params("cars/mx5_na.ini", &p)) return 1;
    /* FWD hatch */
    p.mass = 1250; s3(p.sprung_inertia, 420, 1650, 1800); p.drive = DC_DRIVE_FWD;
    const double xf = 1.00, xr = -1.58, hz = -0.25;
    dc_axle_params* F = &p.axle[0]; dc_axle_params* R = &p.axle[1];
    F->driven = 1; R->driven = 0;
    F->type = DC_SUSP_STRUT; F->nlinks = 3; F->lower_arm_a = 0; F->lower_arm_b = 1; F->rack_link = 2;
    s3(F->hub, xf, 0.745, hz);
    s3(F->link_c[0], xf + 0.000, 0.330, -0.380); s3(F->link_u[0], xf + 0.010, 0.690, -0.390);   /* lower arm front pivot */
    s3(F->link_c[1], xf - 0.330, 0.360, -0.370); s3(F->link_u[1], xf + 0.010, 0.690, -0.390);   /* lower arm rear pivot  */
    s3(F->link_c[2], xf - 0.150, 0.330, -0.250); s3(F->link_u[2], xf - 0.135, 0.680, -0.240);   /* tie rod (rack behind axle) */
    s3(F->strut_top, xf - 0.045, 0.560, 0.380); s3(F->strut_low, xf - 0.010, 0.640, -0.150);
    s3(F->spring_lo, xf - 0.015, 0.630, -0.100); s3(F->spring_hi, xf - 0.045, 0.560, 0.380); F->spring_on_upright = 1;
    F->static_camber_deg = -0.6; F->wheel_rate = 24000; F->arb = 14000; F->brake_torque = 1300; F->handbrake_torque = 0;
    R->type = DC_SUSP_MULTILINK; s3(R->hub, xr, 0.735, hz);
    double dz = hz - (-0.16);
    for (int l = 0; l < 5; l++) { R->link_c[l][0] += xr + 1.178; R->link_u[l][0] += xr + 1.178; R->link_c[l][2] += dz; R->link_u[l][2] += dz;
                                  R->link_c[l][1] += 0.02; R->link_u[l][1] += 0.02; }
    R->spring_lo[0] += xr + 1.178; R->spring_hi[0] += xr + 1.178; R->spring_lo[2] += dz; R->spring_hi[2] += dz;
    R->wheel_rate = 21000; R->arb = 6000; R->brake_torque = 500; R->handbrake_torque = 700; R->static_camber_deg = -1.4;
    p.steer_ratio = 14.5; p.steer_lock_deg = 540; p.final_drive = 3.9;
    for (int a = 0; a < 2; a++) { p.axle[a].tyre.radius = 0.316; p.axle[a].tyre.width = 0.205; p.axle[a].tyre.fz0 = 2600; p.axle[a].unsprung_mass = 38; }
    double bp[8][3] = {{2.0, 0.65, -0.40}, {2.0, -0.65, -0.40}, {-2.1, 0.65, -0.35}, {-2.1, -0.65, -0.35},
                       {0.3, 0.80, -0.42}, {0.3, -0.80, -0.42}, {-0.9, 0.80, -0.42}, {-0.9, -0.80, -0.42}};
    for (int i = 0; i < 8; i++) s3(p.bodypt[i], bp[i][0], bp[i][1], bp[i][2]);
    if (!dc_save_params("cars/hatch_fwd.ini", &p)) return 1;
    puts("wrote cars/mx5_na.ini cars/hatch_fwd.ini");
    return 0;
}
