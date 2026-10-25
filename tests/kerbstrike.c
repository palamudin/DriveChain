/* kerbstrike.c - straight-line strike of one corner on a raised kerb/step (and drops off ledges).
   usage: kerbstrike <car.ini> <solver 0|1> <mode> [args]
     step  <height m> <face ramp m> <approach deg> <speed km/h> [out.csv]   kerb along +x at y=0, raised for y>0
     drop  <height m> <speed km/h> [out.csv]                               ledge at x=0: ground at -height for x>0
   Prints peak per-wheel normal load (tyre + rim), peak rim force, peak deflection and the strike timeline. */
#include "drivechain.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
static double Hk = 0.15, Wf = 0.0, Hd = 1.0; static int mode = 0;
static double road(void* u, double x, double y, double* nx, double* ny, double* nz, double* grip) {
    (void)u; double h = 0;
    if (mode == 0) { if (Wf <= 0) h = y > 0 ? Hk : 0; else h = y <= 0 ? 0 : (y >= Wf ? Hk : Hk * y / Wf); }
    else h = x > 0 ? -Hd : 0;
    if (nx) { *nx = 0; *ny = 0; *nz = 1; } if (grip) *grip = 1; return h;
}
int main(int argc, char** argv) {
    if (argc < 5) { fprintf(stderr, "usage: see source\n"); return 2; }
    dc_car_params p; dc_default_params(&p); char err[256];
    if (!dc_load_params(argv[1], &p, err, sizeof err)) { fprintf(stderr, "%s\n", err); return 1; }
    p.solver = atoi(argv[2]);
    double ang = 0, v = 0; const char* out = NULL;
    if (!strcmp(argv[3], "step")) { mode = 0; Hk = atof(argv[4]); Wf = atof(argv[5]); ang = atof(argv[6]) * M_PI / 180; v = atof(argv[7]) / 3.6; out = argc > 8 ? argv[8] : NULL; }
    else { mode = 1; Hd = atof(argv[4]); v = atof(argv[5]) / 3.6; out = argc > 6 ? argv[6] : NULL; }
    dc_car* c = dc_create(&p);
    dc_set_road(c, road, NULL); dc_set_road_flags(c, DC_ROAD_HEIGHT_ONLY);
    double pos[3], yaw;
    if (mode == 0) { double d0 = 6.0; pos[0] = -d0 * cos(ang) / fmax(sin(ang), 1e-3) * 0; pos[1] = -1.2 - fmin(d0 * sin(ang), 6.0); pos[2] = 0; yaw = ang; }
    else { pos[0] = -6; pos[1] = 0; pos[2] = 0; yaw = 0; }
    dc_reset(c, pos, yaw, v);
    dc_input in; memset(&in, 0, sizeof in); in.steer_mode = DC_STEER_ANGLE; in.hands_k = 20; in.hands_d = 1; in.hands_tmax = 15; in.gear = 0; in.clutch = 1;
    dc_set_input(c, &in);
    FILE* f = out ? fopen(out, "w") : NULL;
    if (f) fprintf(f, "t,x,y,z,vx,fz0,fz1,fz2,fz3,rim0,rim1,rim2,rim3,defl0,defl1,defl2,defl3,trav0,trav1,trav2,trav3,bs0,bs1,bs2,bs3,ke,resid,pe,diss\n");
    double pfw[4][3] = {{0}}, pk[4] = {0}, pr[4] = {0}, pdf[4] = {0}, pbs[4] = {0}, tpk[4] = {0}; dc_telemetry t; int bad = 0; double tend = mode == 0 ? 3.0 : 4.0;
    dc_substep(c); dc_get_telemetry(c, &t); double fz_static = 0; for (int i = 0; i < 4; i++) fz_static = fmax(fz_static, t.w[i].fz);
    for (int k = 0; k < (int)(tend * 2000); k++) {
        dc_substep(c); dc_get_telemetry(c, &t);
        if (!isfinite(t.kinetic) || t.kinetic > 1e7) { bad = 1; printf("BLOWUP at t=%.4f ke=%g\n", t.time, t.kinetic); break; }
        for (int i = 0; i < 4; i++) {
            if (t.w[i].fz > pk[i]) { pk[i] = t.w[i].fz; tpk[i] = t.time; for (int k2 = 0; k2 < 3; k2++) pfw[i][k2] = t.w[i].force_world[k2]; }
            pr[i] = fmax(pr[i], t.w[i].rim_force); pdf[i] = fmax(pdf[i], t.w[i].deflection); pbs[i] = fmax(pbs[i], fabs(t.w[i].bumpstop_force));
        }
        if (f) { dc_xform xa[DC_PART_COUNT]; dc_get_parts(c, xa, DC_PART_COUNT); const dc_xform xf = xa[DC_PART_CHASSIS]; double rr = sqrt(t.resid_force[0]*t.resid_force[0]+t.resid_force[1]*t.resid_force[1]+t.resid_force[2]*t.resid_force[2]);
            fprintf(f, "%.5f,%.4f,%.4f,%.4f,%.3f", t.time, xf.pos[0], xf.pos[1], xf.pos[2], t.vx);
            for (int i = 0; i < 4; i++) fprintf(f, ",%.1f", t.w[i].fz); for (int i = 0; i < 4; i++) fprintf(f, ",%.1f", t.w[i].rim_force);
            for (int i = 0; i < 4; i++) fprintf(f, ",%.5f", t.w[i].deflection); for (int i = 0; i < 4; i++) fprintf(f, ",%.5f", t.w[i].travel);
            for (int i = 0; i < 4; i++) fprintf(f, ",%.1f", t.w[i].bumpstop_force); fprintf(f, ",%.1f,%.3g,%.1f,%.1f\n", t.kinetic, rr, t.potential, t.dissipated); }
    }
    if (f) fclose(f);
    printf("solver %d  static max Fz %.0f N\n", dc_active_solver(c), fz_static);
    for (int i = 0; i < 4; i++) printf("  w%d  peak N %8.0f N (%.1fx) at %.3f s  [world x %6.0f y %6.0f z %6.0f]  rim %8.0f N  defl %5.1f mm  bumpstop %7.0f N\n", i, pk[i], pk[i] / fz_static, tpk[i], pfw[i][0], pfw[i][1], pfw[i][2], pr[i], pdf[i] * 1000, pbs[i]);
    printf("%s\n", bad ? "RESULT BLOWUP" : "RESULT OK");
    dc_destroy(c); return bad;
}
