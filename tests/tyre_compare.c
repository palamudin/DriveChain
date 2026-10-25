#define _POSIX_C_SOURCE 200809L
/* tyre_compare.c - head-to-head rig: envelope/brush tyre (tyre_model 0) vs TARMAC tyre (tyre_model 1).
 * Usage: tyre_compare car.ini outdir
 * Steady state on a flat road, single wheel (front axle tyre): Fy, Mz, trail vs slip angle at 3 loads and camber,
 * Fx vs slip ratio, relaxation length from a 2 deg step, footprint vs load, cost per contact evaluation. */
#include "dc_internal.h"
#include "dc_tyre.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#define D2R (M_PI / 180.0)
#define R2D (180.0 / M_PI)

static double flat(void* u, double x, double y, double* nx, double* ny, double* nz, double* grip) {
    (void)u; (void)x; (void)y; *nx = 0; *ny = 0; *nz = 1; *grip = 1; return 0;
}
static double kz_est;
static void rig(dc_car* c, double Fz, double alpha, double kappa, double gamma, double V, dc_tc* out, int iters) {
    double h = 1.0 / c->P.substep_hz; const dc_tyre_params* T = &c->P.axle[0].tyre;
    dc_tyre_state* S = &c->ty[0];
    S->ex = S->ey = S->phi = 0;
    v3 a = V3(0, cos(gamma), -sin(gamma));
    v3 vc = V3(V * cos(alpha), -V * sin(alpha), 0);
    double z = T->radius - Fz / kz_est;
    dc_tc t;
    for (int n = 0; n < iters; n++) {
        v3 cw = V3(0, 0, z);
        memset(&t, 0, sizeof t); S->omega = 0; dc_tyre_phaseA(c, 0, cw, a, vc, V3(0, 0, 0), h, &t);
        double Om = vdot(vc, t.xc) * (1 + kappa) / t.Re; S->omega = Om;
        memset(&t, 0, sizeof t); dc_tyre_phaseA(c, 0, cw, a, vc, V3(0, 0, 0), h, &t);
        dc_tyre_phaseB(c, 0, cw, a, Om, h, &t, 1);
        z += (t.Ntot - Fz) / kz_est * 0.5;
    }
    *out = t;
}
static dc_car* mkcar(const dc_car_params* P0, int m) {
    dc_car_params P = *P0; int model = m > 0;
    P.axle[0].tyre.model = model; P.axle[1].tyre.model = model;
    if (m == 2) { P.axle[0].tyre.tm_shape = 1; P.axle[1].tyre.tm_shape = 1; }
    dc_car* c = dc_create(&P); dc_set_road(c, flat, NULL); return c;
}
int main(int argc, char** argv) {
    if (argc < 3) { fprintf(stderr, "usage: tyre_compare car.ini outdir\n"); return 2; }
    dc_car_params P; dc_default_params(&P); char err[256];
    if (!dc_load_params(argv[1], &P, err, sizeof err)) { fprintf(stderr, "%s\n", err); return 1; }
    const char* od = argv[2]; char path[512];
    const dc_tyre_params* T = &P.axle[0].tyre;
    double fref = P.mass * P.gravity / 4;
    double loads[4] = {0.5 * fref, fref, 1.5 * fref, 2 * fref};
    const char* nm[3] = {"envelope", "tarmac", "tarmac_brush"};
    snprintf(path, sizeof path, "%s/summary.txt", od); FILE* S = fopen(path, "w");
    for (int m = 0; m < 3; m++) {
        dc_car* c = mkcar(&P, m);
        kz_est = m ? 0 : T->kz_ref;
        if (m) { double A, hl, d; dc_tyre_tm_flat(c, 0, fref, &A, &hl, &d); kz_est = fref / d; }
        fprintf(S, "== %s  (fref %.0f N, front tyre r %.3f w %.3f p %.0f kPa)\n", nm[m], fref, T->radius, T->width, T->pressure / 1000);
        if (m) {
            const dc_tm_tab* B = &c->tm[0];
            fprintf(S, "  TARMAC params: Kz(Rhyne) %.0f N/m, kx %.0f ky %.0f, sigma_x %.3f sigma_y %.3f m, mu0 %.3f ls %.3f, H %.3f tread %.3f, patch0 %.3e, cp %.3e\n",
                    kz_est, B->kx, B->ky, B->sx0, B->sy0, B->mu0, B->ls, B->H, B->tread, B->patch0, B->cp);
            for (int l = 0; l < 4; l++) { double A, hl, d; dc_tyre_tm_flat(c, 0, loads[l], &A, &hl, &d);
                fprintf(S, "  footprint @%.0f N: area %.1f cm2, half length %.1f mm (2a %.0f mm), deflection %.1f mm, mean pressure %.0f kPa\n", loads[l], A * 1e4, hl * 1e3, 2 * hl * 1e3, d * 1e3, loads[l] / A / 1000); }
        }
        snprintf(path, sizeof path, "%s/lat_%s.csv", od, nm[m]); FILE* f = fopen(path, "w");
        fprintf(f, "Fz,gamma_deg,alpha_deg,Fy,Mz,trail_mm,mu,sig_y,a_eff\n");
        for (int g = 0; g < 2; g++) for (int l = 0; l < 4; l++) {
            if (g && l != 1) continue;
            double gam = g ? -3 * D2R : 0, pk = 0, pka = 0, mzpk = 0, mzpka = 0, Cy = 0, tp0 = 0, Fy0 = 0;
            for (int k = 0; k <= 80; k++) {
                double al = 0.25 * k * D2R; dc_tc t; rig(c, loads[l], al, 0, gam, 15, &t, 1500);
                double Mz = t.Mz_brush + t.Mt;
                fprintf(f, "%.0f,%.1f,%.3f,%.2f,%.3f,%.2f,%.4f,%.4f,%.4f\n", loads[l], gam * R2D, al * R2D, t.Fy, Mz, t.tp * 1000, t.Fy / loads[l], t.sig_y, t.a_eff);
                if (k == 0) Fy0 = t.Fy;
                if (k == 2) { Cy = (t.Fy - Fy0) / (0.5 * D2R); tp0 = t.tp; }
                if (t.Fy > pk) { pk = t.Fy; pka = al * R2D; }
                if (fabs(Mz) > mzpk) { mzpk = fabs(Mz); mzpka = al * R2D; }
            }
            fprintf(S, "  Fz %5.0f g %+.0f: C_alpha %.0f N/rad (%.1f /rad norm), peak mu %.3f at %.2f deg, |Mz| peak %.1f N m at %.2f deg, trail(0.5deg) %.1f mm, Fy(alpha=0) %.1f N\n",
                    loads[l], gam * R2D, Cy, Cy / loads[l], pk / loads[l], pka, mzpk, mzpka, tp0 * 1000, Fy0);
        }
        fclose(f);
        snprintf(path, sizeof path, "%s/long_%s.csv", od, nm[m]); f = fopen(path, "w");
        fprintf(f, "Fz,kappa,Fx\n"); double pk = 0, pkk = 0, Cx = 0;
        for (int k = -60; k <= 60; k++) { double kap = 0.01 * k; dc_tc t; rig(c, fref, 0, kap, 0, 15, &t, 1500);
            fprintf(f, "%.0f,%.3f,%.2f\n", fref, kap, t.Fx); if (t.Fx > pk) { pk = t.Fx; pkk = kap; } if (k == 1) Cx = t.Fx / 0.01; }
        fclose(f);
        fprintf(S, "  long @fref: C_kappa %.0f N (%.1f norm), peak mu_x %.3f at kappa %.2f\n", Cx, Cx / fref, pk / fref, pkk);
        /* relaxation: 2 deg step at 15 m/s */
        {
            double h = 1.0 / P.substep_hz, V = 15, al = 2 * D2R; dc_tc t; rig(c, fref, al, 0, 0, V, &t, 1500); double Fyss = t.Fy;
            dc_tyre_state* St = &c->ty[0]; v3 a = V3(0, 1, 0), vc = V3(V * cos(al), -V * sin(al), 0);
            rig(c, fref, 0, 0, 0, V, &t, 1500);   /* settle height at zero slip */
            double z = 0; { /* recover z from the settled state: re-run with same loop storing z */ }
            /* find z by bisection on N(z) at zero slip */
            double lo = T->radius - 0.08, hi = T->radius;
            for (int it = 0; it < 60; it++) { double zm = 0.5 * (lo + hi); St->ex = St->ey = St->phi = 0; St->omega = V / T->radius; memset(&t, 0, sizeof t);
                dc_tyre_phaseA(c, 0, V3(0, 0, zm), a, V3(V, 0, 0), V3(0, 0, 0), h, &t); if (t.Ntot > fref) lo = zm; else hi = zm; }
            z = 0.5 * (lo + hi);
            St->ex = St->ey = St->phi = 0; double d63 = -1;
            snprintf(path, sizeof path, "%s/relax_%s.csv", od, nm[m]); f = fopen(path, "w"); fprintf(f, "dist_m,Fy,Fy_ss\n");
            for (int n = 0; n < 2000; n++) {
                memset(&t, 0, sizeof t); dc_tyre_phaseA(c, 0, V3(0, 0, z), a, vc, V3(0, 0, 0), h, &t);
                double Om = vdot(vc, t.xc) / t.Re; St->omega = Om; dc_tyre_phaseB(c, 0, V3(0, 0, z), a, Om, h, &t, 1);
                double d = V * h * (n + 1); if (n % 4 == 0) fprintf(f, "%.4f,%.2f,%.2f\n", d, t.Fy, Fyss);
                if (d63 < 0 && t.Fy > 0.632 * Fyss) d63 = d;
            }
            fclose(f);
            fprintf(S, "  relaxation (63%% distance, 2 deg step, 15 m/s, fref): %.3f m\n", d63);
            /* cost: phaseA+phaseB at steady contact */
            struct timespec t0, t1; int NN = 200000; clock_gettime(CLOCK_MONOTONIC, &t0);
            for (int n = 0; n < NN; n++) { memset(&t, 0, sizeof t); dc_tyre_phaseA(c, 0, V3(0, 0, z), a, vc, V3(0, 0, 0), h, &t); dc_tyre_phaseB(c, 0, V3(0, 0, z), a, St->omega, h, &t, 1); }
            clock_gettime(CLOCK_MONOTONIC, &t1);
            fprintf(S, "  cost: %.2f us per wheel evaluation (phaseA + phaseB)\n", ((t1.tv_sec - t0.tv_sec) * 1e9 + (t1.tv_nsec - t0.tv_nsec)) / NN / 1000.0);
        }
        dc_destroy(c);
    }
    fclose(S);
    snprintf(path, sizeof path, "%s/summary.txt", od); FILE* r = fopen(path, "r"); char ln[512]; while (fgets(ln, sizeof ln, r)) fputs(ln, stdout); fclose(r);
    return 0;
}
