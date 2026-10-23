/* dc_params.c - defaults (a public-spec MX-5 NA-like roadster) and the INI loader.
 * Every physical number of the vehicle lives in dc_car_params and can be overridden from a text file. */
#include "dc_internal.h"
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <ctype.h>

static void set3(double* d, double x, double y, double z) { d[0] = x; d[1] = y; d[2] = z; }

static void default_tyre(dc_tyre_params* t) {
    t->radius = 0.289; t->rim_radius = 0.190; t->width = 0.185; t->spin_inertia = 0.70;
    t->pressure_ref = 200e3; t->kz_ref = 200e3; t->kz_per_pa = 0.90; t->cz = 400; t->pressure = 200e3;
    t->k_rim = 3.0e6; t->c_rim = 3000; t->rim_pinch = 0.012;
    t->mu_x = 1.12; t->mu_y = 1.15; t->fz0 = 1800; t->ls_exp_x = 0.80; t->ls_exp_y = 0.75;
    t->falloff_level = 0.85; t->falloff_speed = 1.0;
    t->patch_factor = 0.62; t->peak_slip_deg = 8.5; t->cx_mult = 1.05;
    t->carcass_ky = 150e3; t->carcass_kx = 300e3; t->twist_k = 2500; t->twist_relax = 0.06;
    t->camber_gain = 0.12; t->dcamber0 = 1.0; t->dcamber1 = -12.0; t->rr_factor = 0.25;
    t->n_rays = 13; t->n_rows = 3; t->ray_span_deg = 50;
}

void dc_default_params(dc_car_params* p) {
    memset(p, 0, sizeof *p);
    p->mass = 1050; set3(p->sprung_inertia, 300, 1000, 1100);
    p->gravity = 9.81; p->substep_hz = 2000; p->road_resolve = 0.025;
    const double xf = 1.087, xr = -1.178, hz = -0.16;
    for (int a = 0; a < 2; a++) {
        dc_axle_params* A = &p->axle[a];
        A->type = DC_SUSP_DWB; A->nlinks = 5; A->rack_link = 4; A->lower_arm_a = 2; A->lower_arm_b = 3;
        default_tyre(&A->tyre);
        A->unsprung_mass = 30; set3(A->upright_inertia, 0.6, 0.5, 0.6);
        A->damp_fast_bump = 0; A->droop_rate = 200e3;
        A->bumpstop_rate = 40e3;
    }
    dc_axle_params* F = &p->axle[0];
    F->steered = 1; F->driven = 0;
    set3(F->hub, xf, 0.705, hz);
    set3(F->link_c[0], xf + 0.100, 0.310, -0.080); set3(F->link_u[0], xf - 0.013, 0.575, -0.040); /* upper front */
    set3(F->link_c[1], xf - 0.120, 0.310, -0.090); set3(F->link_u[1], xf - 0.013, 0.575, -0.040); /* upper rear  */
    set3(F->link_c[2], xf + 0.020, 0.290, -0.255); set3(F->link_u[2], xf + 0.004, 0.615, -0.252); /* lower front */
    set3(F->link_c[3], xf - 0.320, 0.290, -0.255); set3(F->link_u[3], xf + 0.004, 0.615, -0.252); /* lower rear  */
    set3(F->link_c[4], xf + 0.120, 0.270, -0.240); set3(F->link_u[4], xf + 0.105, 0.630, -0.232); /* tie rod     */
    set3(F->spring_lo, xf - 0.020, 0.530, -0.240); set3(F->spring_hi, xf - 0.030, 0.500, 0.200);
    F->static_camber_deg = -0.5; F->static_toe_deg = 0.0;
    F->wheel_rate = 19000; F->damp_bump = 2200; F->damp_fast_bump = 1300; F->damp_bump_thr = 0.03;
    F->damp_reb = 6000; F->damp_fast_reb = 1300; F->damp_reb_thr = 0.06;
    F->bumpstop_up = 0.060; F->bumpstop_dn = 0.100; F->arb = 9000;
    F->brake_torque = 950; F->handbrake_torque = 0;

    dc_axle_params* R = &p->axle[1];
    R->steered = 0; R->driven = 1;
    set3(R->hub, xr, 0.715, hz);
    set3(R->link_c[0], xr + 0.120, 0.350, -0.047); set3(R->link_u[0], xr + 0.037, 0.563, -0.040);
    set3(R->link_c[1], xr - 0.050, 0.350, -0.047); set3(R->link_u[1], xr + 0.037, 0.563, -0.040);
    set3(R->link_c[2], xr + 0.160, 0.215, -0.228); set3(R->link_u[2], xr + 0.000, 0.578, -0.259);
    set3(R->link_c[3], xr - 0.160, 0.215, -0.228); set3(R->link_u[3], xr + 0.000, 0.578, -0.259);
    set3(R->link_c[4], xr - 0.150, 0.225, -0.235); set3(R->link_u[4], xr - 0.150, 0.578, -0.250); /* toe link */
    set3(R->spring_lo, xr, 0.500, -0.250); set3(R->spring_hi, xr - 0.02, 0.470, 0.170);
    R->static_camber_deg = -1.2; R->static_toe_deg = 0.1;
    R->wheel_rate = 17500; R->damp_bump = 1700; R->damp_fast_bump = 1150; R->damp_bump_thr = 0.03;
    R->damp_reb = 5500; R->damp_fast_reb = 1300; R->damp_reb_thr = 0.06;
    R->bumpstop_up = 0.070; R->bumpstop_dn = 0.150; R->arb = 4000;
    R->brake_torque = 450; R->handbrake_torque = 400;

    p->steer_ratio = 15.0; p->steer_lock_deg = 504;
    p->rack_mass = 3.0; p->sw_inertia = 0.035; p->column_inertia = 0.010;
    p->torsion_k = 1500; p->torsion_c = 0.15; p->rack_friction = 60; p->column_friction = 0.15;
    p->rack_stop_k = 2e6; p->rack_stop_c = 3e3;
    p->assist = 1; p->assist_area = 8.8e-4; p->assist_pmax = 7e6; p->assist_tref = 2.5;
    p->assist_flow = 1.0e-4; p->assist_rpm_full = 1500; p->assist_damping = 1500; p->assist_torsion_k = 115;

    p->drive = DC_DRIVE_RWD; p->ngears = 5;
    double g[5] = {3.136, 1.888, 1.330, 1.000, 0.814}; for (int i = 0; i < 5; i++) p->gear[i] = g[i];
    p->gear_reverse = -3.758; p->final_drive = 4.3;
    p->engine_inertia = 0.12; p->gearbox_inertia = 0.02;
    double lr[] = {0, 1000, 2000, 3000, 4000, 5000, 5500, 6000, 6500, 7000, 7200, 7500, 8000};
    double lt[] = {60, 95, 112, 121, 127, 132, 135, 133, 127, 115, 110, 60, 0};
    p->nlut = 13; for (int i = 0; i < 13; i++) { p->lut_rpm[i] = lr[i]; p->lut_torque[i] = lt[i]; }
    p->coast_torque = 60; p->coast_rpm = 7200; p->idle_rpm = 850; p->limiter_rpm = 7200;
    p->clutch_torque = 260; p->diff_power = 0.30; p->diff_coast = 0.30; p->diff_preload = 20;
    p->awd_front_split = 0.4; p->center_lock = 0.2;
    p->cda = 0.63; p->cla_front = -0.05; p->cla_rear = -0.05; p->aero_front_x = 1.6; p->aero_rear_x = -1.7; p->aero_z = 0.1;
    p->nbodypts = 8;
    double bp[8][3] = {{1.9, 0.6, -0.30}, {1.9, -0.6, -0.30}, {-1.9, 0.6, -0.28}, {-1.9, -0.6, -0.28},
                       {0.5, 0.75, -0.32}, {0.5, -0.75, -0.32}, {-0.6, 0.75, -0.32}, {-0.6, -0.75, -0.32}};
    for (int i = 0; i < 8; i++) set3(p->bodypt[i], bp[i][0], bp[i][1], bp[i][2]);
    p->body_k = 2e5; p->body_c = 4e3; p->body_mu = 0.5;
}

/* ------------------------------------------------------------------ INI loader */
enum { T_D, T_I };
typedef struct { const char* key; int type; size_t off; int n; } fld;
#define CF(k, f, n) {k, T_D, offsetof(dc_car_params, f), n}
#define CI(k, f) {k, T_I, offsetof(dc_car_params, f), 1}
#define AF(k, f, n) {k, T_D, offsetof(dc_axle_params, f), n}
#define AI(k, f) {k, T_I, offsetof(dc_axle_params, f), 1}
#define TF(k, f) {k, T_D, offsetof(dc_axle_params, tyre) + offsetof(dc_tyre_params, f), 1}
#define TI(k, f) {k, T_I, offsetof(dc_axle_params, tyre) + offsetof(dc_tyre_params, f), 1}

static const fld car_f[] = {
    CF("mass", mass, 1), CF("sprung_inertia", sprung_inertia, 3), CF("gravity", gravity, 1), CF("substep_hz", substep_hz, 1), CF("road_resolve", road_resolve, 1),
    CF("steer_ratio", steer_ratio, 1), CF("steer_lock_deg", steer_lock_deg, 1), CF("rack_mass", rack_mass, 1),
    CF("sw_inertia", sw_inertia, 1), CF("column_inertia", column_inertia, 1), CF("torsion_k", torsion_k, 1),
    CF("torsion_c", torsion_c, 1), CF("rack_friction", rack_friction, 1), CF("column_friction", column_friction, 1),
    CF("rack_stop_k", rack_stop_k, 1), CF("rack_stop_c", rack_stop_c, 1),
    CI("assist", assist), CF("assist_area", assist_area, 1), CF("assist_pmax", assist_pmax, 1), CF("assist_tref", assist_tref, 1),
    CF("assist_flow", assist_flow, 1), CF("assist_rpm_full", assist_rpm_full, 1), CF("assist_damping", assist_damping, 1),
    CF("assist_torsion_k", assist_torsion_k, 1),
    CI("drive", drive), CI("ngears", ngears), CF("gears", gear, DC_MAX_GEARS), CF("gear_reverse", gear_reverse, 1),
    CF("final_drive", final_drive, 1), CF("engine_inertia", engine_inertia, 1), CF("gearbox_inertia", gearbox_inertia, 1),
    CF("coast_torque", coast_torque, 1), CF("coast_rpm", coast_rpm, 1), CF("idle_rpm", idle_rpm, 1), CF("limiter_rpm", limiter_rpm, 1),
    CF("clutch_torque", clutch_torque, 1), CF("diff_power", diff_power, 1), CF("diff_coast", diff_coast, 1),
    CF("diff_preload", diff_preload, 1), CF("awd_front_split", awd_front_split, 1), CF("center_lock", center_lock, 1),
    CF("cda", cda, 1), CF("cla_front", cla_front, 1), CF("cla_rear", cla_rear, 1), CF("aero_front_x", aero_front_x, 1),
    CF("aero_rear_x", aero_rear_x, 1), CF("aero_z", aero_z, 1),
    CF("body_k", body_k, 1), CF("body_c", body_c, 1), CF("body_mu", body_mu, 1),
    {NULL, 0, 0, 0}};
static const fld axle_f[] = {
    AI("type", type), AI("steered", steered), AI("driven", driven), AF("hub", hub, 3), AI("nlinks", nlinks),
    AI("rack_link", rack_link), AI("lower_arm_a", lower_arm_a), AI("lower_arm_b", lower_arm_b),
    AF("strut_top", strut_top, 3), AF("strut_low", strut_low, 3), AF("toe_rod_delta", toe_rod_delta, 1),
    AF("static_camber_deg", static_camber_deg, 1), AF("static_toe_deg", static_toe_deg, 1),
    AI("spring_on_upright", spring_on_upright), AF("spring_lo", spring_lo, 3), AF("spring_hi", spring_hi, 3),
    AF("wheel_rate", wheel_rate, 1), AF("spring_rate", spring_rate, 1), AF("preload_travel", preload_travel, 1),
    AF("damp_bump", damp_bump, 1), AF("damp_fast_bump", damp_fast_bump, 1), AF("damp_bump_thr", damp_bump_thr, 1),
    AF("damp_reb", damp_reb, 1), AF("damp_fast_reb", damp_fast_reb, 1), AF("damp_reb_thr", damp_reb_thr, 1),
    AF("bumpstop_up", bumpstop_up, 1), AF("bumpstop_dn", bumpstop_dn, 1), AF("bumpstop_rate", bumpstop_rate, 1),
    AF("droop_rate", droop_rate, 1), AF("arb", arb, 1), AF("unsprung_mass", unsprung_mass, 1),
    AF("upright_inertia", upright_inertia, 3), AF("brake_torque", brake_torque, 1), AF("handbrake_torque", handbrake_torque, 1),
    TF("tyre_radius", radius), TF("tyre_rim_radius", rim_radius), TF("tyre_width", width), TF("tyre_spin_inertia", spin_inertia),
    TF("tyre_kz_ref", kz_ref), TF("tyre_pressure_ref", pressure_ref), TF("tyre_kz_per_pa", kz_per_pa), TF("tyre_cz", cz),
    TF("tyre_pressure", pressure), TF("tyre_k_rim", k_rim), TF("tyre_c_rim", c_rim), TF("tyre_rim_pinch", rim_pinch),
    TF("tyre_mu_x", mu_x), TF("tyre_mu_y", mu_y), TF("tyre_fz0", fz0), TF("tyre_ls_exp_x", ls_exp_x), TF("tyre_ls_exp_y", ls_exp_y),
    TF("tyre_falloff_level", falloff_level), TF("tyre_falloff_speed", falloff_speed), TF("tyre_patch_factor", patch_factor),
    TF("tyre_peak_slip_deg", peak_slip_deg), TF("tyre_cx_mult", cx_mult), TF("tyre_carcass_ky", carcass_ky),
    TF("tyre_carcass_kx", carcass_kx), TF("tyre_twist_k", twist_k), TF("tyre_twist_relax", twist_relax),
    TF("tyre_camber_gain", camber_gain), TF("tyre_dcamber0", dcamber0), TF("tyre_dcamber1", dcamber1),
    TF("tyre_rr_factor", rr_factor), TI("tyre_n_rays", n_rays), TI("tyre_n_rows", n_rows), TF("tyre_ray_span_deg", ray_span_deg),
    {NULL, 0, 0, 0}};

static char* trim(char* s) {
    while (isspace((unsigned char)*s)) s++;
    char* e = s + strlen(s); while (e > s && isspace((unsigned char)e[-1])) *--e = 0;
    return s;
}
static int parse_vals(char* v, double* out, int max) {
    int n = 0; char* p = v;
    while (*p && n < max) {
        char* end; double d = strtod(p, &end);
        if (end == p) break;
        out[n++] = d; p = end;
        while (*p == ',' || isspace((unsigned char)*p)) p++;
    }
    return n;
}
static int apply(const fld* tab, char* base, const char* key, char* val) {
    for (const fld* f = tab; f->key; f++) if (!strcmp(f->key, key)) {
        double tmp[DC_MAX_LUT]; int n = parse_vals(val, tmp, f->n);
        if (n <= 0) return -1;
        if (f->type == T_I) *(int*)(base + f->off) = (int)tmp[0];
        else for (int i = 0; i < n; i++) ((double*)(base + f->off))[i] = tmp[i];
        return 1;
    }
    return 0;
}

int dc_load_params(const char* path, dc_car_params* p, char* err, int errlen) {
    FILE* f = fopen(path, "r");
    if (!f) { if (err) snprintf(err, errlen, "cannot open %s", path); return 0; }
    char line[4096], sect[64] = "car"; int ln = 0, ok = 1;
    while (fgets(line, sizeof line, f)) {
        ln++;
        char* c = strpbrk(line, ";#"); if (c) *c = 0;
        char* s = trim(line); if (!*s) continue;
        if (*s == '[') { char* e = strchr(s, ']'); if (e) *e = 0; snprintf(sect, sizeof sect, "%s", trim(s + 1));
            if (!strcmp(sect, "engine_lut")) p->nlut = 0;
            if (!strcmp(sect, "body")) p->nbodypts = 0;
            continue; }
        char* eq = strchr(s, '='); if (!eq) continue;
        *eq = 0; char* key = trim(s); char* val = trim(eq + 1);
        int r = 0;
        if (!strcmp(sect, "front") || !strcmp(sect, "rear")) {
            dc_axle_params* A = &p->axle[sect[0] == 'r'];
            /* link arrays: link0_c, link0_u ... */
            if (!strncmp(key, "link", 4) && isdigit((unsigned char)key[4]) && key[5] == '_') {
                int li = key[4] - '0'; double tmp[3];
                if (li < DC_MAX_LINKS && parse_vals(val, tmp, 3) == 3) {
                    memcpy(key[6] == 'c' ? A->link_c[li] : A->link_u[li], tmp, sizeof tmp); r = 1;
                } else r = -1;
            } else r = apply(axle_f, (char*)A, key, val);
        } else if (!strcmp(sect, "engine_lut")) {
            double tmp[2];
            if (parse_vals(val, tmp, 1) == 1 && p->nlut < DC_MAX_LUT) {
                p->lut_rpm[p->nlut] = atof(key); p->lut_torque[p->nlut] = tmp[0]; p->nlut++; r = 1;
            }
        } else if (!strcmp(sect, "body")) {
            double tmp[3];
            if (!strncmp(key, "point", 5) && parse_vals(val, tmp, 3) == 3 && p->nbodypts < DC_MAX_BODYPTS) {
                memcpy(p->bodypt[p->nbodypts++], tmp, sizeof tmp); r = 1;
            } else r = apply(car_f, (char*)p, key, val);
        } else r = apply(car_f, (char*)p, key, val);
        if (r <= 0) { if (err) snprintf(err, errlen, "%s:%d unknown/invalid key '%s' in [%s]", path, ln, key, sect); ok = 0; }
    }
    fclose(f);
    return ok;
}
