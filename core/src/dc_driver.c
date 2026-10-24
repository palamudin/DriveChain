/* dc_driver.c - optional driver model ("interface plugin"): finite-strength arms with delayed voluntary command,
   scripted waveforms, closed-loop path following, hands-off and locked brackets. Deterministic; the only
   allocation is in dc_driver_create. See drivechain.h for the contract. */
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "drivechain.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct dc_driver {
    dc_driver_params p;
    dc_driver_cmd c;
    const double* path; int npath, closed, pidx, have_idx;
    double hist_v[DC_DRV_HIST]; double hist_t[DC_DRV_HIST]; int hn, hhead;   /* delay line (ring) */
    double time, filt, filt_init, rate_out, ival, locked_rim, lat_i; int locked_set;
    dc_driver_state st;
};

static double clampd(double x, double a, double b) { return x < a ? a : (x > b ? b : x); }
static double wrapa(double a) { while (a > M_PI) a -= 2 * M_PI; while (a < -M_PI) a += 2 * M_PI; return a; }

void dc_driver_default_params(dc_driver_params* p) {
    memset(p, 0, sizeof *p);
    p->arm_k = 10; p->arm_d = 1.0; p->arm_tmax = 20;   /* relaxed-but-firm grip, as the gamepad "arms" layer */
    p->delay = 0.18; p->bandwidth_hz = 3.0; p->rim_rate_max = 0;
    p->preview_time = 0.75; p->preview_min = 6.0; p->yaw_damp = 0.06; p->catch_gain = 0.5;
    p->speed_kp = 0.5; p->speed_ki = 0.15; p->brake_kp = 0.25; p->lock_frac = 1.0; p->lat_ki = 0.005; p->predict = 1.0;
}
dc_driver* dc_driver_create(const dc_driver_params* p) {
    dc_driver* d = (dc_driver*)calloc(1, sizeof *d);
    if (!d) return NULL;
    if (p) d->p = *p; else dc_driver_default_params(&d->p);
    d->c.steer_mode = DC_DRV_OFF; d->c.pedal_mode = DC_PED_OFF;
    return d;
}
void dc_driver_destroy(dc_driver* d) { free(d); }
void dc_driver_set_params(dc_driver* d, const dc_driver_params* p) { d->p = *p; }
void dc_driver_get_params(const dc_driver* d, dc_driver_params* p) { *p = d->p; }
void dc_driver_reset(dc_driver* d) {
    d->hn = 0; d->hhead = 0; d->filt_init = 0; d->ival = 0; d->lat_i = 0; d->have_idx = 0; d->locked_set = 0; d->time = 0;
    memset(&d->st, 0, sizeof d->st);
}
void dc_driver_set_path(dc_driver* d, const double* xy, int n, int closed) { d->path = xy; d->npath = n; d->closed = closed; d->have_idx = 0; }
void dc_driver_command(dc_driver* d, const dc_driver_cmd* c) {
    int was = d->c.steer_mode;
    d->c = *c;
    if (c->steer_mode != DC_DRV_LOCKED || was != DC_DRV_LOCKED) d->locked_set = 0;
    if (c->pedal_mode != DC_PED_SPEED) d->ival = 0;
    if (c->steer_mode != was) d->lat_i = 0;
}
void dc_driver_get_state(const dc_driver* d, dc_driver_state* s) { *s = d->st; }

/* delay line: push (t, v), read the value at t - delay (linear interpolation; oldest sample if the line is short) */
static double delay_line(dc_driver* d, double v) {
    int i = d->hhead; d->hist_t[i] = d->time; d->hist_v[i] = v;
    d->hhead = (i + 1) % DC_DRV_HIST; if (d->hn < DC_DRV_HIST) d->hn++;
    if (d->p.delay <= 0) return v;
    double tq = d->time - d->p.delay;
    int k = i;
    for (int m = 0; m < d->hn - 1; m++) {
        int kp = (k - 1 + DC_DRV_HIST) % DC_DRV_HIST;
        if (d->hist_t[kp] <= tq) { double t0 = d->hist_t[kp], t1 = d->hist_t[k];
            double a = t1 > t0 ? (tq - t0) / (t1 - t0) : 1; return d->hist_v[kp] + a * (d->hist_v[k] - d->hist_v[kp]); }
        k = kp;
    }
    return d->hist_v[k];   /* not enough history: hold the oldest command (start-up) */
}

static void pose(const dc_car* car, double* x, double* y, double* yaw) {
    dc_xform P[DC_PART_COUNT]; dc_get_parts(car, P, DC_PART_COUNT); const dc_xform* c = &P[DC_PART_CHASSIS];
    double q0 = c->quat[0], q1 = c->quat[1], q2 = c->quat[2], q3 = c->quat[3];
    *yaw = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)); *x = c->pos[0]; *y = c->pos[1];
}
static void ppt(const dc_driver* d, int i, double* x, double* y) {
    int n = d->npath; if (d->closed) i = ((i % n) + n) % n; else i = i < 0 ? 0 : (i >= n ? n - 1 : i);
    *x = d->path[2 * i]; *y = d->path[2 * i + 1];
}
/* nearest segment (local search around the last index once locked on) -> index, fraction, lateral error, tangent */
static void project(dc_driver* d, double x, double y, int* idx, double* frac, double* lat, double* tx, double* ty) {
    int n = d->npath, lo = 0, hi = n - 1;
    if (d->have_idx) { lo = d->pidx - 40; hi = d->pidx + 40; if (!d->closed) { lo = lo < 0 ? 0 : lo; hi = hi > n - 2 ? n - 2 : hi; } }
    else if (!d->closed) hi = n - 2;
    double best = 1e300; int bi = 0; double bf = 0;
    for (int i = lo; i <= hi; i++) {
        double ax, ay, bx, by; ppt(d, i, &ax, &ay); ppt(d, i + 1, &bx, &by);
        double ex = bx - ax, ey = by - ay, l2 = ex * ex + ey * ey + 1e-12;
        double f = clampd(((x - ax) * ex + (y - ay) * ey) / l2, 0, 1);
        double dx = ax + f * ex - x, dy = ay + f * ey - y, dd = dx * dx + dy * dy;
        if (dd < best) { best = dd; bi = i; bf = f; }
    }
    if (d->closed) bi = ((bi % n) + n) % n;
    d->pidx = bi; d->have_idx = 1;
    double ax, ay, bx, by; ppt(d, bi, &ax, &ay); ppt(d, bi + 1, &bx, &by);
    double l = hypot(bx - ax, by - ay) + 1e-12; *tx = (bx - ax) / l; *ty = (by - ay) / l;
    double px = ax + bf * (bx - ax), py = ay + bf * (by - ay);
    *lat = -(x - px) * (*ty) + (y - py) * (*tx);   /* + = car left of path */
    *idx = bi; *frac = bf;
}
/* point at arc distance L ahead of (idx, frac) plus lateral offset */
static void ahead(const dc_driver* d, int idx, double frac, double L, double off, double* x, double* y) {
    double ax, ay, bx, by; ppt(d, idx, &ax, &ay); ppt(d, idx + 1, &bx, &by);
    double seg = hypot(bx - ax, by - ay), rem = (1 - frac) * seg; int i = idx;
    double cx = ax + frac * (bx - ax), cy = ay + frac * (by - ay);
    for (int guard = 0; guard < 100000 && L > rem; guard++) {
        L -= rem; i++; if (!d->closed && i >= d->npath - 1) { L = 0; break; }
        ppt(d, i, &ax, &ay); ppt(d, i + 1, &bx, &by); seg = hypot(bx - ax, by - ay); rem = seg; cx = ax; cy = ay;
    }
    ppt(d, i, &ax, &ay); ppt(d, i + 1, &bx, &by); seg = hypot(bx - ax, by - ay) + 1e-12;
    double tx = (bx - ax) / seg, ty = (by - ay) / seg;
    *x = cx + tx * L - ty * off; *y = cy + ty * L + tx * off;
}
static double curvature_at(const dc_driver* d, int i) {
    double x0, y0, x1, y1, x2, y2; ppt(d, i - 2, &x0, &y0); ppt(d, i, &x1, &y1); ppt(d, i + 2, &x2, &y2);
    double a = hypot(x1 - x0, y1 - y0), b = hypot(x2 - x1, y2 - y1), c = hypot(x2 - x0, y2 - y0);
    double cr = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0);
    return a * b * c > 1e-9 ? 2 * cr / (a * b * c) : 0;
}

void dc_driver_update(dc_driver* d, const dc_car* car, double dt, dc_input* io) {
    const dc_car_params* P = dc_params(car);
    dc_telemetry t; dc_get_telemetry(car, &t);
    const dc_driver_params* p = &d->p;
    const dc_driver_cmd* c = &d->c;
    double ratio = P->steer_ratio, lock = P->steer_lock_deg * M_PI / 180 * p->lock_frac;
    double a_cg = P->axle[0].hub[0], wb = P->axle[0].hub[0] - P->axle[1].hub[0];
    double v = t.vx, va = fmax(fabs(v), 0.5);
    d->time += dt;
    d->st.time = d->time; d->st.beta = atan2(t.vy, va);
    double wave = (c->wave_amp != 0 && d->time >= c->wave_t0) ? c->wave_amp * sin(2 * M_PI * c->wave_freq * (d->time - c->wave_t0)) : 0;
    double cmd = 0; int angle_like = 0;
    switch (c->steer_mode) {
    case DC_DRV_ANGLE: cmd = c->steer_target + wave; angle_like = 1; break;
    case DC_DRV_TORQUE: cmd = c->steer_torque + wave; break;
    case DC_DRV_PATH:
        if (d->path && d->npath >= 2) {
            double x, y, yaw; pose(car, &x, &y, &yaw);
            double rx = x - (wb - a_cg) * cos(yaw), ry = y - (wb - a_cg) * sin(yaw);   /* rear-axle reference (pure pursuit) */
            int idx; double frac, lat, tx, ty; project(d, rx, ry, &idx, &frac, &lat, &tx, &ty);
            double lat_now = lat, tx_now = tx, ty_now = ty, yaw_now = yaw;
            if (p->predict > 0) {   /* internal model: aim from where the car will be when this command reaches the rim */
                double tau = p->predict * (p->delay + (p->bandwidth_hz > 0 ? 1 / (2 * M_PI * p->bandwidth_hz) : 0));
                double ym = yaw + 0.5 * t.yaw_rate * tau, vxw = t.vx * cos(ym) - t.vy * sin(ym), vyw = t.vx * sin(ym) + t.vy * cos(ym);
                x += vxw * tau; y += vyw * tau; yaw += t.yaw_rate * tau;
                rx = x - (wb - a_cg) * cos(yaw); ry = y - (wb - a_cg) * sin(yaw);
                int i2; double f2, l2, tx2, ty2; project(d, rx, ry, &i2, &f2, &l2, &tx2, &ty2);
                d->pidx = idx;   /* keep the tracker anchored on the actual pose */
                idx = i2; frac = f2; (void)l2; (void)tx2; (void)ty2;
            }
            double L = fmax(p->preview_min, p->preview_time * va);
            double gx, gy; ahead(d, idx, frac, L, c->path_offset, &gx, &gy);
            double ex = gx - rx, ey = gy - ry;
            double lx = cos(yaw) * ex + sin(yaw) * ey, ly = -sin(yaw) * ex + cos(yaw) * ey;
            double kap = curvature_at(d, idx);
            double delta = atan2(2 * wb * ly, lx * lx + ly * ly);
            delta -= p->yaw_damp * (t.yaw_rate - va * kap);
            double beta_f = atan2(t.vy + a_cg * t.yaw_rate, va);   /* front-axle sideslip: + = front sliding left */
            double beta_r = atan2(t.vy - (wb - a_cg) * t.yaw_rate, va);
            { double b = clampd(beta_r, -0.6, 0.6), db = fabs(b) - 0.03; delta += p->catch_gain * (db > 0 ? copysign(db, b) : 0.0); }   /* countersteer when the rear steps out (continuous deadband: no switching chatter) */
            (void)beta_f;
            /* slow trim (bounded, frozen while the rear is sliding so it never fights a catch) */
            if (fabs(beta_r) < 0.05) d->lat_i = clampd(d->lat_i + p->lat_ki * (lat_now - c->path_offset) * dt, -0.015, 0.015);
            delta -= d->lat_i;
            cmd = clampd(delta * ratio, -lock, lock); angle_like = 1;
            d->st.lat_err = lat_now - c->path_offset; d->st.head_err = wrapa(yaw_now - atan2(ty_now, tx_now)); d->st.path_index = idx;
            d->st.kappa_path = kap; d->st.preview_dist = L;
        }
        break;
    case DC_DRV_LOCKED: if (!d->locked_set) { d->locked_rim = t.sw_angle; d->locked_set = 1; } break;
    default: break;
    }
    /* voluntary command path: bandwidth -> rate limit -> transport delay */
    if (c->steer_mode == DC_DRV_ANGLE || c->steer_mode == DC_DRV_TORQUE || c->steer_mode == DC_DRV_PATH) {
        if (!d->filt_init) { d->filt = angle_like ? t.sw_angle : cmd; d->rate_out = d->filt; d->filt_init = 1; }
        if (p->bandwidth_hz > 0) d->filt += (cmd - d->filt) * (1 - exp(-dt * 2 * M_PI * p->bandwidth_hz)); else d->filt = cmd;
        double out = d->filt;
        if (angle_like && p->rim_rate_max > 0) { double mx = p->rim_rate_max * dt; out = d->rate_out + clampd(out - d->rate_out, -mx, mx); }
        d->rate_out = out;
        double del = delay_line(d, out);
        d->st.rim_cmd = cmd; d->st.rim_cmd_delayed = del;
        if (angle_like) { io->steer_mode = DC_STEER_ANGLE; io->steer_target = clampd(del, -lock, lock);
                          io->hands_k = p->arm_k; io->hands_d = p->arm_d; io->hands_tmax = p->arm_tmax; }
        else { io->steer_mode = DC_STEER_TORQUE; io->steer_torque = clampd(del, -p->arm_tmax, p->arm_tmax); }
    } else if (c->steer_mode == DC_DRV_HANDS_OFF) {
        io->steer_mode = DC_STEER_TORQUE; io->steer_torque = 0; d->filt_init = 0;
    } else if (c->steer_mode == DC_DRV_LOCKED) {
        io->steer_mode = DC_STEER_ANGLE; io->steer_target = d->locked_rim; io->hands_k = 3000; io->hands_d = 30; io->hands_tmax = 500; d->filt_init = 0;
    } else d->filt_init = 0;
    d->st.saturated = fabs(t.driver_torque) >= 0.98 * p->arm_tmax && c->steer_mode != DC_DRV_OFF && c->steer_mode != DC_DRV_HANDS_OFF;
    /* pedals */
    if (c->pedal_mode == DC_PED_DIRECT) { io->throttle = clampd(c->throttle, 0, 1); io->brake = clampd(c->brake, 0, 1); io->handbrake = clampd(c->handbrake, 0, 1); }
    else if (c->pedal_mode == DC_PED_SPEED) {
        double e = c->speed - v;
        d->ival = clampd(d->ival + p->speed_ki * e * dt, -0.3, 0.6);
        io->throttle = clampd(p->speed_kp * e + d->ival + 0.08, 0, 1);
        io->brake = clampd(-p->brake_kp * e - 0.05, 0, 0.9); if (io->brake > 0) io->throttle = 0;
        io->handbrake = 0;
    }
}
