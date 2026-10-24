#include "sim.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
static double now_us(void) { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return ts.tv_sec * 1e6 + ts.tv_nsec * 1e-3; }
static double clampd(double v, double a, double b) { return v < a ? a : (v > b ? b : v); }

static int make_car(sim_t* S) {
    if (S->car) dc_destroy(S->car);
    S->car = dc_create(&S->P);
    if (!S->car) return 0;
    dc_set_road(S->car, track_road_fn, S->trk);
    return 1;
}

int sim_init(sim_t* S, cfg_t* cfg, track_t* trk, audio_t* audio, char* err, int errlen) {
    memset(S, 0, sizeof *S);
    S->cfg = cfg; S->trk = trk; S->audio = audio;
    dc_default_params(&S->P);
    const char* carini = cfg_str(cfg, "car.ini", "");
    if (*carini && !dc_load_params(carini, &S->P, err, errlen)) return 0;
    S->P.substep_hz = cfg_num(cfg, "physics.substep_hz", S->P.substep_hz);
    int as = cfg_int(cfg, "car.assist", -1); if (as >= 0) S->P.assist = as;
    if (!make_car(S)) { snprintf(err, errlen, "dc_create failed"); return 0; }
    S->rotation_rad = cfg_num(cfg, "input.rotation_deg", 2 * S->P.steer_lock_deg) * M_PI / 180;
    S->hands_k = cfg_num(cfg, "input.wheel_hands_k", 80); S->hands_d = cfg_num(cfg, "input.wheel_hands_d", 1.2);
    S->hands_tmax = cfg_num(cfg, "input.wheel_hands_tmax", 30);
    S->auto_clutch = cfg_int(cfg, "input.auto_clutch", -1);
    S->kb_hands_off = cfg_int(cfg, "input.keyboard_hands_off", 1);
    { const char* gm = cfg_str(cfg, "gamepad.mode", "arms"); S->gp_mode = strcmp(gm, "raw") != 0; }
    S->gp_k = cfg_num(cfg, "gamepad.arms_k", 10);          /* N m/rad at the rim: relaxed grip */
    S->gp_d = cfg_num(cfg, "gamepad.arms_d", 1.0);         /* N m s/rad: hand/forearm damping */
    S->gp_tmax = cfg_num(cfg, "gamepad.arms_tmax", 20);    /* N m: strongest the arms will push */
    S->gp_deadzone = cfg_num(cfg, "gamepad.deadzone", 0.04);     /* stick jitter isolation only: inside it the hands are off the rim */
    S->gp_grip_t = cfg_num(cfg, "gamepad.grip_time", 0.08);      /* s for the hands to close on the rim when the stick leaves the deadzone */
    S->auto_line = cfg_num(cfg, "autopilot.inside_offset", 4.8);
    S->auto_ay = cfg_num(cfg, "autopilot.lat_accel", 5.5);
    S->auto_speed_cap = cfg_num(cfg, "autopilot.speed_cap", 33);
    S->autopilot = cfg_int(cfg, "autopilot.enabled", 0);
    S->auto_straight = cfg_num(cfg, "autopilot.straight_speed", 0);
    {   /* plug-in driver defaults (overridable per command over the protocol) */
        dc_driver_params dp; dc_driver_default_params(&dp);
        dp.arm_k = cfg_num(cfg, "driver.arm_k", dp.arm_k); dp.arm_d = cfg_num(cfg, "driver.arm_d", dp.arm_d);
        dp.arm_tmax = cfg_num(cfg, "driver.arm_tmax", dp.arm_tmax); dp.delay = cfg_num(cfg, "driver.delay", dp.delay);
        dp.bandwidth_hz = cfg_num(cfg, "driver.bandwidth_hz", dp.bandwidth_hz);
        dp.preview_time = cfg_num(cfg, "driver.preview_time", dp.preview_time); dp.catch_gain = cfg_num(cfg, "driver.catch_gain", dp.catch_gain);
        dp.predict = cfg_num(cfg, "driver.predict", dp.predict); dp.lat_ki = cfg_num(cfg, "driver.lat_ki", dp.lat_ki);
        dp.yaw_damp = cfg_num(cfg, "driver.yaw_damp", dp.yaw_damp);
        S->drv = dc_driver_create(&dp);
        S->drv_takeover_enabled = cfg_int(cfg, "driver.takeover", 1);
        S->drv_takeover_stick = cfg_num(cfg, "driver.takeover_stick", 0.25);
        S->drv_takeover_wheel = cfg_num(cfg, "driver.takeover_wheel_deg", 45) * M_PI / 180;
        S->drv_takeover_pedal = cfg_num(cfg, "driver.takeover_pedal", 0.15);
    }
    input_open(cfg, &S->dev);
    if (S->auto_clutch < 0) S->auto_clutch = !(S->dev.kind == 1 && S->dev.hgear != -99);   /* H-shifter rigs: manual clutch */
    S->gear = 1;
    if (cfg_str(cfg, "start.x", NULL))
        sim_reset_at(S, cfg_num(cfg, "start.x", 0), cfg_num(cfg, "start.y", 0), cfg_num(cfg, "start.yaw_deg", 0) * M_PI / 180, cfg_num(cfg, "start.speed", 0));
    else if (track_spot_by_name(trk, cfg_str(cfg, "start.spot", cfg_str(cfg, "start.s", NULL) ? "" : "hub")) >= 0) {   /* default: proving-ground start hub */
        double x, y, yaw; track_spot(trk, track_spot_by_name(trk, cfg_str(cfg, "start.spot", cfg_str(cfg, "start.s", NULL) ? "" : "hub")), &x, &y, &yaw);
        sim_reset_at(S, x, y, yaw, cfg_num(cfg, "start.speed", 0));
    }
    else sim_reset_on_track(S, cfg_num(cfg, "start.s", 30), cfg_num(cfg, "start.d", 0), cfg_num(cfg, "start.speed", 0));
    return 1;
}
void sim_reset_at(sim_t* S, double x, double y, double yaw, double speed) {
    double nx, ny, nz, g;
    double h = track_road_fn(S->trk, x, y, &nx, &ny, &nz, &g);
    double pos[3] = {x, y, h + 0.01};
    dc_reset(S->car, pos, yaw, speed);
    S->kb_sw = 0;
}
int sim_rebuild_car(sim_t* S) {
    dc_xform ch[DC_PART_COUNT]; dc_get_parts(S->car, ch, DC_PART_COUNT);
    dc_telemetry t0; dc_get_telemetry(S->car, &t0);
    double q0 = ch[0].quat[0], q1 = ch[0].quat[1], q2 = ch[0].quat[2], q3 = ch[0].quat[3];
    double yaw = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
    if (!make_car(S)) return 0;
    sim_reset_at(S, ch[0].pos[0], ch[0].pos[1], yaw, t0.vx); return 1;
}
void sim_driver_engage(sim_t* S, int on) {
    if (!S->drv) return;
    if (on && !S->drv_on) dc_driver_reset(S->drv);
    S->drv_on = on; if (on) S->drv_takeover = 0;
}
void sim_reset_on_track(sim_t* S, double s, double d, double speed) {
    double x, y, yaw; track_point(S->trk, s, d, &x, &y, &yaw);
    sim_reset_at(S, x, y, yaw, speed);
}

/* pure-pursuit + curvature speed autopilot (used for scripted screenshots / audio renders) */
static void autopilot(sim_t* S, const dc_telemetry* t, const dc_xform* ch, dc_input* in) {
    if (S->auto_straight > 0) {   /* hands off, hold a speed: shows pure emergent steering response on rough ground */
        double e = S->auto_straight - t->vx;
        in->steer_mode = DC_STEER_TORQUE; in->steer_torque = 0;
        in->throttle = clampd(e * 0.4, 0, 1); in->brake = clampd(-e * 0.3, 0, 1); return;
    }
    const track_t* T = S->trk;
    double px = ch->pos[0], py = ch->pos[1];
    double q0 = ch->quat[0], q1 = ch->quat[1], q2 = ch->quat[2], q3 = ch->quat[3];
    double yaw = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
    double d, s; track_height(T, px, py, NULL, NULL, &d, &s);
    double v = fmax(t->vx, 0);
    double L = 7 + 0.45 * v;
    /* line: towards the inside of upcoming curvature (puts the inside wheels onto the apex kerbs) */
    int k = (int)(fmod(s + L, T->length) / T->ds) % T->n;
    double cv = 0; for (int m = -6; m <= 6; m++) cv += T->curv[(k + m + T->n) % T->n]; cv /= 13;
    double dline = clampd(cv * 400, -1, 1) * S->auto_line;
    double tx, ty; track_point(T, s + L, dline, &tx, &ty, NULL);
    double ex = tx - px, ey = ty - py;
    double lx = cos(yaw) * ex + sin(yaw) * ey, ly = -sin(yaw) * ex + cos(yaw) * ey;
    double wb = S->P.axle[0].hub[0] - S->P.axle[1].hub[0];
    double delta = atan2(2 * wb * ly, lx * lx + ly * ly);
    in->steer_mode = DC_STEER_ANGLE; in->steer_target = clampd(delta * S->P.steer_ratio, -7, 7);
    in->hands_k = 40; in->hands_d = 1.0; in->hands_tmax = 25;
    /* speed: min over the next 80 m of sqrt(ay/|k|) with braking distance */
    double vt = S->auto_speed_cap;
    for (double a = 0; a < 160; a += 2) {
        int kk = (int)(fmod(s + a, T->length) / T->ds) % T->n;
        double c = fabs(T->curv[kk]) + 1e-4, vc = sqrt(S->auto_ay / c);
        double vb = sqrt(vc * vc + 2 * 4.5 * fmax(a - 0.4 * v, 0));
        if (vb < vt) vt = vb;
    }
    double e = vt - v;
    in->throttle = clampd(e * 0.35, 0, 1); in->brake = clampd(-e * 0.6, 0, 0.85);
    if (fabs(t->ay) > 0.8 * S->auto_ay * 1.3) in->throttle *= 0.5;
    /* recovery: autopilot that left the circuit (or spun) is put back on the centre line, like a marshal would */
    if (fabs(d) > T->half_w + T->kerb_w + 4 || (v < 2 && S->sim_time > 5)) {
        if (++S->auto_off_ms > 2500) { S->auto_off_ms = 0; sim_reset_on_track(S, s, 0, 8); S->gear = 1; }
    } else S->auto_off_ms = 0;
}

void sim_tick(sim_t* S, double dt) {
    sim_ctl* C = &S->ctl;
    /* one-shot requests */
    if (C->toggle_assist != S->ctl_seen.toggle_assist) {
        S->ctl_seen.toggle_assist = C->toggle_assist;
        dc_xform ch[DC_PART_COUNT]; dc_get_parts(S->car, ch, DC_PART_COUNT);
        dc_telemetry t0; dc_get_telemetry(S->car, &t0);
        double q0 = ch[0].quat[0], q1 = ch[0].quat[1], q2 = ch[0].quat[2], q3 = ch[0].quat[3];
        double yaw = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
        S->P.assist = !S->P.assist; make_car(S); sim_reset_at(S, ch[0].pos[0], ch[0].pos[1], yaw, t0.vx);
    }
    if (C->reset != S->ctl_seen.reset) {
        S->ctl_seen.reset = C->reset;
        dc_xform ch[DC_PART_COUNT]; dc_get_parts(S->car, ch, DC_PART_COUNT);
        double d, s; track_height(S->trk, ch[0].pos[0], ch[0].pos[1], NULL, NULL, &d, &s);
        for (int i = 0; i < 4; i++) dc_set_pressure(S->car, i, S->P.axle[i / 2].tyre.pressure);
        sim_reset_on_track(S, s, 0, 0); S->gear = 1;
    }
    if (C->spot_req != S->ctl_seen.spot_req) {
        S->ctl_seen.spot_req = C->spot_req;
        double x, y, yaw;
        if (track_spot(S->trk, C->spot, &x, &y, &yaw)) {
            for (int i = 0; i < 4; i++) dc_set_pressure(S->car, i, S->P.axle[i / 2].tyre.pressure);
            sim_reset_at(S, x, y, yaw, 0); S->gear = 1;
        }
    }
    if (C->puncture != S->ctl_seen.puncture) { S->ctl_seen.puncture = C->puncture; dc_puncture(S->car, (C->puncture - 1) & 3, 2.0); }
    if (C->autopilot != S->ctl_seen.autopilot) { S->ctl_seen.autopilot = C->autopilot; S->autopilot = !S->autopilot; }
    if (C->drv_toggle != S->ctl_seen.drv_toggle) { S->ctl_seen.drv_toggle = C->drv_toggle; sim_driver_engage(S, !S->drv_on); }
    if (S->ext_hook) S->ext_hook(S, 0);
    if (C->pause) { S->snap.paused = 1; return; }
    S->snap.paused = 0;

    input_poll(&S->dev);
    dc_telemetry t; dc_get_telemetry(S->car, &t);
    dc_xform parts[DC_PART_COUNT]; dc_get_parts(S->car, parts, DC_PART_COUNT);
    dc_input in; memset(&in, 0, sizeof in);
    const input_state* D = &S->dev;
    double v = fabs(t.vx);
    /* ---- steering ---- */
    if (D->kind == 1) {          /* wheel: device angle is the steering-wheel angle (1:1), stiff hands; FFB = column torque */
        in.steer_mode = DC_STEER_ANGLE; in.steer_target = D->steer * S->rotation_rad * 0.5;
        in.hands_k = S->hands_k; in.hands_d = S->hands_d; in.hands_tmax = S->hands_tmax;
    } else if (D->kind == 2) {   /* gamepad: stick = rim target, linear over the FULL lock at every speed (no speed range, no expo) */
        double x = D->steer, dz = S->gp_deadzone, lock = S->P.steer_lock_deg * M_PI / 180;
        if (fabs(x) <= dz) {     /* stick centred: hands relaxed off the rim, caster trail x Fy / SAT / valve steer the wheel freely */
            in.steer_mode = DC_STEER_TORQUE; in.steer_torque = 0; S->gp_grip = 0;
        } else {
            x = (x - (x > 0 ? dz : -dz)) / (1 - dz);
            S->gp_grip = S->gp_grip_t > 0 ? fmin(1, S->gp_grip + dt / S->gp_grip_t) : 1;   /* hand grip onset, not a range limit */
            in.steer_mode = DC_STEER_ANGLE; in.steer_target = x * lock;
            if (S->gp_mode == 1) { in.hands_k = S->gp_k * S->gp_grip; in.hands_d = S->gp_d * S->gp_grip; in.hands_tmax = S->gp_tmax * S->gp_grip; }   /* arms */
            else { in.hands_k = 25 * S->gp_grip; in.hands_d = 0.8 * S->gp_grip; in.hands_tmax = 15 * S->gp_grip; }                                  /* raw: stiffer legacy hands */
        }
    } else {
        int dir = C->kb_left - C->kb_right;
        double lim = S->P.steer_lock_deg * M_PI / 180;   /* full lock at every speed */
        if (dir) {
            if (S->kb_sw * dir < 0 || fabs(S->kb_sw - t.sw_angle) > 0.6) S->kb_sw = t.sw_angle;   /* grab the wheel where it is */
            S->kb_sw = clampd(S->kb_sw + dir * 6.0 * dt, -lim, lim);
            in.steer_mode = DC_STEER_ANGLE; in.steer_target = S->kb_sw; in.hands_k = 25; in.hands_d = 0.8; in.hands_tmax = 15;
        } else if (S->kb_hands_off) {   /* hands off: caster/SAT centre the wheel and catch slides by themselves */
            in.steer_mode = DC_STEER_TORQUE; in.steer_torque = 0; S->kb_sw = t.sw_angle;
        } else {
            S->kb_sw *= 1 - 3 * dt; in.steer_mode = DC_STEER_ANGLE; in.steer_target = S->kb_sw; in.hands_k = 25; in.hands_d = 0.8; in.hands_tmax = 15;
        }
    }
    /* ---- pedals ---- */
    double thr = C->kb_up, brk = C->kb_down, clu = C->kb_clutch, hb = C->kb_hand;
    if (D->kind) { thr = fmax(thr, D->throttle); brk = fmax(brk, D->brake); clu = fmax(clu, D->clutch); hb = fmax(hb, D->handbrake); }
    in.throttle = thr; in.brake = brk; in.handbrake = hb;
    if (S->autopilot) { autopilot(S, &t, &parts[0], &in); }
    if (S->ext_steer) { in.steer_mode = S->ext_in.steer_mode; in.steer_torque = S->ext_in.steer_torque; in.steer_target = S->ext_in.steer_target;
                        in.hands_k = S->ext_in.hands_k; in.hands_d = S->ext_in.hands_d; in.hands_tmax = S->ext_in.hands_tmax; }
    if (S->ext_pedals) { in.throttle = S->ext_in.throttle; in.brake = S->ext_in.brake; in.handbrake = S->ext_in.handbrake; }
    /* ---- plug-in driver + live human takeover ---- */
    int drv_ped = 0;
    if (S->drv && S->drv_on) {
        int human = C->kb_left || C->kb_right || C->kb_up || C->kb_down;
        if (D->kind == 2) human |= fabs(D->steer) > S->drv_takeover_stick;
        if (D->kind == 1) human |= fabs(D->steer * S->rotation_rad * 0.5 - t.sw_angle) > S->drv_takeover_wheel;   /* wheel held away from the sim rim */
        if (D->kind) human |= D->throttle > S->drv_takeover_pedal || D->brake > S->drv_takeover_pedal;
        if (human && S->drv_takeover_enabled) { S->drv_on = 0; S->drv_takeover = 1; }   /* human input above already mapped into `in` */
        else {
            dc_driver_command(S->drv, &S->drv_cmd);
            dc_driver_update(S->drv, S->car, dt, &in);
            drv_ped = S->drv_cmd.pedal_mode != DC_PED_OFF;
        }
    }
    /* ---- gearbox ---- */
    int up = C->shift_up - S->ctl_seen.shift_up + D->shift_up, dn = C->shift_down - S->ctl_seen.shift_down + D->shift_down;
    S->ctl_seen.shift_up = C->shift_up; S->ctl_seen.shift_down = C->shift_down; S->dev.shift_up = S->dev.shift_down = 0;
    if (D->kind == 1 && D->hgear != -99) {
        if (D->hgear != S->gear) S->gear = D->hgear;
    } else {
        if (S->autopilot || drv_ped) {   /* autopilot / plug-in driver auto-shift */
            if (t.engine_rpm > 6600 && S->gear < S->P.ngears && S->shift_timer_ms <= 0) up = 1;
            if (t.engine_rpm < 2600 && S->gear > 1 && S->shift_timer_ms <= 0) dn = 1;
        }
        if (up > 0 && S->gear < S->P.ngears) { S->gear = S->gear < 0 ? 0 : S->gear + 1; S->shift_timer_ms = 160; }
        if (dn > 0 && S->gear > -1) { S->gear--; S->shift_timer_ms = 160; }
    }
    if (S->shift_timer_ms > 0) S->shift_timer_ms -= (int)(dt * 1000 + 0.5);
    if (S->auto_clutch || S->autopilot || drv_ped) {
        double c = S->shift_timer_ms > 0 ? 1.0 : 0.0;
        if (S->shift_timer_ms > 0 && S->shift_timer_ms < 60) c = S->shift_timer_ms / 60.0;
        /* launch / anti-stall: slip the clutch below ~1.6x idle */
        if (S->gear != 0 && t.engine_rpm < 2200) c = fmax(c, clampd((2200 - t.engine_rpm) / 1300.0, 0, 1) * (in.brake > 0.3 && v < 1 ? 1 : 0.85));
        if (S->gear != 0 && v < 0.5 && in.throttle < 0.05) c = 1;
        clu = fmax(clu, c);
        if (S->shift_timer_ms > 60) in.throttle = 0;
    }
    in.clutch = clu; in.gear = S->gear;
    dc_set_input(S->car, &in);
    double t0 = now_us();
    int n = dc_step(S->car, dt);
    S->us_acc += now_us() - t0; S->steps_acc += n;
    S->sim_time += dt;
    dc_get_telemetry(S->car, &t);
    /* ---- FFB ---- */
    input_ffb(&S->dev, t.column_torque, dt);
    if (D->kind == 2) {
        double sl = 0; for (int i = 0; i < 4; i++) sl += t.w[i].sliding * fmin(t.w[i].fz / 3000, 1.5);
        input_rumble(&S->dev, 0.15 * fabs(t.column_torque) / 5, 0.2 * sl);
    }
    /* ---- surfaces + audio frame ---- */
    int surf[4];
    audio_frame af; memset(&af, 0, sizeof af);
    for (int i = 0; i < 4; i++) {
        track_height(S->trk, t.w[i].contact[0], t.w[i].contact[1], &surf[i], NULL, NULL, NULL);
        af.fz[i] = (float)t.w[i].fz; af.sliding[i] = (float)t.w[i].sliding; af.surf[i] = (float)surf[i]; af.rim[i] = (float)t.w[i].rim_force;
        double sv = fabs(t.speed) * hypot(t.w[i].slip_ratio, tan(fmin(fabs(t.w[i].slip_angle), 1.2)));
        af.slide[i] = (float)sv;
    }
    af.rpm = (float)t.engine_rpm; af.throttle = (float)in.throttle; af.speed = (float)fabs(t.speed);
    af.load = t.engine_running ? (float)(in.throttle > 0.02 ? in.throttle : -0.3) : 0;
    af.clutch = (float)in.clutch; af.gear = t.gear; af.engine_on = t.engine_running;
    af.limiter = t.engine_rpm > S->P.limiter_rpm - 80 ? 1.0f : 0.0f;
    if (S->audio) audio_push(S->audio, &af);
    /* ---- snapshot ---- */
    sim_snap* N = &S->snap;
    N->t = t; dc_get_parts(S->car, N->parts, DC_PART_COUNT); N->nseg = dc_get_segments(S->car, N->seg, SIM_MAXSEG);
    N->in = in; memcpy(N->surf, surf, sizeof surf); N->ffb_level = S->dev.ffb_out; N->sim_time = S->sim_time;
    N->beta = fabs(t.vx) > 1 ? atan2(t.vy, fabs(t.vx)) : 0;
    N->gear_req = S->gear; N->assist = S->P.assist; N->autopilot = S->autopilot;
    N->device_kind = S->dev.kind; N->has_ffb = S->dev.has_ffb; snprintf(N->device, sizeof N->device, "%s", S->dev.name);
    for (int a = 0; a < 8; a++) N->raw_axis[a] = S->dev.axis[a];
    N->drv_on = S->drv_on; N->drv_takeover = S->drv_takeover; N->drv_mode = S->drv_cmd.steer_mode; N->drv_clients = S->ext_clients;
    if (S->drv) dc_driver_get_state(S->drv, &N->drv_st);
    N->raw_buttons = S->dev.buttons_raw; N->raw_naxes = S->dev.naxes; N->hgear = S->dev.hgear;
    if (S->ext_hook) S->ext_hook(S, 1);
    if (S->steps_acc >= 2000) { N->us_per_substep = S->us_acc / S->steps_acc; N->rt_factor = 1e6 / (N->us_per_substep * S->P.substep_hz); S->us_acc = 0; S->steps_acc = 0; }
}
void sim_shutdown(sim_t* S) { input_close(); if (S->car) dc_destroy(S->car); S->car = NULL; if (S->drv) dc_driver_destroy(S->drv); S->drv = NULL; }
