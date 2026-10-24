/* drvproto.c - see drvproto.h and docs/DRIVER_INTERFACE.md */
#include "drvproto.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG (180.0 / M_PI)
#define PROTO_VERSION 1

/* ---- minimal flat-JSON readers (objects of scalars / strings / one numeric array) ---- */
static const char* jval(const char* s, const char* key) {
    char k[96]; snprintf(k, sizeof k, "\"%s\"", key);
    const char* p = s;
    while ((p = strstr(p, k))) {
        const char* q = p + strlen(k); while (*q == ' ' || *q == '\t') q++;
        if (*q == ':') { q++; while (*q == ' ' || *q == '\t') q++; return q; }
        p = q;
    }
    return NULL;
}
static int jnum(const char* s, const char* key, double* v) {
    const char* q = jval(s, key); if (!q) return 0;
    if (!strncmp(q, "true", 4)) { *v = 1; return 1; } if (!strncmp(q, "false", 5)) { *v = 0; return 1; }
    char* e; double d = strtod(q, &e); if (e == q) return 0; *v = d; return 1;
}
static int jstr(const char* s, const char* key, char* out, int max) {
    const char* q = jval(s, key); if (!q || *q != '"') return 0;
    q++; int n = 0; while (*q && *q != '"' && n < max - 1) out[n++] = *q++; out[n] = 0; return 1;
}
static int jarr(const char* s, const char* key, double* out, int max) {
    const char* q = jval(s, key); if (!q || *q != '[') return -1;
    q++; int n = 0;
    while (*q && *q != ']' && n < max) { char* e; double d = strtod(q, &e); if (e == q) { q++; continue; } out[n++] = d; q = e; }
    return n;
}

static double yaw_of(const dc_xform* x) {
    double q0 = x->quat[0], q1 = x->quat[1], q2 = x->quat[2], q3 = x->quat[3];
    return atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
}
static const char* mode_name(int m) {
    static const char* n[] = {"off", "hands_off", "torque", "angle", "path", "locked"};
    return m >= 0 && m <= 5 ? n[m] : "?";
}
static int mode_of(const char* s) {
    static const char* n[] = {"off", "hands_off", "torque", "angle", "path", "locked"};
    for (int i = 0; i < 6; i++) if (!strcmp(s, n[i])) return i;
    return -1;
}

void proto_init(proto_t* P, sim_t* S, int lockstep) {
    memset(P, 0, sizeof *P); P->S = S; P->lockstep = lockstep; P->log_every = 1; P->net.lfd = P->net.cfd = -1;
    S->ext_ctx = P; S->ext_hook = proto_hook;
    if (S->drv) dc_driver_get_params(S->drv, &P->defaults);
}
int proto_listen(proto_t* P, const char* bind_addr, int port) {
    if (!net_listen(&P->net, bind_addr, port)) return 0;
    P->use_net = 1; return 1;
}
void proto_close(proto_t* P) { if (P->log) fclose(P->log); P->log = NULL; if (P->use_net) net_close(&P->net); P->use_net = 0; }

static void metrics_reset(proto_t* P) { memset(&P->M, 0, sizeof P->M); P->M.t0 = P->S->sim_time; }

void proto_telemetry(proto_t* P, char* o, int max) {
    sim_t* S = P->S; dc_telemetry t; dc_get_telemetry(S->car, &t);
    dc_xform chp[DC_PART_COUNT]; dc_get_parts(S->car, chp, DC_PART_COUNT); dc_xform ch = chp[DC_PART_CHASSIS];
    dc_driver_state ds; memset(&ds, 0, sizeof ds); if (S->drv) dc_driver_get_state(S->drv, &ds);
    double d = 0, s = 0; int surf = 0; track_height(S->trk, ch.pos[0], ch.pos[1], &surf, NULL, &d, &s);
    double beta = atan2(t.vy, fmax(fabs(t.vx), 0.5));
    int n = snprintf(o, max,
        "{\"ok\":true,\"t\":%.4f,\"x\":%.4f,\"y\":%.4f,\"z\":%.4f,\"yaw_deg\":%.4f,\"vx\":%.4f,\"vy\":%.4f,\"speed_kmh\":%.3f,"
        "\"yaw_rate_dps\":%.4f,\"beta_deg\":%.4f,\"ax\":%.4f,\"ay\":%.4f,\"sw_deg\":%.4f,\"sw_rate_dps\":%.3f,"
        "\"column_nm\":%.4f,\"hand_nm\":%.4f,\"rack_mm\":%.4f,\"rack_force_n\":%.2f,\"assist_n\":%.2f,"
        "\"rim_cmd_deg\":%.4f,\"rim_cmd_delayed_deg\":%.4f,\"lat_err\":%.4f,\"head_err_deg\":%.4f,\"path_kappa\":%.6f,\"saturated\":%d,"
        "\"track_s\":%.3f,\"track_d\":%.4f,\"surface\":%d,\"gear\":%d,\"rpm\":%.1f,\"throttle\":%.3f,\"brake\":%.3f,"
        "\"driver\":\"%s\",\"engaged\":%d,\"takeover\":%d,",
        t.time, ch.pos[0], ch.pos[1], ch.pos[2], yaw_of(&ch) * DEG, t.vx, t.vy, t.speed * 3.6, t.yaw_rate * DEG, beta * DEG, t.ax, t.ay,
        t.sw_angle * DEG, t.sw_rate * DEG, t.column_torque, t.driver_torque, t.rack_pos * 1e3, t.rack_force, t.assist_force,
        ds.rim_cmd * DEG, ds.rim_cmd_delayed * DEG, ds.lat_err, ds.head_err * DEG, ds.kappa_path, ds.saturated,
        s, d, surf, t.gear, t.engine_rpm, S->snap.in.throttle, S->snap.in.brake,
        mode_name(S->drv_cmd.steer_mode), S->drv_on, S->drv_takeover);
    #define ARR4(name, expr) n += snprintf(o + n, max - n, "\"" name "\":[%.3f,%.3f,%.3f,%.3f],", expr(0), expr(1), expr(2), expr(3))
    #define FZ(i) t.w[i].fz
    #define FY(i) t.w[i].fy
    #define FX(i) t.w[i].fx
    #define MZ(i) t.w[i].mz
    #define AL(i) (t.w[i].slip_angle * DEG)
    #define KA(i) t.w[i].slip_ratio
    #define TR(i) (t.w[i].travel * 1e3)
    ARR4("fz", FZ); ARR4("fx", FX); ARR4("fy", FY); ARR4("mz", MZ); ARR4("alpha_deg", AL); ARR4("kappa", KA); ARR4("travel_mm", TR);
    n += snprintf(o + n, max - n, "\"toe_deg\":[%.4f,%.4f],\"energy_j\":%.2f,\"dissipated_j\":%.2f}",
                  t.w[0].toe * DEG, t.w[1].toe * DEG, t.kinetic + t.potential, t.dissipated);
}

static void metrics_json(proto_t* P, char* o, int max) {
    proto_metrics* M = &P->M; double n = M->n > 0 ? (double)M->n : 1;
    snprintf(o, max, "{\"ok\":true,\"duration\":%.4f,\"ticks\":%lld,\"rms_lat_err\":%.4f,\"max_lat_err\":%.4f,\"max_head_err_deg\":%.3f,"
        "\"max_column_nm\":%.3f,\"max_hand_nm\":%.3f,\"saturated_frac\":%.4f,\"max_beta_deg\":%.3f,\"max_yaw_rate_dps\":%.3f,"
        "\"max_sw_deg\":%.3f,\"max_sw_rate_dps\":%.2f,\"max_ay\":%.3f,\"distance\":%.2f,\"offtrack_frac\":%.4f,\"spin\":%d,\"takeover_ticks\":%lld}",
        P->S->sim_time - M->t0, M->n, sqrt(M->sum_lat2 / n), M->max_lat, M->max_head, M->max_col, M->max_hand, M->sat / n, M->max_beta,
        M->max_r, M->max_sw, M->max_swr, M->max_ay, M->dist, M->offtrack / n, M->max_beta > 45, M->takeover_ticks);
}

static void path_track(proto_t* P) {
    const track_t* T = P->S->trk; int n = T->n < PROTO_MAXPATH ? T->n : PROTO_MAXPATH;
    for (int i = 0; i < n; i++) { P->path_xy[2 * i] = T->px[i]; P->path_xy[2 * i + 1] = T->py[i]; }
    P->npath = n; P->path_closed = 1;
}

/* "drive": merge the given fields into the current command / params, then engage */
static int cmd_drive(proto_t* P, const char* L, char* err, int errmax) {
    sim_t* S = P->S; dc_driver_cmd c = S->drv_cmd; dc_driver_params p; dc_driver_get_params(S->drv, &p);
    char b[32]; double v; dc_driver_state ds; dc_driver_get_state(S->drv, &ds);
    if (jstr(L, "steer", b, sizeof b)) { int m = mode_of(b); if (m < 0) { snprintf(err, errmax, "unknown steer mode '%s'", b); return 0; } c.steer_mode = m; }
    if (jstr(L, "pedals", b, sizeof b)) {
        if (!strcmp(b, "off")) c.pedal_mode = DC_PED_OFF; else if (!strcmp(b, "direct")) c.pedal_mode = DC_PED_DIRECT;
        else if (!strcmp(b, "speed")) c.pedal_mode = DC_PED_SPEED; else { snprintf(err, errmax, "unknown pedal mode '%s'", b); return 0; }
    }
    if (jnum(L, "target_deg", &v)) c.steer_target = v / DEG;
    if (jnum(L, "target", &v)) c.steer_target = v;
    if (jnum(L, "torque", &v)) c.steer_torque = v;
    if (jnum(L, "wave_amp_deg", &v)) c.wave_amp = v / DEG;
    if (jnum(L, "wave_amp", &v)) c.wave_amp = v;
    if (jnum(L, "wave_freq", &v)) c.wave_freq = v;
    if (jnum(L, "wave_delay", &v)) c.wave_t0 = ds.time + v;      /* start the wave v seconds from now */
    if (jnum(L, "offset", &v)) c.path_offset = v;
    if (jnum(L, "throttle", &v)) c.throttle = v;
    if (jnum(L, "brake", &v)) c.brake = v;
    if (jnum(L, "handbrake", &v)) c.handbrake = v;
    if (jnum(L, "speed", &v)) c.speed = v;
    if (jnum(L, "speed_kmh", &v)) c.speed = v / 3.6;
    if (jnum(L, "arm_k", &v)) p.arm_k = v;
    if (jnum(L, "arm_d", &v)) p.arm_d = v;
    if (jnum(L, "arm_tmax", &v)) p.arm_tmax = v;
    if (jnum(L, "delay", &v)) { if (v < 0 || v > 0.5) { snprintf(err, errmax, "delay must be 0..0.5 s"); return 0; } p.delay = v; }
    if (jnum(L, "bandwidth_hz", &v)) p.bandwidth_hz = v;
    if (jnum(L, "rate_max_dps", &v)) p.rim_rate_max = v / DEG;
    if (jnum(L, "preview_time", &v)) p.preview_time = v;
    if (jnum(L, "preview_min", &v)) p.preview_min = v;
    if (jnum(L, "yaw_damp", &v)) p.yaw_damp = v;
    if (jnum(L, "catch_gain", &v)) p.catch_gain = v;
    if (jnum(L, "lock_frac", &v)) p.lock_frac = v;
    if (jnum(L, "lat_ki", &v)) p.lat_ki = v;
    if (jnum(L, "predict", &v)) p.predict = v;
    if (jnum(L, "speed_kp", &v)) p.speed_kp = v;
    if (jnum(L, "speed_ki", &v)) p.speed_ki = v;
    if (jnum(L, "brake_kp", &v)) p.brake_kp = v;
    if (c.steer_mode == DC_DRV_PATH && P->npath < 2) path_track(P), dc_driver_set_path(S->drv, P->path_xy, P->npath, P->path_closed);
    dc_driver_set_params(S->drv, &p);
    S->drv_cmd = c;
    double eng = 1; jnum(L, "engage", &eng);
    sim_driver_engage(S, eng != 0);
    return 1;
}

int proto_handle(proto_t* P, const char* L, char* o, int max) {
    sim_t* S = P->S; char cmd[32] = "", b[256], err[160] = ""; double v;
    while (*L == ' ' || *L == '\t') L++;
    if (!*L || *L == '#') { snprintf(o, max, "{\"ok\":true,\"note\":\"empty\"}"); return 0; }
    if (!jstr(L, "cmd", cmd, sizeof cmd)) { snprintf(o, max, "{\"ok\":false,\"error\":\"missing cmd\"}"); return 0; }
    if (!strcmp(cmd, "hello") || !strcmp(cmd, "info")) {
        const track_t* T = S->trk;
        int n = snprintf(o, max, "{\"ok\":true,\"protocol\":%d,\"name\":\"drivechain\",\"lockstep\":%d,\"tick_dt\":0.001,\"substep_hz\":%.0f,"
            "\"layout\":\"%s\",\"map\":\"%s\",\"track_length\":%.2f,\"half_width\":%.2f,\"s_arc1\":%.2f,\"s_str2\":%.2f,\"s_arc2\":%.2f,"
            "\"skidpad\":[%.3f,%.3f,%.3f],\"steer_ratio\":%.3f,\"steer_lock_deg\":%.2f,\"wheelbase\":%.4f,\"assist\":%d,\"spots\":[",
            PROTO_VERSION, P->lockstep, S->P.substep_hz, T->layout == TRACK_PROVING ? "proving" : T->layout == TRACK_MESH ? "mesh" : "circuit", track_layout_name(T), T->length, T->half_w,
            T->s_arc1, T->s_str2, T->s_arc2, T->skid_cx, T->skid_cy, T->skid_r, S->P.steer_ratio, S->P.steer_lock_deg,
            S->P.axle[0].hub[0] - S->P.axle[1].hub[0], S->P.assist);
        for (int i = 0; i < TRK_NSPOT; i++) {
            double x, y, yaw; const char* nm = track_spot(T, i, &x, &y, &yaw);
            if (nm) n += snprintf(o + n, max - n, "%s{\"name\":\"%s\",\"x\":%.3f,\"y\":%.3f,\"yaw_deg\":%.3f}", i ? "," : "", nm, x, y, yaw * DEG);
        }
        n += snprintf(o + n, max - n, "],\"cones\":[");
        for (int i = 0; i < T->ncone && n < max - 64; i++) n += snprintf(o + n, max - n, "%s[%.3f,%.3f]", i ? "," : "", T->cone_x[i], T->cone_y[i]);
        snprintf(o + n, max - n, "]}");
        return 0;
    }
    if (!strcmp(cmd, "reset")) {
        double speed = 0, x, y, yaw; jnum(L, "speed", &speed); if (jnum(L, "speed_kmh", &v)) speed = v / 3.6;
        if (jstr(L, "spot", b, sizeof b)) {
            int k = track_spot_by_name(S->trk, b);
            if (k < 0 || !track_spot(S->trk, k, &x, &y, &yaw)) { snprintf(o, max, "{\"ok\":false,\"error\":\"unknown spot '%s'\"}", b); return 0; }
            sim_reset_at(S, x, y, yaw, speed);
        } else if (jnum(L, "x", &x) && jnum(L, "y", &y)) {
            double yd = 0; jnum(L, "yaw_deg", &yd); sim_reset_at(S, x, y, yd / DEG, speed);
        } else { double s = 0, d = 0; jnum(L, "s", &s); jnum(L, "d", &d); sim_reset_on_track(S, s, d, speed); }
        for (int i = 0; i < 4; i++) dc_set_pressure(S->car, i, S->P.axle[i / 2].tyre.pressure);
        S->gear = speed > 25 ? 4 : (speed > 17 ? 3 : (speed > 8 ? 2 : 1)); if (jnum(L, "gear", &v)) S->gear = (int)v;
        if (S->drv) {   /* every reset starts from the same driver: default params, neutral command, disengaged */
            v = 0; jnum(L, "keep_driver", &v);
            if (!v) { dc_driver_set_params(S->drv, &P->defaults); memset(&S->drv_cmd, 0, sizeof S->drv_cmd); sim_driver_engage(S, 0); }
            dc_driver_reset(S->drv);
        }
        metrics_reset(P);
        proto_telemetry(P, o, max); return 0;
    }
    if (!strcmp(cmd, "path")) {
        char ty[32] = "track"; jstr(L, "type", ty, sizeof ty);
        if (!strcmp(ty, "track")) path_track(P);
        else if (!strcmp(ty, "circle")) {
            double cx = S->trk->skid_cx, cy = S->trk->skid_cy, r = S->trk->skid_r, dir = 1;
            jnum(L, "cx", &cx); jnum(L, "cy", &cy); jnum(L, "r", &r); jnum(L, "dir", &dir);
            int n = 720; for (int i = 0; i < n; i++) { double a = (dir >= 0 ? 1 : -1) * 2 * M_PI * i / n; P->path_xy[2 * i] = cx + r * sin(a); P->path_xy[2 * i + 1] = cy - r * cos(a); }
            P->npath = n; P->path_closed = 1;
        } else if (!strcmp(ty, "line")) {
            double x0 = 0, y0 = 0, yd = 0, len = 1000; jnum(L, "x", &x0); jnum(L, "y", &y0); jnum(L, "yaw_deg", &yd); jnum(L, "length", &len);
            int n = (int)fmin(PROTO_MAXPATH, len / 1.0 + 1); if (n < 2) n = 2;
            for (int i = 0; i < n; i++) { double s = len * i / (n - 1); P->path_xy[2 * i] = x0 + s * cos(yd / DEG); P->path_xy[2 * i + 1] = y0 + s * sin(yd / DEG); }
            P->npath = n; P->path_closed = 0;
        } else if (!strcmp(ty, "points")) {
            int n = jarr(L, "pts", P->path_xy, 2 * PROTO_MAXPATH);
            if (n < 4) { snprintf(o, max, "{\"ok\":false,\"error\":\"points path needs pts:[x0,y0,x1,y1,...] (>=2 points)\"}"); return 0; }
            P->npath = n / 2; v = 0; jnum(L, "closed", &v); P->path_closed = v != 0;
        } else { snprintf(o, max, "{\"ok\":false,\"error\":\"unknown path type '%s'\"}", ty); return 0; }
        dc_driver_set_path(S->drv, P->path_xy, P->npath, P->path_closed);
        snprintf(o, max, "{\"ok\":true,\"type\":\"%s\",\"points\":%d,\"closed\":%d}", ty, P->npath, P->path_closed); return 0;
    }
    if (!strcmp(cmd, "drive")) {
        if (!cmd_drive(P, L, err, sizeof err)) { snprintf(o, max, "{\"ok\":false,\"error\":\"%s\"}", err); return 0; }
        snprintf(o, max, "{\"ok\":true,\"driver\":\"%s\",\"pedals\":%d,\"engaged\":%d}", mode_name(S->drv_cmd.steer_mode), S->drv_cmd.pedal_mode, S->drv_on); return 0;
    }
    if (!strcmp(cmd, "engage")) { v = 1; jnum(L, "on", &v); sim_driver_engage(S, v != 0); snprintf(o, max, "{\"ok\":true,\"engaged\":%d}", S->drv_on); return 0; }
    if (!strcmp(cmd, "gear")) { if (jnum(L, "gear", &v)) S->gear = (int)v; snprintf(o, max, "{\"ok\":true,\"gear\":%d}", S->gear); return 0; }
    if (!strcmp(cmd, "step")) {
        if (!P->lockstep) { snprintf(o, max, "{\"ok\":false,\"error\":\"step is only valid on a lockstep host (drvhost); the demo runs in real time\"}"); return 0; }
        double n = 1; jnum(L, "n", &n); if (jnum(L, "seconds", &v)) n = v / 0.001 + 0.5;
        for (long k = 0; k < (long)n; k++) sim_tick(S, 0.001);
        proto_telemetry(P, o, max); return 0;
    }
    if (!strcmp(cmd, "get")) { proto_telemetry(P, o, max); return 0; }
    if (!strcmp(cmd, "metrics")) { metrics_json(P, o, max); return 0; }
    if (!strcmp(cmd, "metrics_reset")) { metrics_reset(P); snprintf(o, max, "{\"ok\":true}"); return 0; }
    if (!strcmp(cmd, "stream")) { v = 0; jnum(L, "hz", &v); P->stream_dt = v > 0 ? 1.0 / v : 0; P->stream_next = S->sim_time; snprintf(o, max, "{\"ok\":true,\"hz\":%.1f}", v); return 0; }
    if (!strcmp(cmd, "log")) {
        if (P->log) { fclose(P->log); P->log = NULL; }
        if (jstr(L, "file", b, sizeof b) && *b) {
            /* a TCP control port can be reached by any local process: only relative paths below the working directory */
            if (P->use_net && (b[0] == '/' || b[0] == '\\' || strchr(b, ':') || strstr(b, ".."))) { snprintf(o, max, "{\"ok\":false,\"error\":\"log file must be a relative path without '..'\"}"); return 0; }
            P->log = fopen(b, "w");
            if (!P->log) { snprintf(o, max, "{\"ok\":false,\"error\":\"cannot open log file\"}"); return 0; }
            v = 1; jnum(L, "every", &v); P->log_every = v >= 1 ? (int)v : 1;
            fprintf(P->log, "t,x,y,yaw_deg,vx,vy,yaw_rate_dps,beta_deg,ax,ay,sw_deg,sw_rate_dps,column_nm,hand_nm,rim_cmd_deg,rim_cmd_delayed_deg,"
                            "lat_err,head_err_deg,track_s,track_d,throttle,brake,gear,fz0,fz1,fz2,fz3,alpha0_deg,alpha1_deg,alpha2_deg,alpha3_deg,"
                            "mz0,mz1,toe0_deg,toe1_deg,saturated,engaged,takeover\n");
        }
        snprintf(o, max, "{\"ok\":true,\"logging\":%d}", P->log != NULL); return 0;
    }
    if (!strcmp(cmd, "set")) {   /* car parameter change, e.g. {"cmd":"set","key":"car.assist","value":"0"} -> car rebuilt in place */
        char key[96], val[96], *dot;
        if (!jstr(L, "key", key, sizeof key) || !(dot = strchr(key, '.'))) { snprintf(o, max, "{\"ok\":false,\"error\":\"set needs key:'section.name' and value\"}"); return 0; }
        if (!jstr(L, "value", val, sizeof val)) { if (jnum(L, "value", &v)) snprintf(val, sizeof val, "%.10g", v); else { snprintf(o, max, "{\"ok\":false,\"error\":\"missing value\"}"); return 0; } }
        *dot = 0; int ok = dc_set_param(&S->P, key, dot + 1, val);
        if (!ok) { snprintf(o, max, "{\"ok\":false,\"error\":\"unknown parameter %s.%s\"}", key, dot + 1); return 0; }
        sim_rebuild_car(S);
        snprintf(o, max, "{\"ok\":true,\"rebuilt\":true}"); return 0;
    }
    if (!strcmp(cmd, "quit")) { snprintf(o, max, "{\"ok\":true,\"bye\":true}"); P->quit = 1; return 1; }
    snprintf(o, max, "{\"ok\":false,\"error\":\"unknown cmd '%s'\"}", cmd); return 0;
}

void proto_hook(sim_t* S, int phase) {
    proto_t* P = (proto_t*)S->ext_ctx; if (!P) return;
    if (phase == 0) {
        if (!P->use_net || P->lockstep) return;
        static char line[1 << 16], out[1 << 14];
        int was = P->net.cfd >= 0;
        for (int k = 0; k < 16 && net_read_line(&P->net, line, sizeof line); k++) {
            proto_handle(P, line, out, sizeof out - 2); strcat(out, "\n"); net_send(&P->net, out);
        }
        int now = P->net.cfd >= 0;
        if (was && !now) { sim_driver_engage(S, 0); P->stream_dt = 0; }   /* client gone: hands back to the human */
        S->ext_clients = now;
        return;
    }
    /* phase 1: per-tick metrics, CSV log, stream */
    dc_telemetry t; dc_get_telemetry(S->car, &t);
    dc_driver_state ds; memset(&ds, 0, sizeof ds); if (S->drv) dc_driver_get_state(S->drv, &ds);
    proto_metrics* M = &P->M; M->n++;
    double beta = atan2(t.vy, fmax(fabs(t.vx), 0.5));
    M->sum_lat2 += ds.lat_err * ds.lat_err; M->max_lat = fmax(M->max_lat, fabs(ds.lat_err)); M->max_head = fmax(M->max_head, fabs(ds.head_err) * DEG);
    M->max_col = fmax(M->max_col, fabs(t.column_torque)); M->max_hand = fmax(M->max_hand, fabs(t.driver_torque));
    M->max_beta = fmax(M->max_beta, fabs(beta) * DEG); M->max_r = fmax(M->max_r, fabs(t.yaw_rate) * DEG);
    M->max_sw = fmax(M->max_sw, fabs(t.sw_angle) * DEG); M->max_swr = fmax(M->max_swr, fabs(t.sw_rate) * DEG); M->max_ay = fmax(M->max_ay, fabs(t.ay));
    M->dist += fabs(t.speed) * 0.001; M->sat += ds.saturated; M->takeover_ticks += S->drv_takeover;
    dc_xform chp[DC_PART_COUNT]; dc_get_parts(S->car, chp, DC_PART_COUNT); dc_xform ch = chp[DC_PART_CHASSIS];
    double d = 0, s = 0; int surf = 0; track_height(S->trk, ch.pos[0], ch.pos[1], &surf, NULL, &d, &s);
    if (surf == SURF_GRASS || surf == SURF_ROUGH || surf == SURF_GRAVEL) M->offtrack++;   /* chassis over grass / rough / gravel = off the road */
    P->tick++;
    if (P->log && P->tick % P->log_every == 0)
        fprintf(P->log, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.4f,%.3f,%.3f,%d,"
                        "%.1f,%.1f,%.1f,%.1f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.4f,%.4f,%d,%d,%d\n",
                t.time, ch.pos[0], ch.pos[1], yaw_of(&ch) * DEG, t.vx, t.vy, t.yaw_rate * DEG, beta * DEG, t.ax, t.ay, t.sw_angle * DEG, t.sw_rate * DEG,
                t.column_torque, t.driver_torque, ds.rim_cmd * DEG, ds.rim_cmd_delayed * DEG, ds.lat_err, ds.head_err * DEG, s, d,
                S->snap.in.throttle, S->snap.in.brake, t.gear, t.w[0].fz, t.w[1].fz, t.w[2].fz, t.w[3].fz,
                t.w[0].slip_angle * DEG, t.w[1].slip_angle * DEG, t.w[2].slip_angle * DEG, t.w[3].slip_angle * DEG,
                t.w[0].mz, t.w[1].mz, t.w[0].toe * DEG, t.w[1].toe * DEG, ds.saturated, S->drv_on, S->drv_takeover);
    if (!P->lockstep && P->use_net && P->stream_dt > 0 && S->sim_time >= P->stream_next) {
        static char out[1 << 14];
        P->stream_next += P->stream_dt; if (P->stream_next < S->sim_time) P->stream_next = S->sim_time + P->stream_dt;
        proto_telemetry(P, out, sizeof out - 2); strcat(out, "\n"); net_send(&P->net, out);
    }
}
