/* sim.h - physics side of the demo: car, track, input mapping, gearbox/auto-clutch, autopilot, FFB, audio feed.
   Runs on its own thread at 1 kHz wall-clock (2 kHz substeps inside the core), decoupled from rendering. */
#ifndef DEMO_SIM_H
#define DEMO_SIM_H
#include "drivechain.h"
#include "track.h"
#include "input.h"
#include "audio.h"
#include "cfg.h"
#include <stdio.h>
#define SIM_MAXSEG 64
typedef struct {            /* render thread -> physics */
    int kb_left, kb_right, kb_up, kb_down, kb_clutch, kb_hand;
    int shift_up, shift_down;   /* edge counts */
    int reset, puncture, toggle_assist, pause, autopilot;
    int spot, spot_req;         /* teleport to track_spot(spot) when spot_req changes */
    int drv_toggle;             /* edge count: engage / release the plug-in driver (key G) */
    int rec_toggle;             /* edge count: input record start / stop, or replay stop / restart (key F9) */
} sim_ctl;
/* input record / replay (key F9, [replay] file = ...): a recording starts from a freshly built car placed at the
   current pose and forward speed (flying restart) and stores, per tick, the final dc_input handed to dc_step plus
   puncture events. Replaying it on the same build, car, map, tick rate and backend reproduces the run bit for bit;
   the end-state hash of the recording is checked when the replay ends. */
#define SIM_REC_MAGIC "DCREC01"
typedef struct {
    char magic[8]; int version, nticks;
    double tick_dt, substep_hz, x, y, yaw, speed;
    int gear, model, solver, bricks, layout, threads, backend, bake_bits;
    double track_len;
    unsigned long long end_hash, param_hash;
    char map[96], car[160], setup[32], backend_cfg[16];
} sim_rec_hdr;
typedef struct { dc_input in; int ev, arg; } sim_rec_tick;   /* ev 1 = puncture wheel arg (applied before the step) */
typedef struct {            /* physics -> render */
    dc_telemetry t;
    dc_xform parts[DC_PART_COUNT];
    dc_seg seg[SIM_MAXSEG]; int nseg;
    dc_input in;
    int surf[4];
    double ffb_level, us_per_substep, rt_factor, sim_time, beta; int solver;
    int gear_req, assist, autopilot, paused, device_kind, has_ffb;
    char device[128];
    double raw_axis[8]; int raw_buttons, raw_naxes, hgear;   /* input diagnostics for mapping */
    int drv_on, drv_takeover, drv_mode, drv_clients; dc_driver_state drv_st;
    int rec_on, rep_on, rep_done, rep_match, rep_i, rep_n, rec_n; char rec_msg[200], rep_path[256], rec_path[256];   /* record / replay status */
} sim_snap;
typedef struct sim_s {
    cfg_t* cfg; track_t* trk; audio_t* audio;
    dc_car_params P; dc_car* car;
    input_state dev;
    sim_ctl ctl, ctl_seen;
    double sim_time, kb_sw;
    int gear, shift_timer_ms;
    double rotation_rad, hands_k, hands_d, hands_tmax;
    int auto_clutch, kb_hands_off, autopilot, auto_off_ms;
    /* gamepad steering layer: mode 1 "arms" (default) = the stick is the target of a compliant, finite-strength
       grip (stiffness/damping/max torque of relaxed arms on the rim), always engaged, so the tyres' SAT can still
       turn the rim against the hands (self-steer emerges) while the hands' damping takes out the overshoot.
       mode 0 "raw" = legacy: stiff-ish angle servo, and inside the stick deadzone the hands come off the wheel. */
    int gp_mode; double gp_k, gp_d, gp_tmax, gp_deadzone, gp_grip_t, gp_grip;
    double auto_line, auto_ay, auto_speed_cap, auto_straight;
    double us_acc; long long steps_acc;
    double tick_dt;                       /* host tick (physics.tick_hz; 1 kHz when unset, 240 Hz in demo.ini) */
    double tick_step_us; int tick_n, threads;
    int offload_patch;               /* backend in use for the patch stage (DC_OFFLOAD_*) */
    struct roadbake* bake; double bake_sig[5];   /* [track] bake: prewarm-baked road lattice (demo/roadbake.h, one per process), NULL = live road */
    double shift_left_s;                  /* shift timer remainder below 1 ms (non-ms ticks) */
    double perf_t, perf_wall, perf_step, perf_worst; long long perf_sub, perf_ticks; int perf_log;
    sim_snap snap;
    /* external driver (headless harness / tests): when ext_steer / ext_pedals are set, these fields of ext_in replace
       the device mapping for that channel. The demo never sets them. */
    int ext_steer, ext_pedals; dc_input ext_in;
    /* plug-in driver (dc_driver, see drivechain.h): when drv_on, it drives the steering (finite-strength arms) and/or
       pedals. Live human takeover: any deliberate human input (drive keys, stick/triggers, pedals, or a wheel turned
       away from the simulated rim) switches it off and sets drv_takeover; key G (or the "engage" command) re-engages. */
    dc_driver* drv; dc_driver_cmd drv_cmd; int drv_on, drv_takeover, drv_takeover_enabled;
    double drv_takeover_stick, drv_takeover_wheel, drv_takeover_pedal;
    /* external control hook (JSON-lines protocol, demo/drvproto.c): phase 0 before input mapping, 1 after the step */
    void (*ext_hook)(struct sim_s* S, int phase); void* ext_ctx; int ext_clients;
    /* record / replay (owned by the physics thread; the render thread reads the scalars under the sim lock) */
    char rec_dir[400];                    /* where F9 recordings go (demo: the exe folder) */
    sim_rec_tick* rec_buf; int rec_on, rec_n, rec_cap, rec_ev, rec_arg; sim_rec_hdr rec_h; char rec_path[600];
    sim_rec_tick* rep_buf; int rep_on, rep_i, rep_n, rep_done, rep_match; sim_rec_hdr rep_h; char rep_path[600];
    unsigned long long rep_hash;
    char rec_msg[200];                    /* last record / replay status line (HUD) */
#ifdef DC_PROF
    double pf_poll0, pf_poll1, pf_step0, pf_step1, pf_ffb1;   /* tick-phase stamps (us) */
#endif
} sim_t;
int  sim_apply_setup(dc_car_params* P, const char* setup, char* err, int errlen);   /* car.setup: default (propBR + propD where applicable) | legacy */
int  sim_init(sim_t* S, cfg_t* cfg, track_t* trk, audio_t* audio, char* err, int errlen);
void sim_tick(sim_t* S, double dt);         /* one physics tick (dt = S->tick_dt) incl. input + FFB + audio */
void sim_reset_on_track(sim_t* S, double s, double d, double speed);
void sim_reset_at(sim_t* S, double x, double y, double yaw, double speed);
void sim_shutdown(sim_t* S);
int  sim_rebuild_car(sim_t* S);            /* after editing S->P: recreate the car at the current pose and speed */
void sim_driver_engage(sim_t* S, int on);   /* engage (1) / release (0) the plug-in driver */
int  sim_replay_load(sim_t* S, const char* path, char* err, int errlen);   /* load + start a replay (also [replay] file) */
void sim_record_start(sim_t* S);           /* physics thread (F9 edge); headless tools may call it directly */
void sim_record_stop(sim_t* S);            /* writes the file to rec_dir (or [replay] record_file) */
unsigned long long sim_state_hash(sim_t* S);   /* FNV-1a over every body pose and the wheel forces */
#ifdef DC_PROF
/* profiling builds only (-DDC_PROF): per-tick / per-physics-batch / per-frame records in rings preallocated by
   prof_init (no allocation while running), dumped as CSV by prof_dump. Each ring has a single writer thread. */
typedef struct { double tw, simt, tick_us, poll_us, input_us, step_us, ffb_us, post_us; float st_us[DCP_S_N], wh_us[4], whs_us[DCP_W_TOTAL];   /* per-wheel total; per-lap sum over wheels */
                 int nsub, solves, resolves, sweeps, sweep_max, capped, nbx_max, pool_runs, pool_wakes, nslide;
                 float pool_wait_us, pool_wait_max_us, slide, slip_max_deg, speed, rack_n, column_nm, mz_fl, mz_fr, sw_deg, fz[4], px, py; int surf_mask, tags; } prof_tick_t;
enum { PT_TAG_KERB = 1, PT_TAG_LOAD = 2, PT_TAG_STEER = 4 };
typedef struct { double tw; float sleep_req_us, sleep_us, lockwait_max_us, batch_us, backlog_ms; int ticks, dropped, broke; } prof_batch_t;
typedef struct { double tw; float frame_us, lock_us, draw_us; double sim_time; int new_ticks; } prof_frame_t;
double prof_now_us(void);
void prof_init(int cap_ticks);
void prof_init_ring(int cap_ticks);       /* perf overlay: a wrapping tick ring (allocated once) */
long long prof_ring_copy(prof_tick_t* dst, int max);   /* newest <= max ticks, oldest first (any thread) */
void prof_write_ticks_csv(FILE* f, const prof_tick_t* p, int n);
const prof_tick_t* prof_last(void);       /* newest tick record (NULL if none); prof_rewind() empties the rings (measurement tools) */
void prof_rewind(void);
void prof_batch(const prof_batch_t* b);
void prof_frame(const prof_frame_t* f);
void prof_dump(const char* prefix);        /* <prefix>_ticks.csv, _batches.csv, _frames.csv */
#endif
#endif
