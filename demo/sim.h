/* sim.h - physics side of the demo: car, track, input mapping, gearbox/auto-clutch, autopilot, FFB, audio feed.
   Runs on its own thread at 1 kHz wall-clock (2 kHz substeps inside the core), decoupled from rendering. */
#ifndef DEMO_SIM_H
#define DEMO_SIM_H
#include "drivechain.h"
#include "track.h"
#include "input.h"
#include "audio.h"
#include "cfg.h"
#define SIM_MAXSEG 64
typedef struct {            /* render thread -> physics */
    int kb_left, kb_right, kb_up, kb_down, kb_clutch, kb_hand;
    int shift_up, shift_down;   /* edge counts */
    int reset, puncture, toggle_assist, pause, autopilot;
} sim_ctl;
typedef struct {            /* physics -> render */
    dc_telemetry t;
    dc_xform parts[DC_PART_COUNT];
    dc_seg seg[SIM_MAXSEG]; int nseg;
    dc_input in;
    int surf[4];
    double ffb_level, us_per_substep, rt_factor, sim_time, beta;
    int gear_req, assist, autopilot, paused, device_kind, has_ffb;
    char device[128];
} sim_snap;
typedef struct {
    cfg_t* cfg; track_t* trk; audio_t* audio;
    dc_car_params P; dc_car* car;
    input_state dev;
    sim_ctl ctl, ctl_seen;
    double sim_time, kb_sw;
    int gear, shift_timer_ms;
    double rotation_rad, hands_k, hands_d, hands_tmax;
    int auto_clutch, kb_hands_off, autopilot, auto_off_ms;
    double auto_line, auto_ay, auto_speed_cap, auto_straight;
    double us_acc; long long steps_acc;
    sim_snap snap;
} sim_t;
int  sim_init(sim_t* S, cfg_t* cfg, track_t* trk, audio_t* audio, char* err, int errlen);
void sim_tick(sim_t* S, double dt);         /* one physics tick (dt = 1 ms) incl. input + FFB + audio */
void sim_reset_on_track(sim_t* S, double s, double d, double speed);
void sim_reset_at(sim_t* S, double x, double y, double yaw, double speed);
void sim_shutdown(sim_t* S);
#endif
