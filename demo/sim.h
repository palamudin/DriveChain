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
    int spot, spot_req;         /* teleport to track_spot(spot) when spot_req changes */
    int drv_toggle;             /* edge count: engage / release the plug-in driver (key G) */
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
    double raw_axis[8]; int raw_buttons, raw_naxes, hgear;   /* input diagnostics for mapping */
    int drv_on, drv_takeover, drv_mode, drv_clients; dc_driver_state drv_st;
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
    int gp_mode; double gp_k, gp_d, gp_tmax, gp_deadzone, gp_range_lo, gp_range_v, gp_expo;
    double auto_line, auto_ay, auto_speed_cap, auto_straight;
    double us_acc; long long steps_acc;
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
} sim_t;
int  sim_init(sim_t* S, cfg_t* cfg, track_t* trk, audio_t* audio, char* err, int errlen);
void sim_tick(sim_t* S, double dt);         /* one physics tick (dt = 1 ms) incl. input + FFB + audio */
void sim_reset_on_track(sim_t* S, double s, double d, double speed);
void sim_reset_at(sim_t* S, double x, double y, double yaw, double speed);
void sim_shutdown(sim_t* S);
int  sim_rebuild_car(sim_t* S);            /* after editing S->P: recreate the car at the current pose and speed */
void sim_driver_engage(sim_t* S, int on);   /* engage (1) / release (0) the plug-in driver */
#endif
