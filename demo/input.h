/* input.h - SDL2 joystick / game-controller input and haptic (constant-force) FFB for wheel rigs.
   All SDL calls are made from ONE thread (the physics thread), which also owns the FFB update loop. */
#ifndef DEMO_INPUT_H
#define DEMO_INPUT_H
#include "cfg.h"
typedef struct {
    int kind;                       /* 0 none, 1 wheel/joystick, 2 gamepad */
    char name[128];
    double steer;                   /* -1..1 (full device rotation), + = left */
    double throttle, brake, clutch, handbrake;
    int hgear;                      /* H-shifter: -1 R, 0 neutral, 1..7; -99 = no H-shifter configured */
    int shift_up, shift_down;       /* edge counts since last read */
    int buttons_raw;                /* bitmask of first 32 buttons (diagnostics) */
    int naxes, nbuttons;
    double axis[8];
    int has_ffb, ffb_ok;
    double ffb_out;                 /* last commanded level -1..1 */
} input_state;
int  input_open(const cfg_t* cfg, input_state* st);       /* returns kind */
void input_poll(input_state* st);
void input_ffb(input_state* st, double column_torque_nm, double dt); /* constant force from physics column torque */
void input_rumble(input_state* st, double lo, double hi);
void input_close(void);
#endif
