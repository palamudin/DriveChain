#include "input.h"
#ifndef DEMO_NO_SDL
#include <SDL.h>
#endif
#include <stdio.h>
#include <string.h>
#include <math.h>

#ifndef DEMO_NO_SDL
static SDL_Joystick* js; static SDL_GameController* gc; static SDL_Haptic* hap; static int eff = -1;
static SDL_HapticEffect E;
#endif
static struct {
    int steer_axis, thr_axis, brk_axis, clu_axis, hb_axis;
    int steer_inv, thr_inv, brk_inv, clu_inv, hb_inv;
    double dead, pedal_dead;
    int gear_btn[8], rev_btn, up_btn, dn_btn, hb_btn;
    double ffb_gain, ffb_min, ffb_max_nm, ffb_smooth_hz; int ffb_inv;
    int prev_up, prev_dn;
    double ffb_filt;
} C;

static double pedal(double v, int inv, double dead) {   /* raw axis -1..1 -> 0..1 */
    double p = (v + 1) * 0.5; if (inv) p = 1 - p;
    p = (p - dead) / (1 - 2 * dead); return p < 0 ? 0 : (p > 1 ? 1 : p);
}

int input_open(const cfg_t* cfg, input_state* st) {
    memset(st, 0, sizeof *st); st->hgear = -99;
    C.steer_axis = cfg_int(cfg, "input.steer_axis", 0); C.thr_axis = cfg_int(cfg, "input.throttle_axis", 1);
    C.brk_axis = cfg_int(cfg, "input.brake_axis", 2); C.clu_axis = cfg_int(cfg, "input.clutch_axis", 3);
    C.hb_axis = cfg_int(cfg, "input.handbrake_axis", -1);
    C.steer_inv = cfg_int(cfg, "input.steer_invert", 0); C.thr_inv = cfg_int(cfg, "input.throttle_invert", 1);
    C.brk_inv = cfg_int(cfg, "input.brake_invert", 1); C.clu_inv = cfg_int(cfg, "input.clutch_invert", 1);
    C.hb_inv = cfg_int(cfg, "input.handbrake_invert", 0);
    C.dead = cfg_num(cfg, "input.steer_deadzone", 0.0); C.pedal_dead = cfg_num(cfg, "input.pedal_deadzone", 0.02);
    char k[64];
    for (int g = 1; g <= 7; g++) { snprintf(k, sizeof k, "input.gear%d_button", g); C.gear_btn[g] = cfg_int(cfg, k, -1); }
    C.rev_btn = cfg_int(cfg, "input.reverse_button", -1);
    C.up_btn = cfg_int(cfg, "input.shift_up_button", -1); C.dn_btn = cfg_int(cfg, "input.shift_down_button", -1);
    C.hb_btn = cfg_int(cfg, "input.handbrake_button", -1);
    C.ffb_gain = cfg_num(cfg, "ffb.gain", 1.0); C.ffb_min = cfg_num(cfg, "ffb.min_force", 0.0);
    C.ffb_max_nm = cfg_num(cfg, "ffb.max_torque_nm", 8.0); C.ffb_inv = cfg_int(cfg, "ffb.invert", 0);
    C.ffb_smooth_hz = cfg_num(cfg, "ffb.smoothing_hz", 0.0);
    int any_gear = 0; for (int g = 1; g <= 7; g++) if (C.gear_btn[g] >= 0) any_gear = 1;
    if (any_gear) st->hgear = 0;
#ifdef DEMO_NO_SDL
    return 0;
#else
    const char* mode = cfg_str(cfg, "input.device", "auto");
    if (!strcmp(mode, "keyboard")) return 0;
    SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
    if (SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_GAMECONTROLLER | SDL_INIT_HAPTIC) != 0) {
        if (SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_GAMECONTROLLER) != 0) { fprintf(stderr, "SDL init failed: %s\n", SDL_GetError()); return 0; }
    }
    const char* want = cfg_str(cfg, "input.device_name", "");
    int n = SDL_NumJoysticks(), pick = -1;
    for (int i = 0; i < n; i++) {
        const char* nm = SDL_JoystickNameForIndex(i);
        printf("[input] joystick %d: %s%s\n", i, nm ? nm : "?", SDL_IsGameController(i) ? " (game controller)" : "");
        if (*want && nm && strstr(nm, want)) { pick = i; break; }
        if (pick < 0) {
            int isgc = SDL_IsGameController(i);
            if (!strcmp(mode, "wheel") && isgc) continue;
            if (!strcmp(mode, "gamepad") && !isgc) continue;
            pick = i;
        }
    }
    if (pick < 0) { printf("[input] no device -> keyboard\n"); return 0; }
    int as_gc = SDL_IsGameController(pick) && strcmp(mode, "wheel") != 0;
    if (as_gc) { gc = SDL_GameControllerOpen(pick); js = SDL_GameControllerGetJoystick(gc); st->kind = 2; }
    else { js = SDL_JoystickOpen(pick); st->kind = 1; }
    if (!js) { st->kind = 0; return 0; }
    snprintf(st->name, sizeof st->name, "%s", SDL_JoystickName(js));
    st->naxes = SDL_JoystickNumAxes(js); st->nbuttons = SDL_JoystickNumButtons(js);
    printf("[input] using %s: %s, %d axes, %d buttons\n", st->kind == 2 ? "gamepad" : "wheel/joystick", st->name, st->naxes, st->nbuttons);
    if (cfg_int(cfg, "ffb.enabled", 1) && st->kind == 1 && SDL_JoystickIsHaptic(js) == 1) {
        hap = SDL_HapticOpenFromJoystick(js);
        if (hap && (SDL_HapticQuery(hap) & SDL_HAPTIC_CONSTANT)) {
            if (SDL_HapticQuery(hap) & SDL_HAPTIC_AUTOCENTER) SDL_HapticSetAutocenter(hap, 0);   /* the car does the centring */
            if (SDL_HapticQuery(hap) & SDL_HAPTIC_GAIN) SDL_HapticSetGain(hap, 100);
            memset(&E, 0, sizeof E);
            E.type = SDL_HAPTIC_CONSTANT;
            E.constant.direction.type = SDL_HAPTIC_CARTESIAN; E.constant.direction.dir[0] = 1;  /* +x: force to the right */
            E.constant.length = SDL_HAPTIC_INFINITY; E.constant.level = 0;
            eff = SDL_HapticNewEffect(hap, &E);
            if (eff >= 0 && SDL_HapticRunEffect(hap, eff, 1) == 0) { st->has_ffb = 1; st->ffb_ok = 1; printf("[ffb] constant force OK\n"); }
            else printf("[ffb] effect failed: %s\n", SDL_GetError());
        } else printf("[ffb] device has no constant-force effect\n");
    }
    return st->kind;
#endif
}

void input_poll(input_state* st) {
#ifndef DEMO_NO_SDL
    if (!js) return;
    SDL_JoystickUpdate();
    if (st->kind == 2) {
        double sx = SDL_GameControllerGetAxis(gc, SDL_CONTROLLER_AXIS_LEFTX) / 32767.0;
        st->steer = -sx;
        st->throttle = SDL_GameControllerGetAxis(gc, SDL_CONTROLLER_AXIS_TRIGGERRIGHT) / 32767.0;
        st->brake = SDL_GameControllerGetAxis(gc, SDL_CONTROLLER_AXIS_TRIGGERLEFT) / 32767.0;
        st->handbrake = SDL_GameControllerGetButton(gc, SDL_CONTROLLER_BUTTON_B) ? 1 : 0;
        st->clutch = SDL_GameControllerGetButton(gc, SDL_CONTROLLER_BUTTON_X) ? 1 : 0;
        int up = SDL_GameControllerGetButton(gc, SDL_CONTROLLER_BUTTON_RIGHTSHOULDER), dn = SDL_GameControllerGetButton(gc, SDL_CONTROLLER_BUTTON_LEFTSHOULDER);
        if (up && !C.prev_up) st->shift_up++; if (dn && !C.prev_dn) st->shift_down++;
        C.prev_up = up; C.prev_dn = dn;
        return;
    }
    for (int a = 0; a < st->naxes && a < 8; a++) st->axis[a] = SDL_JoystickGetAxis(js, a) / 32767.0;
    st->buttons_raw = 0;
    for (int b = 0; b < st->nbuttons && b < 32; b++) if (SDL_JoystickGetButton(js, b)) st->buttons_raw |= 1 << b;
    double s = C.steer_axis >= 0 && C.steer_axis < st->naxes ? st->axis[C.steer_axis] : 0;
    if (fabs(s) < C.dead) s = 0; else s = (s - (s > 0 ? C.dead : -C.dead)) / (1 - C.dead);
    st->steer = C.steer_inv ? s : -s;     /* SDL: + = right; ours: + = left */
    st->throttle = C.thr_axis >= 0 && C.thr_axis < st->naxes ? pedal(st->axis[C.thr_axis], C.thr_inv, C.pedal_dead) : 0;
    st->brake = C.brk_axis >= 0 && C.brk_axis < st->naxes ? pedal(st->axis[C.brk_axis], C.brk_inv, C.pedal_dead) : 0;
    st->clutch = C.clu_axis >= 0 && C.clu_axis < st->naxes ? pedal(st->axis[C.clu_axis], C.clu_inv, C.pedal_dead) : 0;
    st->handbrake = C.hb_axis >= 0 && C.hb_axis < st->naxes ? pedal(st->axis[C.hb_axis], C.hb_inv, C.pedal_dead) : 0;
    if (C.hb_btn >= 0 && SDL_JoystickGetButton(js, C.hb_btn)) st->handbrake = 1;
    if (st->hgear != -99) {
        int g = 0;
        for (int k = 1; k <= 7; k++) if (C.gear_btn[k] >= 0 && SDL_JoystickGetButton(js, C.gear_btn[k])) g = k;
        if (C.rev_btn >= 0 && SDL_JoystickGetButton(js, C.rev_btn)) g = -1;
        st->hgear = g;
    }
    int up = C.up_btn >= 0 && SDL_JoystickGetButton(js, C.up_btn), dn = C.dn_btn >= 0 && SDL_JoystickGetButton(js, C.dn_btn);
    if (up && !C.prev_up) st->shift_up++; if (dn && !C.prev_dn) st->shift_down++;
    C.prev_up = up; C.prev_dn = dn;
#else
    (void)st;
#endif
}

void input_ffb(input_state* st, double tq, double dt) {
    /* level = gain * T_column / T_max, clipped; min_force lifts small forces over the wheel's own friction
       deadband (sign-preserving, continuous: level' = sgn*(min + (1-min)*|level|)). Sign: column torque > 0 turns the
       wheel left (CCW) = force to the left = negative SDL x. */
    double lv = C.ffb_gain * tq / (C.ffb_max_nm > 0.1 ? C.ffb_max_nm : 0.1);
    if (C.ffb_smooth_hz > 0) { double a = 1 - exp(-2 * M_PI * C.ffb_smooth_hz * dt); C.ffb_filt += a * (lv - C.ffb_filt); lv = C.ffb_filt; }
    double m = fabs(lv); if (m > 1) m = 1;
    if (m > 1e-4 && C.ffb_min > 0) m = C.ffb_min + (1 - C.ffb_min) * m;
    lv = (lv >= 0 ? 1 : -1) * m;
    if (C.ffb_inv) lv = -lv;
    st->ffb_out = lv;
#ifndef DEMO_NO_SDL
    if (hap && eff >= 0) {
        E.constant.level = (Sint16)(-lv * 32767);
        SDL_HapticUpdateEffect(hap, eff, &E);
    }
#endif
}
void input_rumble(input_state* st, double lo, double hi) {
#ifndef DEMO_NO_SDL
    if (gc) SDL_GameControllerRumble(gc, (Uint16)(fmin(lo, 1) * 65535), (Uint16)(fmin(hi, 1) * 65535), 60);
#endif
    (void)st;
}
void input_close(void) {
#ifndef DEMO_NO_SDL
    if (hap) { if (eff >= 0) SDL_HapticStopEffect(hap, eff); SDL_HapticClose(hap); hap = NULL; }
    if (gc) SDL_GameControllerClose(gc); else if (js) SDL_JoystickClose(js);
    gc = NULL; js = NULL;
    SDL_Quit();
#endif
}
