/* menu.h - the garage / pause menu of the raylib demo (same visual language as the Unity DriveChainUI package:
   smoked glass over the blurred scene, white 1 px edge, gold top hairline, Michroma / Inter / JetBrains Mono).
   Screens: title, hub, car, map, tuning pages (sliders + live info panels from demo/ui/tune_spec.json), setups,
   settings, pause. The menu never touches the physics: it edits cfg keys and writes user/car.ui.ini +
   user/drivechain_overrides.ini (demo/tune.c); main.c applies them (sim re-init or sim_retune). */
#ifndef DEMO_MENU_H
#define DEMO_MENU_H
#include "raylib.h"
#include "sim.h"

typedef enum { MA_NONE = 0, MA_DRIVE, MA_RESUME, MA_APPLY, MA_RESTART, MA_TO_GARAGE, MA_QUIT, MA_LIVE } menu_action;

typedef struct {
    cfg_t* cfg;
    char (*maps)[128]; int nmaps; int* cur_map;   /* map ids (proving, circuit, tracks/<name>) */
    const char* map_title;                       /* current map display name */
    int dev_kind;                                /* SIM.dev.kind: 0 none, 1 wheel, 2 gamepad */
    const sim_snap* view;                        /* latest snapshot (telemetry on the pause screen, wheel axes) */
    const dc_car_params* P;
} menu_host;

void menu_init(menu_host* H);
void menu_open(int in_game);          /* 0 = garage (title screen), 1 = pause, 2 = garage hub */
void menu_close(void);
int  menu_is_open(void);
int  menu_in_game(void);
int  menu_garage_cam(void);           /* the 3D view should orbit the parked car */
int  menu_need_reinit(void);          /* after MA_DRIVE / MA_RESTART: car, map or physics settings changed */
int  menu_map_choice(void);           /* map index picked in the menu (-1 = unchanged) */
void menu_after_init(void);           /* main re-initialised the sim: reload the tune state from the written files */
const char* menu_hint(void);
menu_action menu_update(float dt);
/* scene = the frame already rendered this frame (render texture), drawn under the menu and blurred under the glass */
void menu_draw(RenderTexture2D* scene, int sw, int sh);
/* screenshot setup: "title" | "hub" | "car" | "map" | "setups" | "pause" | "tune.<page>[/row[=value]]" | "settings.<page>[/row[=value]]" */
int  menu_shot(const char* spec);
#endif
