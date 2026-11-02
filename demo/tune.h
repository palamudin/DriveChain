/* tune.h - the garage tuning model shared with the Unity GUI package: demo/ui/tune_spec.json (pages, rows, ranges,
   detents, units, live info-panel formulas), a comment-preserving ini document (car files, setups), the expression
   evaluator for the info panels (same language as tools/gen_tune_spec.py and the C# DcTuneExpr), and the write-through
   of a tune: user/car.ui.ini (base car file + edited keys) and user/drivechain_overrides.ini (demo keys, [car] ini,
   [ui] base_car), read by main.c after demo.ini and before --set. No raylib, no physics: unit-testable. */
#ifndef DEMO_TUNE_H
#define DEMO_TUNE_H
#include "cfg.h"

/* ---- ini document (keeps every line; set() replaces a value in place, keeping the comment) ---- */
typedef struct { char** line; int n, cap; } inidoc;
int  ini_load(inidoc* d, const char* path);          /* 1 ok */
void ini_free(inidoc* d);
int  ini_get(const inidoc* d, const char* sec, const char* key, char* out, int n);   /* key may be "name#i" */
void ini_set(inidoc* d, const char* sec, const char* key, const char* val);
int  ini_save(const inidoc* d, const char* path);
int  ini_lut(const inidoc* d, const char* sec, double* x, double* y, int max);        /* numeric "x = y" lines of a section, sorted */

/* ---- spec ---- */
enum { TR_CAR = 0, TR_DEMO = 1, TR_BIAS = 2 };
typedef struct {
    char k[64], k2[64], l[48], u[24], id[32];
    double min, max, st, fine, s, lim[2]; int has_lim, d, t, ro, rs;
    int nopt; char opt[8][20], val[8][24];
    char info[300];
} tune_row;
typedef struct { char l[48], e[200], u[16]; int d, est, has_good, axle; double good[2]; } tune_out;
typedef struct {
    char id[20], title[48], unity[4], panel[16], note[300], group[16];
    tune_row row[14]; int nrow;
    tune_out out[16]; int nout;
} tune_page;
typedef struct { char n[16]; char e[200]; } tune_var;
typedef struct {
    tune_var var[128]; int nvar;
    tune_page page[20]; int npage;
    double g, rho, crr, eta;
    char ov_when_k[4][48]; double ov_when[4][2]; int nov_when;
    char ov_k[8][32], ov_v[8][24]; int nov;
} tune_spec;
int tune_spec_load(tune_spec* S, const char* path, char* err, int n);   /* 1 ok */
const char* tune_spec_find(void);                                       /* first existing spec path (exe dir, repo) */

/* ---- live values ---- */
typedef struct {
    const tune_spec* S;
    inidoc base;        /* stock car file (ghost marker) */
    inidoc saved;       /* car file as loaded at menu open (user/car.ui.ini, else the base) = delta reference */
    char base_path[256];
    struct { char sec[16], key[48]; double v; } ed[96]; int ned;   /* current edits (car keys) */
    double var[128]; int nvar_ok;
    int overlay_on;
    cfg_t* cfg;         /* demo keys (rows with t = demo) */
    cfg_t demo_saved;   /* demo keys at open */
    int dirty_car, dirty_demo;
} tune_state;
int    tune_open(tune_state* T, const tune_spec* S, const char* base_car, const char* saved_car, cfg_t* cfg, char* err, int n);
void   tune_close(tune_state* T);
/* raw (ini-unit) values of a car key "sec.key[#i]": stock = base car, saved = at open, cur = with edits */
double tune_key(const tune_state* T, const char* k, int which);   /* which 0 cur, 1 saved, 2 stock */
void   tune_key_set(tune_state* T, const char* k, double v);
/* rows in display units (display = ini x s); k2 = 1 for the rear / second key of a pair */
double tune_row_get(const tune_state* T, const tune_row* r, int k2, int which);
void   tune_row_set(tune_state* T, const tune_row* r, int k2, double disp);
double tune_row_quant(const tune_row* r, double disp);             /* clamp + snap to the fine step grid */
void   tune_eval(tune_state* T);                                   /* recompute vars */
double tune_expr(const tune_state* T, const char* e);              /* NaN on error / missing */
int    tune_changed(const tune_state* T);                          /* any value != saved */
void   tune_revert(tune_state* T, int to_stock);
/* write user/car.ui.ini + user/drivechain_overrides.ini; dir = "user" */
int    tune_write(tune_state* T, const char* dir, char* car_out, int n, char* err, int en);
int    tune_overrides_load(cfg_t* cfg, const char* path);          /* apply an overrides file onto cfg (1 = found) */
/* setups: user/setups/<name>.ini = a copy of car.ui.ini + [ui] header */
int    tune_setup_save(tune_state* T, const char* dir, const char* name, char* err, int en);
int    tune_setup_load(tune_state* T, const char* path, char* err, int en);
const char* tune_fmt(double v, int d, char* buf, int n);          /* "-" for NaN */
#endif
