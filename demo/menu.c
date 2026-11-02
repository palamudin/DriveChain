#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-const-variable"
#pragma GCC diagnostic ignored "-Wformat-truncation"
/* menu.c - garage / pause menu for the raylib demo. See menu.h. All layout is on a 1920 x 1080 virtual canvas
   (letterboxed by a uniform scale), fonts are rasterised at the on-screen pixel size, the glass panels sample a
   gaussian-blurred copy of the frame. Visual language = private/ui sketches + the Unity DriveChainUI package. */
#include "menu.h"
#include "tune.h"
#include "rlgl.h"
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/stat.h>

/* ================================================================== palette */
#define RGBA(r, g, b, a) ((Color){r, g, b, a})
static const Color INK0 = RGBA(5, 7, 10, 255), INK1 = RGBA(11, 14, 18, 255), INK2 = RGBA(18, 22, 28, 255), INK3 = RGBA(26, 31, 39, 255);
static const Color GR0 = RGBA(236, 238, 241, 255), GR1 = RGBA(185, 191, 199, 255), GR2 = RGBA(125, 133, 143, 255), GR3 = RGBA(74, 81, 90, 255);
static const Color GO0 = RGBA(236, 212, 156, 255), GO1 = RGBA(201, 165, 92, 255), GO2 = RGBA(139, 111, 55, 255);
static const Color GN0 = RGBA(108, 200, 157, 255), GN1 = RGBA(60, 154, 112, 255);
static const Color RD0 = RGBA(239, 122, 111, 255), RD1 = RGBA(200, 70, 63, 255);
static const Color BL0 = RGBA(141, 184, 232, 255), BL1 = RGBA(79, 140, 201, 255);
static Color ca(Color c, float a) { if (a < 0) a = 0; if (a > 1) a = 1; c.a = (unsigned char)(c.a * a + 0.5f); return c; }
static Color mixc(Color a, Color b, float t) { if (t < 0) t = 0; if (t > 1) t = 1;
    return (Color){(unsigned char)(a.r + (b.r - a.r) * t), (unsigned char)(a.g + (b.g - a.g) * t), (unsigned char)(a.b + (b.b - a.b) * t), (unsigned char)(a.a + (b.a - a.a) * t)}; }
static float clampf(float x, float a, float b) { return x < a ? a : x > b ? b : x; }
static float ease_out(float t) { t = clampf(t, 0, 1); return 1 - (1 - t) * (1 - t) * (1 - t); }
static float approach(float cur, float target, float dt, float rate) { return cur + (target - cur) * (1 - expf(-rate * dt)); }

/* ================================================================== canvas */
static float SC = 1, OX = 0, OY = 0; static int SW = 1920, SH = 1080; static float GA = 1;   /* GA = global alpha (screen fade) */
static float VX(float x) { return OX + x * SC; }
static float VY(float y) { return OY + y * SC; }
static void rect(float x, float y, float w, float h, Color c) { DrawRectangleRec((Rectangle){VX(x), VY(y), w * SC, h * SC}, ca(c, c.a / 255.f * GA)); }
static void rrect(float x, float y, float w, float h, float r, Color c) {
    float m = fminf(w, h); if (m <= 0) return;
    DrawRectangleRounded((Rectangle){VX(x), VY(y), w * SC, h * SC}, fminf(1, 2 * r / m), 12, ca(c, c.a / 255.f * GA)); }
static void rrline(float x, float y, float w, float h, float r, float t, Color c) {
    float m = fminf(w, h); if (m <= 0) return;
    DrawRectangleRoundedLinesEx((Rectangle){VX(x), VY(y), w * SC, h * SC}, fminf(1, 2 * r / m), 12, fmaxf(1, t * SC), ca(c, c.a / 255.f * GA)); }
static void line(float x1, float y1, float x2, float y2, float t, Color c) { DrawLineEx((Vector2){VX(x1), VY(y1)}, (Vector2){VX(x2), VY(y2)}, fmaxf(1, t * SC), ca(c, c.a / 255.f * GA)); }
static void dline(float x1, float y1, float x2, float y2, float t, float dash, Color c) {
    float L = hypotf(x2 - x1, y2 - y1); if (L < 1e-3) return; int n = (int)(L / dash);
    for (int i = 0; i < n; i += 2) { float a = i / (float)n, b = fminf(1, (i + 1) / (float)n); line(x1 + (x2 - x1) * a, y1 + (y2 - y1) * a, x1 + (x2 - x1) * b, y1 + (y2 - y1) * b, t, c); } }
static void circ(float x, float y, float r, Color c) { DrawCircleV((Vector2){VX(x), VY(y)}, r * SC, ca(c, c.a / 255.f * GA)); }
static void ring(float x, float y, float r0, float r1, float a0, float a1, Color c) { DrawRing((Vector2){VX(x), VY(y)}, r0 * SC, r1 * SC, a0, a1, 48, ca(c, c.a / 255.f * GA)); }
static void gradv(float x, float y, float w, float h, Color a, Color b) { DrawRectangleGradientV((int)VX(x), (int)VY(y), (int)(w * SC + 0.5f), (int)(h * SC + 0.5f), ca(a, a.a / 255.f * GA), ca(b, b.a / 255.f * GA)); }
static void gradh(float x, float y, float w, float h, Color a, Color b) { DrawRectangleGradientH((int)VX(x), (int)VY(y), (int)(w * SC + 0.5f), (int)(h * SC + 0.5f), ca(a, a.a / 255.f * GA), ca(b, b.a / 255.f * GA)); }
/* soft glow: stacked rounded rects */
static void glow(float x, float y, float w, float h, float r, float spread, Color c, float a) {
    if (a <= 0.01f) return;
    for (int i = 3; i >= 1; i--) { float e = spread * i / 3; rrect(x - e, y - e, w + 2 * e, h + 2 * e, r + e, ca(c, a * 0.10f * (4 - i))); } }

/* ================================================================== fonts */
enum { F_MICH, F_INTER, F_SEMI, F_MONO, F_N };
static const char* FFILE[F_N] = {"Michroma-Regular.ttf", "Inter-Regular.ttf", "Inter-SemiBold.ttf", "JetBrainsMono-Medium.ttf"};
static char FPATH[F_N][320]; static int FOK[F_N];
typedef struct { int face, px, own; Font f; } fcache;
static fcache FC[72]; static int NFC; static float FC_SC = -1;
static int CP[512], NCP;
static void fonts_find(void) {
    const char* dirs[] = {"fonts", "demo/fonts", "../demo/fonts"};
    const char* app = GetApplicationDirectory();
    for (int f = 0; f < F_N; f++) {
        FOK[f] = 0;
        for (int d = 0; d < 4 && !FOK[f]; d++) {
            if (d < 3) snprintf(FPATH[f], sizeof FPATH[f], "%s/%s", dirs[d], FFILE[f]);
            else snprintf(FPATH[f], sizeof FPATH[f], "%sfonts/%s", app, FFILE[f]);
            if (FileExists(FPATH[f])) FOK[f] = 1;
        }
    }
    NCP = 0; for (int c = 32; c < 127; c++) CP[NCP++] = c;
    for (int c = 0xA0; c < 0x180; c++) CP[NCP++] = c;
    const char* extra = "ζμρωΔ²³·—–−→←↑↓≈±×½°≥≤•‹›▲▼";
    int i = 0; while (extra[i]) { int len = 0; int c = GetCodepoint(extra + i, &len); if (len < 1) len = 1; i += len; int dup = 0; for (int k = 0; k < NCP; k++) if (CP[k] == c) dup = 1; if (!dup && NCP < 512) CP[NCP++] = c; }
}
static void fonts_flush(void) { for (int i = 0; i < NFC; i++) if (FC[i].own) UnloadFont(FC[i].f); NFC = 0; }
static Font* font(int face, float size) {
    int px = (int)lrintf(size * SC); if (px < 6) px = 6;
    if (fabsf(FC_SC - SC) > 1e-4f) { fonts_flush(); FC_SC = SC; }
    for (int i = 0; i < NFC; i++) if (FC[i].face == face && FC[i].px == px) return &FC[i].f;
    if (NFC >= 72) fonts_flush();
    fcache* e = &FC[NFC++]; e->face = face; e->px = px; e->own = 0;
    if (FOK[face]) { e->f = LoadFontEx(FPATH[face], px, CP, NCP); e->own = e->f.texture.id != 0; }
    if (!e->own) e->f = GetFontDefault();
    else SetTextureFilter(e->f.texture, TEXTURE_FILTER_BILINEAR);
    return &e->f;
}
static float fsize(const Font* f, float size) { return f->texture.id == GetFontDefault().texture.id ? size * SC : (float)f->baseSize; }
/* width in virtual px; track = letter-spacing in em */
static float tw(int face, float size, const char* s, float track) { Font* f = font(face, size); return MeasureTextEx(*f, s, fsize(f, size), track * size * SC).x / SC; }
/* y = top of the em box; align 0 left, 1 centre, 2 right */
static void tx(int face, float size, float x, float y, const char* s, Color c, float track, int align) {
    Font* f = font(face, size); float sp = track * size * SC, fs = fsize(f, size);
    Vector2 m = MeasureTextEx(*f, s, fs, sp);
    float px = VX(x); if (align == 1) px -= m.x / 2; else if (align == 2) px -= m.x;
    DrawTextEx(*f, s, (Vector2){roundf(px), roundf(VY(y))}, fs, sp, ca(c, c.a / 255.f * GA));
}
/* centred on a baseline-ish middle line cy */
static void txc(int face, float size, float x, float cy, const char* s, Color c, float track, int align) { tx(face, size, x, cy - size * 0.62f, s, c, track, align); }
/* word wrap, returns the height used */
static float twrap(int face, float size, float x, float y, float w, const char* s, Color c, float lh, int maxl) {
    char buf[600]; snprintf(buf, sizeof buf, "%s", s); char ln[600] = ""; float yy = y; int nl = 0;
    char* save = NULL; for (char* wd = strtok_r(buf, " ", &save); wd; wd = strtok_r(NULL, " ", &save)) {
        char t[600]; snprintf(t, sizeof t, "%s%s%s", ln, *ln ? " " : "", wd);
        if (*ln && tw(face, size, t, 0) > w) { if (nl < maxl) tx(face, size, x, yy, ln, c, 0, 0); nl++; yy += size * lh; snprintf(ln, sizeof ln, "%s", wd); }
        else snprintf(ln, sizeof ln, "%s", t);
    }
    if (*ln) { if (nl < maxl) tx(face, size, x, yy, ln, c, 0, 0); yy += size * lh; }
    return yy - y;
}
static void upcase(char* s) { for (; *s; s++) if (*s >= 'a' && *s <= 'z') *s -= 32; }

/* ================================================================== glass + blur */
static const char* BLUR_FS =
"#version 330\nin vec2 fragTexCoord; in vec4 fragColor; uniform sampler2D texture0; uniform vec2 dir; out vec4 finalColor;\n"
"void main(){ vec3 c = texture(texture0, fragTexCoord).rgb*0.2270270270;\n"
" c += texture(texture0, fragTexCoord + dir*1.3846153846).rgb*0.3162162162; c += texture(texture0, fragTexCoord - dir*1.3846153846).rgb*0.3162162162;\n"
" c += texture(texture0, fragTexCoord + dir*3.2307692308).rgb*0.0702702703; c += texture(texture0, fragTexCoord - dir*3.2307692308).rgb*0.0702702703;\n"
" finalColor = vec4(c, 1.0); }\n";
static const char* GLASS_FS =
"#version 330\nin vec2 fragTexCoord; in vec4 fragColor; uniform sampler2D texture0; uniform sampler2D blurTex;\n"
"uniform vec2 screen; uniform vec4 rect; uniform float radius; uniform float alpha; uniform float hair; uniform float lift; uniform float tint;\n"
"out vec4 finalColor;\n"
"float sdr(vec2 p, vec2 b, float r){ vec2 q = abs(p) - b + r; return length(max(q, 0.0)) + min(max(q.x, q.y), 0.0) - r; }\n"
"void main(){\n"
"  vec2 fc = vec2(gl_FragCoord.x, screen.y - gl_FragCoord.y);\n"
"  vec2 c = rect.xy + rect.zw*0.5; float d = sdr(fc - c, rect.zw*0.5, radius);\n"
"  float a = clamp(0.5 - d, 0.0, 1.0); if (a <= 0.0) discard;\n"
"  vec3 bl = texture(blurTex, gl_FragCoord.xy/screen).rgb;\n"
"  float t = clamp((fc.y - rect.y)/max(rect.w, 1.0), 0.0, 1.0);\n"
"  vec3 g0 = vec3(30.0, 35.0, 43.0)/255.0, g1 = vec3(10.0, 13.0, 17.0)/255.0; float ga = mix(0.66, 0.80, t) + tint;\n"
"  vec3 col = mix(bl*0.62, mix(g0, g1, t), clamp(ga, 0.0, 1.0)) + lift*vec3(0.035, 0.038, 0.045);\n"
"  float edge = clamp(1.0 - abs(d + 0.75), 0.0, 1.0); col = mix(col, vec3(1.0), edge*0.085);\n"
"  float top = fc.y - rect.y; float xr = (fc.x - c.x)/(rect.z*0.5);\n"
"  float hl = (1.0 - smoothstep(0.6, 1.8, top))*clamp(1.0 - xr*xr*0.85, 0.0, 1.0);\n"
"  col = mix(col, vec3(0.925, 0.83, 0.61), clamp(hl*hair, 0.0, 1.0));\n"
"  finalColor = vec4(col, a*alpha); }\n";
static Shader SH_BLUR, SH_GLASS; static int L_DIR, L_SCREEN, L_RECT, L_RAD, L_ALPHA, L_HAIR, L_LIFT, L_TINT, L_BTEX;
static RenderTexture2D RT_A, RT_B; static int RTW, RTH; static int GLASS_OK;
static void glass_init(void) {
    SH_BLUR = LoadShaderFromMemory(NULL, BLUR_FS); SH_GLASS = LoadShaderFromMemory(NULL, GLASS_FS);
    GLASS_OK = SH_BLUR.id > 0 && SH_GLASS.id > 0 && SH_GLASS.id != rlGetShaderIdDefault();
    L_DIR = GetShaderLocation(SH_BLUR, "dir");
    L_SCREEN = GetShaderLocation(SH_GLASS, "screen"); L_RECT = GetShaderLocation(SH_GLASS, "rect"); L_RAD = GetShaderLocation(SH_GLASS, "radius");
    L_ALPHA = GetShaderLocation(SH_GLASS, "alpha"); L_HAIR = GetShaderLocation(SH_GLASS, "hair"); L_LIFT = GetShaderLocation(SH_GLASS, "lift");
    L_TINT = GetShaderLocation(SH_GLASS, "tint"); L_BTEX = GetShaderLocation(SH_GLASS, "blurTex");
}
static void blur_scene(RenderTexture2D* scene, int sw, int sh) {
    int w = sw / 4 > 8 ? sw / 4 : 8, h = sh / 4 > 8 ? sh / 4 : 8;
    if (w != RTW || h != RTH) { if (RTW) { UnloadRenderTexture(RT_A); UnloadRenderTexture(RT_B); }
        RT_A = LoadRenderTexture(w, h); RT_B = LoadRenderTexture(w, h); RTW = w; RTH = h;
        SetTextureFilter(RT_A.texture, TEXTURE_FILTER_BILINEAR); SetTextureFilter(RT_B.texture, TEXTURE_FILTER_BILINEAR); }
    SetTextureFilter(scene->texture, TEXTURE_FILTER_BILINEAR);
    BeginTextureMode(RT_A); DrawTexturePro(scene->texture, (Rectangle){0, 0, (float)scene->texture.width, (float)-scene->texture.height}, (Rectangle){0, 0, (float)w, (float)h}, (Vector2){0, 0}, 0, WHITE); EndTextureMode();
    for (int it = 0; it < 3; it++) {
        float sp = 1.0f + it * 0.75f;
        float dh[2] = {sp / w, 0}, dv[2] = {0, sp / h};
        BeginTextureMode(RT_B); BeginShaderMode(SH_BLUR); SetShaderValue(SH_BLUR, L_DIR, dh, SHADER_UNIFORM_VEC2);
        DrawTexturePro(RT_A.texture, (Rectangle){0, 0, (float)w, (float)-h}, (Rectangle){0, 0, (float)w, (float)h}, (Vector2){0, 0}, 0, WHITE); EndShaderMode(); EndTextureMode();
        BeginTextureMode(RT_A); BeginShaderMode(SH_BLUR); SetShaderValue(SH_BLUR, L_DIR, dv, SHADER_UNIFORM_VEC2);
        DrawTexturePro(RT_B.texture, (Rectangle){0, 0, (float)w, (float)-h}, (Rectangle){0, 0, (float)w, (float)h}, (Vector2){0, 0}, 0, WHITE); EndShaderMode(); EndTextureMode();
    }
}
/* the smoked-glass panel; hair = gold hairline strength, lift = hover brighten */
static void glass(float x, float y, float w, float h, float r, float hair, float lift) {
    if (!GLASS_OK) { rrect(x, y, w, h, r, RGBA(14, 17, 22, 215)); rrline(x, y, w, h, r, 1, RGBA(255, 255, 255, 20)); if (hair > 0) line(x + r, y + 0.5f, x + w - r, y + 0.5f, 1, ca(GO1, hair)); return; }
    float scr[2] = {(float)SW, (float)SH}, rc[4] = {VX(x), VY(y), w * SC, h * SC}, rad = r * SC, al = GA, hr = hair, lf = lift, tn = 0;
    BeginShaderMode(SH_GLASS);
    SetShaderValue(SH_GLASS, L_SCREEN, scr, SHADER_UNIFORM_VEC2); SetShaderValue(SH_GLASS, L_RECT, rc, SHADER_UNIFORM_VEC4);
    SetShaderValue(SH_GLASS, L_RAD, &rad, SHADER_UNIFORM_FLOAT); SetShaderValue(SH_GLASS, L_ALPHA, &al, SHADER_UNIFORM_FLOAT);
    SetShaderValue(SH_GLASS, L_HAIR, &hr, SHADER_UNIFORM_FLOAT); SetShaderValue(SH_GLASS, L_LIFT, &lf, SHADER_UNIFORM_FLOAT); SetShaderValue(SH_GLASS, L_TINT, &tn, SHADER_UNIFORM_FLOAT);
    SetShaderValueTexture(SH_GLASS, L_BTEX, RT_A.texture);
    DrawRectangleRec((Rectangle){VX(x) - 2, VY(y) - 2, w * SC + 4, h * SC + 4}, WHITE);
    EndShaderMode();
}

/* ================================================================== state */
enum { SC_TITLE, SC_HUB, SC_CAR, SC_MAP, SC_TUNE, SC_SETUPS, SC_SETTINGS, SC_PAUSE, SC_N };
static const char* SCN[SC_N] = {"TITLE", "GARAGE", "CAR", "TRACK", "TUNING", "SETUPS", "SETTINGS", "PAUSED"};
typedef struct { char path[256], label[48], tag[48]; double kw, nm, kg, vt, wf, rat; int drive, ok; } car_ent;
typedef struct { char path[300], name[80], when[40]; } setup_ent;
static struct {
    menu_host* H;
    int open, in_game, scr; float scr_t, t, dt;
    tune_spec S; int spec_ok; char err[200];
    tune_state T; int t_ok; tune_state* R;         /* R = saved-reference copy (output deltas) */
    int tp[20], ntp, sp[20], nsp, tpage, spage;
    int col, nav, item, btn;                         /* focus: column 0 nav, 1 items, 2 buttons */
    int link[20][14];
    float scroll;
    car_ent car[32]; int ncar, car_sel, car_cur;
    setup_ent set[48]; int nset, set_sel;
    int map_sel, map_pick;
    int need_reinit, car_changed;
    char status[220]; float status_t; Color status_c;
    int drag_item; float drag_off;
    int last_su, last_sd, thr_arm, brk_arm, wb_last;
    int shot;                                        /* screenshot mode: no input */
} M;
static void status_set(Color c, const char* fmt, ...) __attribute__((format(printf, 2, 3)));
#include <stdarg.h>
static void status_set(Color c, const char* fmt, ...) { va_list ap; va_start(ap, fmt); vsnprintf(M.status, sizeof M.status, fmt, ap); va_end(ap); M.status_t = M.t; M.status_c = c; }

/* ---- keyed tweens ---- */
typedef struct { unsigned id; float v; } anim_e; static anim_e AN[1024];
static float* anim(unsigned id, float init) {
    unsigned h = (id * 2654435761u) & 1023;
    for (int i = 0; i < 1024; i++) { anim_e* e = &AN[(h + i) & 1023]; if (e->id == id) return &e->v; if (!e->id) { e->id = id; e->v = init; return &e->v; } }
    static float dummy; dummy = init; return &dummy;
}
static float tween(unsigned id, float target, float rate) { float* v = anim(id, target); *v = approach(*v, target, M.dt, rate); if (fabsf(*v - target) < 1e-5f * fmaxf(1, fabsf(target))) *v = target; return *v; }
static void kick(unsigned id, float v) { *anim(id, v) = v; }
#define HID(kind, a, b) ((unsigned)(((kind) << 24) | (((a) & 0xFFF) << 12) | ((b) & 0xFFF)) + 1u)
enum { K_NAV = 1, K_ITEM, K_TRACK, K_OPT, K_BTN, K_LINK, K_LIST, K_TAB, K_KNOB = 20, K_VAL, K_PULSE, K_OUT, K_OUTP, K_HOV, K_CH, K_SEL };

/* ---- hit rects (registered while drawing, tested on the next update) ---- */
typedef struct { unsigned id; float x, y, w, h; } hit_t;
static hit_t HIT[2][400]; static int NHIT[2], HB;
static void hit(unsigned id, float x, float y, float w, float h) { if (NHIT[HB] < 400) HIT[HB][NHIT[HB]++] = (hit_t){id, x, y, w, h}; }
static float MXv, MYv; static int MMOVED, MCLICK, MDOWN, MREL; static float MWHEEL; static unsigned HOVER;
static int hk(unsigned id) { return (int)((id - 1u) >> 24); }
static int ha(unsigned id) { return (int)(((id - 1u) >> 12) & 0xFFF); }
static int hb(unsigned id) { return (int)((id - 1u) & 0xFFF); }
static float hov(unsigned id) { return tween(HID(K_HOV, 0, 0) ^ (id * 7919u), HOVER == id ? 1.f : 0.f, 14); }

/* ---- input ---- */
static struct { int up, down, left, right, ok, back, tabl, tabr, x, y, start, fine, coarse, any; } IN;
static float RH[4], RN[4];
static int rep(int d, int held, int accel) {
    if (!held) { RH[d] = 0; return 0; }
    if (RH[d] == 0) { RH[d] = 1e-4f; RN[d] = 0.30f; return 1; }
    RH[d] += M.dt; int n = 0;
    while (RH[d] >= RN[d] && n < 8) { n++; float rate = accel ? fminf(42, 10 + 26 * (RH[d] - 0.30f)) : 12; RN[d] += 1.0f / rate; }
    return n;
}
static void input_gather(void) {
    memset(&IN, 0, sizeof IN);
    Vector2 mp = GetMousePosition(); float nx = (mp.x - OX) / SC, ny = (mp.y - OY) / SC;
    MMOVED = fabsf(nx - MXv) + fabsf(ny - MYv) > 0.5f; MXv = nx; MYv = ny;
    MCLICK = IsMouseButtonPressed(MOUSE_BUTTON_LEFT); MDOWN = IsMouseButtonDown(MOUSE_BUTTON_LEFT); MREL = IsMouseButtonReleased(MOUSE_BUTTON_LEFT); MWHEEL = GetMouseWheelMove();
    int pb = HB ^ 1; HOVER = 0;
    for (int i = NHIT[pb] - 1; i >= 0; i--) { hit_t* h = &HIT[pb][i]; if (MXv >= h->x && MXv < h->x + h->w && MYv >= h->y && MYv < h->y + h->h) { HOVER = h->id; break; } }
    if (M.shot) { HOVER = 0; MCLICK = MDOWN = MREL = 0; MWHEEL = 0; MMOVED = 0; return; }
    int g = IsGamepadAvailable(0);
    float sx = g ? GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_X) : 0, sy = g ? GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_Y) : 0;
    int u = IsKeyDown(KEY_UP) || IsKeyDown(KEY_W) || (g && IsGamepadButtonDown(0, GAMEPAD_BUTTON_LEFT_FACE_UP)) || sy < -0.55f;
    int d = IsKeyDown(KEY_DOWN) || IsKeyDown(KEY_S) || (g && IsGamepadButtonDown(0, GAMEPAD_BUTTON_LEFT_FACE_DOWN)) || sy > 0.55f;
    int l = IsKeyDown(KEY_LEFT) || IsKeyDown(KEY_A) || (g && IsGamepadButtonDown(0, GAMEPAD_BUTTON_LEFT_FACE_LEFT)) || sx < -0.55f;
    int r = IsKeyDown(KEY_RIGHT) || IsKeyDown(KEY_D) || (g && IsGamepadButtonDown(0, GAMEPAD_BUTTON_LEFT_FACE_RIGHT)) || sx > 0.55f;
    /* wheel: steering left / right adjusts, paddles move between rows, pedals = select / back, optional buttons */
    const menu_host* H = M.H; const sim_snap* V = H ? H->view : NULL; cfg_t* C = H ? H->cfg : NULL;
    if (V && H->dev_kind == 1) {
        int sa = cfg_int(C, "input.steer_axis", 0); double s = sa >= 0 && sa < 8 ? V->raw_axis[sa] : 0; if (cfg_int(C, "input.steer_invert", 0)) s = -s;
        double thr_dz = cfg_num(C, "ui.wheel_steer_dz", 0.10);
        if (s < -thr_dz) l = 1; if (s > thr_dz) r = 1;
        if (V->menu_su != M.last_su) { IN.down += V->menu_su - M.last_su > 0 ? V->menu_su - M.last_su : 0; M.last_su = V->menu_su; }
        if (V->menu_sd != M.last_sd) { IN.up += V->menu_sd - M.last_sd > 0 ? V->menu_sd - M.last_sd : 0; M.last_sd = V->menu_sd; }
        int ta = cfg_int(C, "input.throttle_axis", 1), ba = cfg_int(C, "input.brake_axis", 2);
        double tv = ta >= 0 && ta < V->raw_naxes ? V->raw_axis[ta] : (cfg_int(C, "input.throttle_invert", 1) ? 1 : -1);
        double bv = ba >= 0 && ba < V->raw_naxes ? V->raw_axis[ba] : (cfg_int(C, "input.brake_invert", 1) ? 1 : -1);
        double tp = cfg_int(C, "input.throttle_invert", 1) ? (1 - tv) / 2 : (tv + 1) / 2, bp = cfg_int(C, "input.brake_invert", 1) ? (1 - bv) / 2 : (bv + 1) / 2;
        if (tp < 0.3) M.thr_arm = 1; if (bp < 0.3) M.brk_arm = 1;
        if (M.thr_arm && tp > 0.75) { IN.ok = 1; M.thr_arm = 0; }
        if (M.brk_arm && bp > 0.75) { IN.back = 1; M.brk_arm = 0; }
        int ob = cfg_int(C, "ui.wheel_ok_button", -1), bb = cfg_int(C, "ui.wheel_back_button", -1), now = V->raw_buttons;
        if (ob >= 0 && (now & (1 << ob)) && !(M.wb_last & (1 << ob))) IN.ok = 1;
        if (bb >= 0 && (now & (1 << bb)) && !(M.wb_last & (1 << bb))) IN.back = 1;
        M.wb_last = now;
    }
    IN.up += rep(0, u, 0); IN.down += rep(1, d, 0); IN.left += rep(2, l, 1); IN.right += rep(3, r, 1);
    IN.ok |= IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER) || IsKeyPressed(KEY_SPACE) || (g && IsGamepadButtonPressed(0, GAMEPAD_BUTTON_RIGHT_FACE_DOWN));
    IN.back |= IsKeyPressed(KEY_ESCAPE) || IsKeyPressed(KEY_BACKSPACE) || (g && IsGamepadButtonPressed(0, GAMEPAD_BUTTON_RIGHT_FACE_RIGHT));
    IN.tabl = IsKeyPressed(KEY_Q) || IsKeyPressed(KEY_PAGE_UP) || (g && IsGamepadButtonPressed(0, GAMEPAD_BUTTON_LEFT_TRIGGER_1));
    IN.tabr = IsKeyPressed(KEY_E) || IsKeyPressed(KEY_PAGE_DOWN) || (g && IsGamepadButtonPressed(0, GAMEPAD_BUTTON_RIGHT_TRIGGER_1));
    IN.x = IsKeyPressed(KEY_R) || (g && IsGamepadButtonPressed(0, GAMEPAD_BUTTON_RIGHT_FACE_LEFT));
    IN.y = IsKeyPressed(KEY_L) || (g && IsGamepadButtonPressed(0, GAMEPAD_BUTTON_RIGHT_FACE_UP));
    IN.start = g && IsGamepadButtonPressed(0, GAMEPAD_BUTTON_MIDDLE_RIGHT);
    IN.fine = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT) || (g && GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_TRIGGER) > 0.3f);
    IN.coarse = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL) || (g && GetGamepadAxisMovement(0, GAMEPAD_AXIS_RIGHT_TRIGGER) > 0.3f);
    IN.any = IN.up || IN.down || IN.left || IN.right || IN.ok || IN.back || IN.tabl || IN.tabr || MCLICK || GetKeyPressed();
}

/* ================================================================== tune model glue */
static tune_state RC;                                  /* reference copy: saved values (ned = 0) for output deltas */
static double OSV[20][16];                             /* saved output values per page */
static double OPREV[20][16];
static const tune_page* PG(int p) { return &M.S.page[p]; }
static int is_pair(const tune_row* r) { return r->t == TR_CAR && r->k2[0]; }
static void ref_refresh(void) {
    if (!M.t_ok) return;
    RC = M.T; RC.ned = 0; tune_eval(&RC);
    for (int p = 0; p < M.S.npage; p++) for (int o = 0; o < PG(p)->nout; o++) { OSV[p][o] = tune_expr(&RC, PG(p)->out[o].e); OPREV[p][o] = tune_expr(&M.T, PG(p)->out[o].e); }
}
static const char* base_car(void) { const char* b = cfg_str(M.H->cfg, "ui.base_car", NULL); return b && FileExists(b) ? b : cfg_str(M.H->cfg, "car.ini", "cars/mx5_na.ini"); }
static void tune_reload(void) {
    if (M.t_ok) { tune_close(&M.T); M.t_ok = 0; }
    if (!M.spec_ok) return;
    const char* cur = cfg_str(M.H->cfg, "car.ini", "cars/mx5_na.ini");
    char err[200] = "";
    M.t_ok = tune_open(&M.T, &M.S, base_car(), FileExists(cur) ? cur : NULL, M.H->cfg, err, sizeof err);
    if (!M.t_ok) { snprintf(M.err, sizeof M.err, "%s", err); return; }
    tune_eval(&M.T); ref_refresh();
}
static double rowv(const tune_row* r, int k2, int which) { return M.t_ok ? tune_row_get(&M.T, r, k2, which) : NAN; }
static int row_changed(const tune_row* r) {
    if (!M.t_ok) return 0;
    for (int k = 0; k < (is_pair(r) ? 2 : 1); k++) { double a = rowv(r, k, 0), b = rowv(r, k, 1); if (a == a && b == b && fabs(a - b) > 1e-9 * fmax(1, fabs(b))) return 1; }
    return 0;
}
static int page_changed(int p) { for (int r = 0; r < PG(p)->nrow; r++) if (row_changed(&PG(p)->row[r])) return 1; return 0; }
/* step a row by n steps (sign = direction); lands on detents (stock, limits) when it crosses them */
static void row_step(int p, int ri, int k2, int n) {
    const tune_row* r = &PG(p)->row[ri]; if (r->ro || !M.t_ok || !n) return;
    if (r->nopt) { int v = (int)lround(rowv(r, k2, 0)) + (n > 0 ? 1 : -1); if (v < 0 || v >= r->nopt) return;
        tune_row_set(&M.T, r, k2, v); tune_eval(&M.T); return; }
    double st = IN.fine && r->fine > 0 ? r->fine : r->st; if (IN.coarse) st *= 10;
    double v0 = rowv(r, k2, 0), v1 = v0 + n * st;
    double det[3] = {rowv(r, k2, 2), r->has_lim ? r->lim[0] : NAN, r->has_lim ? r->lim[1] : NAN};
    for (int i = 0; i < 3; i++) { double d = det[i]; if (d != d) continue;
        if ((v0 < d - 1e-9 && v1 > d + 1e-9) || (v0 > d + 1e-9 && v1 < d - 1e-9)) { v1 = d; kick(HID(K_PULSE, p * 16 + ri, i), 1); break; } }
    double q = tune_row_quant(r, v1);
    if (fabs(q - v0) < 1e-12) return;
    for (int i = 0; i < 3; i++) if (det[i] == det[i] && fabs(q - det[i]) < 1e-9) kick(HID(K_PULSE, p * 16 + ri, i), 1);
    tune_row_set(&M.T, r, k2, q);
    if (is_pair(r) && M.link[p][ri]) { double o = rowv(r, !k2, 0); tune_row_set(&M.T, r, !k2, o + (q - v0)); }
    tune_eval(&M.T);
}
static void row_set_abs(int p, int ri, int k2, double v, int snap) {
    const tune_row* r = &PG(p)->row[ri]; if (r->ro || !M.t_ok) return;
    double v0 = rowv(r, k2, 0);
    if (!r->nopt && snap) { double stk = rowv(r, k2, 2); if (fabs(v - stk) < 0.012 * (r->max - r->min)) v = stk;
        double st = IN.fine && r->fine > 0 ? r->fine : r->st; if (st > 0 && fabs(v - stk) > 1e-12) v = r->min + st * floor((v - r->min) / st + 0.5); }
    double q = tune_row_quant(r, v); if (fabs(q - v0) < 1e-12) return;
    tune_row_set(&M.T, r, k2, q);
    if (is_pair(r) && M.link[p][ri]) { double o = rowv(r, !k2, 0); tune_row_set(&M.T, r, !k2, o + (q - v0)); }
    tune_eval(&M.T);
}
static void row_revert(int p, int ri, int to_stock) {
    const tune_row* r = &PG(p)->row[ri]; if (r->ro || !M.t_ok) return;
    for (int k = 0; k < (is_pair(r) ? 2 : 1); k++) tune_row_set(&M.T, r, k, rowv(r, k, to_stock ? 2 : 1));
    tune_eval(&M.T);
}
static void fmtv(double v, int d, char* b, int n) { if (v != v) snprintf(b, n, "—"); else snprintf(b, n, "%.*f", d < 0 ? 0 : d, fabs(v) < 0.5 * pow(10, -d) ? 0.0 : v); }
static void fmt_row(const tune_row* r, double v, char* b, int n) {
    if (r->nopt) { int i = (int)lround(v); snprintf(b, n, "%s", i >= 0 && i < r->nopt ? r->opt[i] : "?"); return; }
    fmtv(v, r->d, b, n);
}

/* ================================================================== chrome */
#define NAVX 72
#define TOPY 150
#define PANH 780
#define CENX 404
#define CENW 1000
#define INFX 1428
#define INFW 420
#define BOTY 960
static void topbar(const char* crumb) {
    tx(F_MICH, 20, 72, 52, "TARMAC LAB", GR0, 0.42f, 0);
    float w = tw(F_MICH, 20, "TARMAC LAB", 0.42f);
    line(72 + w + 22, 50, 72 + w + 22, 78, 1, ca(GR3, 0.8f));
    char c[96]; snprintf(c, sizeof c, "%s", crumb); upcase(c);
    tx(F_SEMI, 14, 72 + w + 44, 57, c, GR1, 0.24f, 0);
    /* right: car + map chips */
    char car[64] = "", mp[96]; if (M.car_cur >= 0 && M.car_cur < M.ncar) snprintf(car, sizeof car, "%s", M.car[M.car_cur].label);
    snprintf(mp, sizeof mp, "%s", M.H && M.H->map_title ? M.H->map_title : ""); upcase(mp);
    float x = 1848;
    const char* chips[2] = {mp, car};
    for (int i = 0; i < 2; i++) { if (!chips[i][0]) continue; float cw = tw(F_SEMI, 13, chips[i], 0.2f) + 34; x -= cw;
        rrect(x, 46, cw, 34, 17, RGBA(255, 255, 255, 10)); rrline(x, 46, cw, 34, 17, 1, RGBA(255, 255, 255, 26));
        tx(F_SEMI, 13, x + cw / 2, 56, chips[i], i ? GO0 : GR1, 0.2f, 1); x -= 12; }
    line(72, 104, 1848, 104, 1, RGBA(255, 255, 255, 14));
    gradh(72, 104, 380, 1, ca(GO1, 0.75f), ca(GO1, 0));
}
static void footer(void) {
    tx(F_INTER, 12, 72, 1046, "Flat visual: private development provenance · credit Miki", ca(GR2, 0.9f), 0.04f, 0);
    tx(F_INTER, 12, 1848, 1046, "DriveChain physics · tracks & reference cars: nothke, lemax", ca(GR2, 0.9f), 0.04f, 2);
}
/* button; style 0 ghost, 1 gold primary; returns hover/focus mix */
static void button(unsigned id, float x, float y, float w, float h, const char* label, int style, int focus, int enabled) {
    float hv = hov(id), fo = tween(id ^ 0x5bd1e995u, focus ? 1.f : 0.f, 14), a = enabled ? 1 : 0.4f;
    hit(id, x, y, w, h);
    if (style == 1) {
        glow(x, y, w, h, 10, 10, GO1, (0.35f + 0.65f * fo) * a * (0.4f + 0.6f * hv));
        gradv(x + 2, y, w - 4, h, mixc(GO0, RGBA(255, 240, 205, 255), hv * 0.5f), GO1);
        rrect(x, y, w, h, 10, ca(mixc(GO1, GO0, 0.4f + 0.4f * hv), a));
        rrline(x, y, w, h, 10, 1, ca(RGBA(255, 244, 214, 255), 0.55f * a));
        tx(F_MICH, 14, x + w / 2, y + h / 2 - 9, label, ca(INK0, a), 0.26f, 1);
    } else {
        rrect(x, y, w, h, 10, RGBA(255, 255, 255, (unsigned char)(8 + 14 * hv + 12 * fo)));
        rrline(x, y, w, h, 10, 1, ca(mixc(RGBA(255, 255, 255, 40), GO1, fo), a));
        if (fo > 0.01f) line(x + 12, y + 0.5f, x + w - 12, y + 0.5f, 1, ca(GO0, fo * a));
        tx(F_MICH, 13, x + w / 2, y + h / 2 - 8, label, ca(mixc(GR1, GR0, fmaxf(hv, fo)), a), 0.26f, 1);
    }
}
/* left navigation list (labels + optional right-side tag); returns nothing; focus = column active */
static void navlist(const char* title, const char** items, const char** tags, int n, int sel, int focus, float y0, int flags) {
    glass(NAVX, TOPY, 308, PANH, 16, 0.9f, 0);
    tx(F_SEMI, 12, NAVX + 28, TOPY + 30, title, GR2, 0.3f, 0);
    float sy = tween(HID(K_SEL, 1, 0), y0 + sel * 52, 16);
    float fo = tween(HID(K_SEL, 2, 0), focus ? 1.f : 0.45f, 12);
    rrect(NAVX + 14, sy, 280, 46, 9, RGBA(255, 255, 255, (unsigned char)(10 + 10 * fo)));
    rect(NAVX + 14, sy + 11, 3, 24, ca(GO0, fo));
    for (int i = 0; i < n; i++) {
        unsigned id = HID(K_NAV, i, 0); float y = y0 + i * 52, hv = hov(id); hit(id, NAVX + 14, y, 280, 46);
        if (hv > 0.01f && i != sel) rrect(NAVX + 14, y, 280, 46, 9, RGBA(255, 255, 255, (unsigned char)(8 * hv)));
        Color c = i == sel ? GR0 : mixc(GR2, GR1, hv);
        tx(F_MICH, 13.5f, NAVX + 36 + (i == sel ? 4 * fo : 2 * hv), y + 15, items[i], c, 0.22f, 0);
        if (tags && tags[i] && tags[i][0]) { int dot = tags[i][0] == '*'; if (dot) circ(NAVX + 272, y + 23, 3.5f, GO0); else tx(F_MONO, 12, NAVX + 278, y + 16, tags[i], GR3, 0, 2); }
    }
    (void)flags;
}
static void hints(const char* s) { tx(F_INTER, 14, 72, BOTY + 18, s, GR2, 0.02f, 0); }
static void status_draw(void) {
    float age = M.t - M.status_t; if (!M.status[0] || age > 5) return;
    float a = age < 0.25f ? age / 0.25f : age > 4 ? 5 - age : 1;
    float w = tw(F_SEMI, 14, M.status, 0.04f) + 40, x = 960 - w / 2, y = 116 + 6 * (1 - ease_out(age / 0.35f));
    rrect(x, y, w, 30, 15, ca(RGBA(14, 17, 22, 230), a)); rrline(x, y, w, 30, 15, 1, ca(M.status_c, 0.55f * a));
    tx(F_SEMI, 14, 960, y + 7, M.status, ca(M.status_c, a), 0.04f, 1);
}

/* ================================================================== tune page: rows */
typedef struct { int r, k; float y, cy; } line_t;
static int build_lines(int p, line_t* L, float* total) {
    int n = 0; float y = 0;
    for (int r = 0; r < PG(p)->nrow; r++) {
        const tune_row* R = &PG(p)->row[r];
        if (is_pair(R)) { L[n++] = (line_t){r, 0, y, y + 36}; L[n++] = (line_t){r, 1, y, y + 76}; y += 108; }
        else { L[n++] = (line_t){r, 0, y, y + 34}; y += 68; }
    }
    if (total) *total = y; return n;
}
#define ROWS_Y (TOPY + 158)
#define ROWS_H (PANH - 178)
#define TX0 (CENX + 336)
#define TW 420
static float f_of(const tune_row* r, double v) { return r->max > r->min ? clampf((float)((v - r->min) / (r->max - r->min)), 0, 1) : 0; }
static void slider_line(int p, int ri, int k2, float cy, int focus, int editing) {
    const tune_row* r = &PG(p)->row[ri]; int key = p * 16 + ri;
    double v = rowv(r, k2, 0), sv = rowv(r, k2, 1), st = rowv(r, k2, 2);
    unsigned tid = HID(K_TRACK, key, k2), iid = HID(K_ITEM, key, k2);
    float th = hov(tid), ih = hov(iid);
    float fo = tween(iid ^ 0x2545F491u, focus ? 1.f : 0.f, 16);
    float ro = r->ro ? 0.45f : 1;
    char b[64], bd[64];
    if (r->nopt) {   /* option chips with a sliding highlight */
        int n = r->nopt; float gap = 6, cw = (TW - gap * (n - 1)) / n; int cur = (int)lround(v);
        float hx = tween(HID(K_CH, key, k2), TX0 + cur * (cw + gap), 18);
        rrect(TX0, cy - 17, TW, 34, 9, RGBA(255, 255, 255, 7));
        glow(hx, cy - 17, cw, 34, 9, 6, GO1, 0.6f * fo);
        rrect(hx, cy - 17, cw, 34, 9, mixc(RGBA(201, 165, 92, 60), RGBA(201, 165, 92, 120), fo));
        rrline(hx, cy - 17, cw, 34, 9, 1, ca(GO0, 0.35f + 0.5f * fo));
        int stk = (int)lround(st);
        for (int i = 0; i < n; i++) { float x = TX0 + i * (cw + gap); unsigned oid = HID(K_OPT, key, k2 * 8 + i); float oh = hov(oid); hit(oid, x, cy - 17, cw, 34);
            if (oh > 0.01f && i != cur) rrect(x, cy - 17, cw, 34, 9, RGBA(255, 255, 255, (unsigned char)(14 * oh)));
            char o[24]; snprintf(o, sizeof o, "%s", r->opt[i]); upcase(o);
            tx(F_SEMI, 12.5f, x + cw / 2, cy - 8, o, i == cur ? GO0 : mixc(GR2, GR1, oh), 0.16f, 1);
            if (i == stk) circ(x + cw - 9, cy - 10, 2, ca(GR1, 0.7f)); }
        fmt_row(r, v, b, sizeof b);
    } else {
        float f = f_of(r, v), fk = tween(HID(K_KNOB, key, k2), f, editing ? 30 : 18);
        float x0 = TX0, x1 = TX0 + TW, kx = x0 + fk * TW;
        float act = fmaxf(fo, th);
        /* track + recommended window + detents */
        rrect(x0, cy - 1.5f, TW, 3, 1.5f, ca(mixc(GR3, GO2, th * 0.6f), 0.9f * ro));
        if (r->has_lim) { float a = x0 + f_of(r, r->lim[0]) * TW, c = x0 + f_of(r, r->lim[1]) * TW;
            rect(a, cy - 1.5f, c - a, 3, ca(GR2, 0.45f * ro));
            for (int i = 0; i < 2; i++) { float x = i ? c : a; float pu = tween(HID(K_PULSE, key, i + 1) ^ 77u, 0, 4); float pk = *anim(HID(K_PULSE, key, i + 1), 0); (void)pu;
                *anim(HID(K_PULSE, key, i + 1), 0) = approach(pk, 0, M.dt, 5);
                line(x, cy - 7 - 3 * pk, x, cy + 7 + 3 * pk, 1, ca(mixc(GR2, GO0, pk), 0.9f * ro)); } }
        float org = r->min < 0 && r->max > 0 ? x0 + f_of(r, 0) * TW : x0;
        if (r->min < 0 && r->max > 0) line(org, cy - 4, org, cy + 4, 1, ca(GR2, 0.6f));
        float fa = fminf(org, kx), fb = fmaxf(org, kx);
        rect(fa, cy - 1.5f, fb - fa, 3, ca(mixc(GO1, GO0, act), ro));
        if (act > 0.01f) rect(fa, cy - 3.5f, fb - fa, 7, ca(GO1, 0.10f * act));
        /* ghost stock marker (small down triangle + tick), saved marker dot */
        { float sx = x0 + f_of(r, st) * TW; float pk = *anim(HID(K_PULSE, key, 0), 0); *anim(HID(K_PULSE, key, 0), 0) = approach(pk, 0, M.dt, 5);
          Color gc = ca(mixc(GR1, GO0, pk), 0.75f * ro);
          DrawTriangle((Vector2){VX(sx - 5 - 2 * pk), VY(cy - 15)}, (Vector2){VX(sx), VY(cy - 8)}, (Vector2){VX(sx + 5 + 2 * pk), VY(cy - 15)}, ca(gc, GA * gc.a / 255.f));
          line(sx, cy - 8, sx, cy + 5, 1, ca(gc, 0.7f));
          if (pk > 0.02f) circ(sx, cy, 9 * pk + 3, ca(GO0, 0.18f * pk)); }
        if (fabs(sv - st) > 1e-9 * fmax(1, fabs(st))) { float vx = x0 + f_of(r, sv) * TW; circ(vx, cy + 10, 2.2f, ca(GR1, 0.8f)); }
        /* knob */
        float dragging = (M.drag_item == (int)tid) ? 1.f : 0.f; float kd = tween(tid ^ 0x68E31DA4u, dragging, 20);
        glow(kx - 4, cy - 12, 8, 24, 4, 9, GO0, (0.25f * th + 0.75f * fo) * ro);
        rrect(kx - 3.5f - kd, cy - 11 - 2 * kd, 7 + 2 * kd, 22 + 4 * kd, 3.5f + kd, ca(mixc(GR1, GR0, act), ro));
        rect(kx - 0.5f, cy - 5, 1, 10, ca(GO1, 0.8f * act));
        /* min / max under the track when focused */
        if (fo > 0.02f) { char a[32], c[32]; fmtv(r->min, r->d, a, sizeof a); fmtv(r->max, r->d, c, sizeof c);
            tx(F_MONO, 11, x0, cy + 9, a, ca(GR3, fo), 0, 0); tx(F_MONO, 11, x1, cy + 9, c, ca(GR3, fo), 0, 2); }
        hit(tid, x0 - 12, cy - 16, TW + 24, 32);
        double vr = tween(HID(K_VAL, key, k2), (float)v, 16); if (fabs(vr - v) < 0.5 * pow(10, -r->d)) vr = v;
        fmtv(vr, r->d, b, sizeof b);
    }
    /* value + delta vs saved */
    tx(F_MONO, 17, CENX + 862, cy - 11, b, ca(mixc(GR1, GR0, fmaxf(fo, ih)), ro), 0, 2);
    double d = v - sv;
    if (r->nopt) { if (fabs(d) > 0.5) tx(F_SEMI, 12, CENX + 962, cy - 8, "CHANGED", GO0, 0.14f, 2); else tx(F_MONO, 13, CENX + 962, cy - 9, "—", GR3, 0, 2); }
    else if (fabs(d) >= 0.5 * pow(10, -r->d)) { snprintf(bd, sizeof bd, "%s%.*f", d > 0 ? "+" : "−", r->d, fabs(d)); tx(F_MONO, 13, CENX + 962, cy - 9, bd, d > 0 ? GN0 : RD0, 0, 2); }
    else tx(F_MONO, 13, CENX + 962, cy - 9, "—", GR3, 0, 2);
}
static void tune_rows(int p, int focus_line, int col_focus, float alpha_in) {
    line_t L[32]; float total; int n = build_lines(p, L, &total);
    /* scroll so the focused line is visible */
    float target = M.scroll;
    if (focus_line >= 0 && focus_line < n) { float top = L[focus_line].y, bot = L[focus_line].cy + 34;
        if (top - target < 0) target = top; if (bot - target > ROWS_H) target = bot - ROWS_H; }
    if (total <= ROWS_H) target = 0; else target = clampf(target, 0, total - ROWS_H + 8);
    M.scroll = target; float sc = tween(HID(K_SEL, 3, p), target, 14);
    BeginScissorMode((int)VX(CENX), (int)VY(ROWS_Y - 6), (int)(CENW * SC), (int)((ROWS_H + 12) * SC));
    int lastr = -1;
    for (int i = 0; i < n; i++) {
        const tune_row* R = &PG(p)->row[L[i].r]; int key = p * 16 + L[i].r;
        float by = ROWS_Y + L[i].y - sc, cy = ROWS_Y + L[i].cy - sc;
        int foc = col_focus && focus_line == i;
        int rowfoc = col_focus && focus_line >= 0 && L[focus_line].r == L[i].r;
        if (L[i].r != lastr) {   /* row block: label, unit, link, separator */
            lastr = L[i].r; float bh = is_pair(R) ? 100 : 60;
            float rf = tween(HID(K_SEL, 4, key), rowfoc ? 1.f : 0.f, 14);
            unsigned bid = HID(K_ITEM, key, 0); float bhv = hov(bid) + hov(HID(K_ITEM, key, 1)) + hov(HID(K_TRACK, key, 0)) + hov(HID(K_TRACK, key, 1));
            rrect(CENX + 16, by + 2, CENW - 32, bh, 10, RGBA(255, 255, 255, (unsigned char)(clampf(5 * fminf(1, bhv) + 9 * rf, 0, 255))));
            if (i && L[i - 1].r != L[i].r) line(CENX + 36, by - 2, CENX + CENW - 36, by - 2, 1, RGBA(255, 255, 255, 10));
            char lab[64]; snprintf(lab, sizeof lab, "%s", R->l);
            tx(F_SEMI, 16, CENX + 40 + 3 * rf, by + 16, lab, mixc(GR1, GR0, rf), 0.01f, 0);
            char u[48]; if (R->ro) snprintf(u, sizeof u, "%s · LOCKED", R->u[0] ? R->u : "-"); else snprintf(u, sizeof u, "%s", R->u);
            if (row_changed(R)) circ(CENX + 30, by + 26, 3, GO0);
            tx(F_INTER, 12.5f, CENX + 40 + 3 * rf, by + 38, u, R->ro ? GO2 : GR2, 0.02f, 0);
            if (is_pair(R)) {
                int lk = M.link[p][L[i].r]; unsigned lid = HID(K_LINK, key, 0); float lh = hov(lid); hit(lid, CENX + 240, by + 44, 52, 28);
                float la = tween(HID(K_SEL, 5, key), lk ? 1.f : 0.f, 12);
                Color lc = mixc(mixc(GR3, GR1, lh), GO0, la);
                ring(CENX + 258, by + 58, 4.5f, 6, 0, 360, lc); ring(CENX + 274 - 4 * la, by + 58, 4.5f, 6, 0, 360, lc);
                tx(F_SEMI, 10.5f, CENX + 252, by + 70, lk ? "LINKED" : "LINK", ca(lc, 0.9f), 0.14f, 0);
                if (la > 0.01f) line(TX0 + f_of(R, rowv(R, 0, 0)) * TW, by + 36 + 12, TX0 + f_of(R, rowv(R, 1, 0)) * TW, by + 76 - 12, 1, ca(GO1, 0.45f * la));
            }
            hit(HID(K_ITEM, key, 0), CENX + 16, by + 2, CENW - 32, is_pair(R) ? 50 : bh);
        }
        if (is_pair(R)) { if (L[i].k) hit(HID(K_ITEM, key, 1), CENX + 16, by + 52, CENW - 32, 50);
            tx(F_MONO, 12, CENX + 312, cy - 8, L[i].k ? "R" : "F", foc ? GO0 : GR2, 0, 0); }
        float fb = tween(HID(K_SEL, 6, key * 2 + L[i].k), foc ? 1.f : 0.f, 16);
        if (fb > 0.01f) rect(CENX + 16, cy - 12, 3, 24, ca(GO0, fb));
        slider_line(p, L[i].r, L[i].k, cy, foc, foc && (IN.left || IN.right));
    }
    EndScissorMode();
    if (total > ROWS_H) { float h = ROWS_H * ROWS_H / total, y = ROWS_Y + (ROWS_H - h) * (sc / (total - ROWS_H + 8));
        rrect(CENX + CENW - 10, y, 3, h, 1.5f, RGBA(255, 255, 255, 40)); }
    (void)alpha_in;
}

/* ================================================================== info panel: live outputs + charts */
static double X(const char* e) { return M.t_ok ? tune_expr(&M.T, e) : NAN; }
static double XS(const char* e) { return M.t_ok ? tune_expr(&RC, e) : NAN; }
static float TWV(unsigned id, double v) { return v == v ? tween(id, (float)v, 10) : 0; }
static void axis_lbl(float x, float y, const char* s, int al) { tx(F_MONO, 10.5f, x, y, s, GR3, 0, al); }
static void chart_frame(float x, float y, float w, float h, const char* title) {
    rrect(x, y, w, h, 10, RGBA(0, 0, 0, 46)); rrline(x, y, w, h, 10, 1, RGBA(255, 255, 255, 12));
    tx(F_SEMI, 10.5f, x + 14, y + 11, title, GR2, 0.24f, 0);
}
static void grid(float x, float y, float w, float h, int nx, int ny) {
    for (int i = 1; i < nx; i++) line(x + w * i / nx, y, x + w * i / nx, y + h, 1, RGBA(255, 255, 255, 9));
    for (int i = 1; i < ny; i++) line(x, y + h * i / ny, x + w, y + h * i / ny, 1, RGBA(255, 255, 255, 9));
    line(x, y + h, x + w, y + h, 1, RGBA(255, 255, 255, 30));
}
static void ch_gearing(float x, float y, float w, float h) {
    chart_frame(x, y, w, h, "RPM VS ROAD SPEED");
    float px = x + 40, py = y + 34, pw = w - 56, ph = h - 62;
    double lim = X("car.limiter_rpm"); if (!(lim > 0)) lim = 7000;
    double vg[7]; int ng = 0; char e[8]; for (int i = 1; i <= 6; i++) { snprintf(e, sizeof e, "v%d", i); double v = X(e); if (v > 0) vg[ng++] = TWV(HID(K_OUT, 50, i), v); }
    double vt = TWV(HID(K_OUT, 50, 9), X("vt")); double vmax = 50 * ceil(fmax(ng ? vg[ng - 1] : 200, vt) * 1.08 / 50);
    grid(px, py, pw, ph, (int)(vmax / 50), 4);
    for (int k = 0; k <= (int)(vmax / 50); k++) { char b[16]; snprintf(b, sizeof b, "%d", k * 50); axis_lbl(px + pw * k * 50 / vmax, py + ph + 5, b, 1); }
    char lb[16]; snprintf(lb, sizeof lb, "%.0fk", lim / 1000); axis_lbl(px - 6, py - 5, lb, 2); axis_lbl(px - 6, py + ph - 8, "0", 2);
    float rl = (float)(lim * 1.0);
    for (int i = 0; i < ng; i++) {
        float xa = px + pw * (float)(vg[i] / vmax);
        line(px, py + ph, xa, py, 1, RGBA(255, 255, 255, 18));
        double v0 = i ? vg[i - 1] : 0; float r0 = i ? (float)(lim * vg[i - 1] / vg[i]) : 0;
        float x0 = px + pw * (float)(v0 / vmax), y0 = py + ph - ph * r0 / rl;
        Color c = mixc(GO1, GO0, i / (float)fmax(1, ng - 1));
        line(x0, y0, xa, py, 2, c);
        if (i) line(x0, py, x0, y0, 1, ca(GO2, 0.8f));
        char g[4]; snprintf(g, sizeof g, "%d", i + 1); tx(F_MONO, 10.5f, xa - 4, py + 4, g, ca(GO0, 0.8f), 0, 2);
    }
    if (vt > 0) { float vx = px + pw * (float)(vt / vmax); dline(vx, py - 4, vx, py + ph, 1, 4, ca(GN0, 0.85f)); char b[24]; snprintf(b, sizeof b, "%.0f", vt); tx(F_MONO, 11, vx + 4, py + ph - 18, b, GN0, 0, 0); }
}
static void gauge(float x, float y, float w, const char* lab, double lo, double hi, double g0, double g1, double vf, double vr, const char* fmt) {
    tx(F_SEMI, 10.5f, x, y, lab, GR2, 0.18f, 0);
    float ty = y + 24; rrect(x, ty, w, 6, 3, RGBA(255, 255, 255, 30));
    if (g1 > g0) { float a = x + w * clampf((float)((g0 - lo) / (hi - lo)), 0, 1), b = x + w * clampf((float)((g1 - lo) / (hi - lo)), 0, 1); rrect(a, ty, b - a, 6, 3, ca(GN1, 0.45f)); }
    double vs[2] = {vf, vr}; Color cs[2] = {BL0, GO0}; const char* nm[2] = {"F", "R"};
    for (int i = 0; i < 2; i++) { if (vs[i] != vs[i]) continue; float px = x + w * clampf((float)((vs[i] - lo) / (hi - lo)), 0, 1);
        rrect(px - 2, ty - 6, 4, 18, 2, cs[i]); char b[32]; snprintf(b, sizeof b, fmt, vs[i]);
        rrect(px - 6, ty - 8, 12, 22, 4, ca(cs[i], 0.14f));
        char t[40]; snprintf(t, sizeof t, "%s %s", nm[i], b); tx(F_MONO, 11, x + w - (i ? 0 : 92), y, t, cs[i], 0, 2); }
    char e[16]; snprintf(e, sizeof e, fmt, lo); axis_lbl(x, ty + 12, e, 0); snprintf(e, sizeof e, fmt, hi); axis_lbl(x + w, ty + 12, e, 2);
}
static void ch_springs(float x, float y, float w, float h) {
    chart_frame(x, y, w, h, "RIDE · DAMPING · ROLL");
    float gx = x + 24, gw = w - 48;
    gauge(gx, y + 40, gw, "RIDE FREQUENCY  Hz", 0.6, 3.0, 1.1, 2.0, TWV(HID(K_OUT, 51, 0), X("fnf")), TWV(HID(K_OUT, 51, 1), X("fnr")), "%.2f");
    gauge(gx, y + 106, gw, "DAMPING RATIO BUMP  ζ", 0, 1.0, 0.25, 0.45, TWV(HID(K_OUT, 51, 2), X("zbf")), TWV(HID(K_OUT, 51, 3), X("zbr")), "%.2f");
    double rs = X("rsf"); float rf = TWV(HID(K_OUT, 51, 4), rs);
    float by = y + h - 40; tx(F_SEMI, 10.5f, gx, by - 18, "ROLL STIFFNESS SPLIT", GR2, 0.18f, 0);
    if (rs == rs) { float f = clampf(rf / 100.f, 0, 1); rrect(gx, by, gw * f, 10, 3, ca(BL0, 0.85f)); rrect(gx + gw * f + 2, by, gw * (1 - f) - 2, 10, 3, ca(GO1, 0.85f));
        char b[32]; snprintf(b, sizeof b, "F %.0f%%", rf); tx(F_MONO, 11, gx, by + 14, b, BL0, 0, 0); snprintf(b, sizeof b, "R %.0f%%", 100 - rf); tx(F_MONO, 11, gx + gw, by + 14, b, GO0, 0, 2); }
}
static void ch_tyres(float x, float y, float w, float h) {
    chart_frame(x, y, w, h, "HOT PRESSURE · GRIP WINDOW");
    float px = x + 22, py = y + 40, pw = w - 44, ph = h - 76;
    double lo = 150, hi = 300; grid(px, py, pw, ph, 6, 3);
    double c = 225, wdt = 42; double wf = X("winf"); (void)wf;
    Vector2 prev = {0}; for (int i = 0; i <= 60; i++) { double p = lo + (hi - lo) * i / 60.0; double g = exp(-0.5 * pow((p - c) / wdt, 2));
        float xx = px + pw * i / 60.f, yy = py + ph - ph * (float)g * 0.92f; if (i) line(prev.x, prev.y, xx, yy, 2, ca(GR1, 0.7f)); prev = (Vector2){xx, yy}; }
    for (int k = 0; k <= 6; k++) { char b[8]; snprintf(b, sizeof b, "%.0f", lo + (hi - lo) * k / 6); axis_lbl(px + pw * k / 6, py + ph + 5, b, 1); }
    double hp[2] = {TWV(HID(K_OUT, 52, 0), X("hotf")), TWV(HID(K_OUT, 52, 1), X("hotr"))}; Color cs[2] = {BL0, GO0};
    for (int i = 0; i < 2; i++) { double p = hp[i]; if (!(p > 0)) continue; double g = exp(-0.5 * pow((p - c) / wdt, 2));
        float xx = px + pw * clampf((float)((p - lo) / (hi - lo)), 0, 1), yy = py + ph - ph * (float)g * 0.92f;
        line(xx, yy, xx, py + ph, 1, ca(cs[i], 0.5f)); circ(xx, yy, 5, cs[i]); circ(xx, yy, 9, ca(cs[i], 0.18f));
        char b[24]; snprintf(b, sizeof b, "%s %.0f", i ? "R" : "F", p); tx(F_MONO, 11, xx + (i ? 8 : -8), yy - 18, b, cs[i], 0, i ? 0 : 2); }
    axis_lbl(px + pw, py - 14, "kPa hot", 2);
}
static void ch_brakes(float x, float y, float w, float h) {
    chart_frame(x, y, w, h, "TORQUE VS LOCK LIMIT");
    float gx = x + 74, gw = w - 98;
    double tb[2] = {TWV(HID(K_OUT, 53, 0), X("tbf")), TWV(HID(K_OUT, 53, 1), X("tbr"))}, lm[2] = {TWV(HID(K_OUT, 53, 2), X("limf")), TWV(HID(K_OUT, 53, 3), X("limr"))};
    double mx = 1; for (int i = 0; i < 2; i++) mx = fmax(mx, fmax(tb[i], lm[i])); mx *= 1.15;
    const char* nm[2] = {"FRONT", "REAR"};
    for (int i = 0; i < 2; i++) { float by = y + 44 + i * 44;
        tx(F_SEMI, 10.5f, x + 16, by + 3, nm[i], GR2, 0.18f, 0);
        rrect(gx, by, gw, 14, 4, RGBA(255, 255, 255, 12));
        int over = tb[i] > lm[i]; float f = clampf((float)(tb[i] / mx), 0, 1), l = clampf((float)(lm[i] / mx), 0, 1);
        rrect(gx, by, gw * f, 14, 4, ca(over ? RD0 : (i ? GO1 : BL1), 0.9f));
        line(gx + gw * l, by - 5, gx + gw * l, by + 19, 2, GR0);
        char b[40]; snprintf(b, sizeof b, "%.0f / %.0f N·m", tb[i], lm[i]); tx(F_MONO, 10.5f, gx + gw, by + 18, b, over ? RD0 : GR2, 0, 2); }
    /* bias needle vs ideal */
    double bias = TWV(HID(K_OUT, 53, 4), X("bias")), ideal = lm[0] + lm[1] > 0 ? 100 * lm[0] / (lm[0] + lm[1]) : NAN;
    float bx = x + 24, bw = w - 48, byy = y + h - 46; double lo = 40, hi = 85;
    tx(F_SEMI, 10.5f, bx, byy - 22, "BIAS  % FRONT", GR2, 0.18f, 0);
    rrect(bx, byy, bw, 6, 3, RGBA(255, 255, 255, 30));
    for (int k = 0; k <= 9; k++) line(bx + bw * k / 9, byy + 8, bx + bw * k / 9, byy + 12, 1, RGBA(255, 255, 255, 28));
    if (ideal == ideal) { float ix = bx + bw * clampf((float)((ideal - lo) / (hi - lo)), 0, 1); rrect(ix - 9, byy - 2, 18, 10, 4, ca(GN0, 0.35f)); tx(F_MONO, 10.5f, ix, byy + 14, TextFormat("ideal %.1f", ideal), GN0, 0, 1); }
    if (bias == bias) { float nx = bx + bw * clampf((float)((bias - lo) / (hi - lo)), 0, 1);
        DrawTriangle((Vector2){VX(nx - 6), VY(byy - 12)}, (Vector2){VX(nx), VY(byy - 2)}, (Vector2){VX(nx + 6), VY(byy - 12)}, ca(GO0, GA)); rect(nx - 1, byy - 2, 2, 10, GO0);
        tx(F_MONO, 11, bx + bw, byy - 22, TextFormat("%.1f %% F", bias), GO0, 0, 2); }
}
static void ch_diff(float x, float y, float w, float h) {
    chart_frame(x, y, w, h, "LOCKING TORQUE VS INPUT (1ST)");
    float px = x + 44, py = y + 36, pw = w - 62, ph = h - 66;
    double axt = fmax(100, X("axt1")), pre = X("car.diff_preload"), pwr = X("car.diff_power"), cst = X("car.diff_coast");
    if (pre != pre) pre = 0; if (pwr != pwr) pwr = 0; if (cst != cst) cst = 0;
    float ap = TWV(HID(K_OUT, 54, 0), pwr), ac = TWV(HID(K_OUT, 54, 1), cst), apr = TWV(HID(K_OUT, 54, 2), pre);
    double ymax = fmax(100, apr + fmax(ap, ac) * axt) * 1.1; grid(px, py, pw, ph, 4, 4);
    line(px, py + ph - ph * (float)(apr / ymax), px + pw, py + ph - ph * (float)((apr + ap * axt) / ymax), 2, GO0);
    line(px, py + ph - ph * (float)(apr / ymax), px + pw, py + ph - ph * (float)((apr + ac * axt) / ymax), 2, BL0);
    dline(px, py + ph - ph * (float)(apr / ymax), px + pw, py + ph - ph * (float)(apr / ymax), 1, 4, ca(GR2, 0.8f));
    tx(F_MONO, 10.5f, px + pw - 4, py + ph - ph * (float)((apr + ap * axt) / ymax) - 16, "POWER", GO0, 0, 2);
    tx(F_MONO, 10.5f, px + pw - 4, py + ph - ph * (float)((apr + ac * axt) / ymax) + 4, "COAST", BL0, 0, 2);
    axis_lbl(px + pw, py + ph + 5, TextFormat("%.0f N·m in", axt), 2); axis_lbl(px - 4, py - 4, TextFormat("%.0f", ymax), 2);
}
static void wheel_shape(float cx, float cy, float w, float h, float ang, Color c) {
    Rectangle r = {VX(cx), VY(cy), w * SC, h * SC}; DrawRectanglePro(r, (Vector2){w * SC / 2, h * SC / 2}, ang, ca(c, GA * c.a / 255.f)); }
static void ch_align(float x, float y, float w, float h) {
    chart_frame(x, y, w, h, "CAMBER ×4 · TOE ×12 (EXAGGERATED)");
    float cf = TWV(HID(K_OUT, 55, 0), X("front.static_camber_deg")), cr = TWV(HID(K_OUT, 55, 1), X("rear.static_camber_deg"));
    float tf = TWV(HID(K_OUT, 55, 2), X("front.static_toe_deg")), tr = TWV(HID(K_OUT, 55, 3), X("rear.static_toe_deg"));
    /* front view (left) */
    float fx = x + w * 0.27f, fy = y + h * 0.56f; line(fx - 80, fy + 34, fx + 80, fy + 34, 1, RGBA(255, 255, 255, 40));
    rrect(fx - 44, fy - 8, 88, 10, 3, RGBA(255, 255, 255, 22));
    wheel_shape(fx - 58, fy, 14, 66, -cf * 4, ca(BL0, 0.9f)); wheel_shape(fx + 58, fy, 14, 66, cf * 4, ca(BL0, 0.9f));
    tx(F_MONO, 10.5f, fx, fy + 44, TextFormat("F %.1f°  R %.1f°", cf, cr), GR1, 0, 1);
    /* top view (right) */
    float tx0 = x + w * 0.72f, ty0 = y + h * 0.55f;
    rrect(tx0 - 24, ty0 - 56, 48, 112, 10, RGBA(255, 255, 255, 14));
    float wt[2] = {tf, tr}; for (int i = 0; i < 2; i++) { float yy = ty0 + (i ? 40 : -40);
        wheel_shape(tx0 - 38, yy, 11, 30, wt[i] * 12, ca(i ? GO0 : BL0, 0.9f)); wheel_shape(tx0 + 38, yy, 11, 30, -wt[i] * 12, ca(i ? GO0 : BL0, 0.9f)); }
    tx(F_MONO, 10.5f, tx0, ty0 + 62, TextFormat("toe F %.2f° R %.2f°", tf, tr), GR1, 0, 1);
}
static void ch_aero(float x, float y, float w, float h) {
    chart_frame(x, y, w, h, "AERO LOADS @ 200 KM/H");
    float cx = x + w / 2, gy = y + h - 62, bx = x + 24, bw = w - 48;
    float dff = TWV(HID(K_OUT, 56, 0), X("df200f")), dfr = TWV(HID(K_OUT, 56, 1), X("df200r")), dr = TWV(HID(K_OUT, 56, 2), X("dr200"));
    char b[48];
    /* drag bar */
    float dy = y + 40; tx(F_SEMI, 10.5f, bx, dy, "DRAG", GR2, 0.18f, 0); snprintf(b, sizeof b, "%.0f N", dr); tx(F_MONO, 11, bx + bw, dy, b, GO0, 0, 2);
    rrect(bx, dy + 20, bw, 6, 3, RGBA(255, 255, 255, 30)); if (dr == dr) rrect(bx, dy + 20, bw * clampf(dr / 3000.f, 0.01f, 1), 6, 3, ca(GO1, 0.85f));
    /* ground + silhouette (front to the right) */
    line(x + 20, gy, x + w - 20, gy, 1, RGBA(255, 255, 255, 40));
    rrect(cx - 110, gy - 34, 220, 20, 8, RGBA(255, 255, 255, 30)); rrect(cx - 44, gy - 52, 84, 22, 9, RGBA(255, 255, 255, 22));
    circ(cx - 72, gy - 10, 10, RGBA(255, 255, 255, 50)); circ(cx + 72, gy - 10, 10, RGBA(255, 255, 255, 50));
    float sc = 36.f / fmaxf(300, fmaxf(fabsf(dff), fabsf(dfr)) * 1.1f), ay = gy - 40;
    float v[2] = {dff, dfr}; float ax[2] = {cx + 72, cx - 72};
    for (int i = 0; i < 2; i++) { if (v[i] != v[i]) continue; float L = fabsf(v[i]) * sc; Color c = v[i] >= 0 ? GN0 : RD0; float tip;
        if (v[i] >= 0) { tip = ay - 14 - L; line(ax[i], tip, ax[i], ay - 12, 3, c);
            DrawTriangle((Vector2){VX(ax[i] - 7), VY(ay - 14)}, (Vector2){VX(ax[i]), VY(ay - 4)}, (Vector2){VX(ax[i] + 7), VY(ay - 14)}, ca(c, GA)); }
        else { tip = ay - 14 - L; line(ax[i], ay - 4, ax[i], tip + 8, 3, c);
            DrawTriangle((Vector2){VX(ax[i] + 7), VY(tip + 10)}, (Vector2){VX(ax[i]), VY(tip)}, (Vector2){VX(ax[i] - 7), VY(tip + 10)}, ca(c, GA)); }
        snprintf(b, sizeof b, "%s %+.0f N", i ? "R" : "F", v[i]); tx(F_MONO, 10.5f, ax[i], tip - 18, b, c, 0, 1); }
    double ab = X("abal"); if (ab == ab) { float f = clampf((float)ab / 100.f, 0, 1), by = y + h - 24;
        rrect(bx, by, bw, 5, 2, RGBA(255, 255, 255, 30)); rrect(bx + bw * f - 3, by - 4, 6, 13, 3, GO0);
        tx(F_MONO, 10, bx, by - 15, "REAR", GR3, 0, 0); tx(F_MONO, 10, bx + bw, by - 15, "FRONT", GR3, 0, 2);
        snprintf(b, sizeof b, "BALANCE %.0f%% F", ab); tx(F_MONO, 10, bx + bw / 2, by - 15, b, GO0, 0, 1); }
}
static void ch_weight(float x, float y, float w, float h) {
    chart_frame(x, y, w, h, "MASS DISTRIBUTION · TOP VIEW");
    float cx = x + w / 2, cy = y + h / 2 + 8;
    float L = 150, Wd = 70; rrect(cx - L / 2 - 26, cy - Wd / 2 - 10, L + 52, Wd + 20, 18, RGBA(255, 255, 255, 16));
    for (int i = 0; i < 2; i++) for (int s = -1; s <= 1; s += 2) rrect(cx + (i ? -L / 2 : L / 2) - 12, cy + s * (Wd / 2 + 4) - 5, 24, 10, 3, RGBA(255, 255, 255, 50));
    float wf = TWV(HID(K_OUT, 57, 0), X("wf")); float gx = cx - L / 2 + L * wf;   /* front axle on the right */
    line(cx - L / 2, cy, cx + L / 2, cy, 1, RGBA(255, 255, 255, 30));
    circ(gx, cy, 12, ca(GO0, 0.16f)); circ(gx, cy, 6, GO0); ring(gx, cy, 9, 10, 0, 360, ca(GO0, 0.6f));
    tx(F_MONO, 11, cx + L / 2 + 30, cy - 8, TextFormat("F %.1f%%", 100 * wf), BL0, 0, 0);
    tx(F_MONO, 11, cx - L / 2 - 30, cy - 8, TextFormat("R %.1f%%", 100 - 100 * wf), GO0, 0, 2);
    tx(F_MONO, 12, cx, y + h - 26, TextFormat("%.0f kg", X("m")), GR0, 0, 1);
}
static void ch_engine(float x, float y, float w, float h) {
    chart_frame(x, y, w, h, "TORQUE · POWER");
    float px = x + 40, py = y + 36, pw = w - 80, ph = h - 64;
    double lim = X("car.limiter_rpm"); if (!(lim > 0)) lim = 7000; double tpk = fmax(50, X("tpk")), ppk = fmax(30, X("ppk"));
    grid(px, py, pw, ph, 7, 4);
    Vector2 pt = {0}, pp = {0};
    for (int i = 0; i <= 64; i++) { double rpm = 800 + (lim - 800) * i / 64.0; char e[48]; snprintf(e, sizeof e, "torque(%.1f)", rpm); double t = X(e); if (t != t) t = 0;
        double kw = t * rpm * 2 * M_PI / 60 / 1000; float xx = px + pw * (float)((rpm - 800) / (lim - 800));
        Vector2 a = {xx, py + ph - ph * (float)(t / (tpk * 1.15))}, b = {xx, py + ph - ph * (float)(kw / (ppk * 1.15))};
        if (i) { line(pt.x, pt.y, a.x, a.y, 2, GO0); line(pp.x, pp.y, b.x, b.y, 2, BL0); } pt = a; pp = b; }
    tx(F_MONO, 10.5f, px, py - 14, TextFormat("%.0f N·m", tpk), GO0, 0, 0); tx(F_MONO, 10.5f, px + pw, py - 14, TextFormat("%.0f kW", ppk), BL0, 0, 2);
    axis_lbl(px + pw, py + ph + 5, TextFormat("%.0f rpm", lim), 2); axis_lbl(px, py + ph + 5, "800", 0);
}
static void ch_settings(float x, float y, float w, float h, int rs_focus) {
    chart_frame(x, y, w, h, "WHEN CHANGES APPLY");
    const char* nm[3] = {"LIVE", "ON DRIVE", "RESTART"}; const char* ds[3] = {"applies immediately", "applied when you press Drive / Apply", "takes effect after restarting the demo"};
    Color cs[3] = {GN0, GO0, RD0};
    for (int i = 0; i < 3; i++) { float yy = y + 44 + i * 52; float a = rs_focus == i ? 1 : 0.45f;
        rrect(x + 16, yy, 100, 28, 14, ca(cs[i], 0.16f * a + 0.04f)); rrline(x + 16, yy, 100, 28, 14, 1, ca(cs[i], a));
        tx(F_SEMI, 11, x + 66, yy + 7, nm[i], ca(cs[i], a), 0.16f, 1); tx(F_INTER, 13, x + 130, yy + 6, ds[i], ca(GR1, a), 0, 0); }
}
static void info_panel(int p, const tune_row* fr, int fk) {
    const tune_page* P = PG(p);
    glass(INFX, TOPY, INFW, PANH, 16, 0.9f, 0);
    char t[64]; snprintf(t, sizeof t, "LIVE · %s", P->title); upcase(t);
    tx(F_MICH, 12.5f, INFX + 26, TOPY + 26, t, GO0, 0.28f, 0);
    float pulse = 0.5f + 0.5f * sinf(M.t * 3.2f); circ(INFX + INFW - 34, TOPY + 33, 3.5f, ca(GN0, 0.55f + 0.45f * pulse)); circ(INFX + INFW - 34, TOPY + 33, 7, ca(GN0, 0.12f * pulse));
    float cx = INFX + 20, cy = TOPY + 60, cw = INFW - 40, chh = 232;
    if (!strcmp(P->panel, "gearing")) ch_gearing(cx, cy, cw, chh);
    else if (!strcmp(P->panel, "springs")) ch_springs(cx, cy, cw, chh);
    else if (!strcmp(P->panel, "tyres")) ch_tyres(cx, cy, cw, chh);
    else if (!strcmp(P->panel, "brakes")) ch_brakes(cx, cy, cw, chh);
    else if (!strcmp(P->panel, "diff")) ch_diff(cx, cy, cw, chh);
    else if (!strcmp(P->panel, "align")) ch_align(cx, cy, cw, chh);
    else if (!strcmp(P->panel, "aero")) ch_aero(cx, cy, cw, chh);
    else if (!strcmp(P->panel, "weight")) ch_weight(cx, cy, cw, chh);
    else if (!strcmp(P->panel, "engine")) ch_engine(cx, cy, cw, chh);
    else ch_settings(cx, cy, cw, chh, fr ? fr->rs : -1);
    float oy = cy + chh + 18;
    if (P->nout) { tx(F_SEMI, 10.5f, INFX + 26, oy, "OUTPUT", GR3, 0.24f, 0); tx(F_SEMI, 10.5f, INFX + INFW - 26, oy, "Δ SAVED", GR3, 0.24f, 2); oy += 22; }
    for (int o = 0; o < P->nout && oy < TOPY + PANH - 190; o++) {
        const tune_out* O = &P->out[o]; int pair = o + 1 < P->nout && !P->out[o + 1].l[0];
        double v = tune_expr(&M.T, O->e), v2 = pair ? tune_expr(&M.T, P->out[o + 1].e) : NAN;
        if (v == v && OPREV[p][o] == OPREV[p][o] && fabs(v - OPREV[p][o]) > 1e-9 * fmax(1, fabs(v))) kick(HID(K_OUTP, p, o), 1);
        OPREV[p][o] = v; if (pair) OPREV[p][o + 1] = v2;
        float* fl = anim(HID(K_OUTP, p, o), 0); *fl = approach(*fl, 0, M.dt, 2.5f);
        if (*fl > 0.01f) rrect(INFX + 14, oy - 5, INFW - 28, 30, 7, ca(GO1, 0.10f * *fl));
        tx(F_INTER, 14, INFX + 26, oy, O->l, GR1, 0, 0);
        char b[64], b2[32];
        if (O->axle) snprintf(b, sizeof b, "%s", v != v ? "—" : v < 0.5 ? "FRONT" : "REAR");
        else { float vr = TWV(HID(K_OUT, p, o), v); tune_fmt(v == v ? vr : v, O->d, b, sizeof b);
            if (pair) { float vr2 = TWV(HID(K_OUT, p, o + 1), v2); tune_fmt(v2 == v2 ? vr2 : v2, O->d, b2, sizeof b2); strncat(b, " / ", sizeof b - strlen(b) - 1); strncat(b, b2, sizeof b - strlen(b) - 1); } }
        int good = O->has_good && v == v ? (v >= O->good[0] && v <= O->good[1]) && (!pair || (v2 >= O->good[0] && v2 <= O->good[1])) : -1;
        Color vc = good == 1 ? GN0 : good == 0 ? GO0 : GR0;
        float ux = INFX + INFW - 92; tx(F_MONO, 15, ux, oy - 1, b, mixc(vc, RGBA(255, 246, 222, 255), *fl), 0, 2);
        tx(F_INTER, 12, ux + 5, oy + 2, O->u, GR2, 0, 0);
        if (O->est) { float ex = INFX + 26 + tw(F_INTER, 14, O->l, 0) + 8; rrline(ex, oy + 1, 30, 15, 4, 1, ca(GR3, 0.9f)); tx(F_SEMI, 9, ex + 15, oy + 3.5f, "EST", GR2, 0.1f, 1); }
        double sv = OSV[p][o], d = v - sv;
        if (!O->axle && v == v && sv == sv && fabs(d) >= 0.5 * pow(10, -O->d)) { char db[32]; snprintf(db, sizeof db, "%s%.*f", d > 0 ? "+" : "−", O->d, fabs(d)); tx(F_MONO, 11.5f, INFX + INFW - 26, oy + 1, db, d > 0 ? GN0 : RD0, 0, 2); }
        oy += 31; if (pair) o++;
    }
    /* focused row */
    float fy = TOPY + PANH - 176; line(INFX + 24, fy, INFX + INFW - 24, fy, 1, RGBA(255, 255, 255, 16));
    if (fr) {
        char l[64]; snprintf(l, sizeof l, "%s%s", fr->l, is_pair(fr) ? (fk ? " · REAR" : " · FRONT") : ""); upcase(l);
        tx(F_MICH, 11, INFX + 26, fy + 16, l, GO1, 0.2f, 0);
        twrap(F_INTER, 14, INFX + 26, fy + 40, INFW - 52, fr->info[0] ? fr->info : P->note, GR1, 1.4f, 4);
        char a[32], s[32], c[32]; fmt_row(fr, rowv(fr, fk, 2), a, sizeof a); fmt_row(fr, rowv(fr, fk, 1), s, sizeof s); fmt_row(fr, rowv(fr, fk, 0), c, sizeof c);
        tx(F_MONO, 12, INFX + 26, fy + 142, TextFormat("STOCK %s · SAVED %s · NOW %s %s", a, s, c, fr->nopt ? "" : fr->u), GR2, 0, 0);
    } else twrap(F_INTER, 14, INFX + 26, fy + 18, INFW - 52, P->note, GR1, 1.4f, 6);
}

/* ================================================================== lists: cars, maps, setups */
static tune_state TMP;
static void cars_scan(void) {
    M.ncar = 0; DIR* d = opendir("cars"); char names[32][200]; int n = 0;
    if (d) { struct dirent* e; while ((e = readdir(d)) && n < 32) { size_t L = strlen(e->d_name); if (L > 4 && !strcmp(e->d_name + L - 4, ".ini") && L < 190) snprintf(names[n++], 200, "cars/%s", e->d_name); } closedir(d); }
    for (int i = 0; i < n; i++) for (int j = i + 1; j < n; j++) if (strcmp(names[j], names[i]) < 0) { char t[200]; memcpy(t, names[i], 200); memcpy(names[i], names[j], 200); memcpy(names[j], t, 200); }
    for (int i = 0; i < n; i++) {
        car_ent* c = &M.car[M.ncar]; memset(c, 0, sizeof *c); snprintf(c->path, sizeof c->path, "%s", names[i]);
        char err[100]; if (M.spec_ok && tune_open(&TMP, &M.S, c->path, NULL, M.H->cfg, err, sizeof err)) { tune_eval(&TMP);
            c->kw = tune_expr(&TMP, "ppk"); c->nm = tune_expr(&TMP, "tpk"); c->kg = tune_expr(&TMP, "m"); c->vt = tune_expr(&TMP, "vt"); c->wf = 100 * tune_expr(&TMP, "wf");
            c->drive = (int)tune_expr(&TMP, "car.drive"); c->ok = 1; tune_close(&TMP); }
        const char* drv = c->drive == 1 ? "FWD" : c->drive == 2 ? "AWD" : "RWD";
        int hatch = strstr(c->path, "hatch") != NULL;
        snprintf(c->label, sizeof c->label, "%s · %s", hatch ? "HATCH" : "ROADSTER", drv);
        snprintf(c->tag, sizeof c->tag, "%s", hatch ? "FRONT-ENGINED · 5-SPEED" : "FRONT-MID · 5-SPEED");
        M.ncar++;
    }
    M.car_cur = 0; const char* b = base_car();
    for (int i = 0; i < M.ncar; i++) if (!strcmp(M.car[i].path, b)) M.car_cur = i;
    M.car_sel = M.car_cur;
}
static void map_title_of(int i, char* out, int n) {
    if (!M.H || i < 0 || i >= M.H->nmaps) { snprintf(out, n, "-"); return; }
    const char* m = M.H->maps[i];
    if (!strcmp(m, "proving")) { snprintf(out, n, "PROVING GROUND"); return; }
    if (!strcmp(m, "circuit")) { snprintf(out, n, "CIRCUIT"); return; }
    char p[300]; snprintf(p, sizeof p, "%s/track.cfg", m); FILE* f = fopen(p, "r"); out[0] = 0;
    if (f) { char ln[300]; while (fgets(ln, sizeof ln, f)) { char* s = ln; while (*s == ' ' || *s == '\t') s++;
        if (!strncmp(s, "name", 4)) { char* q = strchr(s, '='); if (q) { q++; while (*q == ' ') q++; char* e = q + strlen(q); while (e > q && (e[-1] == '\n' || e[-1] == '\r' || e[-1] == ' ')) *--e = 0; snprintf(out, n, "%s", q); break; } } } fclose(f); }
    if (!out[0]) { const char* s = strrchr(m, '/'); snprintf(out, n, "%s", s ? s + 1 : m); }
}
static void setups_scan(void) {
    M.nset = 0; DIR* d = opendir("user/setups"); if (!d) return; struct dirent* e;
    while ((e = readdir(d)) && M.nset < 48) { size_t L = strlen(e->d_name); if (L < 5 || strcmp(e->d_name + L - 4, ".ini") || L > 70) continue;
        setup_ent* s = &M.set[M.nset++]; snprintf(s->path, sizeof s->path, "user/setups/%s", e->d_name); snprintf(s->name, sizeof s->name, "%.*s", (int)(L - 4), e->d_name);
        struct stat st; if (!stat(s->path, &st)) { time_t tt = st.st_mtime; struct tm* tmp = localtime(&tt); if (tmp) strftime(s->when, sizeof s->when, "%d %b %Y · %H:%M", tmp); else s->when[0] = 0; } else s->when[0] = 0; }
    closedir(d);
    for (int i = 0; i < M.nset; i++) for (int j = i + 1; j < M.nset; j++) if (strcmp(M.set[j].name, M.set[i].name) < 0) { setup_ent t = M.set[i]; M.set[i] = M.set[j]; M.set[j] = t; }
}

/* ================================================================== navigation + actions */
enum { B_REVERT = 1, B_STOCK, B_SAVE, B_DRIVE, B_BACK };
static int BTN[6], NBTN;
static int home(void) { return M.in_game ? SC_PAUSE : SC_HUB; }
static void go(int s) {
    if (M.scr == SC_SETTINGS && s != SC_SETTINGS && M.t_ok && M.T.dirty_demo) { char out[300], err[200]; if (tune_write(&M.T, "user", out, sizeof out, err, sizeof err)) status_set(GR1, "Settings saved"); }
    M.scr = s; M.scr_t = M.t; M.col = 0; M.btn = NBTN ? NBTN - 1 : 0; M.scroll = 0;
    if (s == SC_HUB || s == SC_TITLE || s == SC_PAUSE) M.nav = 0;
    if (s == SC_CAR) M.car_sel = M.car_cur;
    if (s == SC_MAP) M.map_sel = M.map_pick >= 0 ? M.map_pick : (M.H->cur_map ? *M.H->cur_map : 0);
    if (s == SC_SETUPS) { setups_scan(); M.set_sel = 0; }
}
static int demo_changed(int rs) {
    if (!M.t_ok) return 0;
    for (int p = 0; p < M.S.npage; p++) for (int r = 0; r < PG(p)->nrow; r++) { const tune_row* R = &PG(p)->row[r];
        if (R->t != TR_DEMO || (rs >= 0 && R->rs != rs)) continue;
        const char* a = cfg_str(M.H->cfg, R->k, ""), *b = cfg_str(&M.T.demo_saved, R->k, ""); if (strcmp(a, b)) return 1; }
    return 0;
}
static int car_rows_changed(void) {
    if (!M.t_ok) return 0;
    for (int p = 0; p < M.S.npage; p++) for (int r = 0; r < PG(p)->nrow; r++) { const tune_row* R = &PG(p)->row[r]; if (R->t != TR_DEMO && row_changed(R)) return 1; }
    return 0;
}
static int map_changed(void) { return M.map_pick >= 0 && M.H->cur_map && M.map_pick != *M.H->cur_map; }
static int write_tune(void) {
    char out[300], err[200];
    if (!tune_write(&M.T, "user", out, sizeof out, err, sizeof err)) { status_set(RD0, "%s", err); return 0; }
    cfg_set(M.H->cfg, "ui.base_car", M.T.base_path); cfg_set(M.H->cfg, "car.ini", out); return 1;
}
static menu_action do_drive(void) {
    int car = car_rows_changed(), reinit_demo = demo_changed(1) || demo_changed(2), mp = map_changed(), cc = M.car_changed;
    if (M.t_ok && (car || demo_changed(-1) || cc) && !write_tune()) return MA_NONE;
    if (!M.in_game) { M.need_reinit = car || reinit_demo || mp || cc; return MA_DRIVE; }
    if (reinit_demo || mp || cc) { M.need_reinit = 1; return MA_DRIVE; }
    if (car) { M.need_reinit = 0; return MA_APPLY; }
    return MA_RESUME;
}
static void save_setup(void) {
    if (!M.t_ok) return;
    setups_scan(); char nm[40]; int k = 1;
    for (;; k++) { snprintf(nm, sizeof nm, "tune-%02d", k); int dup = 0; for (int i = 0; i < M.nset; i++) if (!strcmp(M.set[i].name, nm)) dup = 1; if (!dup || k > 98) break; }
    char err[200]; if (tune_setup_save(&M.T, "user/setups", nm, err, sizeof err)) status_set(GN0, "Setup saved as %s", nm); else status_set(RD0, "%s", err);
    setups_scan();
}
static menu_action press_button(int code) {
    int p = M.scr == SC_SETTINGS ? M.sp[M.spage] : M.tp[M.tpage];
    switch (code) {
    case B_REVERT: for (int r = 0; r < PG(p)->nrow; r++) row_revert(p, r, 0); status_set(GR1, "%s reverted to saved", PG(p)->title); break;
    case B_STOCK: for (int r = 0; r < PG(p)->nrow; r++) row_revert(p, r, 1); status_set(GR1, "%s set to stock", PG(p)->title); break;
    case B_SAVE: save_setup(); break;
    case B_DRIVE: return do_drive();
    case B_BACK: go(home()); break;
    }
    return MA_NONE;
}

/* ================================================================== screens: draw */
static const char* RSN[3] = {"LIVE", "ON DRIVE", "RESTART"};
static void bottom_buttons(int focus) {
    float x = 1848; float ws[6];
    for (int i = NBTN - 1; i >= 0; i--) { const char* l = BTN[i] == B_REVERT ? "REVERT" : BTN[i] == B_STOCK ? "STOCK" : BTN[i] == B_SAVE ? "SAVE SETUP" : BTN[i] == B_BACK ? "BACK" : (M.in_game ? "APPLY & RESUME" : "DRIVE");
        ws[i] = BTN[i] == B_DRIVE ? 236 : tw(F_MICH, 13, l, 0.26f) + 56; x -= ws[i];
        button(HID(K_BTN, i, 0), x, BOTY, ws[i], 54, l, BTN[i] == B_DRIVE, focus && M.btn == i, 1); x -= 14; }
}
static void tune_screen(int settings) {
    int* pl = settings ? M.sp : M.tp; int np = settings ? M.nsp : M.ntp, *pi = settings ? &M.spage : &M.tpage;
    if (np <= 0) return;
    if (*pi >= np) *pi = np - 1;
    int p = pl[*pi];
    const char* items[20]; const char* tags[20]; char tg[20][8];
    for (int i = 0; i < np; i++) { items[i] = PG(pl[i])->title; snprintf(tg[i], 8, "%s", page_changed(pl[i]) ? "*" : ""); tags[i] = tg[i]; }
    static char up[20][48]; for (int i = 0; i < np; i++) { snprintf(up[i], 48, "%s", items[i]); upcase(up[i]); items[i] = up[i]; }
    navlist(settings ? "SETTINGS" : "GARAGE · TUNING", items, tags, np, *pi, M.col == 0, TOPY + 62, 0);
    /* centre */
    glass(CENX, TOPY, CENW, PANH, 16, 0.9f, 0);
    char t[64]; snprintf(t, sizeof t, "%s", PG(p)->title); upcase(t);
    tx(F_MICH, 24, CENX + 40, TOPY + 30, t, GR0, 0.18f, 0);
    if (!settings) { char n[24]; snprintf(n, sizeof n, "%02d / %02d", *pi + 1, np); tx(F_MONO, 13, CENX + CENW - 40, TOPY + 38, n, GR3, 0, 2); }
    twrap(F_INTER, 14, CENX + 40, TOPY + 72, CENW - 80, PG(p)->note, GR2, 1.35f, 2);
    float hy = TOPY + 128; line(CENX + 36, hy + 20, CENX + CENW - 36, hy + 20, 1, RGBA(255, 255, 255, 14));
    tx(F_SEMI, 10.5f, CENX + 40, hy, "PARAMETER", GR3, 0.24f, 0); tx(F_SEMI, 10.5f, TX0, hy, "ADJUST", GR3, 0.24f, 0);
    tx(F_SEMI, 10.5f, CENX + 862, hy, "VALUE", GR3, 0.24f, 2); tx(F_SEMI, 10.5f, CENX + 962, hy, "Δ SAVED", GR3, 0.24f, 2);
    if (!PG(p)->nrow) tx(F_INTER, 15, CENX + 40, ROWS_Y + 20, "Read-only page: the engine map is part of the car file. The live panel shows the curves.", GR2, 0, 0);
    tune_rows(p, M.col == 1 ? M.item : -1, M.col == 1, 1);
    /* settings: apply-class badges */
    if (settings) { line_t L[32]; int n = build_lines(p, L, NULL); float sc = *anim(HID(K_SEL, 3, p), 0); int last = -1;
        BeginScissorMode((int)VX(CENX), (int)VY(ROWS_Y - 6), (int)(CENW * SC), (int)((ROWS_H + 12) * SC));
        for (int i = 0; i < n; i++) { if (L[i].r == last) continue; last = L[i].r; const tune_row* R = &PG(p)->row[L[i].r]; int rs = R->rs < 0 || R->rs > 2 ? 1 : R->rs;
            Color c = rs == 0 ? GN0 : rs == 1 ? GO0 : RD0; float by = ROWS_Y + L[i].y - sc; float w = tw(F_SEMI, 9.5f, RSN[rs], 0.14f) + 14;
            rrline(CENX + 300 - w, by + 37, w, 16, 8, 1, ca(c, 0.7f)); tx(F_SEMI, 9.5f, CENX + 300 - w / 2, by + 40, RSN[rs], c, 0.14f, 1); }
        EndScissorMode(); }
    /* right */
    line_t L[32]; int n = build_lines(p, L, NULL);
    const tune_row* fr = M.col == 1 && M.item < n ? &PG(p)->row[L[M.item].r] : NULL;
    info_panel(p, fr, fr ? L[M.item].k : 0);
    NBTN = 0; BTN[NBTN++] = B_REVERT; if (!settings) { BTN[NBTN++] = B_STOCK; BTN[NBTN++] = B_SAVE; } BTN[NBTN++] = B_DRIVE;
    bottom_buttons(M.col == 2);
    hints(M.col == 0 ? "↑↓ page   → / Enter edit   Q E page   Esc back" :
          M.col == 1 ? "↑↓ row   ←→ adjust (hold)   Shift fine · Ctrl coarse   R revert   L link F/R   Esc back" : "←→ choose   Enter press   ↑ rows   Esc back");
}
static void stat_cell(float x, float y, float w, const char* lab, const char* val, const char* unit) {
    tx(F_SEMI, 10.5f, x, y, lab, GR2, 0.24f, 0); tx(F_MONO, 26, x, y + 22, val, GR0, 0, 0); tx(F_INTER, 13, x + tw(F_MONO, 26, val, 0) + 6, y + 34, unit, GR2, 0, 0); (void)w;
}
static void hub_screen(void) {
    static const char* it[] = {"DRIVE", "CAR", "TRACK", "TUNING", "SETUPS", "SETTINGS", "QUIT"};
    const char* tags[7] = {0}; char tun[8] = ""; if (M.t_ok && tune_changed(&M.T)) snprintf(tun, sizeof tun, "*"); tags[3] = tun;
    navlist("GARAGE", it, tags, 7, M.nav, 1, TOPY + 62, 0);
    car_ent* c = M.car_cur < M.ncar ? &M.car[M.car_cur] : NULL;
    tx(F_MICH, 42, CENX + 40, TOPY + 14, c ? c->label : "CAR", GR0, 0.16f, 0);
    tx(F_SEMI, 13, CENX + 44, TOPY + 76, c ? c->tag : "", GO0, 0.3f, 0);
    /* stats strip */
    float sy = TOPY + PANH - 128; glass(CENX, sy, CENW, 128, 16, 0.9f, 0);
    char b[5][24]; snprintf(b[0], 24, "%.0f", X("ppk")); snprintf(b[1], 24, "%.0f", X("tpk")); snprintf(b[2], 24, "%.0f", X("m")); snprintf(b[3], 24, "%.1f", 100 * X("wf")); snprintf(b[4], 24, "%.0f", X("vt"));
    const char* lab[5] = {"PEAK POWER", "PEAK TORQUE", "MASS", "FRONT WEIGHT", "TOP SPEED (EST)"}; const char* un[5] = {"kW", "N·m", "kg", "%", "km/h"};
    for (int i = 0; i < 5; i++) { float x = CENX + 36 + i * (CENW - 72) / 5.f; if (i) line(x - 18, sy + 28, x - 18, sy + 100, 1, RGBA(255, 255, 255, 14)); stat_cell(x, sy + 34, 160, lab[i], b[i], un[i]); }
    /* session */
    glass(INFX, TOPY, INFW, PANH, 16, 0.9f, 0);
    tx(F_MICH, 12.5f, INFX + 26, TOPY + 26, "SESSION", GO0, 0.28f, 0);
    char mp[96]; map_title_of(M.map_pick >= 0 ? M.map_pick : (M.H->cur_map ? *M.H->cur_map : 0), mp, sizeof mp);
    const char* dev = M.H->dev_kind == 1 ? "Wheel" : M.H->dev_kind == 2 ? "Gamepad" : "Keyboard";
    char setup[64]; snprintf(setup, sizeof setup, "%s", M.t_ok && tune_changed(&M.T) ? "Edited (unsaved)" : (strstr(cfg_str(M.H->cfg, "car.ini", ""), "user/") ? "Garage tune" : "Stock"));
    const char* kv[][2] = {{"Car", c ? c->label : "-"}, {"Track", mp}, {"Tune", setup}, {"Input", dev}, {"Physics", cfg_str(M.H->cfg, "physics.model", "fast")}, {"Camera", cfg_str(M.H->cfg, "camera.mode", "chase")}};
    for (int i = 0; i < 6; i++) { float y = TOPY + 76 + i * 46; tx(F_INTER, 14, INFX + 26, y, kv[i][0], GR2, 0, 0); tx(F_SEMI, 15, INFX + INFW - 26, y - 1, kv[i][1], GR0, 0, 2);
        line(INFX + 26, y + 30, INFX + INFW - 26, y + 30, 1, RGBA(255, 255, 255, 10)); }
    twrap(F_INTER, 14, INFX + 26, TOPY + 380, INFW - 52, "Tune the car in the garage: every slider updates the live panel from the same model the physics loads. Drive writes the tune and starts the session; Esc in the car opens the pause menu.", GR2, 1.45f, 6);
    NBTN = 0; BTN[NBTN++] = B_DRIVE; bottom_buttons(0);
    hints("↑↓ choose   Enter select   Esc title");
}
static void list_screen(const char* title, int n, int sel, const char** labels, const char** tags) {
    navlist(title, labels, tags, n, sel, 1, TOPY + 62, 0);
}
static void car_screen(void) {
    const char* lb[32]; const char* tg[32]; for (int i = 0; i < M.ncar; i++) { lb[i] = M.car[i].label; tg[i] = i == M.car_cur ? "*" : ""; }
    list_screen("CAR", M.ncar, M.car_sel, lb, tg);
    car_ent* c = M.car_sel < M.ncar ? &M.car[M.car_sel] : NULL; if (!c) return;
    tx(F_MICH, 42, CENX + 40, TOPY + 14, c->label, GR0, 0.16f, 0); tx(F_SEMI, 13, CENX + 44, TOPY + 76, c->tag, GO1, 0.3f, 0);
    glass(INFX, TOPY, INFW, PANH, 16, 0.9f, 0); tx(F_MICH, 12.5f, INFX + 26, TOPY + 26, "SPECIFICATION", GO0, 0.28f, 0);
    const char* lab[5] = {"Peak power", "Peak torque", "Mass", "Front weight", "Top speed (est)"}; double v[5] = {c->kw, c->nm, c->kg, c->wf, c->vt}; const char* u[5] = {"kW", "N·m", "kg", "%", "km/h"};
    double mx[5] = {250, 350, 1800, 70, 300};
    for (int i = 0; i < 5; i++) { float y = TOPY + 80 + i * 70; tx(F_INTER, 14, INFX + 26, y, lab[i], GR2, 0, 0);
        char b[24]; snprintf(b, sizeof b, i == 3 ? "%.1f %s" : "%.0f %s", v[i], u[i]); tx(F_MONO, 15, INFX + INFW - 26, y - 1, b, GR0, 0, 2);
        float f = clampf(TWV(HID(K_OUT, 60, i), v[i] / mx[i]), 0, 1); rrect(INFX + 26, y + 28, INFW - 52, 4, 2, RGBA(255, 255, 255, 14)); rrect(INFX + 26, y + 28, (INFW - 52) * f, 4, 2, GO1); }
    twrap(F_INTER, 14, INFX + 26, TOPY + 470, INFW - 52, "Switching car resets the tune to that car's stock file. Drive builds it.", GR2, 1.45f, 4);
    NBTN = 0; BTN[NBTN++] = B_DRIVE; bottom_buttons(0);
    hints("↑↓ choose   Enter select   Esc back");
}
static void map_screen(void) {
    static char t[32][96]; const char* lb[32]; const char* tg[32]; int n = M.H->nmaps < 32 ? M.H->nmaps : 32; int cur = M.map_pick >= 0 ? M.map_pick : *M.H->cur_map;
    for (int i = 0; i < n; i++) { map_title_of(i, t[i], 96); upcase(t[i]); lb[i] = t[i]; tg[i] = i == cur ? "*" : ""; }
    list_screen("TRACK", n, M.map_sel, lb, tg);
    if (M.map_sel < n) { tx(F_MICH, 42, CENX + 40, TOPY + 14, lb[M.map_sel], GR0, 0.16f, 0);
        const char* m = M.H->maps[M.map_sel]; tx(F_SEMI, 13, CENX + 44, TOPY + 76, !strcmp(m, "proving") ? "FLAT PAD · TEST GROUND" : !strcmp(m, "circuit") ? "GENERATED CIRCUIT" : "MESH TRACK", GO1, 0.3f, 0); }
    NBTN = 0; BTN[NBTN++] = B_DRIVE; bottom_buttons(0);
    hints("↑↓ choose   Enter select   Esc back");
}
static void setups_screen(void) {
    static char lb2[49][96]; const char* lb[49]; const char* tg[49]; int n = 1 + M.nset; if (n > 12) n = 12;
    snprintf(lb2[0], 96, "+ SAVE CURRENT"); lb[0] = lb2[0]; tg[0] = "";
    for (int i = 1; i < n; i++) { snprintf(lb2[i], 96, "%s", M.set[i - 1].name); upcase(lb2[i]); lb[i] = lb2[i]; tg[i] = ""; }
    list_screen("SETUPS", n, M.set_sel, lb, tg);
    glass(INFX, TOPY, INFW, PANH, 16, 0.9f, 0); tx(F_MICH, 12.5f, INFX + 26, TOPY + 26, "SETUP", GO0, 0.28f, 0);
    if (M.set_sel == 0) twrap(F_INTER, 15, INFX + 26, TOPY + 70, INFW - 52, "Saves every tuning value of the current car (edits included) as a new named setup under user/setups. Load one to replace the current edits; Drive applies it.", GR1, 1.45f, 8);
    else { setup_ent* s = &M.set[M.set_sel - 1]; tx(F_MICH, 22, INFX + 26, TOPY + 70, lb[M.set_sel], GR0, 0.16f, 0); tx(F_MONO, 13, INFX + 26, TOPY + 110, s->when, GR2, 0, 0);
        twrap(F_INTER, 15, INFX + 26, TOPY + 150, INFW - 52, "Enter loads this setup into the garage (unsaved until you Drive).", GR1, 1.45f, 4); }
    NBTN = 0; BTN[NBTN++] = B_DRIVE; bottom_buttons(0);
    hints("↑↓ choose   Enter save / load   Esc back");
}
static void title_screen(void) {
    float e = ease_out((M.t - M.scr_t) / 0.6f);
    gradh(0, 0, 1100, 1080, RGBA(5, 7, 10, 210), RGBA(5, 7, 10, 0));
    tx(F_MICH, 84, 140, 330 + 18 * (1 - e), "TARMAC LAB", GR0, 0.34f, 0);
    gradh(144, 450, 520, 1, ca(GO0, 0.9f), ca(GO0, 0));
    tx(F_SEMI, 15, 146, 470, "DRIVECHAIN · GARAGE & TUNING", GO1, 0.38f, 0);
    static const char* it[] = {"START", "SETTINGS", "QUIT"};
    for (int i = 0; i < 3; i++) { unsigned id = HID(K_NAV, i, 0); float y = 560 + i * 64, hv = hov(id); hit(id, 140, y, 360, 52);
        float f = tween(HID(K_SEL, 7, i), M.nav == i ? 1.f : 0.f, 14);
        if (f > 0.01f) { glow(140, y, 360, 52, 10, 8, GO1, 0.25f * f); rrect(140, y, 360, 52, 10, RGBA(255, 255, 255, (unsigned char)(14 * f))); rect(140, y + 14, 3, 24, ca(GO0, f)); }
        tx(F_MICH, 17, 168 + 6 * f, y + 15, it[i], mixc(GR2, GR0, fmaxf(f, hv)), 0.3f, 0); }
    hints("↑↓ choose   Enter select   mouse, keyboard, gamepad or wheel (paddles + pedals)");
}
static void pause_screen(void) {
    rect(0, 0, 1920, 1080, RGBA(5, 7, 10, 90));
    float x = 72, y = TOPY, w = 520; glass(x, y, w, PANH, 16, 0.9f, 0);
    tx(F_MICH, 30, x + 36, y + 32, "PAUSED", GR0, 0.24f, 0);
    char mp[96]; map_title_of(*M.H->cur_map, mp, sizeof mp); upcase(mp); tx(F_SEMI, 12, x + 38, y + 82, mp, GO1, 0.28f, 0);
    int ch = M.t_ok && (car_rows_changed() || demo_changed(-1));
    const char* it[7] = {"RESUME", ch ? "APPLY TUNE & RESUME" : "APPLY TUNE", "TUNING", "SETTINGS", "RESTART SESSION", "QUIT TO GARAGE", "QUIT TO DESKTOP"};
    for (int i = 0; i < 7; i++) { unsigned id = HID(K_NAV, i, 0); float yy = y + 138 + i * 60, hv = hov(id); hit(id, x + 20, yy, w - 40, 50);
        float f = tween(HID(K_SEL, 8, i), M.nav == i ? 1.f : 0.f, 14); int dis = i == 1 && !ch;
        if (f > 0.01f) { rrect(x + 20, yy, w - 40, 50, 10, RGBA(255, 255, 255, (unsigned char)(14 * f))); rect(x + 20, yy + 13, 3, 24, ca(GO0, f)); }
        tx(F_MICH, 14, x + 44 + 4 * f, yy + 16, it[i], ca(mixc(GR2, GR0, fmaxf(f, hv)), dis ? 0.4f : 1), 0.24f, 0); }
    /* telemetry */
    const sim_snap* V = M.H->view; glass(INFX, TOPY, INFW, PANH, 16, 0.9f, 0); tx(F_MICH, 12.5f, INFX + 26, TOPY + 26, "TELEMETRY", GO0, 0.28f, 0);
    if (V) { const dc_telemetry* T = &V->t; char g[8]; snprintf(g, sizeof g, "%s", T->gear < 0 ? "R" : T->gear == 0 ? "N" : TextFormat("%d", T->gear));
        tx(F_MONO, 64, INFX + 26, TOPY + 66, TextFormat("%.0f", fabs(T->speed) * 3.6), GR0, 0, 0); tx(F_INTER, 15, INFX + 30, TOPY + 140, "km/h", GR2, 0, 0);
        tx(F_MONO, 64, INFX + INFW - 26, TOPY + 66, g, GO0, 0, 2); tx(F_INTER, 15, INFX + INFW - 30, TOPY + 140, "gear", GR2, 0, 2);
        float rf = clampf((float)(T->engine_rpm / fmax(1000, X("car.limiter_rpm"))), 0, 1);
        rrect(INFX + 26, TOPY + 180, INFW - 52, 6, 3, RGBA(255, 255, 255, 14)); rrect(INFX + 26, TOPY + 180, (INFW - 52) * rf, 6, 3, rf > 0.92f ? RD0 : GO1);
        tx(F_MONO, 12, INFX + INFW - 26, TOPY + 192, TextFormat("%.0f rpm", T->engine_rpm), GR2, 0, 2);
        char tb[6][32]; snprintf(tb[0], 32, "%.2f g", T->ay / 9.81); snprintf(tb[1], 32, "%.2f g", T->ax / 9.81); snprintf(tb[2], 32, "%.1f °/s", T->yaw_rate * 57.2958);
        snprintf(tb[3], 32, "%.0f °", T->sw_angle * 57.2958); snprintf(tb[4], 32, "%.1f s", V->sim_time); snprintf(tb[5], 32, "%.2f ×", V->rt_factor);
        const char* kv[][2] = {{"Lateral", tb[0]}, {"Longitudinal", tb[1]}, {"Yaw rate", tb[2]}, {"Steering", tb[3]}, {"Sim time", tb[4]}, {"Realtime", tb[5]}};
        for (int i = 0; i < 6; i++) { float yy = TOPY + 236 + i * 42; tx(F_INTER, 14, INFX + 26, yy, kv[i][0], GR2, 0, 0); tx(F_MONO, 14, INFX + INFW - 26, yy, kv[i][1], GR0, 0, 2);
            line(INFX + 26, yy + 28, INFX + INFW - 26, yy + 28, 1, RGBA(255, 255, 255, 10)); }
        float cxx = INFX + INFW / 2, cyy = TOPY + 600; ring(cxx, cyy, 70, 71, 0, 360, RGBA(255, 255, 255, 30)); ring(cxx, cyy, 35, 36, 0, 360, RGBA(255, 255, 255, 20));
        line(cxx - 76, cyy, cxx + 76, cyy, 1, RGBA(255, 255, 255, 16)); line(cxx, cyy - 76, cxx, cyy + 76, 1, RGBA(255, 255, 255, 16));
        float gx = clampf((float)(T->ay / 9.81 / 1.2), -1, 1) * 70, gy = clampf((float)(-T->ax / 9.81 / 1.2), -1, 1) * 70;
        circ(cxx + gx, cyy + gy, 6, GO0); circ(cxx + gx, cyy + gy, 12, ca(GO0, 0.18f)); tx(F_SEMI, 10, cxx, cyy + 84, "G-G · 1.2 g RING", GR3, 0.2f, 1); }
    hints("↑↓ choose   Enter select   Esc resume");
}

/* ================================================================== screens: update */
static int PREV_SCR = SC_HUB, LIVE_HIT;
static int line_of(int p, int ri, int k) { line_t L[32]; int n = build_lines(p, L, NULL); for (int i = 0; i < n; i++) if (L[i].r == ri && L[i].k == k) return i; return 0; }
static void touched(int p, int ri) { if (PG(p)->row[ri].t == TR_DEMO && PG(p)->row[ri].rs == 0) LIVE_HIT = 1; }
static menu_action upd_tune(int settings) {
    int* pl = settings ? M.sp : M.tp; int np = settings ? M.nsp : M.ntp, *pi = settings ? &M.spage : &M.tpage;
    if (np <= 0) { if (IN.back) go(settings ? PREV_SCR : home()); return MA_NONE; }
    int p = pl[*pi]; line_t L[32]; int n = build_lines(p, L, NULL); menu_action act = MA_NONE;
    if (M.item >= n) M.item = n ? n - 1 : 0;
    /* mouse */
    if (M.drag_item) {
        unsigned id = (unsigned)M.drag_item; int key = ha(id), k2 = hb(id), pp = key >> 4, ri = key & 15;
        if (MDOWN && pp < M.S.npage) { const tune_row* r = &PG(pp)->row[ri]; float f = clampf((MXv - TX0) / TW, 0, 1); row_set_abs(pp, ri, k2, r->min + f * (r->max - r->min), 1); touched(pp, ri); }
        else M.drag_item = 0;
    } else if (HOVER) {
        int k = hk(HOVER), a = ha(HOVER), b = hb(HOVER);
        int rp = a >> 4, ri = a & 15;
        if ((k == K_ITEM || k == K_TRACK || k == K_OPT) && rp == p) {
            int kk = k == K_OPT ? b / 8 : b;
            if (MMOVED || MCLICK) { M.col = 1; M.item = line_of(p, ri, kk); }
            if (MCLICK && k == K_TRACK) { M.drag_item = (int)HOVER; const tune_row* r = &PG(p)->row[ri]; float f = clampf((MXv - TX0) / TW, 0, 1); row_set_abs(p, ri, kk, r->min + f * (r->max - r->min), 1); touched(p, ri); }
            if (MCLICK && k == K_OPT) { row_set_abs(p, ri, kk, b % 8, 0); touched(p, ri); }
            if (MWHEEL != 0) { row_step(p, ri, kk, MWHEEL > 0 ? 1 : -1); touched(p, ri); }
        }
        if (MCLICK && k == K_NAV && a < np) { if (a != *pi) { *pi = a; M.item = 0; M.scroll = 0; } M.col = 0; }
        if (MCLICK && k == K_LINK && rp == p) M.link[p][ri] ^= 1;
        if (MCLICK && k == K_BTN && a < NBTN) { M.col = 2; M.btn = a; act = press_button(BTN[a]); if (settings && BTN[a] == B_REVERT) LIVE_HIT = 1; }
    }
    if (act) return act;
    /* keys / pad / wheel */
    if (IN.tabl || IN.tabr) { *pi = (*pi + (IN.tabr ? 1 : np - 1)) % np; M.item = 0; M.scroll = 0; if (M.col == 2) M.col = 1; return MA_NONE; }
    if (M.col == 0) {
        if (IN.up) *pi = *pi > 0 ? *pi - 1 : 0;
        if (IN.down) *pi = *pi < np - 1 ? *pi + 1 : np - 1;
        if (IN.up || IN.down) { M.item = 0; M.scroll = 0; }
        if (IN.right || IN.ok) { M.col = n ? 1 : 2; M.item = 0; M.btn = NBTN - 1; }
        if (IN.back) go(settings ? PREV_SCR : home());
    } else if (M.col == 1) {
        if (!n) { M.col = 0; return MA_NONE; }
        const tune_row* r = &PG(p)->row[L[M.item].r];
        for (int i = 0; i < IN.up; i++) if (M.item > 0) M.item--;
        for (int i = 0; i < IN.down; i++) { if (M.item < n - 1) M.item++; else { M.col = 2; M.btn = NBTN - 1; break; } }
        if (M.col == 1 && !IN.up && !IN.down) {
            int d = IN.right - IN.left; if (d) { row_step(p, L[M.item].r, L[M.item].k, d); touched(p, L[M.item].r); }
            if (IN.ok && r->nopt) { int v = (int)lround(rowv(r, L[M.item].k, 0)); row_set_abs(p, L[M.item].r, L[M.item].k, (v + 1) % r->nopt, 0); touched(p, L[M.item].r); }
            if (IN.x) { row_revert(p, L[M.item].r, IN.fine); touched(p, L[M.item].r); status_set(GR1, "%s %s", r->l, IN.fine ? "set to stock" : "reverted"); }
            if (IN.y && is_pair(r)) M.link[p][L[M.item].r] ^= 1;
            if (IN.back) M.col = 0;
        }
    } else {
        if (IN.left && M.btn > 0) M.btn--;
        if (IN.right && M.btn < NBTN - 1) M.btn++;
        if (IN.up) { M.col = n ? 1 : 0; M.item = n ? n - 1 : 0; }
        if (IN.ok) { act = press_button(BTN[M.btn]); if (settings && BTN[M.btn] == B_REVERT) LIVE_HIT = 1; }
        if (IN.back) M.col = 0;
    }
    return act;
}
static int list_nav(int* sel, int n) {   /* returns 1 when the selection is activated */
    for (int i = 0; i < IN.up; i++) if (*sel > 0) (*sel)--;
    for (int i = 0; i < IN.down; i++) if (*sel < n - 1) (*sel)++;
    if (HOVER && hk(HOVER) == K_NAV && ha(HOVER) < n) { if (MMOVED) *sel = ha(HOVER); if (MCLICK) { *sel = ha(HOVER); return 1; } }
    return IN.ok;
}
static int btn_click(void) { return MCLICK && HOVER && hk(HOVER) == K_BTN && ha(HOVER) < NBTN; }
static menu_action upd_screen(void) {
    switch (M.scr) {
    case SC_TITLE: if (list_nav(&M.nav, 3)) { if (M.nav == 0) go(SC_HUB); else if (M.nav == 1) { PREV_SCR = SC_TITLE; go(SC_SETTINGS); } else return MA_QUIT; } break;
    case SC_HUB:
        if (btn_click()) return do_drive();
        if (list_nav(&M.nav, 7)) switch (M.nav) {
            case 0: return do_drive();
            case 1: go(SC_CAR); break; case 2: go(SC_MAP); break; case 3: go(SC_TUNE); break; case 4: go(SC_SETUPS); break;
            case 5: PREV_SCR = SC_HUB; go(SC_SETTINGS); break; case 6: return MA_QUIT; }
        else if (IN.back) go(SC_TITLE);
        break;
    case SC_CAR:
        if (btn_click()) return do_drive();
        if (list_nav(&M.car_sel, M.ncar)) {
            if (M.car_sel != M.car_cur) { car_ent* c = &M.car[M.car_sel]; cfg_set(M.H->cfg, "ui.base_car", c->path); cfg_set(M.H->cfg, "car.ini", c->path);
                M.car_cur = M.car_sel; M.car_changed = 1; tune_reload(); status_set(GO0, "%s selected · stock tune", c->label); }
            go(SC_HUB); M.nav = 1;
        } else if (IN.back) { go(SC_HUB); M.nav = 1; }
        break;
    case SC_MAP: {
        if (btn_click()) return do_drive();
        int n = M.H->nmaps < 32 ? M.H->nmaps : 32;
        if (list_nav(&M.map_sel, n)) { M.map_pick = M.map_sel == *M.H->cur_map ? -1 : M.map_sel; char t[96]; map_title_of(M.map_sel, t, sizeof t); status_set(GO0, "Track: %s", t); go(SC_HUB); M.nav = 2; }
        else if (IN.back) { go(SC_HUB); M.nav = 2; }
        break; }
    case SC_SETUPS: {
        if (btn_click()) return do_drive();
        int n = 1 + M.nset; if (n > 12) n = 12;
        if (list_nav(&M.set_sel, n)) { if (M.set_sel == 0) save_setup();
            else { char err[200]; if (tune_setup_load(&M.T, M.set[M.set_sel - 1].path, err, sizeof err)) status_set(GN0, "Loaded %s · Drive to apply", M.set[M.set_sel - 1].name); else status_set(RD0, "%s", err); } }
        else if (IN.back) { go(home()); M.nav = M.in_game ? 2 : 4; }
        break; }
    case SC_TUNE: return upd_tune(0);
    case SC_SETTINGS: return upd_tune(1);
    case SC_PAUSE:
        if (IN.start) return MA_RESUME;
        if (list_nav(&M.nav, 7)) switch (M.nav) {
            case 0: return MA_RESUME;
            case 1: if (M.t_ok && (car_rows_changed() || demo_changed(-1))) return do_drive(); break;
            case 2: go(SC_TUNE); break; case 3: PREV_SCR = SC_PAUSE; go(SC_SETTINGS); break;
            case 4: if (M.t_ok && (car_rows_changed() || demo_changed(-1))) write_tune(); M.need_reinit = 1; return MA_RESTART;
            case 5: return MA_TO_GARAGE; case 6: return MA_QUIT; }
        else if (IN.back) return MA_RESUME;
        break;
    }
    return MA_NONE;
}

/* ================================================================== API */
void menu_init(menu_host* H) {
    memset(&M, 0, sizeof M); M.H = H; M.map_pick = -1; M.scr = SC_TITLE;
    fonts_find(); glass_init();
    const char* sp = tune_spec_find(); char err[200] = "";
    static char spa[400]; if (!sp) { snprintf(spa, sizeof spa, "%sui/tune_spec.json", GetApplicationDirectory()); if (FileExists(spa)) sp = spa; }
    M.spec_ok = sp && tune_spec_load(&M.S, sp, err, sizeof err);
    if (!M.spec_ok) { snprintf(M.err, sizeof M.err, "tune spec: %s", sp ? err : "demo/ui/tune_spec.json not found"); fprintf(stderr, "[menu] %s\n", M.err); }
    for (int p = 0; p < M.S.npage; p++) { if (!strcmp(M.S.page[p].group, "settings")) { if (M.nsp < 20) M.sp[M.nsp++] = p; } else if (M.ntp < 20) M.tp[M.ntp++] = p; }
    tune_reload(); cars_scan();
    if (H->view) { M.last_su = H->view->menu_su; M.last_sd = H->view->menu_sd; }
}
void menu_open(int in_game) {
    M.open = 1; M.in_game = in_game == 1; M.drag_item = 0; memset(RH, 0, sizeof RH);
    if (!M.t_ok) tune_reload();
    go(in_game == 1 ? SC_PAUSE : in_game == 2 ? SC_HUB : SC_TITLE);
    if (M.H->view) { M.last_su = M.H->view->menu_su; M.last_sd = M.H->view->menu_sd; }
    M.thr_arm = M.brk_arm = 0;
}
void menu_close(void) { if (M.scr == SC_SETTINGS) go(home()); M.open = 0; }
int menu_is_open(void) { return M.open; }
int menu_in_game(void) { return M.in_game; }
int menu_garage_cam(void) { return M.open && !M.in_game; }
int menu_need_reinit(void) { return M.need_reinit; }
int menu_map_choice(void) { return M.map_pick; }
void menu_after_init(void) {
    M.need_reinit = 0; M.car_changed = 0; M.map_pick = -1; tune_reload(); cars_scan();
    if (M.H->view) { M.last_su = M.H->view->menu_su; M.last_sd = M.H->view->menu_sd; }
}
const char* menu_hint(void) { return M.err[0] ? M.err : ""; }
menu_action menu_update(float dt) {
    M.dt = dt > 0.1f ? 0.1f : dt < 0 ? 0 : dt; M.t += M.dt;
    if (!M.open) return MA_NONE;
    input_gather(); LIVE_HIT = 0;
    menu_action a = upd_screen();
    if (a == MA_DRIVE || a == MA_RESUME || a == MA_APPLY || a == MA_RESTART) { if (M.scr == SC_SETTINGS) go(home()); M.open = 0; }
    if (a == MA_NONE && LIVE_HIT) a = MA_LIVE;
    return a;
}
void menu_draw(RenderTexture2D* scene, int sw, int sh) {
    SW = sw; SH = sh; SC = fminf(sw / 1920.f, sh / 1080.f); OX = (sw - 1920 * SC) / 2; OY = (sh - 1080 * SC) / 2;
    NHIT[HB] = 0; GA = 1;
    if (scene) { if (GLASS_OK) blur_scene(scene, sw, sh);
        DrawTexturePro(scene->texture, (Rectangle){0, 0, (float)scene->texture.width, (float)-scene->texture.height}, (Rectangle){0, 0, (float)sw, (float)sh}, (Vector2){0, 0}, 0, WHITE); }
    rect(0, 0, 1920, 1080, RGBA(5, 7, 10, M.scr == SC_TITLE ? 40 : 80));
    if (M.scr != SC_PAUSE) { gradh(0, 0, 760, 1080, RGBA(5, 7, 10, 150), RGBA(5, 7, 10, 0)); gradh(1300, 0, 620, 1080, RGBA(5, 7, 10, 0), RGBA(5, 7, 10, 120)); }
    gradv(0, 860, 1920, 220, RGBA(5, 7, 10, 0), RGBA(5, 7, 10, 190)); gradv(0, 0, 1920, 140, RGBA(5, 7, 10, 150), RGBA(5, 7, 10, 0));
    if (M.scr != SC_TITLE) {
        char crumb[96]; int sp = M.scr == SC_SETTINGS;
        if ((M.scr == SC_TUNE || sp) && (sp ? M.nsp : M.ntp)) snprintf(crumb, sizeof crumb, "%s / %s / %s", M.in_game ? "PAUSED" : "GARAGE", SCN[M.scr], PG(sp ? M.sp[M.spage] : M.tp[M.tpage])->title);
        else snprintf(crumb, sizeof crumb, "%s%s", M.in_game && M.scr != SC_PAUSE ? "PAUSED / " : "", SCN[M.scr]);
        topbar(crumb);
    }
    float e = ease_out((M.t - M.scr_t) / 0.32f), oy0 = OY; OY += (1 - e) * 24 * SC; GA = e;
    switch (M.scr) {
    case SC_TITLE: title_screen(); break;
    case SC_HUB: hub_screen(); break;
    case SC_CAR: car_screen(); break;
    case SC_MAP: map_screen(); break;
    case SC_SETUPS: setups_screen(); break;
    case SC_TUNE: tune_screen(0); break;
    case SC_SETTINGS: tune_screen(1); break;
    case SC_PAUSE: pause_screen(); break;
    }
    OY = oy0; GA = 1;
    if (M.err[0]) tx(F_MONO, 12, 960, 1020, M.err, RD0, 0, 1);
    footer(); status_draw();
    HB ^= 1;
}
int menu_shot(const char* spec) {
    M.shot = 1; M.open = 1; char s[160]; snprintf(s, sizeof s, "%s", spec);
    char* val = strchr(s, '='); if (val) *val++ = 0;
    char* row = strchr(s, '/'); if (row) *row++ = 0;
    if (!strcmp(s, "title")) { M.in_game = 0; go(SC_TITLE); return 1; }
    if (!strcmp(s, "hub")) { M.in_game = 0; go(SC_HUB); return 1; }
    if (!strcmp(s, "car")) { M.in_game = 0; go(SC_CAR); return 1; }
    if (!strcmp(s, "map")) { M.in_game = 0; go(SC_MAP); return 1; }
    if (!strcmp(s, "setups")) { M.in_game = 0; go(SC_SETUPS); return 1; }
    if (!strcmp(s, "pause")) { M.in_game = 1; go(SC_PAUSE); return 1; }
    int settings = !strncmp(s, "settings.", 9); if (!settings && strncmp(s, "tune.", 5)) return 0;
    const char* id = s + (settings ? 9 : 5); int* pl = settings ? M.sp : M.tp; int np = settings ? M.nsp : M.ntp;
    M.in_game = 0; go(settings ? SC_SETTINGS : SC_TUNE);
    for (int i = 0; i < np; i++) if (!strcmp(PG(pl[i])->id, id)) { if (settings) M.spage = i; else M.tpage = i;
        int p = pl[i]; if (row) { int ri = atoi(row), k = strchr(row, 'r') ? 1 : 0; if (ri < PG(p)->nrow) { M.col = 1; M.item = line_of(p, ri, k);
            if (val) { if (!M.t_ok) return 0; row_set_abs(p, ri, k, atof(val), 0); } } }
        return 1; }
    return 0;
}
