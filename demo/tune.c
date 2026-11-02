/* tune.c - see tune.h */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "tune.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <strings.h>
#include <math.h>
#include <sys/stat.h>
#ifdef _WIN32
#include <direct.h>
#define MKDIR(p) _mkdir(p)
#else
#define MKDIR(p) mkdir(p, 0755)
#endif
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* =============================================================== mini JSON (DOM, enough for the spec) */
typedef struct jv { int t; double n; char* s; struct jv* kid; char** key; int nk; } jv;   /* t: 0 null 1 num 2 str 3 arr 4 obj 5 bool */
typedef struct { const char* p; int err; } jp;
static void jws(jp* P) { while (*P->p && isspace((unsigned char)*P->p)) P->p++; }
static char* jstr(jp* P) {
    if (*P->p != '"') { P->err = 1; return NULL; }
    P->p++; size_t cap = 64, n = 0; char* o = malloc(cap);
    while (*P->p && *P->p != '"') {
        unsigned c = (unsigned char)*P->p++;
        if (c == '\\') {
            char e = *P->p++;
            if (e == 'n') c = '\n'; else if (e == 't') c = '\t'; else if (e == 'r') c = '\r'; else if (e == 'b') c = '\b'; else if (e == 'f') c = '\f';
            else if (e == 'u') {
                unsigned u = 0; for (int i = 0; i < 4 && *P->p; i++) { char h = *P->p++; u = u * 16 + (isdigit((unsigned char)h) ? h - '0' : (tolower(h) - 'a' + 10)); }
                if (u >= 0xD800 && u < 0xDC00 && P->p[0] == '\\' && P->p[1] == 'u') {   /* surrogate pair */
                    unsigned lo = 0; P->p += 2; for (int i = 0; i < 4 && *P->p; i++) { char h = *P->p++; lo = lo * 16 + (isdigit((unsigned char)h) ? h - '0' : (tolower(h) - 'a' + 10)); }
                    u = 0x10000 + ((u - 0xD800) << 10) + (lo - 0xDC00); }
                char b[4]; int k = 0;
                if (u < 0x80) b[k++] = (char)u; else if (u < 0x800) { b[k++] = (char)(0xC0 | (u >> 6)); b[k++] = (char)(0x80 | (u & 63)); }
                else if (u < 0x10000) { b[k++] = (char)(0xE0 | (u >> 12)); b[k++] = (char)(0x80 | ((u >> 6) & 63)); b[k++] = (char)(0x80 | (u & 63)); }
                else { b[k++] = (char)(0xF0 | (u >> 18)); b[k++] = (char)(0x80 | ((u >> 12) & 63)); b[k++] = (char)(0x80 | ((u >> 6) & 63)); b[k++] = (char)(0x80 | (u & 63)); }
                for (int i = 0; i < k; i++) { if (n + 2 > cap) o = realloc(o, cap *= 2); o[n++] = b[i]; }
                continue;
            } else c = (unsigned char)e;
        }
        if (n + 2 > cap) o = realloc(o, cap *= 2);
        o[n++] = (char)c;
    }
    if (*P->p == '"') P->p++; else P->err = 1;
    o[n] = 0; return o;
}
static void jval(jp* P, jv* v);
static void jpush(jv* v, jv* k, char* key) {
    v->kid = realloc(v->kid, sizeof(jv) * (v->nk + 1)); v->kid[v->nk] = *k;
    if (v->t == 4) { v->key = realloc(v->key, sizeof(char*) * (v->nk + 1)); v->key[v->nk] = key; }
    v->nk++;
}
static void jval(jp* P, jv* v) {
    memset(v, 0, sizeof *v); jws(P);
    char c = *P->p;
    if (c == '{' || c == '[') {
        v->t = c == '{' ? 4 : 3; P->p++; jws(P);
        if (*P->p == (c == '{' ? '}' : ']')) { P->p++; return; }
        for (;;) {
            char* key = NULL; jws(P);
            if (v->t == 4) { key = jstr(P); jws(P); if (*P->p != ':') { P->err = 1; free(key); return; } P->p++; }
            jv k; jval(P, &k); if (P->err) { free(key); return; }
            jpush(v, &k, key); jws(P);
            if (*P->p == ',') { P->p++; continue; }
            if (*P->p == (v->t == 4 ? '}' : ']')) { P->p++; return; }
            P->err = 1; return;
        }
    }
    if (c == '"') { v->t = 2; v->s = jstr(P); return; }
    if (!strncmp(P->p, "true", 4)) { v->t = 5; v->n = 1; P->p += 4; return; }
    if (!strncmp(P->p, "false", 5)) { v->t = 5; v->n = 0; P->p += 5; return; }
    if (!strncmp(P->p, "null", 4)) { v->t = 0; P->p += 4; return; }
    char* e; v->n = strtod(P->p, &e); if (e == P->p) { P->err = 1; return; } v->t = 1; P->p = e;
}
static void jfree(jv* v) {
    for (int i = 0; i < v->nk; i++) { jfree(&v->kid[i]); if (v->key) free(v->key[i]); }
    free(v->kid); free(v->key); free(v->s);
}
static const jv* jget(const jv* o, const char* k) {
    if (!o || o->t != 4) return NULL;
    for (int i = 0; i < o->nk; i++) if (!strcmp(o->key[i], k)) return &o->kid[i];
    return NULL;
}
static double jnum(const jv* o, const char* k, double def) { const jv* v = jget(o, k); return v && (v->t == 1 || v->t == 5) ? v->n : def; }
static void jcpy(const jv* o, const char* k, char* out, int n) { const jv* v = jget(o, k); snprintf(out, n, "%s", v && v->t == 2 ? v->s : ""); }

/* =============================================================== ini document */
static void ln_push(inidoc* d, const char* s) {
    if (d->n >= d->cap) { d->cap = d->cap ? d->cap * 2 : 256; d->line = realloc(d->line, sizeof(char*) * d->cap); }
    d->line[d->n++] = strdup(s);
}
static void ln_insert(inidoc* d, int at, const char* s) {
    ln_push(d, s); char* t = d->line[d->n - 1];
    memmove(d->line + at + 1, d->line + at, sizeof(char*) * (d->n - 1 - at)); d->line[at] = t;
}
int ini_load(inidoc* d, const char* path) {
    memset(d, 0, sizeof *d);
    FILE* f = fopen(path, "rb"); if (!f) return 0;
    char buf[1024];
    while (fgets(buf, sizeof buf, f)) { size_t L = strlen(buf); while (L && (buf[L - 1] == '\n' || buf[L - 1] == '\r')) buf[--L] = 0; ln_push(d, buf); }
    fclose(f); return 1;
}
void ini_free(inidoc* d) { for (int i = 0; i < d->n; i++) free(d->line[i]); free(d->line); memset(d, 0, sizeof *d); }
static void split_key(const char* key, char* name, int nn, int* item) {
    const char* h = strchr(key, '#'); *item = -1;
    if (h) { snprintf(name, nn, "%.*s", (int)(h - key), key); *item = atoi(h + 1); } else snprintf(name, nn, "%s", key);
}
static const char* skipws(const char* s) { while (*s == ' ' || *s == '\t') s++; return s; }
/* finds sec/key; value [vs, vs+vl) in line ln */
static int ini_find(const inidoc* d, const char* sec, const char* key, int* ln, int* vs, int* vl) {
    char cur[64] = "";
    for (int i = 0; i < d->n; i++) {
        const char* s = skipws(d->line[i]);
        if (!*s || *s == ';' || *s == '#') continue;
        if (*s == '[') { const char* e = strchr(s, ']'); snprintf(cur, sizeof cur, "%.*s", e ? (int)(e - s - 1) : 0, s + 1); continue; }
        const char* eq = strchr(s, '='); if (!eq) continue;
        if (strcasecmp(cur, sec)) continue;
        const char* ke = eq; while (ke > s && isspace((unsigned char)ke[-1])) ke--;
        if ((int)strlen(key) != (int)(ke - s) || strncasecmp(s, key, ke - s)) continue;
        const char* v = skipws(eq + 1); const char* e = v; while (*e && *e != ';' && *e != '#') e++;
        while (e > v && isspace((unsigned char)e[-1])) e--;
        *ln = i; *vs = (int)(v - d->line[i]); *vl = (int)(e - v); return 1;
    }
    return 0;
}
int ini_get(const inidoc* d, const char* sec, const char* key, char* out, int n) {
    char name[64]; int item, ln, vs, vl; split_key(key, name, sizeof name, &item);
    if (!ini_find(d, sec, name, &ln, &vs, &vl)) return 0;
    char v[512]; snprintf(v, sizeof v, "%.*s", vl, d->line[ln] + vs);
    if (item < 0) { snprintf(out, n, "%s", v); return 1; }
    char* p = v; for (int i = 0; i < item; i++) { p = strchr(p, ','); if (!p) return 0; p++; }
    char* e = strchr(p, ','); if (e) *e = 0;
    p = (char*)skipws(p); size_t L = strlen(p); while (L && isspace((unsigned char)p[L - 1])) p[--L] = 0;
    snprintf(out, n, "%s", p); return 1;
}
void ini_set(inidoc* d, const char* sec, const char* key, const char* val) {
    char name[64]; int item, ln, vs, vl; split_key(key, name, sizeof name, &item);
    if (ini_find(d, sec, name, &ln, &vs, &vl)) {
        char old[512], nv[512]; snprintf(old, sizeof old, "%.*s", vl, d->line[ln] + vs);
        if (item >= 0) {
            char parts[16][48]; int np = 0; char* p = old;
            while (np < 16) { char* e = strchr(p, ','); int L = e ? (int)(e - p) : (int)strlen(p); char t[64]; snprintf(t, sizeof t, "%.*s", L, p);
                char* a = (char*)skipws(t); size_t al = strlen(a); while (al && isspace((unsigned char)a[al - 1])) a[--al] = 0;
                snprintf(parts[np++], 48, "%s", a); if (!e) break; p = e + 1; }
            while (np <= item && np < 16) snprintf(parts[np++], 48, "0");
            if (item < 16) snprintf(parts[item], 48, "%s", val);
            nv[0] = 0; for (int i = 0; i < np; i++) { strcat(nv, parts[i]); if (i + 1 < np) strcat(nv, ", "); }
        } else snprintf(nv, sizeof nv, "%s", val);
        const char* rest = d->line[ln] + vs + vl; int pad = 0; while (rest[pad] == ' ' || rest[pad] == '\t') pad++;
        const char* tail = rest + pad; int np2 = pad - ((int)strlen(nv) - vl); if (np2 < 1) np2 = *tail ? 1 : 0;
        char out[1100]; snprintf(out, sizeof out, "%.*s%s%*s%s", vs, d->line[ln], nv, *tail ? np2 : 0, "", tail);
        free(d->line[ln]); d->line[ln] = strdup(out); return;
    }
    int at = -1; char cur[64] = "";
    for (int i = 0; i < d->n; i++) {
        const char* s = skipws(d->line[i]);
        if (*s == '[') { const char* e = strchr(s, ']'); snprintf(cur, sizeof cur, "%.*s", e ? (int)(e - s - 1) : 0, s + 1);
            if (!strcasecmp(cur, sec)) at = i; else if (at >= 0) break; }
        else if (at >= 0 && *s && *s != ';') at = i;
    }
    char out[600]; snprintf(out, sizeof out, "%s = %s", name, val);
    if (at < 0) { ln_push(d, ""); char h[80]; snprintf(h, sizeof h, "[%s]", sec); ln_push(d, h); ln_push(d, out); }
    else ln_insert(d, at + 1, out);
}
int ini_save(const inidoc* d, const char* path) {
    FILE* f = fopen(path, "wb"); if (!f) return 0;
    for (int i = 0; i < d->n; i++) fprintf(f, "%s\n", d->line[i]);
    fclose(f); return 1;
}
int ini_lut(const inidoc* d, const char* sec, double* x, double* y, int max) {
    char cur[64] = ""; int n = 0;
    for (int i = 0; i < d->n && n < max; i++) {
        const char* s = skipws(d->line[i]);
        if (*s == '[') { const char* e = strchr(s, ']'); snprintf(cur, sizeof cur, "%.*s", e ? (int)(e - s - 1) : 0, s + 1); continue; }
        if (strcasecmp(cur, sec) || !*s || *s == ';') continue;
        char* e; double a = strtod(s, &e); if (e == s) continue; e = (char*)skipws(e); if (*e != '=') continue;
        double b = strtod(e + 1, NULL); x[n] = a; y[n] = b; n++;
    }
    for (int i = 1; i < n; i++) for (int j = i; j > 0 && x[j] < x[j - 1]; j--) { double t = x[j]; x[j] = x[j - 1]; x[j - 1] = t; t = y[j]; y[j] = y[j - 1]; y[j - 1] = t; }
    return n;
}

/* =============================================================== spec */
static void row_load(tune_row* r, const jv* o) {
    memset(r, 0, sizeof *r);
    jcpy(o, "k", r->k, sizeof r->k); jcpy(o, "k2", r->k2, sizeof r->k2); jcpy(o, "l", r->l, sizeof r->l); jcpy(o, "u", r->u, sizeof r->u);
    jcpy(o, "id", r->id, sizeof r->id); jcpy(o, "i", r->info, sizeof r->info);
    r->min = jnum(o, "min", 0); r->max = jnum(o, "max", 1); r->st = jnum(o, "st", 0.01); r->fine = jnum(o, "fine", r->st / 10);
    r->s = jnum(o, "s", 1); r->d = (int)jnum(o, "d", 2); r->ro = (int)jnum(o, "ro", 0); r->rs = (int)jnum(o, "rs", 0);
    char t[16]; jcpy(o, "t", t, sizeof t); r->t = !strcmp(t, "demo") ? TR_DEMO : !strcmp(t, "bias") ? TR_BIAS : TR_CAR;
    const jv* lim = jget(o, "lim"); if (lim && lim->t == 3 && lim->nk == 2) { r->lim[0] = lim->kid[0].n; r->lim[1] = lim->kid[1].n; r->has_lim = 1; }
    const jv* op = jget(o, "opts"); const jv* va = jget(o, "vals");
    if (op && op->t == 3) for (int i = 0; i < op->nk && i < 8; i++) {
        snprintf(r->opt[i], sizeof r->opt[i], "%s", op->kid[i].s ? op->kid[i].s : "");
        snprintf(r->val[i], sizeof r->val[i], "%s", va && i < va->nk && va->kid[i].s ? va->kid[i].s : r->opt[i]); r->nopt = i + 1; }
}
int tune_spec_load(tune_spec* S, const char* path, char* err, int n) {
    memset(S, 0, sizeof *S);
    FILE* f = fopen(path, "rb"); if (!f) { snprintf(err, n, "no %s", path); return 0; }
    fseek(f, 0, SEEK_END); long L = ftell(f); fseek(f, 0, SEEK_SET);
    char* txt = malloc(L + 1); if (fread(txt, 1, L, f) != (size_t)L) L = 0; txt[L] = 0; fclose(f);
    jp P = {txt, 0}; jv root; jval(&P, &root); free(txt);
    if (P.err || root.t != 4) { snprintf(err, n, "%s: JSON parse error", path); jfree(&root); return 0; }
    const jv* c = jget(&root, "consts");
    S->g = jnum(c, "g", 9.81); S->rho = jnum(c, "rho", 1.225); S->crr = jnum(c, "crr", 0.012); S->eta = jnum(c, "eta", 0.9);
    const jv* vs = jget(&root, "vars");
    for (int i = 0; vs && i < vs->nk && S->nvar < 128; i++) { jcpy(&vs->kid[i], "n", S->var[S->nvar].n, 16); jcpy(&vs->kid[i], "e", S->var[S->nvar].e, 200); S->nvar++; }
    const jv* ov = jget(&root, "overlay_default");
    const jv* w = jget(ov, "when"); const jv* ks = jget(ov, "keys");
    for (int i = 0; w && i < w->nk && i < 4; i++) { snprintf(S->ov_when_k[i], 48, "%s", w->key[i]); S->ov_when[i][0] = w->kid[i].kid[0].n; S->ov_when[i][1] = w->kid[i].kid[1].n; S->nov_when++; }
    for (int i = 0; ks && i < ks->nk && i < 8; i++) { snprintf(S->ov_k[i], 32, "%s", ks->key[i]); snprintf(S->ov_v[i], 24, "%s", ks->kid[i].s ? ks->kid[i].s : ""); S->nov++; }
    const jv* pg = jget(&root, "pages");
    for (int i = 0; pg && i < pg->nk && S->npage < 20; i++) {
        const jv* o = &pg->kid[i]; tune_page* p = &S->page[S->npage++]; memset(p, 0, sizeof *p);
        jcpy(o, "id", p->id, sizeof p->id); jcpy(o, "title", p->title, sizeof p->title); jcpy(o, "unity", p->unity, sizeof p->unity);
        jcpy(o, "panel", p->panel, sizeof p->panel); jcpy(o, "note", p->note, sizeof p->note); jcpy(o, "group", p->group, sizeof p->group);
        if (!p->group[0]) snprintf(p->group, sizeof p->group, "tune");
        const jv* rs = jget(o, "rows"); for (int k = 0; rs && k < rs->nk && p->nrow < 14; k++) row_load(&p->row[p->nrow++], &rs->kid[k]);
        const jv* os = jget(o, "out");
        for (int k = 0; os && k < os->nk && p->nout < 16; k++) {
            const jv* q = &os->kid[k]; tune_out* t = &p->out[p->nout++]; memset(t, 0, sizeof *t);
            jcpy(q, "l", t->l, sizeof t->l); jcpy(q, "e", t->e, sizeof t->e); jcpy(q, "u", t->u, sizeof t->u);
            t->d = (int)jnum(q, "d", 1); t->est = (int)jnum(q, "est", 0);
            const jv* g = jget(q, "good"); if (g && g->t == 3 && g->nk == 2) { t->good[0] = g->kid[0].n; t->good[1] = g->kid[1].n; t->has_good = 1; }
            char tone[16]; jcpy(q, "tone", tone, sizeof tone); t->axle = !strcmp(tone, "axle");
        }
    }
    jfree(&root);
    if (!S->npage) { snprintf(err, n, "%s: no pages", path); return 0; }
    return 1;
}
const char* tune_spec_find(void) {
    static const char* P[] = {"ui/tune_spec.json", "demo/ui/tune_spec.json", "../demo/ui/tune_spec.json", NULL};
    for (int i = 0; P[i]; i++) { FILE* f = fopen(P[i], "rb"); if (f) { fclose(f); return P[i]; } }
    return P[0];
}

/* =============================================================== values */
static int split_sk(const char* k, char* sec, int ns, char* key, int nk) {
    const char* dot = strchr(k, '.'); if (!dot) return 0;
    snprintf(sec, ns, "%.*s", (int)(dot - k), k); snprintf(key, nk, "%s", dot + 1); return 1;
}
static double doc_num(const inidoc* d, const char* sec, const char* key) {
    char v[128]; if (!ini_get(d, sec, key, v, sizeof v)) return NAN;
    char* e; double x = strtod(v, &e); return e == v ? NAN : x;
}
static int overlay_key(const tune_state* T, const char* sec, const char* key, double* v) {
    if (!T->overlay_on || (strcmp(sec, "front") && strcmp(sec, "rear"))) return 0;
    for (int i = 0; i < T->S->nov; i++) if (!strcmp(T->S->ov_k[i], key)) { *v = atof(T->S->ov_v[i]); return 1; }
    return 0;
}
double tune_key(const tune_state* T, const char* k, int which) {
    char sec[16], key[48]; if (!split_sk(k, sec, sizeof sec, key, sizeof key)) return NAN;
    if (which == 0) for (int i = 0; i < T->ned; i++) if (!strcmp(T->ed[i].sec, sec) && !strcmp(T->ed[i].key, key)) return T->ed[i].v;
    double ov; if (overlay_key(T, sec, key, &ov)) return ov;
    return doc_num(which == 2 ? &T->base : &T->saved, sec, key);
}
void tune_key_set(tune_state* T, const char* k, double v) {
    char sec[16], key[48]; if (!split_sk(k, sec, sizeof sec, key, sizeof key)) return;
    for (int i = 0; i < T->ned; i++) if (!strcmp(T->ed[i].sec, sec) && !strcmp(T->ed[i].key, key)) { T->ed[i].v = v; T->dirty_car = 1; return; }
    if (T->ned >= 96) return;
    snprintf(T->ed[T->ned].sec, 16, "%s", sec); snprintf(T->ed[T->ned].key, 48, "%s", key); T->ed[T->ned].v = v; T->ned++; T->dirty_car = 1;
}
static double demo_get(const tune_state* T, const tune_row* r, int which) {
    const cfg_t* c = which == 0 ? T->cfg : &T->demo_saved;
    const char* v = cfg_str(c, r->k, NULL);
    if (r->nopt) { for (int i = 0; i < r->nopt; i++) if (v && !strcmp(v, r->val[i])) return i; return v && *v ? atof(v) : 0; }
    return v && *v ? atof(v) * r->s : r->min;
}
double tune_row_get(const tune_state* T, const tune_row* r, int k2, int which) {
    if (r->t == TR_DEMO) return demo_get(T, r, which ? 1 : 0);
    if (r->t == TR_BIAS) { double f = tune_key(T, "front.brake_torque", which), b = tune_key(T, "rear.brake_torque", which); return 100 * f / (f + b); }
    double v = tune_key(T, k2 && r->k2[0] ? r->k2 : r->k, which);
    if (r->nopt) { for (int i = 0; i < r->nopt; i++) if (fabs(v - atof(r->val[i])) < 1e-9) return i; return 0; }
    return v * r->s;
}
double tune_row_quant(const tune_row* r, double x) {
    if (r->nopt) { int i = (int)lround(x); return i < 0 ? 0 : i >= r->nopt ? r->nopt - 1 : i; }
    if (x < r->min) x = r->min;
    if (x > r->max) x = r->max;
    double q = r->fine > 0 ? r->fine : r->st;
    if (q > 0) x = r->min + q * floor((x - r->min) / q + 0.5);
    double p = pow(10, r->d); x = floor(x * p + 0.5) / p;
    return x < r->min ? r->min : x > r->max ? r->max : x;
}
void tune_row_set(tune_state* T, const tune_row* r, int k2, double x) {
    if (r->ro) return;
    x = tune_row_quant(r, x);
    if (r->t == TR_DEMO) {
        char b[64]; if (r->nopt) snprintf(b, sizeof b, "%s", r->val[(int)x]); else snprintf(b, sizeof b, "%.*f", r->d + (r->s != 1 ? 3 : 0), x / r->s);
        cfg_set(T->cfg, r->k, b); T->dirty_demo = 1; return;
    }
    if (r->t == TR_BIAS) {
        double f = tune_key(T, "front.brake_torque", 0), b = tune_key(T, "rear.brake_torque", 0), tot = f + b;
        tune_key_set(T, "front.brake_torque", floor(tot * x / 100 + 0.5)); tune_key_set(T, "rear.brake_torque", tot - floor(tot * x / 100 + 0.5)); return;
    }
    double v = r->nopt ? atof(r->val[(int)x]) : x / r->s;
    tune_key_set(T, k2 && r->k2[0] ? r->k2 : r->k, v);
}
int tune_open(tune_state* T, const tune_spec* S, const char* base_car, const char* saved_car, cfg_t* cfg, char* err, int n) {
    memset(T, 0, sizeof *T); T->S = S; T->cfg = cfg;
    snprintf(T->base_path, sizeof T->base_path, "%s", base_car);
    if (!ini_load(&T->base, base_car)) { snprintf(err, n, "cannot read %s", base_car); return 0; }
    if (!saved_car || !ini_load(&T->saved, saved_car)) ini_load(&T->saved, base_car);
    memcpy(&T->demo_saved, cfg, sizeof *cfg);
    /* car.setup = default overlays the tyre set on matching cars (sim.c SETUP_D) */
    if (!strcmp(cfg_str(cfg, "car.setup", "legacy"), "default") && S->nov_when) {
        int ok = 1;
        for (int i = 0; i < S->nov_when; i++) { char sec[16], key[48]; split_sk(S->ov_when_k[i], sec, 16, key, 48); double v = doc_num(&T->saved, sec, key);
            if (!(v >= S->ov_when[i][0] && v <= S->ov_when[i][1])) ok = 0; }
        T->overlay_on = ok;
    }
    tune_eval(T); return 1;
}
void tune_close(tune_state* T) { ini_free(&T->base); ini_free(&T->saved); }
int tune_changed(const tune_state* T) {
    for (int i = 0; i < T->ned; i++) { char k[80]; snprintf(k, sizeof k, "%s.%s", T->ed[i].sec, T->ed[i].key);
        double s = tune_key(T, k, 1); if (!(fabs(s - T->ed[i].v) <= 1e-9 * fmax(1, fabs(s)))) return 1; }
    for (int p = 0; p < T->S->npage; p++) for (int r = 0; r < T->S->page[p].nrow; r++) { const tune_row* R = &T->S->page[p].row[r];
        if (R->t == TR_DEMO && fabs(demo_get(T, R, 0) - demo_get(T, R, 1)) > 1e-9) return 1; }
    return 0;
}
void tune_revert(tune_state* T, int to_stock) {
    T->ned = 0; T->dirty_car = 1;
    if (to_stock) {   /* edit every spec key back to the base file */
        for (int p = 0; p < T->S->npage; p++) for (int r = 0; r < T->S->page[p].nrow; r++) { const tune_row* R = &T->S->page[p].row[r];
            if (R->t != TR_CAR || R->ro) continue;
            double v = tune_key(T, R->k, 2); if (v == v) tune_key_set(T, R->k, v);
            if (R->k2[0]) { v = tune_key(T, R->k2, 2); if (v == v) tune_key_set(T, R->k2, v); } }
        double f = tune_key(T, "front.brake_torque", 2), b = tune_key(T, "rear.brake_torque", 2);
        if (f == f) { tune_key_set(T, "front.brake_torque", f); tune_key_set(T, "rear.brake_torque", b); }
    } else {
        for (int p = 0; p < T->S->npage; p++) for (int r = 0; r < T->S->page[p].nrow; r++) { const tune_row* R = &T->S->page[p].row[r];
            if (R->t == TR_DEMO) { const char* v = cfg_str(&T->demo_saved, R->k, NULL); if (v) cfg_set(T->cfg, R->k, v); } }
    }
    tune_eval(T);
}

/* =============================================================== expressions */
typedef struct { const tune_state* T; const char* p; int err; } ex;
static double ex_cmp(ex* E);
static void ex_ws(ex* E) { while (*E->p == ' ') E->p++; }
static double lut_torque(const tune_state* T, double rpm) {
    double x[64], y[64]; int n = ini_lut(&T->saved, "engine_lut", x, y, 64);
    if (n < 1) return NAN;
    if (rpm <= x[0]) return y[0];
    for (int i = 1; i < n; i++) if (rpm <= x[i]) return y[i - 1] + (y[i] - y[i - 1]) * (rpm - x[i - 1]) / (x[i] - x[i - 1]);
    return y[n - 1];
}
static double k_(const tune_state* T, const char* k) { return tune_key(T, k, 0); }
static double gear_(const tune_state* T, int i) {
    double ng = k_(T, "car.ngears"); int n = ng == ng ? (int)ng : 5;
    if (i < 1 || i > n) return NAN;
    char b[32]; snprintf(b, sizeof b, "car.gears#%d", i - 1); return k_(T, b);
}
static double drv_r(const tune_state* T) { return k_(T, "car.drive") == 1 ? k_(T, "front.tyre_radius") : k_(T, "rear.tyre_radius"); }
static double vgear_(const tune_state* T, int i) { return k_(T, "car.limiter_rpm") * M_PI / 30 * drv_r(T) / (gear_(T, i) * k_(T, "car.final_drive")) * 3.6; }
static double tmax_(const tune_state* T) { double x[64], y[64]; int n = ini_lut(&T->saved, "engine_lut", x, y, 64); double m = NAN; for (int i = 0; i < n; i++) if (!(y[i] <= m)) m = y[i]; return m; }
static double pmax_(const tune_state* T) {
    double lim = k_(T, "car.limiter_rpm"), best = 0;
    for (double rpm = 500; rpm <= lim + 1e-9; rpm += 25) { double p = lut_torque(T, rpm) * rpm * M_PI / 30 / 1000; if (p > best) best = p; }
    return best;
}
static double vtop_(const tune_state* T) {
    const tune_spec* S = T->S;
    double m = k_(T, "car.mass"), cda = k_(T, "car.cda"), fd = k_(T, "car.final_drive"), lim = k_(T, "car.limiter_rpm"), r = drv_r(T);
    int n = (int)k_(T, "car.ngears"); double best = 0;
    for (int gi = 1; gi <= n; gi++) {
        double gr = gear_(T, gi);
        for (double v = 1.0; v < 120; v += 0.05) {   /* same accumulation as the reference evaluator */
            double rpm = v / r * gr * fd * 30 / M_PI; if (rpm > lim) break;
            double f = lut_torque(T, rpm) * gr * fd * S->eta / r - 0.5 * S->rho * cda * v * v - S->crr * m * S->g;
            if (f < 0) break;
            if (v > best) best = v;
        }
    }
    return best * 3.6;
}
static double axis_(const tune_state* T, int w) {
    int la = (int)k_(T, "front.lower_arm_a"), lb = (int)k_(T, "front.lower_arm_b"), rk = (int)k_(T, "front.rack_link");
    double lo[3], up[3]; char b[48];
    for (int j = 0; j < 3; j++) { snprintf(b, sizeof b, "front.link%d_u#%d", la, j); lo[j] = k_(T, b); }
    if (k_(T, "front.type") == 1) { for (int j = 0; j < 3; j++) { snprintf(b, sizeof b, "front.strut_top#%d", j); up[j] = k_(T, b); } }
    else {
        int n = (int)k_(T, "front.nlinks"), u = -1;
        for (int i = 0; i < n; i++) if (i != la && i != lb && i != rk) { u = i; break; }
        if (u < 0) return NAN;
        for (int j = 0; j < 3; j++) { snprintf(b, sizeof b, "front.link%d_u#%d", u, j); up[j] = k_(T, b); }
    }
    return atan2(w == 0 ? lo[0] - up[0] : lo[1] - up[1], up[2] - lo[2]);
}
static double ex_call(ex* E, const char* f, double* a, int n) {
    const tune_state* T = E->T;
    if (!strcmp(f, "sqrt")) return a[0] >= 0 ? sqrt(a[0]) : NAN;
    if (!strcmp(f, "abs")) return fabs(a[0]);
    if (!strcmp(f, "min")) { double m = a[0]; for (int i = 1; i < n; i++) if (a[i] < m) m = a[i]; return m; }
    if (!strcmp(f, "max")) { double m = a[0]; for (int i = 1; i < n; i++) if (a[i] > m) m = a[i]; return m; }
    if (!strcmp(f, "pow")) return pow(a[0], a[1]);
    if (!strcmp(f, "sin")) return sin(a[0]);
    if (!strcmp(f, "cos")) return cos(a[0]);
    if (!strcmp(f, "tan")) return tan(a[0]);
    if (!strcmp(f, "atan")) return atan(a[0]);
    if (!strcmp(f, "atan2")) return atan2(a[0], a[1]);
    if (!strcmp(f, "deg")) return a[0] * 180 / M_PI;
    if (!strcmp(f, "rad")) return a[0] * M_PI / 180;
    if (!strcmp(f, "clamp")) return fmin(fmax(a[0], a[1]), a[2]);
    if (!strcmp(f, "sel")) return a[0] > 0 ? a[1] : a[2];
    if (!strcmp(f, "torque")) return lut_torque(T, a[0]);
    if (!strcmp(f, "gear")) return gear_(T, (int)lround(a[0]));
    if (!strcmp(f, "vgear")) return vgear_(T, (int)lround(a[0]));
    if (!strcmp(f, "vtop")) return vtop_(T);
    if (!strcmp(f, "tmax")) return tmax_(T);
    if (!strcmp(f, "pmax")) return pmax_(T);
    if (!strcmp(f, "axis")) return axis_(T, (int)lround(a[0]));
    E->err = 1; return NAN;
}
static double ex_atom(ex* E) {
    ex_ws(E);
    if (*E->p == '(') { E->p++; double v = ex_cmp(E); ex_ws(E); if (*E->p == ')') E->p++; else E->err = 1; return v; }
    if (isdigit((unsigned char)*E->p) || *E->p == '.') { char* e; double v = strtod(E->p, &e); E->p = e; return v; }
    if (isalpha((unsigned char)*E->p) || *E->p == '_') {
        char id[80]; int n = 0;
        while ((isalnum((unsigned char)*E->p) || *E->p == '_' || *E->p == '.') && n < 70) id[n++] = *E->p++;
        if (*E->p == '#') { id[n++] = *E->p++; while (isdigit((unsigned char)*E->p) && n < 78) id[n++] = *E->p++; }
        id[n] = 0; ex_ws(E);
        if (*E->p == '(') {
            E->p++; double a[8]; int na = 0; ex_ws(E);
            if (*E->p != ')') for (;;) { double v = ex_cmp(E); if (na < 8) a[na++] = v; ex_ws(E); if (*E->p == ',') { E->p++; continue; } break; }
            if (*E->p == ')') E->p++; else E->err = 1;
            for (int i = na; i < 8; i++) a[i] = NAN;
            return ex_call(E, id, a, na);
        }
        if (!strcmp(id, "pi")) return M_PI;
        if (!strcmp(id, "g")) return E->T->S->g;
        if (!strcmp(id, "rho")) return E->T->S->rho;
        if (!strcmp(id, "crr")) return E->T->S->crr;
        if (!strcmp(id, "eta")) return E->T->S->eta;
        for (int i = 0; i < E->T->nvar_ok; i++) if (!strcmp(E->T->S->var[i].n, id)) return E->T->var[i];
        return tune_key(E->T, id, 0);
    }
    E->err = 1; return NAN;
}
static double ex_un(ex* E);
static double ex_pw(ex* E) {
    double a = ex_atom(E); ex_ws(E);
    if (*E->p == '^') { E->p++; double b = ex_un(E); if (a < 0 && b != floor(b)) return NAN; return pow(a, b); }
    return a;
}
static double ex_un(ex* E) { ex_ws(E); if (*E->p == '-') { E->p++; return -ex_un(E); } if (*E->p == '+') { E->p++; return ex_un(E); } return ex_pw(E); }
static double ex_mul(ex* E) {
    double a = ex_un(E);
    for (;;) { ex_ws(E); char o = *E->p; if (o != '*' && o != '/') return a; E->p++; double b = ex_un(E); a = o == '*' ? a * b : (b != 0 ? a / b : NAN); }
}
static double ex_add(ex* E) {
    double a = ex_mul(E);
    for (;;) { ex_ws(E); char o = *E->p; if (o != '+' && o != '-') return a; E->p++; double b = ex_mul(E); a = o == '+' ? a + b : a - b; }
}
static double ex_cmp(ex* E) {
    double a = ex_add(E);
    for (;;) {
        ex_ws(E); int le = 0, ge = 0, lt = 0, gt = 0;
        if (E->p[0] == '<' && E->p[1] == '=') { le = 1; E->p += 2; } else if (E->p[0] == '>' && E->p[1] == '=') { ge = 1; E->p += 2; }
        else if (E->p[0] == '<') { lt = 1; E->p++; } else if (E->p[0] == '>') { gt = 1; E->p++; } else return a;
        double b = ex_add(E); a = (le ? a <= b : ge ? a >= b : lt ? a < b : gt ? a > b : 0) ? 1 : 0;
    }
}
double tune_expr(const tune_state* T, const char* e) {
    ex E = {T, e, 0}; double v = ex_cmp(&E); ex_ws(&E);
    if (E.err || *E.p) return NAN;
    return v;
}
void tune_eval(tune_state* T) {
    T->nvar_ok = 0;
    for (int i = 0; i < T->S->nvar; i++) { T->var[i] = tune_expr(T, T->S->var[i].e); T->nvar_ok = i + 1; }
}
const char* tune_fmt(double v, int d, char* b, int n) {
    if (!(v == v) || isinf(v)) { snprintf(b, n, "—"); return b; }
    if (fabs(v) < 0.5 * pow(10, -d)) v = 0;
    snprintf(b, n, "%.*f", d, v); return b;
}

/* =============================================================== write-through */
static void fmt_ini(double v, char* b, int n) {
    snprintf(b, n, "%.6g", v);
    if (strchr(b, 'e')) snprintf(b, n, "%.9g", v);
}
static void mkdirs(const char* dir) {
    char b[300]; snprintf(b, sizeof b, "%s", dir);
    for (char* p = b + 1; *p; p++) if (*p == '/' || *p == '\\') { char c = *p; *p = 0; MKDIR(b); *p = c; }
    MKDIR(b);
}
static int write_car(tune_state* T, const char* path, const char* hdr) {
    inidoc d; if (!ini_load(&d, T->base_path)) return 0;
    for (int i = 0; i < T->ned; i++) {
        char k[80]; snprintf(k, sizeof k, "%s.%s", T->ed[i].sec, T->ed[i].key);
        double st = tune_key(T, k, 2); if (fabs(st - T->ed[i].v) <= 1e-12 * fmax(1, fabs(st))) continue;   /* = stock: keep the line */
        char b[64]; fmt_ini(T->ed[i].v, b, sizeof b); ini_set(&d, T->ed[i].sec, T->ed[i].key, b);
    }
    /* keys edited earlier (in saved) and not touched now must survive */
    for (int p = 0; p < T->S->npage; p++) for (int r = 0; r < T->S->page[p].nrow; r++) {
        const tune_row* R = &T->S->page[p].row[r]; if (R->t == TR_DEMO || R->ro) continue;
        const char* ks[4] = {R->k, R->k2, "front.brake_torque", "rear.brake_torque"};
        for (int j = 0; j < (R->t == TR_BIAS ? 4 : 2); j++) { if (!ks[j][0] || (R->t == TR_BIAS && j < 2)) continue;
            double cur = tune_key(T, ks[j], 0), st = tune_key(T, ks[j], 2);
            if (cur == cur && fabs(cur - st) > 1e-12 * fmax(1, fabs(st))) { char sec[16], key[48]; split_sk(ks[j], sec, 16, key, 48); char b[64]; fmt_ini(cur, b, sizeof b); ini_set(&d, sec, key, b); } }
    }
    ln_insert(&d, 0, hdr);
    int ok = ini_save(&d, path); ini_free(&d); return ok;
}
int tune_write(tune_state* T, const char* dir, char* car_out, int n, char* err, int en) {
    mkdirs(dir);
    char car[300], ovp[300]; snprintf(car, sizeof car, "%s/car.ui.ini", dir); snprintf(ovp, sizeof ovp, "%s/drivechain_overrides.ini", dir);
    char hdr[400]; snprintf(hdr, sizeof hdr, "; DriveChain garage tune: %s + the edits made in the menu (written by the demo / the Unity GUI)", T->base_path);
    if (!write_car(T, car, hdr)) { snprintf(err, en, "cannot write %s", car); return 0; }
    FILE* f = fopen(ovp, "wb"); if (!f) { snprintf(err, en, "cannot write %s", ovp); return 0; }
    fprintf(f, "; DriveChain UI overrides (committed in the menu). Read after demo.ini, before --set; delete to go back to demo.ini\n");
    fprintf(f, "[ui]\nbase_car = %s\n\n[car]\nini = %s\n", T->base_path, car);
    /* demo keys that differ from demo.ini, grouped by section */
    const cfg_t* c = T->cfg; char done[64][64]; int nd = 0; char cursec[64] = "car";
    for (int p = 0; p < T->S->npage; p++) for (int r = 0; r < T->S->page[p].nrow; r++) {
        const tune_row* R = &T->S->page[p].row[r]; if (R->t != TR_DEMO) continue;
        int dup = 0; for (int i = 0; i < nd; i++) if (!strcmp(done[i], R->k)) dup = 1;
        if (dup || nd >= 64) continue;
        snprintf(done[nd++], 64, "%s", R->k);
        const char* v = cfg_str(c, R->k, NULL); if (!v) continue;
        char sec[32], key[48]; if (!split_sk(R->k, sec, 32, key, 48)) continue;
        if (!strcmp(sec, "car") && !strcmp(key, "ini")) continue;
        if (strcmp(sec, cursec)) { fprintf(f, "\n[%s]\n", sec); snprintf(cursec, sizeof cursec, "%s", sec); }
        fprintf(f, "%s = %s\n", key, v);
    }
    fclose(f);
    snprintf(car_out, n, "%s", car);
    T->dirty_car = T->dirty_demo = 0;
    return 1;
}
int tune_overrides_load(cfg_t* cfg, const char* path) {
    cfg_t o; memset(&o, 0, sizeof o);
    if (!cfg_load(&o, path)) return 0;
    for (int i = 0; i < o.n; i++) cfg_set(cfg, o.key[i], o.val[i]);
    return 1;
}
int tune_setup_save(tune_state* T, const char* dir, const char* name, char* err, int en) {
    mkdirs(dir);
    char p[300], hdr[400]; snprintf(p, sizeof p, "%s/%s.ini", dir, name);
    snprintf(hdr, sizeof hdr, "; DriveChain setup '%s' for %s (garage tune; load from the Setups page)", name, T->base_path);
    if (!write_car(T, p, hdr)) { snprintf(err, en, "cannot write %s", p); return 0; }
    return 1;
}
int tune_setup_load(tune_state* T, const char* path, char* err, int en) {
    inidoc d; if (!ini_load(&d, path)) { snprintf(err, en, "cannot read %s", path); return 0; }
    T->ned = 0;
    for (int p = 0; p < T->S->npage; p++) for (int r = 0; r < T->S->page[p].nrow; r++) {
        const tune_row* R = &T->S->page[p].row[r]; if (R->t == TR_DEMO || R->ro) continue;
        const char* ks[4] = {R->k, R->k2, "front.brake_torque", "rear.brake_torque"};
        for (int j = 0; j < 4; j++) { if (!ks[j][0] || (j >= 2 && R->t != TR_BIAS) || (R->t == TR_BIAS && j < 2)) continue;
            char sec[16], key[48]; split_sk(ks[j], sec, 16, key, 48); double v = doc_num(&d, sec, key); if (v == v) tune_key_set(T, ks[j], v); }
    }
    ini_free(&d); tune_eval(T); return 1;
}
