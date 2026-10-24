#include "cfg.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
static char* trim(char* s) {
    while (*s && isspace((unsigned char)*s)) s++;
    char* e = s + strlen(s);
    while (e > s && isspace((unsigned char)e[-1])) *--e = 0;
    return s;
}
void cfg_set(cfg_t* c, const char* key, const char* val) {
    for (int i = 0; i < c->n; i++)
        if (!strcmp(c->key[i], key)) { snprintf(c->val[i], sizeof c->val[i], "%s", val); return; }
    if (c->n >= 256) return;
    snprintf(c->key[c->n], sizeof c->key[0], "%s", key);
    snprintf(c->val[c->n], sizeof c->val[0], "%s", val);
    c->n++;
}
int cfg_load(cfg_t* c, const char* path) {
    FILE* f = fopen(path, "r");
    if (!f) return 0;
    char line[512], sec[64] = "";
    while (fgets(line, sizeof line, f)) {
        char* h = strpbrk(line, ";#"); if (h) *h = 0;
        char* s = trim(line);
        if (!*s) continue;
        if (*s == '[') { char* e = strchr(s, ']'); if (e) *e = 0; snprintf(sec, sizeof sec, "%s", trim(s + 1)); continue; }
        char* eq = strchr(s, '='); if (!eq) continue;
        *eq = 0;
        char key[160];
        if (*sec) snprintf(key, sizeof key, "%s.%s", sec, trim(s)); else snprintf(key, sizeof key, "%s", trim(s));
        cfg_set(c, key, trim(eq + 1));
    }
    fclose(f);
    return 1;
}
const char* cfg_str(const cfg_t* c, const char* key, const char* def) {
    for (int i = 0; i < c->n; i++) if (!strcmp(c->key[i], key)) return c->val[i];
    return def;
}
double cfg_num(const cfg_t* c, const char* key, double def) {
    const char* v = cfg_str(c, key, NULL); return v && *v ? atof(v) : def;
}
int cfg_int(const cfg_t* c, const char* key, int def) {
    const char* v = cfg_str(c, key, NULL); return v && *v ? atoi(v) : def;
}
