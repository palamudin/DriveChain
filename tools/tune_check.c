/* tune_check.c - prints the garage info-panel vars / outputs of a car file with the C evaluator (demo/tune.c), in the
   same format as `python3 tools/gen_tune_spec.py --check <car.ini>`, so the C, C# and Python evaluators can be diffed.
   Also self-tests the ini document (in-place set keeps comments, list items, appends). Build: make tune_check */
#include "../demo/tune.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
static int selftest(void) {
    const char* p = "/tmp/dc_tune_selftest.ini";
    FILE* f = fopen(p, "w"); fprintf(f, "; head\n[car]\nmass = 1000      ; kg\ngears = 3.1, 2.0, 1.4\n\n[rear]\nx = 1\n"); fclose(f);
    inidoc d; ini_load(&d, p); char v[64]; int bad = 0;
    ini_set(&d, "car", "mass", "1012.5"); ini_set(&d, "car", "gears#1", "2.15"); ini_set(&d, "car", "new_key", "7"); ini_set(&d, "front", "y", "2");
    if (!ini_get(&d, "car", "mass", v, 64) || strcmp(v, "1012.5")) bad++;
    if (!ini_get(&d, "car", "gears#1", v, 64) || strcmp(v, "2.15")) bad++;
    if (!ini_get(&d, "car", "gears#2", v, 64) || strcmp(v, "1.4")) bad++;
    if (!ini_get(&d, "car", "new_key", v, 64) || strcmp(v, "7")) bad++;
    if (!ini_get(&d, "front", "y", v, 64) || strcmp(v, "2")) bad++;
    if (!strstr(d.line[2], "; kg")) bad++;
    if (strncmp(d.line[2], "mass = 1012.5    ; kg", 21)) bad++;
    ini_free(&d);
    printf("selftest %s (%d)\n", bad ? "FAIL" : "ok", bad);
    return bad;
}
int main(int argc, char** argv) {
    if (argc < 2) { fprintf(stderr, "usage: tune_check <car.ini> [spec.json] [legacy]\n"); return 2; }
    if (!strcmp(argv[1], "--selftest")) return selftest();
    tune_spec S; char err[300];
    if (!tune_spec_load(&S, argc > 2 ? argv[2] : "demo/ui/tune_spec.json", err, sizeof err)) { fprintf(stderr, "%s\n", err); return 1; }
    cfg_t cfg; memset(&cfg, 0, sizeof cfg); cfg_set(&cfg, "car.setup", argc > 3 ? argv[3] : "default");
    tune_state T; if (!tune_open(&T, &S, argv[1], NULL, &cfg, err, sizeof err)) { fprintf(stderr, "%s\n", err); return 1; }
    for (int i = 0; i < S.nvar; i++) printf("var %-8s %.6g\n", S.var[i].n, T.var[i]);
    for (int p = 0; p < S.npage; p++) for (int o = 0; o < S.page[p].nout; o++)
        printf("out %s.%-24.24s %.6g\n", S.page[p].id, S.page[p].out[o].e, tune_expr(&T, S.page[p].out[o].e));
    tune_close(&T); return 0;
}
