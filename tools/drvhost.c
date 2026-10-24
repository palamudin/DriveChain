/* drvhost.c - headless, deterministic, lockstep host for the JSON-lines driver protocol (demo/drvproto.h).
   Runs the exact demo tick (demo/sim.c) on the proving ground with no window, no audio and no devices, so an
   agent, CI job or Python script can drive the car and measure it reproducibly. Time only advances on "step".

     ./build/drvhost [--car cars/mx5_na.ini] [--set key=value]... [--layout proving|circuit]
                     [--tcp PORT [--bind ADDR]] [--script file.jsonl]
   Default transport is stdin/stdout (one JSON object per line each way). --script runs a file of commands,
   then continues with stdin unless the script ends with {"cmd":"quit"}.                                      */
#include "../demo/drvproto.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static track_t TRK;
static char LINE[1 << 20], OUT[1 << 16];

static int run_stream(proto_t* P, FILE* in, FILE* out) {
    while (fgets(LINE, sizeof LINE, in)) {
        size_t n = strlen(LINE); while (n && (LINE[n - 1] == '\n' || LINE[n - 1] == '\r')) LINE[--n] = 0;
        int q = proto_handle(P, LINE, OUT, sizeof OUT);
        fputs(OUT, out); fputc('\n', out); fflush(out);
        if (q) return 1;
    }
    return 0;
}

int main(int argc, char** argv) {
    static cfg_t cfg; static sim_t S; static proto_t P;
    const char* layout = "proving"; const char* script = NULL; const char* bind = "127.0.0.1"; int port = 0;
    cfg_set(&cfg, "autopilot.enabled", "0");
    cfg_set(&cfg, "car.ini", "cars/mx5_na.ini");
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--car") && i + 1 < argc) cfg_set(&cfg, "car.ini", argv[++i]);
        else if (!strcmp(argv[i], "--layout") && i + 1 < argc) layout = argv[++i];
        else if (!strcmp(argv[i], "--script") && i + 1 < argc) script = argv[++i];
        else if (!strcmp(argv[i], "--tcp") && i + 1 < argc) port = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--bind") && i + 1 < argc) bind = argv[++i];
        else if (!strcmp(argv[i], "--set") && i + 1 < argc) {
            char kv[256]; snprintf(kv, sizeof kv, "%s", argv[++i]); char* eq = strchr(kv, '=');
            if (eq) { *eq = 0; cfg_set(&cfg, kv, eq + 1); }
        } else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            fprintf(stderr, "usage: drvhost [--car INI] [--set k=v]... [--layout proving|circuit] [--tcp PORT [--bind ADDR]] [--script FILE]\n");
            return 0;
        } else { fprintf(stderr, "drvhost: unknown argument %s\n", argv[i]); return 2; }
    }
    track_build(&TRK, cfg_num(&cfg, "track.rough_scale", 1.0), !strcmp(layout, "circuit") ? TRACK_CIRCUIT : TRACK_PROVING);
    char err[256];
    if (!sim_init(&S, &cfg, &TRK, NULL, err, sizeof err)) { fprintf(stderr, "drvhost: sim_init: %s\n", err); return 1; }
    S.drv_takeover_enabled = 0;     /* no human devices on a headless host */
    proto_init(&P, &S, 1);
    int quit = 0;
    if (script) {
        FILE* f = fopen(script, "r"); if (!f) { fprintf(stderr, "drvhost: cannot open %s\n", script); return 1; }
        quit = run_stream(&P, f, stdout); fclose(f);
    }
    if (!quit) {
        if (port) {
            if (!net_listen(&P.net, bind, port)) { fprintf(stderr, "drvhost: cannot listen on %s:%d\n", bind, port); return 1; }
            fprintf(stderr, "drvhost: listening on %s:%d (lockstep)\n", bind, port);
            while (!quit && net_read_line_blocking(&P.net, LINE, sizeof LINE)) {
                quit = proto_handle(&P, LINE, OUT, sizeof OUT - 2); strcat(OUT, "\n"); net_send(&P.net, OUT);
            }
            net_close(&P.net);
        } else run_stream(&P, stdin, stdout);
    }
    proto_close(&P);
    sim_shutdown(&S);
    return 0;
}
