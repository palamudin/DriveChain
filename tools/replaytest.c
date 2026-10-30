/* replaytest.c - input record / replay determinism check (demo/sim.c record + replay, the exact demo tick).
   Records RT_SECONDS (default 8) of an autopilot lap start at physics.tick_hz 960, fast model, then replays the
   recording (made on RT_REC_BACKEND, default cpu) on each backend in RT_BACKENDS (default "cpu,cpu_simd,gpu,auto") and on
   1 and 4 threads, then does a ref-recorded / ref-replayed pass (skip with RT_NO_REF=1), and checks the
   end-state hash against the recording. Also prints the lane counters (planar skips, gpu phase split, cross-check).
   Build: make build/replaytest   Run: ./build/replaytest [car.ini] [out.dcr]   Extra cfg: CFGSET="k=v;k=v"    */
#include "../demo/sim.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
static track_t TRK;
static double wall(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + 1e-9 * t.tv_nsec; }
static void base_cfg(cfg_t* c, const char* car) {
    memset(c, 0, sizeof *c);
    cfg_set(c, "car.setup", "default"); cfg_set(c, "physics.tick_hz", "960"); cfg_set(c, "physics.substeps", "4");
    cfg_set(c, "physics.model", "fast"); cfg_set(c, "physics.threads", "4"); cfg_set(c, "autopilot.enabled", "1");
    cfg_set(c, "start.spot", "start"); cfg_set(c, "input.device", "keyboard");
    if (car && *car) cfg_set(c, "car.ini", car);
    if (getenv("CFGSET")) { char buf[2048]; snprintf(buf, sizeof buf, "%s", getenv("CFGSET"));
        for (char* tok = strtok(buf, ";"); tok; tok = strtok(NULL, ";")) { char* eq = strchr(tok, '='); if (eq) { *eq = 0; cfg_set(c, tok, eq + 1); } } }
}
static void lane_line(sim_t* S, const char* tag, double us_tick) {
    dc_offload_stats o; dc_get_offload(S->car, DC_STAGE_PATCH, &o);
    printf("  %-22s tick %6.1f us | lane %s: %lld batches %.2f us/call, skips %lld, live us cpu %.2f simd %.2f gpu %.2f",
           tag, us_tick, dc_offload_name(o.backend), o.calls, o.calls ? 1e6 * o.seconds / o.calls : 0, o.skips, o.live_us[0], o.live_us[1], o.live_us[2]);
    if (o.gpu_calls || o.gpu_fail) printf(" | gpu %lld: up %.2f exec %.2f read %.2f us, failed %lld", o.gpu_calls, o.gpu_phase_us[0], o.gpu_phase_us[1], o.gpu_phase_us[2], o.gpu_fail);
    if (o.xcheck) printf(" | xcheck 1/%d: %lld checked, flips %lld, max dth %.3g dxy/grip %.3g", o.xcheck, o.xc_calls, o.xc_flips, o.xc_max_dth, o.xc_max_dother);
    printf("\n");
}
/* record on rb, replay on each backend in list (x 4 and 1 threads); returns 1 on any mismatch */
static int pass(const char* car, const char* out, double secs, const char* rb, const char* list) {
    static sim_t S; static cfg_t C; char err[300], bl[256]; snprintf(bl, sizeof bl, "%s", list);
    base_cfg(&C, car); cfg_set(&C, "replay.record", "1"); cfg_set(&C, "replay.record_file", out); cfg_set(&C, "offload.patch", rb);
    if (!sim_init(&S, &C, &TRK, NULL, err, sizeof err)) { fprintf(stderr, "sim_init: %s\n", err); return 1; }
    int n = (int)(secs / S.tick_dt + 0.5);
    double w0 = wall(); for (int k = 0; k < n; k++) sim_tick(&S, S.tick_dt); double w1 = wall();
    sim_record_stop(&S);
    unsigned long long h0 = S.rec_h.end_hash;
    printf("record: %d ticks (%.1f s, v end %.1f km/h) on %s -> %s, end hash %016llx\n", S.rec_n, S.rec_n * S.tick_dt, S.snap.t.speed * 3.6, dc_offload_name(S.offload_patch), out, h0);
    lane_line(&S, "record", 1e6 * (w1 - w0) / n);
    sim_shutdown(&S);
    int fail = 0;
    for (char* b = strtok(bl, ","); b; b = strtok(NULL, ",")) for (int th = 4; th >= 1; th -= 3) {
        char tb[8]; snprintf(tb, sizeof tb, "%d", th);
        base_cfg(&C, car); cfg_set(&C, "offload.patch", b); cfg_set(&C, "physics.threads", tb); cfg_set(&C, "replay.file", out);
        if (getenv("RT_XCHECK")) cfg_set(&C, "offload.xcheck", getenv("RT_XCHECK"));
        if (!sim_init(&S, &C, &TRK, NULL, err, sizeof err) || !S.rep_on) { fprintf(stderr, "replay init failed (%s): %s\n", b, S.rec_msg); return 1; }
        int k = 0; w0 = wall();
        while (!S.rep_done && k < n + 10) { sim_tick(&S, S.tick_dt); k++; }
        w1 = wall();
        char tag[64]; snprintf(tag, sizeof tag, "replay %s t%d", b, th);
        printf("%s: %s (hash %016llx)\n", tag, S.rep_done && S.rep_match ? "BIT-IDENTICAL" : "DIFFERS", S.rep_hash);
        lane_line(&S, tag, 1e6 * (w1 - w0) / (k ? k : 1));
        fail |= !(S.rep_done && S.rep_match);
        sim_shutdown(&S);
    }
    return fail;
}
int main(int argc, char** argv) {
    const char* car = argc > 1 ? argv[1] : "cars/mx5_na.ini";
    const char* out = argc > 2 ? argv[2] : "/tmp/replaytest.dcr";
    double secs = getenv("RT_SECONDS") ? atof(getenv("RT_SECONDS")) : 8;
    track_build(&TRK, 1.0, TRACK_PROVING);
    /* the lane backends (cpu / cpu_simd / gpu / auto) share bits with each other; ref rounds differently, so it gets its own pass */
    int fail = pass(car, out, secs, getenv("RT_REC_BACKEND") ? getenv("RT_REC_BACKEND") : "cpu", getenv("RT_BACKENDS") ? getenv("RT_BACKENDS") : "cpu,cpu_simd,gpu,auto");
    if (!getenv("RT_NO_REF")) fail |= pass(car, out, secs, "ref", "ref");
    printf(fail ? "replaytest: FAIL\n" : "replaytest: all replays bit-identical\n");
    return fail;
}
