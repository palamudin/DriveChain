/* Step-cost regression under drive torque (headless, through the demo's own sim_tick).
   usage: perf_tests <car.ini> <layout: proving|circuit|<track dir>> [spot] [budget_us]
   Per scenario (full-throttle launch, part throttle, rolling full throttle in 4th):
     - machine-independent: box-LCP solves that end at the sweep cap (must stay <= 0.1 % of solves), mean box-LCP
       sweeps per tick <= 10 (the unconverged friction solve that froze the stage averaged ~21), and no NaN;
     - wall clock: the worst 100 ms window of 1 ms demo ticks must cost less than budget_us per tick (default
       4000 us, env PERF_BUDGET_US), i.e. a stall can never hide behind a good average. */
#include "../demo/drvproto.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + 1e-9 * t.tv_nsec; }
static track_t TRK; static char OUT[1 << 16];
static cfg_t cfg; static sim_t S; static proto_t P;
static int run(const char* name, const char* spot, double v0, int gear, double thr, double secs, double budget) {
    char L[512];
    snprintf(L, sizeof L, "{\"cmd\":\"reset\",\"spot\":\"%s\",\"speed_kmh\":%g}", spot, v0); proto_handle(&P, L, OUT, sizeof OUT);
    if (gear > 0) { snprintf(L, sizeof L, "{\"cmd\":\"gear\",\"gear\":%d}", gear); proto_handle(&P, L, OUT, sizeof OUT); }
    snprintf(L, sizeof L, "{\"cmd\":\"drive\",\"steer\":\"angle\",\"target_deg\":0,\"pedals\":\"direct\",\"throttle\":%g,\"brake\":0}", thr); proto_handle(&P, L, OUT, sizeof OUT);
    dc_telemetry t; dc_get_telemetry(S.car, &t); int cap0 = t.pgs_capped;
    int n = (int)(secs * 1000 + .5), bad = 0, sweeps = 0; double tot = 0, win = 0, wworst = 0, tmax = 0;
    for (int k = 0; k < n; k++) {
        double a = now(); sim_tick(&S, 0.001); double w = now() - a;
        tot += w; win += w; if (w > tmax) tmax = w;
        if ((k + 1) % 100 == 0) { if (win > wworst) wworst = win; win = 0; }
        dc_get_telemetry(S.car, &t); sweeps += t.pgs_iters;
        if (!(t.speed == t.speed) || !(t.engine_rpm == t.engine_rpm)) bad++;
    }
    int capped = t.pgs_capped - cap0, solves = 2 * n;   /* >= 2 MB solves per 1 ms tick (2 kHz substeps) */
    double wus = wworst / 100 * 1e6;
    int ok = !bad && capped * 1000 <= solves && (double)sweeps / n <= 10 && wus < budget;
    printf("%-28s v_end %6.1f km/h  mean %5.0f us/tick  worst-100ms %5.0f us/tick  max tick %5.1f ms  sweeps/tick %5.1f  capped %d/%d  %s%s\n",
           name, t.speed * 3.6, tot / n * 1e6, wus, tmax * 1e3, (double)sweeps / n, capped, solves, bad ? "NAN " : "", ok ? "PASS" : "FAIL");
    return ok;
}
int main(int argc, char** argv) {
    if (argc < 3) { fprintf(stderr, "usage: %s car.ini layout [spot] [budget_us]\n", argv[0]); return 2; }
    const char* spot = argc > 3 ? argv[3] : (strcmp(argv[2], "proving") == 0 ? "hub" : "start");
    double budget = argc > 4 ? atof(argv[4]) : (getenv("PERF_BUDGET_US") ? atof(getenv("PERF_BUDGET_US")) : 4000);
    char e[256];
    cfg_set(&cfg, "autopilot.enabled", "0"); cfg_set(&cfg, "car.ini", argv[1]);
    if (!track_select(&TRK, argv[2], 1.0, e, sizeof e)) { fprintf(stderr, "%s\n", e); return 2; }
    if (!sim_init(&S, &cfg, &TRK, NULL, e, sizeof e)) { fprintf(stderr, "%s\n", e); return 2; }
    S.drv_takeover_enabled = 0; proto_init(&P, &S, 1);
    printf("perf_tests %s on %s (spot %s), budget %.0f us per 1 ms tick\n", argv[1], argv[2], spot, budget);
    int ok = 1;
    ok &= run("launch, full throttle", spot, 0, 0, 1.0, 12, budget);
    ok &= run("launch, 40% throttle", spot, 0, 0, 0.4, 8, budget);
    ok &= run("rolling 100 km/h, 4th, WOT", spot, 100, 4, 1.0, 6, budget);
    printf("PERF %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
