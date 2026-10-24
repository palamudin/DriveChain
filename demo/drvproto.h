/* drvproto.h - JSON-lines driver protocol: lets a program (Python client, the agent, a CI job, Unity) drive the
   sim through the plug-in driver (dc_driver) the way a human would: finite-strength hands on the rim plus
   pedals. One JSON object per line in, one JSON object per line out. See docs/DRIVER_INTERFACE.md. */
#ifndef DEMO_DRVPROTO_H
#define DEMO_DRVPROTO_H
#include <stdio.h>
#include "sim.h"
#include "net.h"
#define PROTO_MAXPATH 8192
typedef struct {
    long long n; double t0;
    double sum_lat2, max_lat, max_col, max_hand, max_beta, max_r, max_swr, max_ay, max_head, max_sw, dist;
    long long sat, offtrack, takeover_ticks;
} proto_metrics;
typedef struct proto_s {
    sim_t* S;
    int lockstep;                    /* 1: the client advances time with "step" (deterministic); 0: free-running (demo) */
    net_srv net; int use_net;
    FILE* log; int log_every; long long tick;
    double stream_dt, stream_next;   /* free-running: push telemetry every stream_dt s (0 = off) */
    proto_metrics M;
    double path_xy[2 * PROTO_MAXPATH]; int npath, path_closed;
    dc_driver_params defaults;      /* params at attach time (demo: driver.* config); restored by reset */
    int quit;
} proto_t;
void proto_init(proto_t* P, sim_t* S, int lockstep);
int  proto_listen(proto_t* P, const char* bind_addr, int port);   /* demo: attach a TCP control port */
/* handle one command line; writes exactly one JSON line (no trailing newline) to out. Returns 1 on "quit". */
int  proto_handle(proto_t* P, const char* line, char* out, int outmax);
void proto_telemetry(proto_t* P, char* out, int outmax);
void proto_hook(sim_t* S, int phase);   /* install as S->ext_hook with S->ext_ctx = proto_t* */
void proto_close(proto_t* P);
#endif
