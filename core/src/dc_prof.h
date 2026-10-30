/* dc_prof.h - optional fine-grained stage timers and solver/pool counters for profiling builds (compile with
   -DDC_PROF). Without DC_PROF every macro expands to nothing, so normal builds are unchanged (no extra clock reads,
   no globals, same code). The record is process-wide (one car per process in the demo/perf tools); stage times are
   written by the stepping thread, the per-wheel phase-A slots by whichever pool thread runs that wheel (one writer
   per slot, cache-line padded). Read and reset with dc_prof_take(). */
#ifndef DC_PROF_H
#define DC_PROF_H
#include "drivechain.h"
#ifdef DC_PROF
double dc_wall(void);
extern dc_prof_rec dc_prof_g;
#define DCP_T(v) double v = dc_wall()
#define DCP_LAP(stage, t) do { double _dcp_n = dc_wall(); dc_prof_g.st[stage] += _dcp_n - (t); (t) = _dcp_n; } while (0)
#define DCP_ADD(stage, t) (dc_prof_g.st[stage] += dc_wall() - (t))
#define DCP_WLAP(i, slot, t) do { double _dcp_n = dc_wall(); dc_prof_g.wh[i][slot] += _dcp_n - (t); (t) = _dcp_n; } while (0)
#define DCP_CNT(f, n) (dc_prof_g.f += (n))
#define DCP_MAX(f, n) do { if ((n) > dc_prof_g.f) dc_prof_g.f = (n); } while (0)
#define DCP_RESET(t) ((t) = dc_wall())
#define DCP(x) x
#else
#define DCP_T(v)
#define DCP_LAP(stage, t) ((void)0)
#define DCP_ADD(stage, t) ((void)0)
#define DCP_WLAP(i, slot, t) ((void)0)
#define DCP_CNT(f, n) ((void)0)
#define DCP_MAX(f, n) ((void)0)
#define DCP_RESET(t) ((void)0)
#define DCP(x)
#endif
#endif
