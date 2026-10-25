# Performance: cost, mechanics rate, threads

All timings: wall time on one x86-64 server core (8-core Xeon VM), `-O2`, default car `cars/mx5_na.ini` (multibody
core) on the procedural proving ground, wide-open throttle from 100 km/h in 4th, 1 ms host tick. "us/ms" means
microseconds of wall time per millisecond of simulated time; real time is 1000 us/ms.

## Where the time went, and what changed

| stage (default 2 kHz mechanics) | us/ms |
|---|---|
| start of the pass | 858 |
| sparse Schur build of the box rows (elimination-tree forward solves instead of one full K^-1 solve per row) | 609-618 |
| opt-in persistent per-wheel road-node cache (`DC_ROAD_STATIC`, demo opts in) | 445-452 |

Both changes are bit-identical to the previous code path (`schur_dense = 1` keeps the reference Schur build;
`tests/road_cache_tests.c` checks the cache, including a host edit of the road followed by `dc_road_changed()`).

Profile at ~450 us/ms (`-pg`, `-fno-inline` build): tyre/road contact preparation (phase A: road lattice queries and
contact geometry, all four wheels) ~30-40 % of a substep, block LDL^T factor ~11 %, box-row Schur build (forward
solves + dots) ~10 %, back/forward solves ~5 %, chassis body-contact normals ~4 % (deliberately not parallelised yet).

## Mechanics rate under a 240 Hz outer tick (`make rateconv`, `tools/rate_conv.c`)

The host ticks at 240 Hz; the mechanics substep rate is `substep_hz` (240/480/960/1920 = 1/2/4/8 substeps per tick).
Stiff elements (tyre carcass, bushings, joint stops, friction) are implicit in the coupled solve, so lower rates stay
stable without softening anything. Each metric is compared with the 1920 Hz reference; tolerances are listed in the
table. Cost is one core, serial.

`cars/mx5_na.ini` (multibody core)

| metric | 240 Hz | 480 Hz | 960 Hz | 1920 Hz (ref) | tolerance |
|---|---|---|---|---|---|
| skidpad ay [m/s2] | 6.492 | 6.501 | 6.508 | 6.503 | 0.03 or 1 % |
| skidpad yaw rate [deg/s] | 22.323 | 22.354 | 22.379 | 22.361 | 0.2 or 1 % |
| step steer: yaw-rate peak [deg/s] | 19.592 | 19.662 | 19.698 | 19.696 | 0.3 or 2 % |
| step steer: t90 [ms] | 208.4 | 205.9 | 204.7 | 204.0 | 8 ms |
| step steer: yaw rate at 3 s [deg/s] | 15.162 | 15.173 | 15.187 | 15.183 | 0.2 or 1 % |
| lift-off: extra yaw in 1 s [deg] | 3.047 | 3.083 | 3.095 | 3.087 | 0.2 or 3 % |
| lift-off: yaw-rate rise [deg/s] | 4.121 | 4.167 | 4.161 | 4.125 | 0.3 or 5 % |
| 40 mm bump: peak Fz front [N] | 7137 | 6898 | 6933 | 6943 | 5 % |
| 40 mm bump: peak Fz rear [N] | 6957 | 6780 | 6810 | 6800 | 5 % |
| 40 mm bump: body z range [mm] | **8.22 (fail)** | 8.68 | 8.90 | 8.99 | 0.5 mm or 5 % |
| understeer gradient [deg/g] | 0.595 | 0.590 | 0.588 | 0.590 | 0.1 |
| WOT 0-8 s v_end [km/h] | 94.25 | 94.26 | 94.26 | 94.25 | 0.3 or 0.5 % |
| **cost [us/ms, 1 core]** | 53 | **105** | 204 | 394 | |

Minimum converging rate: **480 Hz** (2 substeps per 240 Hz tick), 105 us/ms on one core. 240 Hz misses only the body
heave range over a short (0.6 m) bump by 8.5 %: the 4 ms substep under-resolves the ~25 ms bump pass.

`cars/hatch_fwd.ini`: every metric converges already at **240 Hz** (26 us/ms); see `results/rate/`.

**Proposal (not applied):** low-end default = 240 Hz outer tick with 2 substeps (480 Hz) for the default car. The
shipped default stays 2 kHz until this is accepted.

## Worker threads (`dc_set_threads`, opt-in, default 1)

`dc_set_threads(car, n)` creates a persistent pool (n-1 helper threads + the calling physics thread, n <= 4) once,
outside the step. No heap allocation or thread creation in the step. Per substep:

* tyre/road contact preparation (phase A) runs one wheel per task with a fixed wheel -> thread partition
  (wheel k on thread k mod n);
* the box-row Schur columns (forward solves) and the friction-bound source rows are built in parallel per row.

Every task writes only its own data and there is no cross-thread reduction, so the trajectory is bit-identical for
any thread count (`make threadtest`: 1 vs 2/3/4 threads, both cores x both tyre models, with and without the road
cache, mid-run switching, plus a pool start-up stress test). Helpers spin for up to 0.5 ms after a job, then sleep on a
condition variable. The physics thread's SSE control word (FTZ/DAZ, rounding) is copied into the helpers for every job.
With n > 1 the road callback is called from helper threads and must be thread-safe (a pure function of x, y). The demo
tracks are; a Unity raycast callback is not, so Unity should stay at 1 thread unless it bakes a heightfield.

### Contact preparation, isolated (same form as the reference: serial vs 3 helpers + physics thread)

| | serial | 3 helpers + physics thread | speed-up | efficiency |
|---|---|---|---|---|
| contact preparation per 2 kHz substep (4 wheels) | 94.7 us | 28.1 us | **3.4x** | **84 %** |

### Whole step, us/ms (median of 3)

| mechanics rate | 1 thread | 2 | 3 | 4 |
|---|---|---|---|---|
| 240 Hz | 67 | 53 | 51 | 47 |
| 480 Hz | 121 | 99 | 97 | 92 |
| 960 Hz | 255 | 205 | 196 | 163 |
| 1920 Hz | 454 | 356 | 342 | 292 |
| 2000 Hz (current default) | 481 | 403 | 379 | 332 |

3 threads gain little over 2 because four wheels on three threads leave two wheels on the critical path; 8 requested
threads are clamped to 4 (321 us/ms at 2 kHz, the same as 4). The remaining serial part is the multibody factor/solve
and the PGS (~135 us per substep). Splitting the factorisation by corner subtree is the next step; it changes the
summation order into the chassis block, so it has to be the single code path for all thread counts to stay
bit-identical.
