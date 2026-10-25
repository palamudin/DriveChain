# Throttle freeze: root cause and fix (reference car mx5_na, public maps)

**Symptom.** On a mesh map or the proving ground, the demo ran smoothly until you opened the throttle. Then it seized up abruptly.

**Root cause.**
1. Under drive torque and wheelspin, the joint-friction box/cone LCP stopped converging. This LCP covers ball-joint friction cones, rack friction, and column and strut bearing friction.
   - Its sweep loop waited for a velocity residual that friction rows on the edge of slipping never reach.
   - Every solve therefore ran to the 400-sweep cap.
   - Each sweep also ran a cone Newton solve, and that solve ran to its own 60-iteration cap.
   - Under wheelspin each substep solves twice.
2. As a result, a 1 ms tick cost 5 to 42 ms.
3. The demo's physics thread then ran up to 50 of those ticks back to back, taking the mutex once per tick. On Windows the mutex is unfair, so the render thread starved and the picture froze.

**Fix.** Same physics, solved properly:
1. `cone_block` now solves the sliding case in the block's eigenbasis, using the secular equation with a safeguarded Newton step.
2. Hard constraint rows still need a velocity residual below 1e-8 m/s. Friction rows stop once their remaining work error is below 1e-8 J per solve.
3. Telemetry has a new `pgs_capped` counter.
4. The demo's catch-up loop is now bounded by wall time (8 ms per batch). The physics thread yields the mutex to the render thread, and a backlog over 50 ms is dropped. An over-budget step now shows up as slow motion, not a freeze.

**Regression test.** `make perftest` (tests/perf_tests.c) runs on proving and circuit with three manoeuvres: a WOT launch, a 40 % launch, and a rolling WOT run in 4th from 100 km/h. It checks four things:
- no NaN;
- sweep-capped solves are no more than 0.1 %;
- mean sweeps per tick are no more than 10 (the bug averaged about 21);
- the worst 100 ms window stays under 4000 µs per 1 ms tick (`PERF_BUDGET_US`).

Results are in `perf_summary.txt`.

**Before and after.** mx5_na, proving, hub, on an 8-core Xeon (Linux, -O2):

| scenario | before: worst tick | after: worst tick | before: worst 100 ms | after: worst 100 ms |
|---|---|---|---|---|
| WOT launch 12 s | 18.2 ms | 3.6 ms | 2156 µs/tick | 1244 µs/tick |
| 40 % launch 8 s | 9.1 ms | 5.2 ms | 3073 µs/tick | 1354 µs/tick |
| rolling WOT, 4th | 11.7 ms | 2.6 ms | 1146 µs/tick | 992 µs/tick |

On the worst mesh-map case, the worst 100 ms window went from 12.0 ms to 2.1 ms per 1 ms tick, and the worst single tick from 41 ms to 3 ms.

**Default behaviour change.** This is a solver-convergence change only: the same LCP is solved to tolerance. Comparison against the previous build over a WOT launch, a step steer at 80 km/h, lift-off, braking, and a parking steer, sampled every 250 ms (max |Δ| / max |value|):

| metric | mx5_na |
|---|---|
| speed | 1.0e-6 m/s / 26.8 m/s |
| yaw rate | 1.3e-6 rad/s / 0.50 rad/s |
| ay | 2.0e-5 m/s² / 7.9 m/s² |
| steering-wheel angle | 7.7e-7 rad / 3.4 rad |
| column torque (FFB) | 1.0e-4 N m / 22 N m |
| rack friction force | 2.0 N / 223 N |

The rack friction force differs by up to 2 N because, in stick-slip, the stuck friction multiplier is only resolved to the work tolerance.
