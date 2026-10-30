# Bricks: tabulated model-reduction blocks (physics.bricks)

A *brick* replaces a pure kinematic map inside the multibody car by a table baked at load: a C1 Hermite
interpolant with stored derivatives. Only pure maps are tabulated (angle transfer, ratios, lever arms against
travel and rack). Stateful dynamics (inertia, compliance, friction, tyre) and the coupling stay live, so the
car is still one coupled solve and Newton 3 holds. `physics.bricks` is a bitmask. 0 (the default) is the full
model. The reference model stays selectable and unchanged.

Order of work: least contact-patch influence first (steering chain, then the suspension top-down, the patch last).

## Two-way table chain

Each table hands its output (the interface variable) down to the next brick. The reaction comes back up into
the upper brick (substructuring). The handoff is **implicit**. The table row sits in the same LDLᵀ / PGS solve
as everything else, and its stored derivative is the row Jacobian (the interface impedance), so it cannot ring.

### Explicit staggered handoff: why not

At the torsion bar (input shaft to pinion, h = 1/960 s), a one-pass-lagged force adds a negative damping
c = −k h, i.e. ζ_lag = −ω h / 2:

| torsion k (N m/rad) | lower inertia | ω (rad/s) | ω h | ζ physical | ζ lag | net |
|---|---|---|---|---|---|---|
| 115 (assist) | pinion + rack | 544 | 0.57 | 0.355 | −0.284 | +0.07 |
| 115 (assist) | rack + steered corners | 97 | 0.10 | 0.063 | −0.051 | +0.01 |
| 1500 (manual) | pinion + rack | 1966 | 2.05 | 0.098 | −1.02 | unstable (ω h > 2) |
| 1500 (manual) | rack + steered corners | 351 | 0.37 | 0.018 | −0.18 | −0.17 (rings, grows) |

Rigid interfaces (tie rod, rack-pinion mesh) cannot be handed off explicitly at all without penalty springs.
Explicit stability (ω h < 1) caps those springs at k < m/h²: 2.8e6 N/m for the 3 kg rack, and 184 N m/rad for the
pinion. Both are 1–2 orders below the real stiffness. The implicit row has no such limit.

## Brick 1: steering column (DC_BRICK_COLUMN = 1)

The intermediate shaft and its two cardan joints become one custom row, θ_in − f(θ_rim) = 0. The table f is baked
from the exact cross-pin solve into a periodic 128-node C1 Hermite table, with the shaft angle tabulated as well.
The up pass is the exact self-equilibrated cardan wrench on the rim and input bodies: cross-pin torques plus a
transverse force pair, which reproduces the bearing reactions. The shaft's spin inertia is lumped on the rim
(J ψ′²).

Bake fidelity (mx5_na demo car):

| quantity | value |
|---|---|
| max \|f − θ\| | 0.0184 rad |
| table error | 4.0e-9 rad |
| f′ error | 2.5e-7 |
| up-pass wrench vs table f′ | 2.8e-7 |

Model size drops from N 331 to 319 rows, L-block doubles from 3685 to 3521, and one body is removed.

Results (fast + brick vs reference, fastcheck):

| manoeuvre | result |
|---|---|
| slalom, skidpad, liftoff, step, braking, cobbles (pooled), bump | PASS |
| drift_entry_exit | FAIL: rack 165/2675 N, bands −0.29 / +0.18 / −0.93 / +0.02 dB |

The drift failure is a shifted single-tick stick/impact rack spike. The full model without the brick shows the same
failure for a ±20% change in shaft inertia or a 25% change in column inertia, so the deviation lies inside the
parameter-uncertainty floor. It is still not a strict PASS.

Cost (slalom, alternating rounds of 5 repeats): step mean 667 → 642 µs (−3.8%). The paired differences are −44, +3
and −35 µs against a noise of about 5%. The factor and Schur stages are both about 5% lower, which is consistent
with N −3.6%. In the closed-loop slalom the result is mixed: −2% to +5% at 240×4, and −5.6% at 240×8.

**Verdict: kept selectable, not default.** It is not a strict fastcheck PASS, and the saving is not measurable
above the noise.

## Rack/pinion and tie rods

These are already minimal one-row exact interfaces: the rack-pinion ratio is constant, and each tie rod is one
distance row. There is nothing to tabulate. A toe/steer-arm table needs the corner reduction below.

## Brick 2 candidate: knuckle-pose table P(s, r) per corner, rejected

The candidate tabulates the knuckle pose against suspension travel and rack (64×32 bicubic: 393 kB per corner,
96 FMA per lookup). It would remove the arms, bushes and ball joints, about 23 rows per corner (~28% of N). But it is
a rigid-kinematics map, so it drops the bush compliance. Upper-bound test: the fast rig with rigid bushes
(`mb_bush_k* = 0` builds exact revolutes) against the default reference fails **8 of 8** fastcheck manoeuvres:

| manoeuvre | deviation |
|---|---|
| slalom | +26 dB rack 40–120 Hz |
| skidpad | pos 16.8 m |
| step | yaw 2.9 dps |
| braking | −3.6 dB |
| cobbles | rack −7.8% |

Any kinematic table of the corner must fail at least this badly. **Rejected.**

## Whole-corner table (all state as axes): estimate

| axes per corner | count |
|---|---|
| corner configuration (knuckle, arms, wheel; bushes compliant, spin angle excluded) | 11 |
| corner velocities | 12 |
| chassis velocity (at the corner) / attitude to the road | 6 / 2 |
| rack position, velocity | 2 |
| drive, brake torque | 2 |
| tyre state (carcass ex, ey, twist phi, pressure) | 4 |
| road (height, normal 2, grip) for a planar road | 4 |
| anti-roll bar twist, rate | 2 |
| **total d** | **45** |

Outputs: state increments plus the interface wrench and rack/ARB loads (about 35). An implicit, non-ringing
coupling also needs the interface Jacobian (8×8 = 64), so about 100 in total.

- **Size:** 8^45 ≈ 4e40 nodes at only 8 nodes per axis. With C1 Hermite (2^d coefficients per node), that is about
  1e57 bytes.
- **Lookup:** multilinear needs 2^45 ≈ 3.5e13 node reads, and C1 tensor Hermite needs 4^45 terms. The direct
  corner costs about 30 µs per substep.
- **Rough road:** off a plane, the patch sees a field of ray heights (hundreds of values), not 4 numbers.
- **Quasi-static cut to d = 8** (travel, travel rate, rack, slip angle, slip ratio, speed, road tilt, grip) at 16
  nodes per axis: 4.3e9 nodes. That is 344 GB as values only, and 88 TB with C1 derivatives. A lookup is 256
  scattered reads × 10 outputs, which misses RAM entirely. It also drops tyre transients, bush compliance and every
  history state by construction.

**Conclusion:** a whole-corner table is not buildable or faster, and the only tabulable bricks are the low-dimensional
pure maps above.

## Closed-loop response: does saved time improve control?

`build/slalomcl` runs the same closed-loop driver on every version: pure pursuit with a 0.5 s look-ahead, steering
angle through the hands, and a speed-hold PI. The course is the 18 m pitch slalom with A = 1.2 m. Tick is 240 Hz;
substeps are listed per version. Tick cost is the minimum of 3 runs per tick. Bands are rack-force dB vs the
reference at 0–5 / 5–15 / 15–40 / 40–120 Hz.

**50 km/h (f0 0.386 Hz)**

| version | path err rms/max m | yaw lag deg / ms | peak ay g | rack bands dB | bitwise vs ref | tick µs mean/p99/max |
|---|---|---|---|---|---|---|
| reference ×4 | 0.363 / 0.548 | 12.61 / 90.8 | 0.691 | 0 / 0 / 0 / 0 | n/a (base) | 934 / 1030 / 4841 |
| reference + column ×4 | 0.363 / 0.547 | 12.60 / 90.8 | 0.691 | −0.01 / +0.06 / +0.08 / −0.36 | no, Δpos 1 mm | 922 / 1023 / 4955 |
| fast ×4 (demo) | 0.363 / 0.548 | 12.61 / 90.8 | 0.691 | 0 / 0 / 0 / −0.01 | no, Δpos 1 mm | 632 / 705 / 969 |
| fast + column ×4 | 0.363 / 0.547 | 12.60 / 90.8 | 0.691 | −0.01 / +0.06 / +0.08 / −0.36 | no, Δpos 1 mm | 661 / 733 / 1018 |
| reference ×8 | 0.366 / 0.553 | 12.61 / 90.8 | 0.692 | −1.39 / −5.28 / −9.67 / −9.87 | no, Δpos 15 mm | 1880 / 2023 / 2673 |
| fast ×8 | 0.366 / 0.553 | 12.61 / 90.8 | 0.692 | −1.39 / −5.28 / −9.67 / −9.87 | no, Δpos 15 mm | 1347 / 1471 / 1933 |
| fast + column ×6 | 0.365 / 0.551 | 12.61 / 90.8 | 0.692 | −0.39 / +2.09 / +1.03 / −3.62 | no, Δpos 10 mm | 984 / 1091 / 1435 |
| fast + column ×8 | 0.366 / 0.553 | 12.60 / 90.7 | 0.692 | −1.39 / −5.29 / −9.67 / −9.84 | no, Δpos 14 mm | 1271 / 1381 / 1723 |

**60 km/h (f0 0.463 Hz)**

| version | path err rms/max m | yaw lag deg / ms | peak ay g | rack bands dB | tick µs mean/p99/max |
|---|---|---|---|---|---|
| reference ×4 | 0.775 / 1.144 | 15.40 / 92.4 | 0.756 | base | 972 / 1088 / 4710 |
| fast ×4 | 0.775 / 1.144 | 15.40 / 92.4 | 0.756 | 0 / 0 / 0 / 0 | 659 / 738 / 1015 |
| fast + column ×4 | 0.775 / 1.144 | 15.39 / 92.3 | 0.756 | −0.01 / 0 / −0.01 / +0.03 | 648 / 727 / 985 |
| fast ×8 | 0.777 / 1.146 | 15.45 / 92.7 | 0.759 | −1.45 / −8.07 / −6.60 / −11.88 | 1322 / 1481 / 1873 |
| fast + column ×8 | 0.777 / 1.146 | 15.45 / 92.7 | 0.759 | −1.45 / −8.01 / −6.57 / −11.85 | 1282 / 1395 / 1819 |

No version is bit-identical to the reference. The fast path differs from tick 0 at the 1e-3 m level.

**Findings**

1. **Response and control are unchanged.** Path error, yaw-rate lag (about 91 ms behind the wheel angle) and peak
   lateral g agree to the millimetre and to 0.05° across every model and substep rate. The lag is vehicle and tyre
   dynamics, not numerics. Substeps do not change input latency: the driver and inputs run at the 240 Hz tick, so a
   tick is 4.2 ms, under 5% of the 91 ms response delay. Halving it needs `tick_hz` 480, not more substeps.
2. **What the reinvested time buys is rack-force detail.** At 240×8 the rack band power above 5 Hz drops by 5–12 dB
   in both the reference and fast models (identically). So the 240×4 rack spectrum above 5 Hz is not converged in
   the step size: this is a force-feedback feel question, not a handling one. 240×8 fits the 2 ms budget on this
   box (fast ×8 max tick 1.93 ms, p99 1.47 ms; with the column brick, max 1.72 ms), with little margin. That is a
   proposal for review, not a default change.

## Steady skidpad (R 40 m, closed loop, ramp 0.25 km/h/s, both directions)

`build/skidmax` uses a pure pursuit plus radius integral driver. A speed counts as steady when the car stays within
the lanes (±2 m) with speed held (±1 km/h) for 2 s.

| setup + model | v max steady (km/h) | ay, 2 s mean (g) | steering wheel at limit | body slip |
|---|---|---|---|---|
| legacy + reference | 65.6 | 0.80 | 224° | 2.8° |
| default + reference | 66.1 | 0.79–0.80 | 252° | 1.8° |
| default + fast | 66.1 | 0.79–0.80 | 252° | 1.8° |

Fast matches reference to the printed precision in both directions; the traces are not bit-identical. The limit is
understeer (steering wheel 125° → 250° over the last 3 km/h). This driver holds a grip line, so it does not measure
power-on drift.

## Offload lane (tyre-patch rays) and road bake

The lane is fast-model only; the reference model and `patch = ref` keep the double ray march. The road bake is a demo
road option for either model; fastcheck applies both only to its fast rig, so it still compares against an
untouched reference.

**Offload lane (`[offload] patch`).** Each wheel keeps a patch table: the road heights and grip codes under its contact
patch, filled in double and stored as float32. The ray march over that table runs on one of three backends: `cpu`
(scalar), `cpu_simd` (AVX2, 8 rays per instruction) or `gpu` (OpenCL, loaded at run time). Every backend runs the same
float32 operations in the same order, so all three give the same bits. A ray that leaves the table or runs past the
cell guard is handed back to the double path ("handed back" in the stats; 0.00 % on every manoeuvre).
The per-ray state, the patch forces and every solver accumulator stay double.

`patch = auto` (the demo default) picks a backend from the live batches, not from a synthetic self-test. The
self-test favoured `cpu` by about 10 %; on real batches `cpu_simd` is about 2× faster. Each wheel runs the candidates
(bit-identical backends within 3× of the best self-test cost) in blocks of 32 calls for its first 384 calls, then
keeps the lowest mean. The F7 overlay shows the pick, the live cost per backend, and "(auto, idle)" while no wheel
has made a lane call (flat ground). The library's fallback default stays `ref`.

| 960 × 4, wheel 0 | cpu µs/batch | cpu_simd µs/batch | gpu (pocl on CPU) µs/call |
|---|---|---|---|
| cobbles t1 / t4 | 5.39 / 5.58 | 2.60 / 2.84 | 73.7 (max 11 ms) |
| bump_pothole t1 / t4 | 5.67 / 5.84 | 2.56 / 2.70 | — |

On this box there is no real GPU, so `gpu` runs on pocl's CPU device and is never picked; it still passes fastcheck.

**Error against double rays (cobbles, 43 M rays):** float32 table, 33 contact flips, max contact-height error
9.4e-5 m; pooled rack bands −0.41 / +0.01 / −0.03 / −0.02 dB, rms rack −0.01 %, PASS. A 16-bit table (heights on a
2^-16 m grid, grip on 2^-14; `DC_LANE_Q16=1`, measurement only) gives 3286 flips and a max error of 3.2e-3 m on
cobbles, with band 0 −0.60 dB and yaw rms −1.35 %. That is outside the error budget, so the lane stays float32.
A long closed-loop lap on a mesh track with float32 rays passes the lap check against double rays (lap time and
rack bands).

**Zero-snap.** When a wheel is airborne its free tread shear deflections (ex, ey, phi) relax towards zero; once one
is below 1e-9 (m or rad, far below the brush's deflection resolution) it is set to exactly 0 instead of decaying
forever towards denormals. On by default in the fast model. It
fires on 20–27 % of the airborne decays on cobbles and 0.5 % on bump_pothole; fastcheck passes with it on.

**Road bake (`[track] bake = 0 | 32 | 16`, default 0).** At load, the road inside a band around the centreline
(half width + kerb + 1 m) is sampled on the road's own lattice (1.6 m resolve, 64 × 64 node tiles) and stored as
offsets: float32 from a base aligned to 2^-10 m (`32`), or uint16 with a per-tile power-of-two step (`16`). Grip is a
u8 index into an exact palette. Every exact zero in the source decodes to exactly 0 (the bake refuses to finish
otherwise). Only height queries that land exactly on a lattice node use the table; everything else calls the live
road. One bake is shared per process and rebuilt on a map switch (F6).

| proving ground | memory | prewarm | max / rms height error | exact zeros kept |
|---|---|---|---|---|
| bake = 32 | 313 MB | 8.0 s | 1.5e-8 / 1.7e-9 m | 32495 of 32495 |
| bake = 16 | 188 MB | 7.5–7.9 s | 3.8e-6 / 7.3e-7 m | 32495 of 32495 |

The grip palette is 64 MB of each. With `bake = 16` the full fastcheck passes (8 of 8, cobbles pooled), and a
closed-loop mesh-track lap passes the lap check (lap time and rack bands).

**What the bake buys.** Same box, 960 × 4, fast model with `patch = auto`, bake off vs `bake = 16`, t1: cobbles
mean 628 → 620 µs, worst 969 → 836 µs, `w_cull` 27 → 23 µs, `w_rays` 112 → 108 µs; bump_pothole mean 525 → 521 µs.
That is a 1–2 % gain for 188 MB and an 8 s load, so the bake stays off (proposal only, not a default change).

## Before / after (960 × 4, intrinsic tick, min of 3 runs, µs)

"Before" is the build at the start of this pass; "after" is HEAD with `patch = auto` and `bake = 16`.
mean / p99 / worst:

| manoeuvre | model | t1 before | t1 after | t4 before | t4 after |
|---|---|---|---|---|---|
| slalom | fast | 603 / 675 / 961 | 503 / 567 / 897 | 501 / 568 / 896 | 504 / 595 / 918 |
| step_steer | fast | 608 / 707 / 951 | 513 / 600 / 853 | 529 / 626 / 905 | 493 / 582 / 789 |
| cobbles | fast | 774 / 854 / 980 | 623 / 689 / 831 | 564 / 625 / 870 | 523 / 594 / 762 |
| bump_pothole | fast | 653 / 713 / 982 | 522 / 590 / 908 | 546 / 611 / 833 | 486 / 552 / 858 |
| cobbles | reference | 1016 / 1168 / 1417 | 934 / 1062 / 1227 | 645 / 790 / 1171 | 623 / 757 / 1278 |

Per stage, fast, t1 (mean µs, before → after): cobbles `w_rays` 185 → 110 (plus `w_cull` 24), factor 144 → 111,
schur 162 → 110, ksolve 22 → 18, backsub 23 → 18; slalom `w_rays` 50 → 17 (plus `w_cull` 38). PGS and assemble
are unchanged. At t4 the solver stages are split across threads and the slalom totals are within noise.
