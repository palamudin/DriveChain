# Fast physics model (`physics.model = fast`)

The demo ticks the car at 240 Hz with 4 substeps. The target is a **physics tick that never takes more than 2 ms**
(p100), with mean and p99 well below, while the car behaves the same as the reference model.
The demo runs `physics.model = fast` by default (demo.ini). `physics.model = reference` stays selectable and is unchanged bit for bit (checked against the previous commit with
`FC_REFDUMP`, below). The legacy solver (`solver = 0`) is untouched.

## What the fast model changes

All changes are exact or bounded reformulations of the same physics: one coupled solve, Newton 3 holds, no gates.

| Stage | Reference | Fast |
|---|---|---|
| Tyre road ray / rim contact on a planar road patch | general ray march | closed form (exact on a plane; the general path is still used off-plane) |
| TARMAC brush ring threshold | iterated | exact Newton on the ring threshold |
| TARMAC brush stick scale | iterated | closed form |
| Contact PGS (projected Gauss-Seidel) | up to 400 sweeps per solve, every solve | per-tick sweep budget (default 160) shared by the tick's solves: each solve may use max(8, budget left / substeps left), clipped to 400; unused sweeps carry over; exact limit-cycle exit kept |
| Second solve when a tyre leaves the friction circle | full refactor + Schur of the changed box rows | Sherman-Morrison-Woodbury low-rank update of the existing factor for the changed rows (exact up to round-off); the updated operator is also used by the post-stabilisation solve of the same substep |
| PGS row dot products | sequential sum | 4 partial sums (same terms, different round-off order) |

Both models also cache, per PGS solve, the eigen-decomposition and elimination factor of each isotropic friction
group's diagonal block (the block is constant during a solve, so the sweep recomputed the same numbers). The
operations are identical, so the reference stays bit for bit the same.

The PGS budget is the only change that is not exact. The reference itself stops at 400 sweeps without converging on
slalom, bumps and setts, and those capped solves were the 6-10 ms ticks. Bounding the iterations of an iterative
complementarity solver is the standard real-time trade (Erleben; Silcowitz, Niebe and Erleben; Poulsen et al.,
below). The fidelity suite below decides whether the bound is acceptable.

## Fidelity suite (`make fastcheck`)

`tools/fastcheck.c` runs the reference and the fast model side by side through the exact demo tick (`demo/sim.c`)
with the same scripted, open-loop inputs. It compares them tick by tick: trajectory, yaw rate, slip angles, wheel
loads, rack force, and the rack-force spectrum (Welch band powers in 0-5, 5-15, 15-40 and 40-120 Hz).

Manoeuvres: slalom, skidpad (J266-style steady state), lift-off in a turn, step steer, drift entry and exit,
straight braking, cobbles, and a bump/pothole. A proving-ground lap is checked with `FC_LAP`.

**Thresholds (fixed before the fast path was tuned).** Values for the drift manoeuvre are in brackets.
- Position: max 0.10 m [0.5 m].
- Yaw rate: rms 0.5 deg/s [1.5], max 2 deg/s [6].
- Slip angle: rms 0.15 deg [0.5], max 0.75 deg [3].
- Wheel load: rms 40 N, max 200 N (cobbles/bump: 80 N, 600 N).
- Rack force: rms max(15 N, 1 % of rack rms), max max(60 N, 4 % of rack rms).
- Rack spectrum: each band within ±0.5 dB (bands under 1 N rms in both models are ignored).

**Amendments.** Both were added before tuning and are printed by the tool.
1. *Chaos floor.* On cobbles and bumps, the reference does not reproduce itself when the start yaw changes by
   1e-15 rad: contact chatter makes it diverge. Fast cannot be asked to be closer to the reference than the reference
   is to itself, so each limit becomes max(threshold, ensemble spread of 8 perturbed reference runs). Cobbles is
   additionally judged on pooled statistics over 12 passes (4 lateral offsets × 3 speeds). The pooled rack band
   powers must be within ±0.5 dB, and the pooled rms of rack, yaw rate, load fluctuation and slip angle within ±3 %
   (half the force-feel JND).
2. *Solver floor.* The reference's own 400-sweep cap is a tolerance, not ground truth. The same reference run with
   caps of 800 and 1600 is an equally valid reference, so those runs join the floor ensemble. Drift never hits the
   reference cap, so its floor is empty and drift is judged on the fixed thresholds.

**Result (mx5_na, 240 Hz × 4, 4 threads, budget 160): all eight manoeuvres pass.**

| Manoeuvre | Verdict | Rack error rms / max |
|---|---|---|
| Slalom | PASS | 0.02 / 1.0 N |
| Skidpad | PASS | 1.4 / 12.5 N |
| Lift-off | PASS | 0.2 / 0.9 N |
| Step steer | PASS | 0.2 / 0.9 N |
| Drift entry/exit | PASS (fixed thresholds) | 0.6 / 21 N |
| Braking | PASS | 0.3 / 2.3 N |
| Cobbles | PASS (pooled; reference chaos floor) | |
| Bump/pothole | PASS (within the chaos + solver floor) | 6.1 / 281 N (floor 460 N) |

Budgets under 128 sweeps fail drift on peak rack force (limit 60 N; a cap of 32 per solve gives more than 60 N).
The budget of 160 leaves margin on that metric.

## In the demo (real time)

Measured on a shared 8-vCPU virtual machine with software OpenGL, 40 s autopilot runs, native Linux and Wine:
- Fast, 1 physics thread: step mean 1.0-1.2 ms, p99 1.7-2.0 ms. Most ticks over 2 ms are the physics thread being
  descheduled (one stage that normally takes 25-300 µs takes 5-16 ms), not solver work. The intrinsic worst tick is
  1.17 ms.
- Fast, 4 physics threads: in real time the worker pool's wake-up waits reach 13-35 ms, and p99 is 1.4-1.7 ms
  natively but 6-7 ms under Wine. On this machine 1 thread has the better tail. The demo default stays 4 threads;
  1 thread is a proposal.
- The renderer no longer waits on a physics tick. The physics thread publishes a copy of the render state after every
  tick, and the renderer posts its controls through a lock held only for that copy. The renderer's lock wait went from
  p99 1.0-1.7 ms (max 11 ms) to p99 1 µs (max 8 µs).

## Timing method

- **Intrinsic tick cost** (`FC_REP=n`): each manoeuvre is repeated n times and the per-tick minimum is kept. This
  removes box noise (page faults, preemption) and leaves the cost of the maths. The raw maximum is reported as well.
  `FC_STAGES=1` adds per-stage costs at the mean and at the worst tick. This mode does not check fidelity.
- **In the demo** (`DC_PROF` builds, `DC_PROF_OUT`): the tick, batch and frame rings are dumped to CSV. This runs
  natively and under Wine on the same box, with 1 and 4 physics threads. Render contention is separated by comparing
  the step cost (physics only) with the tick wall time and the frame times.

Intrinsic tick, mx5_na, 4 threads pinned to 4 cores, `FC_REP=3` (µs, mean / p99 / max). "Before" is the fast model
with the PGS budget and tyre closed forms only; "after" adds the Woodbury re-solve, the block cache and the 4-sum dots.
The reference column is after (the block cache also speeds up the reference, bit-identically).

| Manoeuvre | Reference | Fast before | Fast after |
|---|---|---|---|
| Slalom | 965 / 1093 / 3899 | 728 / 846 / 1418 | 676 / 783 / 1057 |
| Skidpad | 1020 / 1316 / 5052 | 740 / 912 / 1468 | 694 / 799 / 1043 |
| Lift-off | 994 / 1241 / 4903 | 766 / 1012 / 1445 | 677 / 794 / 1002 |
| Step steer | 1012 / 1376 / 5088 | 770 / 1186 / 1514 | 689 / 814 / 1038 |
| Drift | 1151 / 1474 / 2362 | 974 / 1398 / 1828 | 796 / 1023 / 1165 |
| Braking | 1017 / 2276 / 5334 | 723 / 1130 / 1473 | 673 / 887 / 1054 |
| Cobbles | 1139 / 1301 / 1557 | 915 / 1070 / 1476 | 893 / 989 / 1123 |
| Bump | 1021 / 1210 / 5237 | 760 / 1015 / 1503 | 726 / 837 / 1111 |

PGS at the worst fast tick fell from 650-780 µs to 75-330 µs. The worst fast tick (drift, about 1.17 ms) is now
spread over several stages: PGS about 0.33 ms, Schur complement about 0.19 ms, factorisation about 0.15 ms, contact and
tyre (phase A) about 0.13 ms, assembly about 0.12 ms. The TARMAC brush force itself is about 5 µs per tick for all
four wheels.

## Not done, and why

- **Pre-baked tyre tables** (a blind sweep of the tyre over the road surface, Hermite-interpolated, cached by hash;
  also a single-precision structure-of-arrays tyre path). Measured: the brush force costs about 5 µs per tick for all
  four wheels, so a table or SIMD path could save at most that. Not built; the fidelity risk (transient brush state,
  interpolation across stick/slide) was not worth 5 µs. No bake step exists.
- **Aligning-moment table and multi-rate bristle loop.** The aligning-moment solve is about 20 µs per tick; a
  multi-rate bristle loop would add work at this step size. Not built.
- **Saturation predictor** (guessing which tyres leave the friction circle so the first solve already uses the right
  rows). It mispredicted often enough during transitions that the second solve was still needed; removed.
- **Incremental PGS** (keeping the row residuals up to date by column updates instead of row dot products). With
  about 30 box rows that all change every sweep, the updates cost as much as the dots. No gain; removed.
- **Friction-bound update every m sweeps** (Anitescu-Tasora style fixed-point on the bounds). m = 2 passes but doubles
  the drift rack error (21 to 45 N) for about 40 µs; m = 4 fails drift on rack force (about 210 N). Not adopted.

## References

- M. Gipser, M. Baumann, *FTire on the Driving Simulator*, VI-grade users' conference 2018 (FTire/realtime: reduced
  resolution, smarter contact search, efficient Jacobian update, 1 ms global step). cosin scientific software,
  FTire hard-real-time HiL product notes (cosin.eu).
- A. J. C. Schmeitz, H. B. Pacejka, *A semi-empirical three-dimensional tyre model for rolling over arbitrary road
  unevennesses*, Vehicle System Dynamics 41 (suppl.), 2004. MF-Tyre/MF-Swift 6.2 manual (effective road plane from
  elliptical cams; rigid ring valid to 60-100 Hz).
- Fraunhofer ITWM, CDTire (scalable structural tyre models; the real-time variants reduce structural resolution).
- Michelin TameTire (thermo-mechanical, real-time capable). Assessed in *Vehicle System Dynamics* 48 (2010),
  doi:10.1080/00423111003706730; patent WO2008025892A1 (iterative brush with local sliding, about 9000 tyre
  evaluations per second on a 2008 PC).
- H. B. Pacejka, *Tire and Vehicle Dynamics*, 3rd ed., 2012, ch. 3 (brush model) and ch. 9 (short-wavelength
  enveloping).
- M. Silcowitz, S. Niebe, K. Erleben, *Projected Gauss-Seidel subspace minimization method for interactive rigid body
  dynamics*, GRAPP 2010, doi:10.5220/0002830700380045; *A nonsmooth nonlinear conjugate gradient method for
  interactive contact force problems*, The Visual Computer 2010.
- M. Poulsen, S. Niebe, K. Erleben, *Heuristic convergence rate improvements of the projected Gauss-Seidel method for
  frictional contact problems*, WSCG 2010.
- D. Baraff, *Linear-time dynamics using Lagrange multipliers*, SIGGRAPH 1996 (sparse factorisation of
  constrained multibody systems). R. Featherstone, *Rigid Body Dynamics Algorithms*, Springer 2008.
- W. W. Hager, *Updating the inverse of a matrix*, SIAM Review 31(2), 1989, doi:10.1137/1031049 (Sherman-Morrison-
  Woodbury low-rank update).
- J. C. Simo, T. J. R. Hughes, *Computational Inelasticity*, Springer 1998 (return mapping onto a convex admissible
  set, consistent tangent).
- M. Anitescu, A. Tasora, *An iterative approach for cone complementarity problems for nonsmooth dynamics*,
  Computational Optimization and Applications 47, 2010, doi:10.1007/s10589-008-9223-4.
