# Braking in a turn: why the car "straightens"

Investigation of the report "under braking the whole car straightens up instead of braking in the state it is in"
(steady left turn under throttle, no slip, then the brakes with the steering held left). Tool: `make braketurn`,
`./build/braketurn <car.ini> turn45|turn70|drift|yawst <pedal 0..1> out.csv` (per-tick yaw, body slip, wheel steer
angles, rack, and per-wheel Fz/Fx/Fy/slip/omega/brake torque/sliding flag). Numbers below: cars/mx5_na.ini, 60 km/h,
45 deg at the steering wheel (~0.35 g), pedal ramped in over 0.1 s at t = 5 s, wheel held; k = yaw rate / speed (path
curvature), k0 its value before the brake.

## What happens

| pedal | default (fast = reference) | legacy setup |
|---|---|---|
| 0.3 | k/k0 1.04 / 1.19 / 1.17 at +0.25 / 0.5 / 1 s | |
| 0.6 | 0.99 / 1.46 / 1.30 (trail-brake rotation) | 0.90 / 1.65 / 1.84 |
| 0.8 | 0.66 / 0.72 / 1.07: inner front locks, yaw rate drops ~40 % | 0.50 / 0.50 / 0.75 |
| 1.0 | 0.09 / -0.23 / -0.02: both fronts locked within 0.25 s, the car goes straight | 0.10 / -0.19 / 0.07 |

* The steering does not self-centre: the steering-wheel angle stays at 43-45 deg and the rack keeps its position under
  braking in every model. The column, rack and kingpin geometry are not the cause.
* Fast and reference are identical to plotting accuracy (largest difference ~1e-3 in the logged columns); car.solver=0
  matches the multibody legacy setup. Not a solver, fast-path or Woodbury effect.
* Below the lock threshold the car rotates *more* when braked (load transfer to the front axle), as a real car does.
* At and above ~0.8 pedal in a 0.35-0.4 g turn the inner front wheel locks first (equal left/right brake torque, the
  inner wheel carries the least load), then the outer one. A locked wheel slides; its friction force opposes its sliding
  velocity, which is nearly straight back, so the lateral force left is only ~tan(slip angle) x the braking force
  (a few percent). The front axle loses its cornering force, the rear still has it, the yaw rate dies in ~0.15-0.25 s
  (tau ~ Iz v / (C_r b^2)) and the car carries straight on. This is the physical behaviour of a car without ABS at a
  locking pedal force, not a model artefact; the keyboard brake is 0/1 (full pedal), so on the keyboard every brake
  application in a corner locks the fronts.

## Model defect found and fixed: the sliding direction of the legacy brush tyre

In full sliding the brush tyre's carcass deflection was returned to the friction limit by a radial scaling
`e /= lam`. That keeps the deflection parallel to the trial (which at lock is parallel to the sliding velocity), so the
force K e came out parallel to (kx Vsx, ky Vsy) rather than opposite Vs: with the carcass twice as stiff in x as in y
the lateral force of a sliding wheel was 2-5x below the Coulomb value tan(alpha)/|kappa| x Fx. The fix is the implicit
plastic return of the brush state equation with sliding along the friction force,

    de/dt = -Vs - (Vr/sigma) e - nu K e   ->   e_i = e*_i / (1 + nu K_i / a_i),   |K e| = 3 mu N,

solved for nu >= 0 by monotone Newton (convex, from nu = 0, fixed step count). It is applied before the brush
evaluation (so the applied force has the right direction) and at the commit. Radial scaling is the special case of
equal axis stiffnesses. Locked front wheels, legacy setup, pedal 1.0: Fy/Fx relative to Coulomb 0.55 -> 0.96 (mean of
both fronts); front-axle lateral force at +0.25 s 262 -> 442 N. Small yaw effect (k/k0 0.01 -> 0.10 at +0.25 s): a
locked front axle has little lateral force either way. Remaining offset: the brush keeps its camber slip in sliding
(inner/outer front +-40 %), which a non-rolling tread would not have (not changed; proposal).

The TARMAC tyre (default setup) relaxes its deflection with the travel speed and returns along the trial too; its
locked-wheel force is 0.81 of the Coulomb ratio, the ratio of its x/y slip stiffness scaling. Not changed (owner's
model).

## Secondary: braking compliance steer of the default bushings (propBR)

At pedal 0.6 (2.1 kN brake force per front) the default bushings toe both fronts in: inner front 2.04 -> 1.39 deg,
outer 2.44 -> 2.78 deg (rigid pivots: 2.68 -> 2.61 / 2.84 -> 2.68). The inner front's slip angle and lateral force drop
(756 -> 330 N). The yaw response is similar with and without (k/k0 1.46 vs 1.49 at +0.5 s), so it is not the cause
of the straightening, but ~0.3 deg/kN is more braking toe-in than usually designed in; a stiffer axial bushing rate
is a candidate proposal (not changed).

## Regression

fastcheck manoeuvre `brake_in_turn`: 45 deg steady left turn at 60 km/h, pedal 0.8 with the wheel held, fast vs
reference.
