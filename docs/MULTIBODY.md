# Multibody core (solver = 1)

`solver = 1` in a car `.ini` selects the coupled multibody core (`core/src/dc_mb.c`, `core/src/dc_mbcar.c`).
`solver = 0` selects the legacy reduced 12-DOF core (`core/src/dc_car.c`), kept switchable until Phase 7 of
[MULTIBODY_PLAN.md](MULTIBODY_PLAN.md). `dc_active_solver()` reports the core actually running (AWD cars
fall back to the legacy core for now). The demo HUD shows `physics MULTIBODY` or `physics legacy`.

## What it is

Every part is a rigid body with its own mass and inertia. Every joint transmits only the reactions its design
allows. All of it is one linearly implicit solve per substep (no gates, no effective-value shortcuts):

- chassis (root), per corner A-arms / rods / strut, knuckle, wheel (revolute bearing);
- steering: rim and column, two U-joints, intermediate shaft, valve input shaft, torsion bar, pinion,
  rack-and-pinion gear row, rack (prismatic in its housing, exact stick/slip Coulomb friction), tie rods;
- coil-over spring, bilinear damper and stops along the real spring line, reaction at the top mount;
- anti-roll bar halves with a torsion element and drop links;
- engine and gearbox rotors in the body (reaction torque, gyroscopics), clutch and brakes as Coulomb box rows,
  open/clutch-pack/viscous differential;
- brush tyre on the wheel body, longitudinal force linearly implicit in the solve.

Solver: block-sparse LDLᵀ with tree elimination ordering, box rows by Schur-complement projected Gauss-Seidel,
symplectic position update. About 130 µs per substep at 2 kHz on one core: real time. No heap allocation in the step.

## Audits (every step, in telemetry)

- **Newton-3:** each joint's two sides sum to zero to round-off (≈1e-12 N·m).
- **Momentum:** |dP/dt − F_ext| ≈ 2e-8 N static, 1.6e-3 N cornering. F_ext includes tyre contact forces and moments.
- **Energy:** `dissipated = W_ext − ΔE`, with E = kinetic + gravity + spring/bar/stop + tyre carcass energy, all
  evaluated at the same instant (start of step). W_ext = engine, aero, the driver's hands and
  the active assist. Dissipation is non-negative and non-decreasing to within 2 mJ per 10 ms (0.2 W) at rest.

## Validation (`make mbcartest`, part of `make test`)

| Check | Result |
|---|---|
| Static ΣFz vs m·g | 4e-6 relative |
| Knuckle pose vs closed-loop kinematics (`dc_kin`) | 5e-16 m static, 4e-12 m cornering |
| Newton-3 per joint | ≈1e-12 N·m |
| Self-centring, rim released at 88° at 15 m/s | 0.3° after 1.5 s |

Driven (plug-in driver with finite-strength hands, `tools/dcdrive.py`, and `tools/drivetest.c`), stock 1990 NA
reference car `cars/mx5_na.ini`, legacy → multibody:

| Manoeuvre | Legacy | Multibody |
|---|---|---|
| Skidpad rms path error / ay / β | 0.361 m / 5.51 / 1.27° | 0.376 m / 5.60 / 1.29° |
| Slalom rms path error / ay / β | 0.757 m / 8.10 / 2.82° | 0.761 m / 8.07 / 2.58° |
| Lane change rms path error / peak yaw | 0.551 m / 21.7°/s | 0.559 m / 19.2°/s |
| Single-wheel bump: rim kick / column torque | 2.05° / 2.80 N·m | 1.62° / 2.81 N·m |
| Lift-off, 55 km/h: β / yaw | 3.01° / 34.4°/s | 3.00° / 34.3°/s |
| Lift-off, 60 km/h | spin | spin |
| Hands-off countersteer, lift 85 km/h: β_front / δ / rim | 34.2° / 35.4° / 503° | 35.7° / 36.3° / 514° |
| Hands-off, power-on 60 km/h R40 | spin (β 50°) | caught (β 34.8°, δ 33.3°) |
| Sustained drift, arms driver, β 27°: hold | 5.0 s | 7.5 s (full run) |
| Understeer gradient | 6.82 °sw/g | 7.16 °sw/g |
| Bounce damping ratio | 0.337 | 0.337 |

In both cores the front wheels follow the front axle's slip angle up to full lock when the rim is released
(emergent countersteer). Plots: `results/plots/` (`tools/plot_s0s1.py`, `tools/plot_compare.py`).

## Known gaps

- Valve: by default the steady-state open-centre law. `mb_valve_model = 1/2` adds the rotary-valve bridge with chamber
  and supply pressure states, hose compliance and emergent lag/back-drive (opt-in, docs/VALVE.md).
- Bushings: arm pivots are rigid revolutes by default. `mb_bush_kr/ka/kt > 0` makes them compliant rows (`MBJ_BUSH`, opt-in,
  docs/BUSHINGS.md). Rack mount, ARB and top-mount bushes are still rigid.
- Driveline: the propshaft and diff carrier are massless (one overall-ratio row) by default. `mb_diff_inertia` / `mb_gb_out_inertia > 0`
  adds them as geared rotors (opt-in, docs/CARRIER.md). Half-shafts are still rigid.
- Flex: all parts are rigid. The compliant-beam element (`MBJ_BEAM`, a rigid-finite-element rod, validated against beam
  theory) exists in the library but is not wired into any car part yet (docs/BEAM.md).
- Tyre lateral force still explicit (Phase 4). Both cores creep backward at about 1 mm/s when parked with no brake
  on a flat road (low-speed tyre regularisation). Phase 4 adds the 0 km/h slope-hold test.
- Bushings rigid (Phase 5). Half-shafts/CV joints not separate bodies yet (Phase 6). AWD uses the legacy core.
- The rack and column frictions are exact stick/slip. With the proportional-only torque path driver this gives a
  ±0.45 m hunting cycle around a straight line at 20 m/s (legacy core: smoothed friction, converges). This is a
  control-layer gap, not fixed in the physics.
- Static suspension travel differs from the legacy core by about 1.5 mm (cause not yet isolated; corner loads are identical).
