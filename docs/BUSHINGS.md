# Arm-pivot bushings as compliant rows (MB core)

Status: implemented, opt-in. The default stays rigid (all `mb_bush_*` rates are 0, so each arm pivot is a revolute and the results are
bit-identical to the rigid core). The dist ships `<car>_propB.ini` variants with the estimates below as a proposal.

## Model

- Each arm's two inner bushes are lumped at the midpoint of the pivot axis as one `MBJ_BUSH` joint (`mb_bushing`, dc_mb.c).
  The joint has 5 rows in a frame fixed in the chassis: axial, two radial, and two conical (tilt). Rotation about the pivot axis
  is free, because the road spring carries the wheel rate. Torsional bush rate is therefore not modelled.
- Every row is an implicit (backward-Euler) spring-damper solved in the same KKT system as the rigid rows. It uses the standard
  soft-constraint form (Catto, "Soft Constraints", GDC 2011; Lacoursière 2007):
  `J v⁺ + λ / (h²k + hc) = −k φ / (hk + c)`, which is exactly `f = −k (φ + h J v⁺) − c J v⁺`.
  There is no gate and no explicit force. The rows are not position-projected: their φ is the bush deflection.
- Both translation forces act at the common midpoint of the two marker points. A massless rubber element transmits an equal
  and opposite wrench. Applying the forces at the two deflected markers would leave a spurious couple (pa − pb) × F.
  This was caught by the per-joint Newton-3 test.
- Lumping two bushes a distance L apart: translation 2kr (radial) and 2ka (axial); tilt 2kt + kr L²/2, because the pair resists
  tilt by opposite radial deflection. This is the "two colinear spatially separated bushes" approximation that K&C practitioners
  use for coning.
- Damping is a loss factor: c = `mb_bush_damp` · k / (2π · 10 Hz), with default 0.15. Rubber loss factors are typically 0.1–0.3.
- Energy: ½ Σ k φ² is included in the stored-energy audit. The implicit rows can only dissipate (tests below).

## Parameters (per bush, front and rear)

| key | proposal | basis |
|---|---|---|
| `mb_bush_kr` | 2e6 N/m (2 kN/mm) | Mid-range of measured passenger-car arm bushes, 0.57–5.5 kN/mm. Source: Skoda MacPherson and double-wishbone bushes, FEA vs test. Stiffness analysis of the rubber bushings of MacPherson and double wishbone suspensions, ETR 2024, doi:10.17770/etr2024vol1.7993 |
| `mb_bush_ka` | 5e5 N/m | Axial rate is typically ¼–⅕ of radial for a plain cylindrical bush. Estimate. |
| `mb_bush_kt` | 300 N·m/rad | Own conical rate of one bush. Below the ≈5000 N·m/rad that one analytic estimate gives, which its author judged too stiff. The coning of the pair (kr L²/2) dominates anyway. Estimate. |
| `mb_bush_damp` | 0.15 | Loss factor. Estimate. |

NA-specific bush rates are not public, so these are documented estimates in the sense of the project rules.

## Validation

- `make mbtest` (`t_bushing`):
  - Static deflection equals F/k in the radial and axial modes (1e-15) and the conical mode (3e-3; the midpoint force adds a second-order moment).
  - Stored energy equals ½F²/k.
  - A free pair conserves linear momentum (2e-14 N·s).
  - With zero damping, energy never increases from one step to the next.
- `mbcar_tests` (bushed section, all 8 arm pivots compliant):
  - Static ΣFz = mg.
  - Dissipation is ≥ 0 and never decreases.
  - Newton 3 holds per joint (1e-12 N·m static, 3e-11 m relative in cornering).
  - Momentum residual is 3e-3 N.
  - At 4.2 m/s² the pose departs from the rigid kinematics by 0.9 mm (bush deflection).
- Default rates: all 8 `mbcompare` manoeuvre CSVs for mx5_na (MB core) are byte-identical to the previous commit.

## Effect on handling (mx5_na, MB core, `mbcompare … 1 all`, rigid → propB)

| metric | rigid | bushed |
|---|---|---|
| understeer gradient (steering wheel) | 7.1 deg/g | 18.5 deg/g |
| step yaw-rate steady state | 15.9 deg/s | 11.9 deg/s |
| step overshoot | 4.8 % | 10.3 % |
| skidpad ay max | 0.78 g | 0.78 g |
| bump heading change | 0.46 deg | 0.23 deg |
| bounce ζ estimate | 0.39 | 0.41 |
| CPU per sim-second | 4.4 | 4.0 |

Compliance steer from generic bushes adds understeer. This direction is expected; production cars use compliance steer
as an understeer contribution (Milliken & Milliken, *Race Car Vehicle Dynamics*, 1995). However, a lumped bush cannot reproduce a designed
toe-control compliance (voided or rate-split bushes, rear toe-compliance links). So the size of the effect depends on the
estimates, which is why the proposal stays opt-in.

Fishtail-settle check (`drivetest fishcheck`, propB rates):
- lift_85kmh_R65 gp_human now passes: ζ 0.07, settle 2.34 s (rigid: ζ 0.03, 3.16 s, FAIL). The bush damping is in the steer–yaw loop.
- lift_85kmh_R65 ideal settles in 0.61 s, against the 0.6 s limit (rigid: 0.40 s), so that run fails.
- The free-wheel 85 km/h known residual passes.

## Not done yet

- Rack-mount compliance (`mb_rack_mount_k/c` are parsed but unused). It needs the rack housing as a body on soft rows.
- Torsional bush rate, frequency-dependent (fractional or Maxwell) rubber damping, the ARB bushes, and the strut top mount.
