# Driveline carrier inertia (MB core)

Status: implemented, opt-in. The defaults are `mb_diff_inertia = mb_gb_out_inertia = 0`. With both at 0 the driveline is the
previous single overall-ratio row, and results are bit-identical. The dist ships `<car>_propC.ini` with the proposal values.

## Model

When either inertia is > 0, two rotors are added, each a 1 kg body on a revolute bearing to the chassis:

- `gbox_out`: gearbox output shaft + propshaft (FWD: final-drive pinion/countershaft), spin axis chassis x.
- `diff_carrier`: diff case + ring gear + spider gears, spin axis chassis y, at the driven axle.

The one row `ω_in/Gr − (ω_L + ω_R)/2 = 0` becomes three ideal-mesh rows in series. All speeds are relative to the
casing (chassis), so the chassis column carries the reaction and Newton 3 holds per row:

| row | equation | in neutral |
|---|---|---|
| gearbox | ω_in / g − ω_out = 0 | inert (compliance 1) |
| final drive | ω_out / fd − ω_carrier = 0 | active |
| open diff | ω_carrier − (ω_L + ω_R)/2 = 0 | active |

Here g·fd = Gr, so the steady-state ratio is unchanged. Physical consequences that emerge without extra code:

- The propshaft and carrier stay geared to the driven wheels in neutral and with the clutch open.
- Their inertia adds I_d + I_go·fd² to the driven axle's mean-speed mode (0.02 + 0.01·4.3² = 0.205 kg·m² for mx5_na).
  It does not affect the left–right differential mode.
- The bearing and mesh reaction torques (including the propshaft's spin-up torque) reach the chassis through the solve.
- The LSD and telemetry semantics are unchanged: the diff row multiplier is still the total axle torque.

## Proposal values (estimates, no public measurement)

| key | proposal | basis |
|---|---|---|
| `mb_diff_inertia` | 0.02 kg·m² | 7-inch ring gear ≈3 kg at gyration radius ≈0.075 m (0.017) + case and spiders (≈0.003). |
| `mb_gb_out_inertia` | 0.01 kg·m² | Steel propshaft tube ≈5 kg at r ≈0.032 m (0.005) + output shaft and the gears that turn with it (≈0.005). |

## Validation (`mbcar_tests`, mx5_na RWD and hatch_fwd FWD)

- Reset kinetic energy rises by exactly ½I_d·ω_c² + ½I_go·(fd·ω_c)²: 490.66 J at 20 m/s on mx5_na, error < 1e-6.
- The ratio chain holds under 2nd-gear drive and after shifting to neutral (relative error 3e-7).
- Newton 3 holds per joint (relative moment 1e-11 m).
- Dissipation never decreases. Momentum residual is 6e-4 N.
- Defaults: all 8 `mbcompare` CSVs are byte-identical to the previous commit.
- Effect on the `mbcompare` suite with the proposal values: changes are below 1 % everywhere (skidpad ay 0.784 → 0.781 g,
  lift-off yaw gain 8.27 → 8.20). The main effect is on driven-axle spin-up (launch, lock-up, wheelspin), not on chassis
  balance.
