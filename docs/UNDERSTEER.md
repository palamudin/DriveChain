# Understeer gradient: measurement and reference values

## How `mbcompare` measures it (constant-radius method, SAE J266)

- Path-held 40 m circle, speed ramped at 0.25 m/s². The radius is fixed, so the Ackermann angle L/R is constant and the
  slope of steer angle against ay is the understeer gradient directly.
- Least-squares fit over 0.2–0.45 g (linear range), skipping the first 3 s (the run starts at 8 m/s, which is already
  0.16 g). Samples with path error > 1 m are excluded.
- Reported values:
  - `usg_sw_fit`: steering-wheel angle gradient, deg SW/g (J266 "steering-wheel angle gradient").
  - `usg_K_j266`: `usg_sw_fit / steer_ratio`, deg/g. This is the reference steer angle gradient, i.e. the understeer
    gradient K that road-car data quote. It includes all compliance between the rim and the tyre.
  - `usg_rw_fit`: gradient of the actual mean front wheel angle (knuckle pose relative to the chassis). Front
    compliance steer moves the wheel relative to the rack, so the driver has to supply it at the rim and it never
    shows up in the actual wheel angle. This value therefore holds only the tyre, rear-steer and roll-steer parts.
    It is a diagnostic, not K.
- The old `understeer_grad_sw` (kept, labelled OLD) took two single samples at the first crossings of 3 and 7 m/s².
  - The path driver hunts by about ±5–15° SW around the mean, so those samples are mostly noise.
  - 7 m/s² is 0.71 g, which is non-linear range.
  - It was a steering-wheel number. Quoting it next to road-car K values (road-wheel deg/g) overstated it by the
    steering ratio (×15).

## Results (mx5_na, steer_ratio 15)

| car / core | usg_sw_fit (deg SW/g) | K = usg_K_j266 (deg/g) | actual-wheel gradient (deg/g) |
|---|---|---|---|
| rigid, MB core (default) | 5.5 | 0.36 | 0.28 |
| rigid, legacy core | 3.6 | 0.24 | 0.21 |
| rigid, MB, TARMAC tyre | 5.1 | 0.34 | 0.26 |
| bushed (`_propB`), MB | 28.7 | 1.91 | 0.63 |
| rigid + rack mount (`_propR`), MB | 7.0 | 0.47 | 0.27 |
| bushed + rack mount (`_propB` + `_propR` keys), MB | 36.7 | 2.45 | 0.99 |

- In the bushed car, the gap between K (1.91) and the actual-wheel gradient (0.63) is front compliance steer from the arm
  bushes. The overall SW/wheel ratio rises with load (16.7 at 0.1 g to 18.2 at 0.7 g), whereas the rigid car stays flat
  at about 15.5. The bushes do not simply shift the steering angle by a constant amount: the compliance steer grows with
  load, as real compliance steer does.

## Reference values

No measured NA MX-5 understeer gradient has been found in public sources. Sourced ranges for road cars:

- Most cars of the 2000s: 2–3 deg/g, with sporty cars at the low end ("low 2") and "virtually no vehicles below 1 deg/g",
  from many hundreds of OEM measurements. Eng-Tips thread "Understeer gradient of modern passenger cars"
  (contributors cibachrome and G. Locock), https://www.eng-tips.com/threads/understeer-gradient-of-modern-passenger-cars.190555/
- 2007 Acura TSX, stock, measured: K = 3.6 deg/g. K. Tse, "Cornering Compliances of a Typical Road Vehicle", 2019,
  https://kktse.github.io/jekyll/update/2019/06/01/typical-cornering-compliances.html
- Instrumented passenger car, constant-steer and constant-speed tests: 2.4–3.0 deg/g (constant steer), 3.3–5.1 deg/g (constant speed, falling with speed). "Evaluation
  of Vehicle Understeer Gradient Definitions", MS thesis, Ohio State University, 2009 (osu1259700998).
- Tyre and weight-distribution term only (bicycle model, no compliance), 162 vehicles: mean 0.81 deg/g, range −1.0 to
  2.5. EDC, HVE-WP-2019-1. The compliance terms (steering system, bushes, roll steer) supply the rest. See also
  Gillespie, *Fundamentals of Vehicle Dynamics*, ch. 6 (understeer budget).

## Assessment

- **Rigid model.** The rigid MB and legacy cars, at K ≈ 0.25–0.35 deg/g, are too neutral for a road car. This is
  consistent with the HVE tyre-only figure, because a rigid model has no steering-system or bush compliance.
- **Bushed proposal.** The `_propB` bush estimates give 1.9 deg/g, inside the sourced range for a sporty car. The
  bushes are not too soft by this measure.
- **Rack mount.** The `_propR` proposal (rack housing on a compliant mount, 8 kN/mm along the rack axis, docs/BUSHINGS.md)
  adds about 0.1 deg/g on the rigid car (0.36 to 0.47) and about 0.5 deg/g with the bushes (1.91 to 2.45). At 0.49 g
  the housing moves 0.13 mm under about 1 kN rack force. Sensitivity (skidpad only): 4 / 8 / 16 kN/mm give rigid
  K 0.55 / 0.47 / 0.41 and bushed 2.39 / 2.34 / 2.23 deg/g. Bushes plus rack mount land in the middle of the sourced
  2–3 deg/g band. Column compliance (other than the torsion bar) is still not modelled.
- The NA-specific target remains an estimate: low 2s deg/g for a light, near-50/50, sporty RWD car.
