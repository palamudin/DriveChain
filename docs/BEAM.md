# Compliant beams (MB core): groundwork

Status: library element plus validation only. No car part uses it yet, so every car result is unchanged
(the `mbcompare` CSVs are byte-identical). This is the base for flexible parts that use the same pivot math as the joints
and bushings: arm, tie-rod and column flex, anti-roll bars, twist-beam axles and chassis torsion.

## Model: rigid finite elements joined by compliant 6-DOF rows

A flexible member is split into N node bodies. Each node carries the mass and rotary inertia of its segment, and
neighbouring nodes are connected by one `MBJ_BEAM` joint at the element midpoint. This is the rigid-finite-element /
pseudo-rigid-body discretisation:
- J. Wittbrodt, I. Adamiec-Wójcik, S. Wojciech, *Dynamics of Flexible Multibody Systems: Rigid Finite Element Method*, Springer 2006.
- L. L. Howell, *Compliant Mechanisms*, Wiley 2001.

It slots into the existing solver without a new formulation. Each element is one more node in the same sparse KKT solve,
so contact, joints, friction and the beam are still one coupled solve.

`mb_beam(W, name, a, b, p, axis, t1)` stores the design relative orientation, as a weld does. It has 6 rows in a frame
fixed in body b:

| row | direction | rate (`mb_beam_section`) |
|---|---|---|
| 0 | axial translation | EA / l |
| 1, 2 | shear along t1, axis × t1 | κGA / l (κ ≤ 0: shear-rigid, Euler–Bernoulli) |
| 3 | torsion | GJ / l |
| 4, 5 | bending about t1, axis × t1 | EI₁ / l, EI₂ / l |

Each row uses the same implicit spring-damper as `MBJ_BUSH`:
- λ = h·f with f = −k(φ + h·Jv⁺) − c·Jv⁺, giving the KKT diagonal −1/(h(hk + c)) and rhs −kφ/(hk + c).
- Damping is Kelvin–Voigt, c = τk.
- The translation rows act at the midpoint of the two anchors, so each element transmits an equal and opposite wrench
  (Newton 3).
- The rotation rows use the relative-orientation error against the design orientation, so large overall rotations of the
  member are exact and only the per-element deflection is linearised.
- A row with k ≤ 0 is rigid, so a beam with no rates is a weld.
- Elastic energy ½Σkφ² is available through `mb_bushing_energy` for the energy audit.

Placing the joints at element midpoints, with node bodies at the nodes and half mass at the free tip, makes the cantilever
error second order. Tip deflection is P·l³/EI·(N³/3 − N/12), which is −1/(4N²) relative to PL³/3EI. Axial, torsion and
shear compliance in series are exact for any N.

## Validation (`make test` → `mb_tests`, section `[beam]`)

| case | result |
|---|---|
| Cantilever 1 m, steel 20 × 40 mm, N = 10: axial F·L/EA | rel err 2e-11 |
| Bending in the stiff plane (P·L³/3EI_y) vs the discrete closed form | 2e-6 (continuum: −0.25 % = −1/(4N²), as predicted) |
| Bending in the weak plane (checks the body-fixed section frame) | 9e-6 (continuum −0.25 %) |
| Torsion T·L/GJ (J by Roark, β(2:1) = 0.229) | 5e-9 |
| Timoshenko, κ = 5/6: adds P·L/(κGA) | 2e-6 vs discrete |
| Stored energy = ½·P·δ (Clapeyron) | 1e-6 |
| Zero rates → rigid weld chain | tip 3e-12 m |
| First bending mode, round bar d = 20 mm, N = 20 | 14.451 Hz vs Euler–Bernoulli 14.472 Hz (−0.14 %) |
| Passivity: undamped free vibration, max per-step energy gain | 0 |
| Free pair with unequal rates: momentum change | 3e-14 N·s; angular 9e-4 relative over 5 s (same as bushing and joint tests) |

## Caveats for the car applications

- **Implicit Euler damps flex modes numerically,** ζ_num ≈ ωh/2: 0.45 % at 14 Hz with h = 0.1 ms, but about 5 % at the
  car's 1 kHz step. Undamped beam energy fell to 43 % after 1 s in the 0.1 ms test.
  - For chassis or column modes that matter for feel, this is the same order as the few-percent structural damping of
    welded steel, so it must be accounted for, not added on top. Either set τ so the physical plus numerical damping is reported, or
    move the steering/structure subsystem to a θ-method (MULTIBODY_PLAN risk 5).
- **Cost:** each element is one 6-row node. A 4-element anti-roll bar adds 24 unknowns, and the bench car is at 65 µs of
  a 250 µs budget.
- **No data yet:** section properties are fine for tubes and bars (geometry + steel). Chassis torsional stiffness and
  arm-flex compliance need K&C or FE data, so the same rule as bushings applies: documented estimates, opt-in, each
  switchable.

## Planned uses (each opt-in, default rigid, validated by driving)

1. **Anti-roll bar as a torsion beam** (2–4 elements plus drop links) instead of the scalar ARB element. Rate from bar
   diameter and lever geometry (GJ/L of a tube, a closed form).
2. **Steering column and intermediate shaft** bending/torsion, alongside the torsion bar already in the chain.
3. **Tie-rod and arm flex:** split a link into 2 nodes. Buckling and bending of a bent tie-rod emerge from the same rows.
4. **Twist-beam rear axle** (hatch_fwd): the cross-member is an open section with low GJ and high EI, which gives the
   characteristic roll-steer.
5. **Chassis torsion:** split the chassis into front and rear bodies joined by one beam element.
