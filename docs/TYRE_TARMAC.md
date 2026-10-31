# TARMAC tyre model (`tyre_model = 1`)

C port of the tyre of **TARMAC Vehicle Dynamics Lab 0.5.0**, written by the project owner (reference JavaScript in
`core/ref/tarmac/`). Implementation: `core/src/dc_tyre_tm.c`. It is a selectable alternative to the default
envelope/brush tyre (`tyre_model = 0`, `core/src/dc_tyre.c`); both run in either core (`solver = 0` legacy,
`solver = 1` multibody) through the same contact struct, so they can be compared head-to-head on one car.

```ini
[tyre_front]            ; and [tyre_rear]
tyre_model = 1          ; 0 envelope (default), 1 TARMAC
tyre_tm_shape = 0       ; 0 faithful TARMAC brush; 1 brush (Fiala) saturation option (see below)
```

## What is ported (faithful)

Contact (`tyre.js`):
- 61 radial rays in the wheel plane: ±65°, 1.5° pitch inside ±33°, 4° outside. They are repeated over 7 lanes across
  the contact width `Wcp = (1.03 - 0.004 AR%) W`.
- The penetrated volume per lane is mapped back to the flat-road deflection with the same volume. This is an exact
  piecewise-linear inversion, so `N = Kz δ` on a flat road and short features are enveloped.
- Radial stiffness from Rhyne (2005) (see below), plus radial damping `cz` (300 N s/m).
- Footprint by membrane equilibrium: the ring displacement is chosen so that the contact area is `N / (κ p)`, with
  κ = 0.8. Pressure is the overlap minus the ring (parabolic along the patch). The load point and normal are
  pressure-weighted means. The half length comes from the pressure second moment (`Var = a²/5`).
- Belt shift (the brush deflection moves the ray origins) and carcass roll (0.6 per unit shift / section height).

Tangential (`physics.js`):
- Brush (carcass) deflection `b`, `F = K b_new + c_d s_new`, with `b_new = (b + h s) / (1 + h|vx|/σ)`. This is
  implicit, and the damping acts on the slip velocity.
- `σ = σ_ref g`, with `g = max(0.05, 2ρ/(1+ρ))` and `ρ = (A a)/(A a)_ref`: the tread brush in series with the carcass.
- Camber slip velocity `|vx| a/(3 R0) sin γ`.
- Load sensitivity `μ = μ0 (F_ref / max(0.15 F_ref, N))^ls`.
- Sliding-friction decay toward `floor·μ` (0.7) with an e-fold of 5 m/s of sliding speed beyond the brush full-slide
  onset `|vx| 3μN/(Kσ)`.
- Plastic return map of `b` onto `|K∘b| ≤ μN`.
- Rolling resistance `crr N Re` (0.012) as a spin torque.
- Rim-flange pinch below `bottom = H - 17.5 mm (ETRTO J flange) - 12 mm`, with `k_rim` 1.2e6 N/m and `c_rim` 1500 N s/m.

## Rhyne vertical stiffness: verification

TARMAC uses `Kz [kgf/mm] = 0.00028 · p[kPa] · sqrt(Wcp[mm] · D[mm]) + 3.45` (Rhyne, T.B., "Development of a Vertical
Stiffness Relationship for Belted Radial Tires", *Tire Science and Technology* 33(3), 2005, pp. 136-155,
doi:10.2346/1.2174340; the form as given in Jazar, *Vehicle Dynamics*, 2017). Checks:

- **Structural term.** The constant 3.45 kgf/mm = 33.84 N/mm is the zero-pressure (structural) term.
- **Independent SI form.** A published SI transcription is `Kz [N/mm] = 2.68 p[MPa] sqrt(W D) + 33.1`, with W and D
  in mm. Converting the TARMAC form gives 2.75 p[MPa] sqrt(W D) + 33.8. The two agree to within 3 %, so the units are
  kgf/mm, kPa and mm.
- **Width.** W is the footprint width, as in Rhyne. Later work replaces it with a geometric estimate; TARMAC's
  `(1.03 - 0.004 AR) W` is such an estimate.
- **mx5_na check** (185/60 R14, 200 kPa): Wcp 144 mm, D 578 mm. This gives Kz = 19.6 kgf/mm = 192 kN/m, a plausible
  value for a passenger tyre of this size. No measured value for this exact tyre was available to compare.

`tyre_tm_kz_mode = 1` replaces Rhyne with the envelope model's pressure-dependent `kz` law, for like-for-like
comparisons.

## Deviations from TARMAC (host differences and fixes)

| Area | TARMAC | Port | Why |
|---|---|---|---|
| Frame / road | y-up, own road sampler | z-up; rays are cast exactly against our bilinear road grid (`dc_rgrid.h`), walked cell by cell with a quadratic root per cell | host |
| Longitudinal | explicit | `Fx` linearised in the wheel spin and solved as an implicit row with the wheel (as the envelope model) | host solver; same steady state |
| Rim-flange pinch | `k_rim (δmax - bottom) + c_rim rate`, added to N along the footprint normal | Separate contact row from a conservative column potential of the flange circle (radius `R0 - bottom`), the same construction as the envelope tyre's rim. It gives the TARMAC value on a flat road. | The deepest-ray form has no potential: its depth and line of action switch from ray to ray at a step riser or a wall's top edge. That pumped energy into wall strikes (crash suite: +155 % KE at a 1.3 m wall, +68 % at 0.3 m). Now 0.57-0.96 %, inside the 1 % criterion. |
| Lanes inside a wall | the ray from inside the solid reads full depth | A ray whose origin is already inside the solid (a face taller than the wheel centre beside that lane) carries no load and no volume | It has no depth gradient, and its top-surface normal produced a full-radius upward pinch (a car launched off a 1.3 m wall). A sidewall pressed against a wall is outside a tread-column model and is left to the flange row and to body contact. |
| Mz | none | Self-aligning moment and pneumatic trail from the footprint shear distribution (below) | required |
| Turn slip | none | Tread-twist (parking) moment: torsional brush of the footprint, relaxing over the half length | required for parking / turn-in steering torque |
| Damage, thermal, wear, gas temperature | modelled | not ported (constant grip and pressure; pressure comes from the host's own model) | out of scope; host has its own pressure state |
| Rolling-resistance sign at standstill | `sign(Ω)` | `tanh(Ω Re / 0.5 m/s)` | no chatter at rest |
| Sliding direction (`tyre_tm_slide_dir = 1`, option, default 0) | radial return of the deflection onto the friction circle | isotropic Coulomb return (same as the envelope model) | a sliding tread's force must oppose its slip velocity (below) |

### Sliding direction option (`tyre_tm_slide_dir = 1`, not in TARMAC; default 0)

TARMAC caps the brush deflection by scaling it radially onto `|K b| = μN`. The trial deflection keeps its elastic
ratio `Kx σx sx : Ky σy sy`, so with the carcass stiffer longitudinally the force of a fully sliding tyre leans toward
the wheel plane instead of opposing the slip velocity. Rubber sliding friction is isotropic to first order (Coulomb;
Pacejka, *Tire and Vehicle Dynamics*, 3rd ed., ch. 3 brush model: in the sliding region the shear stress opposes the local sliding
velocity; Schallamach 1963): a locked wheel's force is anti-parallel to its contact-patch slip velocity whatever its steer
angle, which is why a car with locked front wheels does not respond to the steering.

Measured on the mx5_na front, full lock (34° road wheel), 20 km/h, brake locking the fronts:

| | force vs slip velocity | yaw while sliding (steer held left) |
|---|---|---|
| TARMAC radial (0) | 168-172° (8-12° toward the wheel plane) | 72 → −9 deg/s: the nose turns away from the steer |
| isotropic (1) | 179.6-180° | 71 → 0 deg/s: rotation stops, the car carries on along its momentum |

The option returns the trial deflection along the Coulomb flow (backward Euler of `db/dt = s - |vx| b/σ - ν K b`,
the same `dc_tyre_carcass_return` as the envelope model) in phase A and at commit. Elastic (non-sliding) states are
untouched: the return acts only when the trial exceeds `μN`, so it changes a tyre at the friction limit and in full slide only.
`make locktest` checks it (force within 4° of anti-slip on locked fronts, no yaw against the held steer). The demo
exposes it as `car.tyre_slide_dir` (proposal propL, off until approved).

### Mz and pneumatic trail (addition)

TARMAC returns no aligning moment. The port adds it from brush theory applied to the actual footprint cells:
- In each lane, the tread deflection grows linearly from the leading edge (front when rolling forward, rear in
  reverse).
- The shear in a cell is `min(c_p s ξ, μ p)` along the force direction. `s` is solved by bisection so that the shear
  integral equals the brush resultant `|F|`: the carcass sets the force, the footprint sets where it acts.
- `Mz = Σ (x τ_y - y τ_x)` about the load point.
- In the linear range with parabolic pressure this gives trail `a/3`. The trail falls to 0 at full sliding.
- `c_p` is set so the tread brush equals the carcass at the reference load (in series).
- Rig, mx5_na front at 2575 N: trail at 0.5° is 15.3 mm, and `|Mz|` peaks at 13.9 N·m at 1.25°. The envelope model
  gives 23.6 mm and 19.7 N·m at 2.25°. The lower TARMAC peak follows from its earlier force saturation.

### Brush saturation option (`tyre_tm_shape = 1`, not in TARMAC)

TARMAC's deflection obeys `db/dt = s - |vx| b/σ`. The tread absorbs slip at the linear rate `b/σ = F/C` right up to
the friction limit. The force curve is therefore linear to `μN` and peaks at `α ≈ μN/C` (≈ 2.5° on the mx5_na front),
much earlier than measured passenger tyres (≈ 5-8°).

With a brush tread (Fiala 1954; Pacejka, *Tire and Vehicle Dynamics*, 3rd ed., §3.2), the tread slip that carries
force F is `F/C · φ(f)`, with `f = F/(μN)` and `φ = 3(1 - (1-f)^(1/3))/f`. This is the exact inverse of the Fiala
curve: φ = 1 at small force and 3 at full sliding. The option applies φ to the transport term (3 fixed-point passes).
The curve then rounds over and peaks at the brush full-slide slip `3μN/C` (3.75° at the reference load, 7.25° at
twice the load). Cost: +40 % per wheel.

## Validation (mx5_na, `results/tyre_compare/`, `make tyrecompare`)

Steady state at 15 m/s, front tyre, `F_ref` 2575 N:

| | envelope | TARMAC | TARMAC shape=1 |
|---|---|---|---|
| C_α at F_ref (N/rad) | 44.2k | 54.9k | 52.1k |
| peak μ_y / slip at F_ref | 0.914 / 10.25° | 0.914 / 2.5° | 0.914 / 3.75° |
| peak slip at 2 F_ref | 7.0° | 2.75° | 7.25° |
| peak μ_y at ½, 1½, 2 F_ref | 1.086, 0.826, 0.769 | 1.087, 0.826, 0.769 | same |
| \|Mz\| peak / trail at 0.5° | 19.7 N·m / 23.6 mm | 13.9 N·m / 15.3 mm | 13.9 N·m / 15.5 mm |
| camber −3°: Fy(α=0) | −282 N | −203 N | −198 N |
| C_κ / peak κ | 45.4k / 0.20 | 55.9k / 0.05 | 52.5k / 0.07 |
| relaxation length (63 %, 2° step) | 0.263 m | 0.323 m | 0.210 m |
| cost per wheel (phase A+B) | 21.6 µs | 26.3 µs | 37.1 µs |

Whole-car cost (`make test`, `results/perf.csv`, one core, 2 kHz substep):
- envelope: 101.5 µs per substep on a flat road (4.9x real time) and 153 µs on rough terrain (3.3x).
- TARMAC: 115.9 µs on a flat road (4.3x real time, inside the 4x budget) and 212.8 µs on rough terrain (2.3x). On
  rough terrain every lane walks the road grid.

Peak μ and load sensitivity are identical by construction (TARMAC μ0/ls are derived from the envelope parameters).

Vehicle tests (`mbcompare`, multibody core, mx5_na):

| | envelope | TARMAC | TARMAC shape=1 |
|---|---|---|---|
| skidpad ay | 0.78 g | 0.92 g | 0.88 g |
| understeer gradient (° sw/g) | 7.3 | 3.4 | 5.6 |
| lift-off yaw gain | 6.4 % | 0 % | 1.8 % |
| slalom ay | 0.65 g | 0.77 g | 0.69 g |
| hands-off countersteer, max body slip | 28° (recovers) | 87° (spins) | 87° (spins) |
| drift hold (`drivetest drift`, arms) | 7.7 s | 0.3 s | 0.3 s |

Kerbs, walls, drops (`tests/crash_suite.py`): 26/26 for both TARMAC variants in both cores.
- Peak tyre load on a 135 mm kerb head-on: 15.2-16.2 kN at 25 km/h, 22.4-26.5 kN at 40 km/h.
- Peak tyre load on a 50 mm kerb at 5°/60 km/h: 4.5-4.6 kN.
- Peak tyre load on drops of 0.5/1/1.5 m: 8.6-16.9 kN, with KE gain below mgh in every case.
- Wall-strike energy rise (24 cases per core): at most 0.96 % of KE0 (legacy core) and 0.57 % (multibody core).

Notes on the vehicle results:
- **Hands-off and drift.** The spin is caused by TARMAC's sliding-friction decay, not by the force curve. With
  `tyre_tm_slide_floor = 1` (no decay), hands-off recovers like the envelope tyre: max slip 27° (TARMAC) and 23°
  (shape=1). The drift hold rises to 1.1 s and 2.1 s.
  - The decay is a drop to 0.81 μ after 5 m/s of sliding, beginning at the full-slide onset. That is at or beyond the
    steep end of wet-pavement skid-number speed gradients (Penn State model, `SN(v) = SN0 exp(-PNG v)`, with PNG of
    order 0.5-1.5 %/(km/h); Henry, NCHRP Synthesis 291, 2000).
  - Dry asphalt shows less.
  - Combined with the 2.5° peak, the rear lets go abruptly once it saturates. The countersteer is emergent and
    correct in direction, but it runs to lock without recovering.
  - The drift driver was tuned against the envelope tyre, so drift-hold numbers are indicative.
- **Lift-off yaw gain.** The comparison is at equal ay, so the TARMAC car is at a lower fraction of its limit. This
  is not like-for-like.
- **Telemetry slip angle (fixed).** Reported `slip_angle`/`slip_ratio` used to be the deflection state over the
  relaxation length, which saturates at μFz/Cα in a slide (≈ 0.02 rad for TARMAC, a few degrees for the envelope). For
  all tyre models they are now the relaxed *kinematic* slip, dα/dt = |Vx|/σ·(α_ss − α), integrated implicitly
  (Pacejka 2012, §7.2.1). Telemetry only; it does not feed the force. `tests/reset_tests.c` checks reported against
  kinematic slip in a handbrake slide (gain 0.8-1.1).
- **Reset state.** `dc_reset` re-initialises every tyre state (brush/LuGre deflections, relaxation and telemetry
  states, thermal). `make resettest` verifies that a reset after abuse and a teleport is bit-identical to a fresh
  car, for envelope, TARMAC, brush and brush+LuGre on both cores.
- **Closed-loop laps on a mesh circuit.** An earlier "TARMAC spins on power after a reset" finding was traced to the
  test, not the tyre or its state. The harness reset the car on the grid slot, 4.7 m off the racing line, so the path
  driver lunged back to the line with large rim angles at full throttle. Started on the line, TARMAC (both shapes
  tested) laps cleanly with no spin. From the off-line start the envelope tyre recovers and TARMAC spins, which
  matches the hands-off result above: TARMAC has a higher Cα/Fz (≈ 22 vs 14 /rad) and so a larger yaw response,
  plus slide decay. The off-line start is kept as a stress case (`--start spot`).
  Re-checked on the current build: unchanged. The opt-in propD set ([TYRE_DATA.md](TYRE_DATA.md)) laps cleanly from
  both starts and raises the hands-off recovery limit from below 0.3 to about 0.47 throttle.
- **Very large single-wheel loads in a spin.** Spikes of 40-90 kN were traced to a wheel sliding sideways into a
  raised concrete kerb with a near-vertical lip of about 22 cm at the track edge. The tyre bottoms and the rim
  contact carries the load, so this is a real rim strike. Wheel and rim damage is not modelled.
- **"Fell through the world".** The car had left the track during a spin. The lap harness's windowed nearest-point
  search then lost the line, and the path driver drove straight at full throttle until the car left the mesh,
  which has no geometry beyond its bounding area. This was not a contact failure. The harness now re-acquires the
  line globally and stops when the car leaves the map.

## Recommendation

**Keep the envelope tyre as the default. Keep TARMAC selectable as ported (`tyre_tm_shape = 0`) for faithfulness.
Prefer `tyre_tm_shape = 1` with a gentler slide decay (e.g. `tyre_tm_slide_floor` 0.85-1.0) when using TARMAC for
driving.**

Reasons:
- The envelope tyre has the measured-like slip-angle peak.
- It has the higher pneumatic trail (steering feel).
- It is the only one that recovers hands-off and holds a drift on the current driver tuning.
- It is ~20 % cheaper per wheel.

TARMAC's strengths, which the port keeps:
- the physically sized footprint (membrane equilibrium)
- the Rhyne stiffness from size and pressure alone
- belt shift and carcass roll

These remain available and make it a good second model for comparison.
