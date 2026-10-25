# Wheel spin inertia: provenance

`tyre_spin_inertia` is the inertia about the axle of everything that spins with the wheel: tyre, rim, brake disc,
hub and the rotating bearing race. Driveline inertia behind the hub (half-shafts, differential, propshaft) is *not*
included. The engine and gearbox are separate DOFs (`engine_inertia`, `gearbox_inertia`).

## Sources

* A. Ubysz, *Problems of rotational mass in passenger vehicles*, Transport Problems (Silesian University of
  Technology), Table 3. It gives a superposition/CAD calculation for four tyre/rim sets, checked against a measured chart:
  165/70R13 0.39 (chart 0.435), 175/65R14 0.60 (0.56), 175/70R14 0.71 (0.69), 195/60R15 0.774 (0.77) kg m².
  70–75 % of the tyre's inertia sits in the tread band, and the tyre provides 72–77 % of the wheel's inertia.
* SAE 900760, *Moments of Inertia of Mounted and Unmounted Passenger Car and Motorcycle Tires*: torsional
  pendulum measurements of 44 tyres, with a linear correlation of inertia against tyre weight and diameter.

## Component estimate (`python3 tools/wheel_inertia.py`)

Tyre and rim are scaled from Ubysz's 195/60R15 row by m·r². The disc is an annulus. Masses are documented
estimates: 185/60R14 tyre 7.5 kg, 14×5.5 cast alloy 6.0 kg, 235 mm vented / 231 mm solid discs 4.0 / 2.8 kg; 205/55R16
tyre 8.5 kg, 16×6.5 alloy 8.0 kg, 280 / 264 mm discs 6.0 / 4.0 kg.

| wheel | tyre | rim | disc | hub | total (kg m²) | in car file |
|---|---|---|---|---|---|---|
| mx5_na front, 185/60R14 | 0.51 | 0.12 | 0.036 | 0.005 | 0.67 | 0.70 |
| mx5_na rear, 185/60R14 | 0.51 | 0.12 | 0.025 | 0.005 | 0.66 | 0.70 |
| hatch_fwd front, 205/55R16 | 0.69 | 0.21 | 0.076 | 0.006 | 0.98 | 0.70 |
| hatch_fwd rear, 205/55R16 | 0.69 | 0.21 | 0.046 | 0.006 | 0.95 | 0.70 |
| check: 165/70R13 steel | 0.35 | 0.11 | – | – | 0.45 | Ubysz 0.39 / 0.435 |

Uncertainty is about ±0.05 kg m². The 13-inch check shows that the m·r² scaling reads 5–15 % high for small tyres.

* **mx5_na: 0.70 is consistent.** It sits at the top of the 0.62–0.70 band and between Ubysz's 175/65R14 and
  175/70R14 sets. No change. (For scale, a 13-inch steel wheel with a 165–175 section tyre is 0.4–0.45 kg m², not 0.6.
  The ~0.6 class belongs to 14-inch sets.)
* **hatch_fwd: 0.70 is low for a 16-inch wheel.** The component estimate is 0.95–0.98, and Ubysz's chart puts rd 0.31 m
  above 0.8. Proposal (not applied): `tyre_spin_inertia = 0.95`.
  Its `tyre_rim_radius = 0.19` is also the 14-inch flange radius. The 16-inch value would be about 0.22 (bead seat
  0.203 + flange). Proposal (not applied).
* Not modelled: rear driveline inertia behind the hub (half-shafts, differential, propshaft). Reflected through
  the final drive it is roughly 0.1 kg m² per driven wheel. The car files have only engine and gearbox inertia.
  Candidate for a later driveline pass.
