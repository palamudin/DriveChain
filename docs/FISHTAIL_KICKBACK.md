# Fishtail, relaxation length and kickback

The car is the public NA Miata reference (cars/mx5_na.ini) unless stated otherwise. "AC car" means the private, AC-calibrated Miata. Its numbers are quoted here, but its files stay in private/.

## 1. Fishtail ("erratic oversteer correction, throws left-right even at low speed")
**Harness:** `drivetest fishtail|fishcheck`.
- **Scenarios:**
  - lift-off in a corner at 45, 60 and 85 km/h
  - power oversteer at 40 and 60 km/h
- **Drivers:**
  - `hands_off`: free wheel
  - `locked`
  - `ideal`
  - `arms`
  - gamepad `gp_hold`, `gp_release` (stick released at the slide, the reported case) and `gp_human` (delayed counter-steer)
- **Metrics:**
  - yaw-rate swings
  - log-decrement ζ
  - settle time (|r| < 3 deg/s and |β| < 1°)
  - peak r and β, rim rate, column torque, spin

### Root causes
1. **The legacy gamepad mapping.**
   - Inside the deadzone, the stick switched the rim to *hands off*. Outside it, the stick commanded the rim angle directly.
   - Releasing the stick mid-slide therefore left a free, lightly damped wheel. The self-aligning torque spun it to opposite lock (rim rates of 800–1300 deg/s), the car snapped back the other way, and the cycle repeated.
   - This is the "fighting" the owner felt: the pad and the physical self-steer took turns owning the rim.
2. **The open-centre valve boosts the wheel-inertia torque.**
   - When the tyres back-drive the rack, the valve twists and adds assist in the *same* direction as the motion. This acts as a negative centring stiffness/damping on a free wheel. The manual rack is visibly calmer: hands-off peak β 23° vs 40° with PAS.
3. Relaxation length and tyre lag are **not** the cause. Tripling ky (σ ≈ 0.08 m) left the swings unchanged (see §2).

### Physical fixes (65eb263, retuned in f2cb3f0)
- **Gamepad "arms" layer:**
  - The stick sets where the **hands** want the rim. Finite-strength arms (PD: k 10 N·m/rad, d 1, cap 20 N·m) push the rim there. The SAT can still move the rim through a relaxed grip, as with a real driver, but the wheel is never left free.
  - Superseded (countersteer work, see docs/DRIFT.md): the stick maps **linearly to full lock at every speed** (no speed range, no expo), and inside the 0.04 deadzone the hands **let go** (hand torque 0), so a released stick is hands off and the SAT steers the rim freely. Grip is re-established over `grip_time` 0.08 s when the stick leaves the deadzone, so the deadzone jitter is isolated without a recentre snap.
  - `[gamepad] mode = raw` keeps the legacy mapping. The FFB is untouched: it is the pure column torque.
- **Valve orifice back-drive law:**
  - When the rack is back-driven against the valve, the displaced oil must pass the metering orifices (dP ∝ Q²). The incremental resistance is 2·dP/Q, giving a rack damping of `c = g·2|F|A/Qe`, applied linearly-implicitly and capped by the relief valve at 1.25·pmax·A.
  - The gain is g = 0.5 because the back-driven flow splits across the **two parallel legs** of the open-centre bridge.
  - g = 1 (single orifice) over-damped the free-wheel self-steer: the dc_tests hands-off recovery spun, with peak β 89°.
  - Sweep of g against the hands-off recovery test: 0 → β 54° (recovers), 0.25 → 71° (recovers), 0.4 → 65° (recovers), **0.5 → 40° (recovers)**, 0.6 → 79° (recovers), 0.75 → spins, 1.0 → spins.
  - The fishtail metrics are identical at g = 0.5 and 1.0, so the gamepad improvement comes from the arms layer.

### Before → after (fishcheck, `gp_release` = stick released mid-slide)
| scenario | RAW (before): swings / ζ / settle | arms (after): swings / ζ / settle |
|---|---|---|
| lift 45 km/h R20 | 1 / 0.29 / 0.75 s | **0 / 1.00 / 0.29 s** |
| lift 60 km/h R32 | 3 / 0.16 / 1.46 s | **0 / 1.00 / 0.49 s** |
| lift 85 km/h R65 | 7 / −0.13 / 6.71 s (β 38°) | **1 / 0.42 / 0.63 s** (β 2.8°) |
| power 40 km/h R20 | 1 / 0.48 / 0.64 s | **0 / 1.00 / 0.27 s** |
| power 60 km/h R40 | 2 / 0.22 / 0.92 s | **0 / 1.00 / 0.26 s** |

- **At 1.2× speed:** RAW lift at 72 km/h gives 7 swings with ζ −0.10, and at 102 km/h it SPINS. With the arms layer these become 0 swings (settle 0.72 s) and 1 swing (ζ 0.37, settle 0.87 s).
- **AC car:** fishcheck also PASSes.
- **Known residual:** lift-off at 85 km/h with completely free hands (`hands_off`) still spins (5 swings, β 88°). A free PAS wheel at that speed is genuinely unstable; the test flags it as a known residual rather than hiding it.
- **Files:**
  - results/fishtail/check/fishcheck.txt (the `make fishtest` regression)
  - private/fish_after_{raw,arms}_vs{1.0,1.2}.txt (full before/after tables; private run directory)

## 2. Relaxation length: kept at σ ≈ 0.23 m (not tightened to 0.067 m)
- σ = Cα/ky comes from the brush model with carcass lateral stiffness. For the 185/60R14 that is ≈ 0.23 m.
- **Why not 0.067 m:**
  - 0.067 m is about the half contact-patch length, the rigid-carcass limit.
  - Measured passenger tyres sit at 0.2–0.6 m (Pacejka, *Tyre and Vehicle Dynamics*, ch. 7).
  - Setting ky×3 (σ ≈ 0.08 m) did **not** remove the fishtail, because the fault was in the steering, not the tyre lag.
  - A shorter σ also stiffens the 2 kHz numerics with no feel benefit.
- Plot: results/00_tyre_relaxation.png.

## 3. Kickback and undulations ("bumps overpower the wheel, steering re-centres but the heading changes")
**Harness:** `drivetest kickback <car> <out>`, then `python3 tools/plot_kickback.py <out>`.
- **Road features:**
  - undulations (25 m ±90 mm, 9 m ±35 mm, 3 m twist)
  - single-side bumps and potholes (left and right)
  - a crossfall change
  - ruts
  - a bumpy corner at 70 km/h
- **Drivers:**
  - finite-strength arms at 15 / 20 / 25 N·m with 250 / 180 / 150 ms reaction delay
  - gamepad arms
  - the brackets: `hands_off` and `locked`
- **Per event:**
  - peak column torque
  - rim excursion and rate
  - hand torque and saturation
  - heading change (peak and at re-centre)
  - lateral offset at the end
  - recovery time
  - a source breakdown of kingpin moments (peaks and impulses): Fy·trail, Fx·scrub radius, tyre Mz, Fz·jacking and wheel gyroscopic torque

### Source breakdown (kingpin moments, single-wheel bump/pothole)
| source | moment |
|---|---|
| **Fx × scrub radius** (bump longitudinal impact) | 46–101 N·m |
| Fy × trail | 14–36 N·m |
| tyre Mz | 14–30 N·m |
| Fz jacking | < 1 N·m |
| wheel gyroscopic torque | 60–114 N·m peaks, lasting only ~10–20 ms |

- The left and right wheels largely cancel through the rack. The column sees the net torque, filtered by the rack/column inertia and the torsion bar.

### Results, PAS car (default)
| event | column pk | arms20 rim / heading / recover | hands-off rim / heading / Δlat | locked heading / Δlat |
|---|---|---|---|---|
| bump L 60 | 2.9 N·m | 1.0° / 0.22° / 0.11 s | 5.2° / 0.52° / 0.54 m | 0.23° / 0.28 m |
| bump L 100 | 3.1 N·m | 1.0° / 0.28° / 0.16 s | 3.1° / 0.43° / 0.36 m | 0.41° / 0.42 m |
| pothole L 60 | 3.1 N·m | 1.4° / 0.25° / 0.09 s | 6.4° / 0.79° / 0.82 m | 0.21° / 0.25 m |
| pothole L 100 | 3.2 N·m | 1.2° / 0.20° / 0.12 s | 4.8° / 0.45° / 0.43 m | 0.18° / 0.24 m |
| crossfall 60 | 0.5 N·m | 1.7° / 0.41° | 2.0° / −1.26° / −1.33 m | −0.22° / −0.15 m |
| undulation 25 m, 100 | 0.06 N·m | 0.13° / 0.08° | 0.12° / 0.04° | 0.16° / 0.23 m |
| undulation 9 m, 100 | 0.23 N·m | 0.06° / 0.05° | 0.18° / 0.01° | 0.05° |
| bumpy corner 70 | 0.6 N·m | 5.6° / 1.7° / tracks within 0.08 m | leaves the road (−56°, −120 m) | 7.5°, runs 12 m wide |

**Why the heading changes even though the wheel re-centres.** During the 50–100 ms the front wheels are toed by the kick, the car yaws. The integral of that yaw (0.2–0.4°) stays when the wheel returns. Even a *locked* wheel shows it (0.2–0.4°), because the source is compliance steer, bump steer and Fx asymmetry at the contact patch, not the rim. A driver corrects it within ~0.1–0.2 s. With the arms model the lateral offset stays below 2 cm.

### NA Miata sanity check (manual rack, `assist = 0`: many NAs were sold without power steering)
| event | column pk (arms20 / locked) | arms20 rim | hands-off rim | heading |
|---|---|---|---|---|
| bump L 60 | 9.6 / 11.5 N·m | 1.9° | 8.5° | 0.23° |
| bump L 100 | 15.0 / 17.2 N·m | 1.9° | 5.6° | 0.34° |
| pothole L 60 | 16.9 / 18.7 N·m | 3.8° | 9.6° | 0.28° |
| pothole L 100 | 15.1 / 16.4 N·m | 3.0° | 5.6° | 0.21° |

- The AC car (manual) gives bump 60: 11.1 / 13.1 N·m and pothole 60: 17.4 / 19.4 N·m.
- These sit in the expected 10–20 N·m range for an unassisted NA over a sharp single-wheel edge.
- The 15–25 N·m arms hold the rim to 2–4°, and recovery is about one reaction delay.
- With PAS, the torsion bar and valve cap what reaches the column at ~3 N·m. The rack-side kick is absorbed by the assist, which is the real PAS behaviour. The FFB is therefore much calmer with assist on (H toggles it).

### Plots
In results/kickback/ (PAS) and results/kickback/default_manual/ (manual rack):
- `kick_overview.png`
- `kick_traces_<event>.png`: rim, column torque, heading and lateral offset for each driver
- `kick_attrib_<event>.png`: the source breakdown

Summaries are in `*/kickback_summary.csv`. The AC car versions are in private/kick_ac{,_manual}/.

## 4. Residuals
- Lift-off at 85 km/h with a *free* PAS wheel spins. The hands-off recovery test (90° step) is chaotic near the limit: peak β varies from 40° to 80° with small parameter changes.
- PAS caps the column kickback at ~3 N·m, which is physical but may feel numb. Use the manual rack (H) for a raw NA feel.
- Gyroscopic kingpin spikes of 60–114 N·m last 10–20 ms. They are mostly cancelled by the column inertia, so their impulse is small.
- Near the limit (56–60 km/h on the skidpad) the plug-in driver hunts by 23–25°. The 2 m step overshoots by 0.9 m, and the delayed-human driver shows mild pilot-induced oscillation (PIO).
