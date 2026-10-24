# Countersteer tracking and sustained drift

## 1. Countersteer with zero input (hands off / stick released)

**Metric.**
- β_f = atan2(vy + a·r, vx) is the slip of the front-axle velocity.
- A free, self-aligning steering should point the wheels along that velocity: δ ≈ β_f.
- `t_track` is the time from slide onset until |δ − β_f| < 3° holds for 0.2 s.
- `rms` is the error from onset + 0.3 s onwards.

The scenarios are lift-off and power-on slides at 40, 60 and 85 km/h on the default car (`./build/drivetest counter`).

**Causes found (attribution runs, results/countersteer/attr/; hands-off mean rms over 6 scenarios):**

| variant | mean rms | spins |
|---|---|---|
| base (column_inertia 0.01) | 9.9° | 3 |
| no valve back-drive law | 6.4° | 2 |
| no back-drive + no assist damping | 3.5° | 2 |
| no column friction | 3.6° | 2 |
| **column_inertia 0.002** | **2.6°** | **1** |
| rim inertia halved | 1.3° | 0 |
| no power assist at all | 1.4° | 0 |

- On a free wheel, the open-centre valve amplifies the inertia that the tyres have to accelerate.
- When the tyres back-drive the rack, the torsion bar twists and the valve adds assist in the direction of motion. The rim/column inertia then has to be swung by the SAT through a soft spring (k 115 N·m/rad), so the effective inertia is about 2.65× the bare value.
- The old 0.01 kg·m² for the shaft below the torsion bar (intermediate shaft, valve rotor, pinion) is about 5× a real column of this size. 0.002 kg·m² is realistic, and it removes most of the lag without any added damping.

**Second cause: gating in the input layer.**
- The gamepad "arms" pulled the rim towards the stick (0 when released), and the stick range shrank with speed (25 % at speed, expo 0.5). A released stick therefore *held the wheel straight*.
- Now:
  - The stick is linear to full lock at every speed.
  - Inside the 0.04 deadzone the hands let go (hand torque 0).
  - Leaving the deadzone re-grips over `grip_time` 0.08 s, so stick jitter around centre does not kick the rim and there is no recentre snap.
- The FFB is still the pure column torque.

**Result.** Plot: `results/countersteer/countersteer_before_after.png`. CSV files: `countersteer_before.csv` and `countersteer_after.csv`; the Miata run is `countersteer_after_mx5.csv`.

| scenario | before rms (hands off / stick 0) | after rms | after t_track | spin before → after |
|---|---|---|---|---|
| lift 60 | 4.3 / 27.7° | 3.4° | 0.56 s | 0/1 → 0 |
| lift 85 | 25.9 / 29.2° | 2.8° | 0.57 s | 1/1 → 0 |
| power 40 | 2.8 / 23.0° | 2.4° | 0.32 s | 0 → 0 |
| power 60 | 4.7 / 27.2° | 2.8° | 1.08 s | 1/1 → 0 |
| power 85 | 21.4 / – | 3.5° | 0.38 s | 1 → 1 |

- Spins went from 6 to 1 (power 85: β runs to 83° under full throttle with no driver, which is expected). The gamepad's zero-stick result is now identical to hands off.

**Residuals:**
- t_track is 0.3–0.6 s, and 1.1 s for power 60, against a 0.3 s target. The lag is about 0.04–0.07 s, and most of the remaining error is the first overshoot.
- Hands off, the coupled car-plus-free-steering yaw mode is lightly damped (ζ ≈ 0 to 0.12, 2–7 swings) once the slide is caught. The wheels follow β_f throughout; the oscillation is the car's own weave with a free wheel, which a real car also shows with hands off.
- Nothing was added to damp the yaw mode away.
- fishcheck: `gp_release` (stick released = hands off) is now judged by the hands-off criterion. lift 85 is a known residual (ζ −0.05, β 4°; before, hands off spun with β 88°).

## 2. Sustained power drift (NA Miata, `cars/mx5_na.ini`)

**Test.** `./build/drivetest drift cars/mx5_na.ini results/drift`
- Skidpad R 40 m, entered from a 60 km/h grip corner in 2nd with a 0.35 s handbrake flick (foot off).
- A human-like closed-loop driver then holds β*:
  - Perception and neuromuscular delay of 0.15 s, with first-order anticipation.
  - Finite arms on the rim (k 40 N·m/rad, d 1.5, cap 20 N·m), with full lock available.
- Steering is a yaw-rate law: r_ref = V/R + 1.5(β* − β), and the front slip target is 5° + ∫ + 0.2·(r_ref − r).
- Throttle holds β with a gentle PI, at a foot rate of 0.12 s for full travel. The driver upshifts at 6900 rpm.
- Hold = the longest stretch with β inside 20–35°. β* is 22, 27 or 32°, plus a figure-8 transition.
- `gp_*` runs use the demo's gamepad path (6 Hz thumb, arms k 10).

**Is the drift physically possible?** Hand calculation at β 27°, R 40 m:
- V ≈ 65 km/h. The front axle needs Fy ≈ 1.17 × the rear's, about 75 % of its μFz.
- The rear needs Fx ≈ 2.6 kN and Fy ≈ 3.7 kN against μFz ≈ 4.5 kN, at slip ratio κ ≈ 0.39, about 6000 rpm in 2nd. That is within the B6 engine's 135 N·m.
- So an equilibrium exists.
- With an ideal servo driver (no lag) the sim holds:
  - β 27.04 ± 0.02°, δ 19.4°, rim 283°, front α 5.8° at Fy/μFz 0.92.
  - Rear κ 0.26, throttle 0.86, 58 km/h.

**Why a human could not hold it (before).**
- With the driver frozen open-loop, β leaves ±5° within **0.13 s** of the freeze (`tyre_falloff_speed` 1).
- Divergence ends in a snap: the rear regrips, front slip reverses to −15° at full μ, and yaw rate swings +5 → −108°/s in 0.4 s. This is the "tail slide / spinout" and the tank-slapper.
- The driver's delay margin was about 0.12 s, so a 0.15 s human lost it. The front had nothing left because it was being asked to fight a rear whose grip fell away as it slid.

**Sensitivity** (hold time, three marginal driver settings):

| variant | hold (s) |
|---|---|
| base | 2.3 / 3.2 / 2.0 |
| no sliding falloff | 12.2 / 10.6 / 12.5 |
| falloff_speed 3 | 8.2 / 14.4 / 12.4 |

- Load sensitivity, LSD, yaw inertia, ARBs and peak slip did not help consistently.
- **The sliding-friction falloff is the dominant cause.**

**Cause.** Past full sliding, the tyre's μ is scaled by `level + (1−level)/(1+x²)`, with x = (slip/slip_sliding − 1)/`falloff_speed`.
- With `falloff_speed` 1, μ is already down about 11 % at twice the sliding slip, which is a drifting rear (α ≈ 25–30°).
- That gives the rear a *negative* slope (more slide → less grip → more slide). This Stribeck-like negative damping is what made the rear "step out" faster than any human correction.
- **FALLOFF_SPEED 4** (with a falloff level around 0.85) is a value consistent with published passenger-tyre sliding-friction fits: LuGre/Stribeck fits give a gradual decay over several m/s of slip velocity, not a cliff at the peak.

**Fix.** `cars/mx5_na.ini`: `tyre_falloff_speed` 1 → **4**, front and rear.
- falloff_level 0.85 is kept; it is a value consistent with published passenger-tyre sliding-friction fits.
- Peak μ, peak slip and load sensitivity are unchanged, so grip limits, the skidpad (0.93 g) and all dc_tests are unchanged.
- The built-in default car and `hatch_fwd` are untouched.

**Result.** Plot: `results/drift/drift_mx5.png`. Data: `drift_after.txt` and `drift_before_falloff1.txt`, with per-run CSVs.

| run (arms) | hold before | hold after | after: β | δ (req / max) | front α, Fy/μFz | rear κ, α | throttle | v |
|---|---|---|---|---|---|---|---|---|
| β* 27 | 1.4 s | **11.8 s (whole window)** | 26.6 ± 1.8° | 17.8° (24.9 / 24.1 of 33.6) | 7.1°, 0.96 | 0.28, −28° | 0.98 | 65 km/h |
| β* 32 | 0.9 s | 6.0 s | 26.2 ± 4.6° | 17.1° (max 29.2) | 7.6°, 0.95 | 0.29 | 0.99 (saturated) | 66 km/h |
| β* 22 | 0.8 s | 1.2 s | 24.0 ± 2.1° | 16.7° | 5.8°, 0.90 | 0.23 | 0.92 | 68 km/h |
| transition 27 → −27 | 1.4 s | 11.8 s + reached the other side in 3.5 s, held 2.7 s | | | | | | |

- The open-loop freeze now stays within ±5° for **3.3 s** (before: 0.13 s).
- Required countersteer stays inside lock (max about 25–33° of 33.6°). The front now works at 0.95 of its μFz at 7° slip, so the front axle carries its share.

**Residuals (honest):**
- **β* 32 is power-limited.** Throttle is pinned at 0.99. A 116 hp NA on 1.15-μ street tyres cannot keep more than about 27–30° at R 40 in 2nd, and β droops. This matches the real car, which needs a clutch kick or a lower speed for big angles.
- **β* 22 runs off on speed.** The driver law does not control speed or radius, so the car gains speed, grips up, and drifts out to R 100+ at the limiter.
- **The transition is held for only 2.7 s.** The driver law is tuned for the steady state.
- **Gamepad runs (6 Hz thumb, arms k 10) hold about 1.1–1.5 s.** That input layer is too soft and slow for a 0.15 s-delay drift. This is the control layer that will be dealt with after the physics, as agreed.

### Entry sensitivity and power margin (Oct 2026)

The 11.8 s hold above is **entry-sensitive**. Robustness sweep (arms driver, handbrake pull `DRIFT_HB` 0.25–0.45 s × β* 22/27/32, 15 runs):

| car | passes (hold ≥ 10 s) | mean hold (capped 15 s) | β* 27 hold at HB 0.25 / 0.30 / 0.35 / 0.40 / 0.45 s |
|---|---|---|---|
| mx5_na | 2/15 | 4.4 s | 1.8 / 6.9 / 11.8 / 4.8 / 2.1 |

- Every held run uses throttle 0.93–1.00. A 28° drift at 60 km/h dissipates roughly 55–60 kW (rear lateral sliding ~35 kW, wheelspin ~11 kW, front ~9 kW) against ~69 kW available at 5000 rpm, so the R40 / 60 km/h / 2nd-gear scenario sits at the NA's power limit with no throttle margin.
- At this margin the entry is bistable: a pull that overshoots β drops the car into the regrip branch (rear slip ratio falls, revs rise, the throttle relights, β overshoots again), a pull that lands near β* holds.
- Smaller, slower circles help (`DRIFT_R` / `DRIFT_V0` / `DRIFT_GEAR` env probes in `tools/drivetest.c`): mx5_na passes 4/15 at R30 / 50 km/h and R25 / 45 km/h (mean 6.2 / 7.2 s), and 0/15 at R40 / 70 km/h. Throttle stays ≥ 0.97 in all of them, so the default scenario is kept; the residual is the power margin, not the tyre model.
- `PSET` now accepts up to 16 KB of overrides (was silently truncated at 1 KB).
