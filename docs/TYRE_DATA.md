# Tyre data study: 185/60R14-class passenger tyre vs our two tyre models

Goal: pin the lateral peak slip angle, the post-peak decay vs sliding speed on dry tarmac, and the pneumatic trail of
a 1990s 185/60R14-class passenger tyre from measured data. Document where the envelope tyre (`tyre_model = 0`) and the
TARMAC tyre (`tyre_model = 1`) deviate, and propose corrected parameters. **No defaults are changed by this study.**
The proposals are opt-in ini sets (below) for the project owner to accept or reject.

Tool: `tools/tyre_reference.py <tyre_compare outdir> [png] [csv]` evaluates the reference curves and compares them with
the `build/tyre_compare` rig: single front wheel, flat road, 15 m/s, steady state, loads 1288/2575/3863/5150 N =
0.5/1/1.5/2 × mx5_na front static load. Results: `results/tyre_ref/`.

## Reference data

1. **Measured Magic Formula fit, 205/60R15 passenger tyre** (MF 6.1, Fz0 = 4000 N, 200 kPa, measured at 16.7 m/s).
   This is the public MF-Tyre example parameter set, published with Pacejka, *Tire and Vehicle Dynamics* (3rd ed.,
   2012, App. 3) and redistributed in open tools, e.g. `sample.tir` in the BSD-licensed Modelica ExternData library.
   Only pure-slip, zero-camber, nominal-pressure coefficients are used (PCY1, PDY1-2, PEY1-3, PKY1/2/4, QBZ1-3, QCZ1,
   QDZ1-2, QEZ1-4; PCX1, PDX1-2, PEX1-4, PKX1-3). Ply-steer and conicity shifts are dropped so the curves are odd.
   It is the closest measured, publicly available fit to our class: 20 mm wider, one inch larger rim, same aspect ratio
   and era of construction.
2. **Burckhardt dry-asphalt μ(slip) fit** (Burckhardt 1993, *Fahrwerktechnik: Radschlupf-Regelsysteme*, Vogel; as
   tabulated in Kiencke & Nielsen, *Automotive Control Systems*): μ(s) = c1(1 − e^(−c2 s)) − c3 s with
   c = (1.2801, 23.99, 0.52). It is the standard locked-wheel reference: μ_peak 1.17 at s = 0.17, and μ(s = 1)/μ_peak = 0.65
   (full sliding at vehicle speed).
3. **Stribeck form of sliding friction**: μ(v) = μ_k + (μ_s − μ_k) exp(−(v/v_s)^δ) with δ ≈ 2 (Armstrong-Hélouvry et al.
   1994; Canudas de Wit et al. 1995, the LuGre paper). This is the shape needed to reproduce references 1 and 2 together (below).
4. **Pneumatic trail**: Pacejka, *Tire and Vehicle Dynamics*, §3.2 (brush: a/3 for parabolic pressure, i.e. 1/6 of
   the contact length). Measured trails are somewhat larger because the real pressure distribution is shifted rearward.
5. **Vehicle level**: Car and Driver measured 0.82 g skidpad on 185/60HR14 tyres (see `RESEARCH_FINDINGS.md`).
6. **KIT tyre measurement dataset** (Karlsruhe Institute of Technology, CC BY-NC-SA 4.0, DOI
   [10.35097/p0rr2jc5wmf0drf8](https://doi.org/10.35097/p0rr2jc5wmf0drf8)). It is cited as the next source for a
   direct passenger-tyre fit. Its raw data is not redistributed here and no numbers from it are used in this study.

**Class scaling (estimate).** A 185/60R14 at the same pressure and load has a ~10 % narrower, longer footprint than the
205/60R15. Expect a cornering stiffness 5-15 % lower, a similar or slightly larger peak slip angle, similar μ, and a
slightly longer trail. The 205/60R15 numbers are used as the reference, with that band as the tolerance.

## Measured reference (MF 205/60R15, pure lateral, γ = 0)

| Fz N | Cα N/rad | μ_y peak | α at peak | Fy(15°)/peak | Fy(20°)/peak | trail at 0.5° mm |
|---|---|---|---|---|---|---|
| 1288 | 22 070 | 0.922 | 8.4° | 0.972 | 0.951 | 9.4 |
| 2575 | 40 107 | 0.901 | 8.6° | 0.972 | 0.950 | 18.5 |
| 3863 | 52 212 | 0.881 | 9.2° | 0.976 | 0.954 | 27.3 |
| 5150 | 58 716 | 0.860 | 10.2° | 0.982 | 0.960 | 35.6 |

Longitudinal at 2575 N (braking): C_κ 49.6 kN, μ_x 1.07 at κ = 0.13, Fx(0.3)/peak 0.894, **Fx(0.6)/peak 0.776**.

Key facts for our class:

* **Peak slip angle 8.5-10°**, rising with load.
* **Load sensitivity is mild.** μ falls 7 % from 0.5 to 2 × load, i.e. μ ∝ Fz^−0.05; other passenger fits reach ~Fz^−0.12.
* **Post-peak decay in the lateral range is small**: 3 % at 15° and 5 % at 20° at 16.7 m/s, i.e. up to ~5.7 m/s
  sliding. In the longitudinal range, which reaches larger sliding speeds (κ = 0.6 at 15 m/s is ~9 m/s sliding), it is
  22 %. Burckhardt's locked wheel gives 35 % at full sliding.
* **Pneumatic trail** at small slip grows almost linearly with load: 18.5 mm at 2.6 kN, which is ≈ 0.18 × our footprint
  length at that load. That is between brush a/3 (0.167) and the often quoted ¼. The trail crosses zero near 7.5-8° and
  goes slightly negative at large slip (Mz reversal).

## Where our tyres deviate (mx5_na defaults)

| Fz 2575 N | Cα | μ_y | α_peak | Fy15/pk | Fy20/pk | trail 0.5° | Fx(0.6)/pk |
|---|---|---|---|---|---|---|---|
| Reference (MF) | 40.1 k | 0.901 | 8.6° | 0.972 | 0.950 | 18.5 | 0.776 (Burckhardt 0.83) |
| Envelope default | 44.2 k | 0.914 | 10.3° | 0.995 | 0.984 | 23.6 | 0.873 |
| TARMAC default (`tm_shape 0`) | 54.9 k | 0.914 | **2.5°** | 0.903 | 0.857 | 15.3 | 0.776 |
| TARMAC `tm_shape 1` | 52.1 k | 0.914 | 3.5° | 0.903 | 0.857 | 15.5 | 0.776 |

Deviations:

1. **Load sensitivity is ~5× too strong in both models.** μ_y goes 1.085 → 0.769 over 0.5-2 × load (Fz^−0.25), against
   0.922 → 0.860 measured. Light, unloaded inside tyres have too much grip and loaded outside tyres too little. This
   overstates the grip lost to load transfer and over-weights the ARB balance effect.
2. **Envelope: almost no lateral post-peak decay** (1.6 % at 20° vs 5 % measured), and too little longitudinal decay
   (0.87 vs 0.78-0.83). Its peak-slip trend with load is reversed: 12° → 7° vs 8.4° → 10.2° measured.
3. **TARMAC (faithful, `tm_shape 0`): peak far too early** (2.5° vs 8.5°), because the linear-plastic tread carries
   force linearly up to μN. Lateral decay is too strong (14 % at 20° vs 5 %), while the longitudinal decay matches.
   With `tm_shape 1` the peak only moves to 3.5° at default μ and load sensitivity.
4. **Cornering stiffness** is 10 % (envelope) and 37 % (TARMAC) above the 205 reference. For a 185 tyre we would
   expect it 5-15 % *below* the 205. Open item: not proposed until rig-tested, since it couples to the peak slip.
5. **Trail**: envelope default (`trail_factor 1.5`) is +28 % at nominal load but too flat with load. TARMAC is −15 %
   with the right load slope. Neither model produces the negative Mz at large slip (both clamp trail at 0).
   This is a small effect on rim feel past the limit.

## Proposed corrections (opt-in; defaults unchanged)

**Envelope tyre, "propE"** (`[front]` and `[rear]`):

```ini
tyre_mu_y = 0.955          ; with the new load sensitivity: mu_y 0.917 at the 2575 N reference (MF 0.901)
tyre_mu_x = 0.926
tyre_ls_exp_y = 0.90       ; mu ~ Fz^-0.10 (MF -0.05 .. other passenger fits -0.12)
tyre_ls_exp_x = 0.92
tyre_falloff_level = 0.75  ; large-slip floor from MF Fx(0.6) and Burckhardt locked wheel
tyre_falloff_speed = 2.5   ; keeps the lateral decay at MF level (Fy20/pk 0.944 vs 0.950)
tyre_trail_factor = 1.3    ; trail 20.4 mm vs 18.5 mm at nominal load
```

**TARMAC tyre, "propD"**:

```ini
tyre_model = 1
tyre_tm_shape = 1          ; brush (Fiala) saturation: peak at 7-9 deg instead of 2.5
tyre_tm_damp_mode = 1      ; LuGre deflection-rate damping: no viscous share of the mu N cap in steady rolling
tyre_tm_load_sens = 0.10   ; was 0.25
tyre_tm_slide_floor = 0.70
tyre_tm_slide_exp = 2      ; Stribeck/Gaussian decay (new opt-in key; 1 = TARMAC exponential)
tyre_tm_slide_speed = 8    ; m/s
(+ the same mu_y / mu_x as propE)
```

Rig result with the proposals (2575 N; full table in `results/tyre_ref/*.txt`):

| Fz 2575 N | Cα | μ_y | α_peak | Fy15/pk | Fy20/pk | trail 0.5° | Fx(0.6)/pk |
|---|---|---|---|---|---|---|---|
| Reference (MF) | 40.1 k | 0.901 | 8.6° | 0.972 | 0.950 | 18.5 | 0.776 / 0.83 |
| Envelope propE | 44.5 k | 0.917 | 11.3° | 0.982 | 0.944 | 20.4 | 0.770 |
| TARMAC propD | 46.1 k | 0.921 | 7.0° | 0.982 | 0.954 | 15.8 | 0.844 |

Across 0.5-2 × load, propE and propD give μ 0.98-0.86 and 0.99-0.86 (MF 0.92-0.86). PropD gives α_peak 7.0-9.0° with
the right trend (MF 8.4-10.2°). PropE gives α_peak 10.8-11.3°, a flat top 1-2.5° late, which is acceptable for a
plateau-shaped curve.

**Why a Stribeck exponent?** TARMAC's exponential decay drops fastest right after the peak. One exponential cannot
give the small lateral decay at a few m/s *and* the large longitudinal and locked-wheel decay at 9-20 m/s. The best
exponential compromise (floor 0.90) gives Fy15/pk 0.958, which is already too low, and Fx(0.6)/pk 0.915, which is too high. With δ = 2
(Gaussian) the decay is flat at small sliding speeds and steep later. Fitting v_s on the rig gives v_s = 6 m/s:
lateral 0.969/0.923, Fx(0.6) 0.781. v_s = 8 m/s gives 0.982/0.954 and 0.844, and v_s = 10 m/s gives 0.989/0.970 and
0.887. v_s = 8 matches the lateral reference exactly and lands between MF and Burckhardt longitudinally. This also
matches rubber physics: viscoelastic and flash-temperature friction loss is small at the cm/s-to-1 m/s sliding of a
partially sliding footprint, and grows over several m/s (Grosch 1963; Persson 2006).

(`results/tyre_ref/mx5_envelope_propE.txt` was run with the v_s = 10 TARMAC variant; use only its envelope rows.
`mx5_tarmac_propD.txt` holds the TARMAC proposal.)

Driving validation of the proposals is in the section below.
