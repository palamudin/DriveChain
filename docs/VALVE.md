# Hydraulic rotary valve with pressure states (MB core, `mb_valve_model`)

Opt-in (`mb_valve_model = 0` is the default and is bit-identical to the previous build). In the dist,
`cars/<car>_propV.ini` enables it with the pump at full flow from idle.

| `mb_valve_model` | assist |
|---|---|
| 0 (default) | algebraic boost law `dP = pmax·x⁴/(1+x⁴)`, `x = T_tb/assist_tref`, with the flow limit and the back-drive law (`assist_damping`, `assist_backdrive`) |
| 1 | pressure states; closing edges **contoured** so that the steady bridge reproduces the model-0 law at rated flow with the rack held |
| 2 | pressure states; closing edges are a **linear chamfer** (`a = a0·(1 − θ/θc)`). The boost curve is fully emergent: soft, then abrupt. |

## Model

An open-centre rotary valve is a Wheatstone bridge of four variable orifices. They sit between the pump supply S,
the tank T and the two cylinder chambers A (pushes the rack +y) and B (Rösth 2007, ch. 2-3; Merritt 1967).
Valve twist θ is the torsion-bar twist (rim side ahead of the pinion). θ > 0 closes S→B and A→T and opens S→A and B→T.

- Orifice flow is turbulent, `Q = cd·a·sqrt(2|Δp|/ρ)`. It is regularised to laminar below 10 kPa so that the
  Jacobian stays finite at Δp = 0.
- Chambers follow continuity with the effective bulk modulus: `(V/β)·dP/dt = ΣQ_in − dV/dt`. Here
  `V = chamber_vol + A_p·(stroke/2 ± r)` and `dV/dt = ±A_p·ṙ`.
- The supply node has capacitance `Cs = supply_vol/β + hose_compliance`. The pump is a flow source,
  `Q_p = assist_flow·min(1, rpm/assist_rpm_full)` (flow-controlled fixed-displacement pump). A relief valve
  passes rated flow at 5 % over `assist_pmax`. A chamber cannot fall below vapour pressure (cavitation).
- **Opening edge:** `a = min(a0·(1+|θ|/θc), max(25 mm², a0))`. The chamfer uncovers until the valve-body ports
  limit the area. Rösth measured opening areas levelling off at 20–30 mm².
- **Closing edge, model 1:** designed so that `dP = K·(1/a_c² − 1/a_o²)` equals the boost law at rated flow, where
  `K = ½ρ(Q_r/2c_d)²`. That gives `1/a_c² = dP/K + 1/a_o²`. Leak floor is `valve_amin`.
- **Integration:** the three pressures are integrated by backward Euler with a damped Newton (3×3, ≤ 60
  iterations). A failure is counted in telemetry `valve_nfail` (expected 0).
- **Coupling:** the cylinder force `A_p·(P_A − P_B)` enters the one coupled solve as a linearly implicit
  rack↔chassis element. It uses its sensitivity to rack velocity (`dP/dṙ` from the same Jacobian). The pressures
  are then committed with the solved rack velocity. Newton 3 holds: the reaction acts on the rack housing (body).
- **Energy:** pump power `Q_p·P_S` is external work. Oil and hose compression
  `½(V_A P_A² + V_B P_B²)/β + ½·Cs·P_S²` is stored energy (telemetry `hyd_energy`, included in `potential`).
  Orifice and relief losses appear as dissipation. Per-step dissipation stays ≥ 0 (`make valvetest`).
- `assist_damping` / `assist_backdrive` are not used in models 1/2. Back-drive resistance is emergent: rack
  motion pushes chamber flow through the bridge.

## Parameters and sources

| key | default | source / estimate |
|---|---|---|
| `mb_valve_a0` | 4.4e-6 m² | Neutral open area, sized so that the standby pressure at 6 l/min is ≈0.3 MPa. That matches Rösth's Fig. 2.25 standby loss (≈40 W at 8.2 l/min). |
| `mb_valve_theta_c` | 0.05 rad | twist to close a linear chamfer (model 2), typical 3° valve travel |
| `mb_valve_cd` | 0.6 | sharp-edged orifice |
| `mb_oil_beta` | 3e8 Pa | Effective bulk modulus of mineral oil with entrained air (Merritt). Pure oil is ~1.5e9. |
| `mb_chamber_vol` | 5e-5 m³ | dead volume per chamber including the valve-to-cylinder lines (estimate) |
| `mb_supply_vol` | 1e-4 m³ | pump outlet, hose and valve inlet oil volume (estimate) |
| `mb_hose_compliance` | 2e-12 m³/Pa | PS pressure hose is **built to expand** (SAE J188 "high volumetric expansion", 10.3 MPa class; Rösth's "ECA"). Manufacturer listings give 8–17 cc/ft. The test pressure is not stated; taking it at the rating gives ≈3.8e-12 m³/Pa per metre. Plain 2-braid hose is ≈9e-13 per metre (Parker cat. 4660, 3/8"). The default is ≈0.5 m of J188 hose. |
| `assist_rpm_full` | car ini (1500) | Engine rpm at full pump flow. Rösth: "pumps are normally dimensioned to deliver full flow at engine idle". `_propV` sets 800. |

## What the pressure states add (validated, `make valvetest`, results/valve/)

- **Steady boost (model 1):** equals the model-0 law to 1 kPa over 0–6 N·m (bench, rack held, rated flow).
  The relief caps the supply. Newton converges in ≤ 27 iterations.
- **Parking at idle, braked, `assist_rpm_full = 800`:** rim torque for 30/60° of wheel angle:

  | car | model 0 | model 1 | model 2 |
  |---|---|---|---|
  | mx5_na | 3.36 / 3.89 N·m | 3.52 / 4.05 N·m | 4.62 / 5.05 N·m |
  | hatch | 1.04 / 2.64 N·m | 1.05 / 2.72 N·m | 1.12 / 3.92 N·m |

  With the car's own `assist_rpm_full = 1500`, the pump gives 58 % flow at idle and the bridge pressure scales
  with Q². Parking gets very heavy (MX-5: 28° at 6 N·m). This is why `_propV` puts full flow at idle, as Rösth
  describes.
- **Hydraulic lag (emergent):** a torque step to a 3.5 MPa operating point with the rack held gives a
  supply-pressure t63 of ≈150 ms (model 1). This is the time the pump takes to fill the
  supply capacitance. In the car, the rack moving under the step makes it shorter. Rösth names the ECA hose as
  the main cause of hydraulic lag, with added damping and softened pressure peaks as its benefit.
- **Rack/pressure ringing.** Rack-friction break-away and torque release set off a lightly damped ~35–40 Hz
  rack/oil-column mode. Rösth discusses chattering and the operating-point dependence of stability. Hose compliance is
  what damps it. Worst 100 ms high-pass rack-force rms in the 25 m/s rim-torque step:

  | hose compliance | mx5_na | hatch |
  |---|---|---|
  | supply oil only (0) | ≈350 N | ≈300 N |
  | 1e-12 | 146 N | 166 N |
  | 2e-12 (default) | 56 N | 99 N |
  | model 0 (reference) | 31 N | 32 N |

  Every case decays; none is sustained. The test limit is 120 N, plus < 5 N in the last 250 ms.
  Rubber rack-mount bushes (the next phase: bushings as compliant rows) are the other physical damper of this
  mode.
- **On-centre (`mbcompare hyst`, 0 g sine):** model 1 equals model 0. Rim torque gradient is 0.3006 vs
  0.3005 N·m/deg and equivalent friction is 0.90 N·m, because the x⁴ law gives no boost near centre. Model 2's
  linear chamfer already boosts on-centre: 0.265 N·m/deg (−12 %).
- **Steady state is unchanged:** the MX-5 skidpad gives the same road-wheel angle, speed and ay with models 0/1/2.
  Only the rim torque differs (model 2 is heavier mid-range).
- **Hands-off countersteer is weaker.** The emergent back-drive resistance of the open-centre bridge is
  `c ≈ A_p²·2ΔP_standby/Q ≈ 4.8 kN·s/m`. The model-0 law used 1.5 kN·s/m. The MX-5 hands-off power-on test
  (`HO_THR`) gives β max:

  | throttle | model 0 | model 1 | model 2 |
  |---|---|---|---|
  | 0.3 | 15° | 27° | 25° |
  | 0.4 | 18° | 52° | 51° |
  | 0.5 | 25° | 87° (spin) | 87° (spin) |

  The resistance follows from the standby pressure, which matches Rösth's measured standby loss, so this is the
  physics of the parameter set, not a numerical artefact. A larger `mb_valve_a0` (lower standby) or more pump flow
  lowers it.
- **Determinism:** a repeat after `dc_reset` is bit-identical.

## References

- Rösth, M. (2007). *Hydraulic Power Steering System Design in Road Vehicles*. Linköping Studies in Science and
  Technology, Dissertation 1094.
- Merritt, H. E. (1967). *Hydraulic Control Systems*. Wiley.
- SAE J188, *Power Steering Pressure Hose — High Volumetric Expansion Type*.
- Parker Hannifin, Catalogue 4660 (Hydraulic Hose), section G (volumetric expansion).
