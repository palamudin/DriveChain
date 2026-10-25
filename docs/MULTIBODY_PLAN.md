# Multibody plan: one coupled solve with joint reactions

Status: Phases 0–2 implemented and validated (see [MULTIBODY.md](MULTIBODY.md)); parts of Phase 3 (column, U-joints, torsion bar, rack friction) and Phase 6 (engine, clutch, diff and brakes as bodies and rows in the solve) are in. Open: valve pressure states, implicit tyre, bushings, half-shaft bodies, retiring the reduced core. The research basis and the audit are in [RESEARCH_FINDINGS.md](RESEARCH_FINDINGS.md); audit items A1–A11 are referred to below.

## 1. Principle

Every part is a body. Every connection is a joint that removes exactly the relative motions its hardware prevents, and so carries exactly that many reaction components. Every force element (spring, damper, bush, torsion bar, hydraulic piston, tyre) acts equal and opposite on the two bodies it connects.

The step is one implicit solve of all of it. The joint reactions come out of that solve as Lagrange multipliers λ, so they are unknowns of the same system, not post-processing. There are no gates, no clamps on state, and no sequential sub-solvers.

The chassis, and the game engine, receive the sum of the reactions at the hardpoints:

```
F_body = Σ_joints Jᵀ_body · λ  +  Σ_force elements f  +  m·g
```

That is the "unified output of three vectors" (force, moment, point), built from parts that can each express only their own allowed components.

Formulation (Baraff 1996, https://www.cs.cmu.edu/~baraff/papers/sig96.pdf; Featherstone RBDA ch. 8):

```
[ M + h·D + h²·K    Jᵀ ] [ Δu ]   [ h·(f_ext + f_int(q,u)) ]
[ J                −C  ] [ hλ ] = [ −J·u − (β/h)·φ(q)       ]
```

- M: block-diagonal 6×6 rigid-body masses.
- D, K: linearised damping and stiffness of force elements and of the tyre (implicit).
- J: constraint Jacobian.
- C: compliance of soft rows (bushes; Catto soft-constraint form, https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf).
- φ: position error, plus a post-step projection (https://doi.org/10.1016/j.cma.2011.01.007).
- The matrix is symmetric quasi-definite, so LDLᵀ is stable under a fixed (static) ordering.

## 2. Bodies (NA Miata, double wishbone front and rear)

| Group | Bodies | Count |
|---|---|---|
| Chassis | sprung body, full inertia tensor including Ixz | 1 |
| Front corner ×2 | UCA, LCA, knuckle, hub+wheel (spinning), tie rod | 10 |
| Rear corner ×2 | UCA, LCA, knuckle, hub+wheel, toe link | 10 |
| Steering | rim, upper column, intermediate shaft, pinion (with valve rotor), rack (with piston) | 5 |
| Anti-roll bars | front and rear: 2 bar halves each (torsion element between them) | 4 |
| Driveline | crank+flywheel, gearbox output (ratio rows), diff case, 2 half-shafts | 5 |
| **Total** | | **35 bodies → 210 velocity coordinates** |

Hub+wheel and the 1-DOF shafts can be reduced to one rotational coordinate each on their parent, through a revolute in reduced form. That cuts about 50 coordinates. It is an optimisation, not a change of physics.

## 3. Joints and reaction rows

| Connection | Joint | Rows |
|---|---|---|
| UCA–chassis, LCA–chassis | bushing (6 soft); rigid limit = revolute (5) | 6 each |
| UCA–knuckle, LCA–knuckle | spherical | 3 each |
| Knuckle–hub | revolute (wheel bearing) | 5 |
| Tie rod–knuckle | spherical | 3 |
| Tie rod–rack | universal (inner joint) | 4 |
| Rear toe link | spherical + bush | 3 + 6 |
| Spring, damper | force elements LCA↔chassis at real mounts (NA spring on lower arm), nonlinear 2-stage damper, bump/droop stops as unilateral rows | 0 rows (forces) / 1 unilateral |
| ARB halves–chassis | revolute (or bush) | 5 each |
| ARB drop link | distance (two-force member) | 1 each |
| Rack–housing | prismatic, housing on chassis through rack-mount bushes | 5 (+ 6 soft if the housing becomes a body) |
| Pinion–housing | revolute | 5 |
| Pinion–rack | gear (tooth force) | 1 |
| Column, intermediate shaft | revolute + 2 universal joints (cos β torque law emerges) | 5 + 4 + 4 |
| Rim–column | revolute; torsion bar as a force element rim↔pinion | 5 |
| Valve / piston | 2 chamber pressures as states. Valve orifice areas are functions of torsion-bar twist; flow Q = Cd·A(Δθ)·√(2ΔP/ρ); V/β·Ṗ = Q − A·ṙ; piston force A·ΔP on rack ↔ housing (Newton-3 into the chassis) | 0 rows, 2 states |
| Half-shafts | CV joints at the diff and the hub (4 rows each) | 8 per side |
| Diff | ratio rows + viscous coupling T = c(T°)·Δω (NA option) or a clutch pack | 2–3 |
| Tyre | brush force element on the hub; carcass deflection states; contact envelope | 0 rows, 3 states |
| Rear brake, front brake | capped friction impulse knuckle↔hub (unilateral, box-bounded λ) | 1 each |
| **Total** | | **≈ 180 rows** (≈ 130 hard, ≈ 50 soft or unilateral) |

DOF check, per front corner: 5 bodies × 6 = 30, minus 6+6 (bushes, soft) treated as 5+5 in the rigid limit, minus 3+3+5+3+4 = 28. That leaves 2: travel, wheel spin. Steer comes from the rack. The tie-rod spin is removed by the universal joint. The structure has no redundancy, so λ is unique.

Per-joint output telemetry, appended to the C ABI:

```c
typedef struct { int32_t id, type, nrows; double lambda[6]; double frame[9]; double pos[3]; } dc_joint_reaction;
int dc_get_joint_reactions(const dc_car*, dc_joint_reaction* out, int max);
```

- λ is expressed in the joint frame, so a ball joint reports 3 forces and nothing else.

## 4. Tyre inside the implicit step

- The brush model stays as it is: combined slip, falloff, carcass states, multi-ray contact envelope.
- What changes is where it is evaluated. The contact-point velocity is a linear function of the knuckle/hub body velocities. The tyre force is linearised about the current state, F ≈ F0 + (∂F/∂v_c)·J_c·Δu, and ∂F/∂v_c is added to the D block of the system matrix. Body motion and tyre force are then simultaneous (closes A6).
- The carcass deflection e is a state with ė = v_c − v_slide(F). Its implicit update is part of the same linear system, so relaxation length σ = Cα/ky emerges.
- At low speed the carcass spring holds the car, so the |Vcx| ≥ 0.5 and Vref ≥ 1 floors can be retired once Phase 4 validates it.
- The friction limit becomes a box bound on the linearised force, inside the solve rather than after it.
- Pneumatic trail: add a contact-pressure asymmetry, or a trail shape factor, so that small-slip trail ≈ ¼ of the contact length (Pacejka). This is a tyre change, flagged separately.

## 5. Solver and cost

**Factorisation**
- Sparse LDLᵀ of the KKT matrix. The symbolic factorisation and the elimination order are computed once at `dc_create`: corner bodies first, then the steering chain, then the chassis as the separator.
- That ordering is Baraff's O(n) tree elimination with the loops (DWB through the chassis, steering through the rack) closed inside each corner block.
- Fill is bounded by the per-corner blocks (≈ 30 coordinates + 28 rows). All arrays are static, so the step does no allocation.

**Stabilisation**
- Velocity-level constraints with a small Baumgarte bias (β ≈ 0.1–0.2).
- One Newton position-projection pass per substep, reusing the same factor. Expected drift is below 10⁻⁹ m.
- The post-hoc momentum projection (A9) is deleted. Its job is now done by a consistent integrator, and the test is momentum conservation without it.

**Integrator**
- Linearly-implicit Euler (one Newton step). Optional second iteration at the friction limit.
- Numerical dissipation is accounted in its own energy bucket and must stay below 1 % of physical dissipation in the standard tests.

**Cost per 2 kHz substep (single thread, doubles)**

| Item | Estimate |
|---|---|
| Assembly (Jacobians, force elements, tyre linearisation) | 20–30 k flops |
| Corner blocks ×4 (sparse LDL ≈ 58×58, structured) | 60–100 k flops |
| Steering, chassis, driveline separator | 10–20 k flops |
| Solve, projection, telemetry | 10–20 k flops |
| **Total** | ≈ 100–170 k flops → **≈ 150–300 µs** (today: ≈ 80 µs) |

- Real-time factor at 2 kHz: ≈ 1.7–3.3×, against about 6× today.
- Mitigations: reduced coordinates for spinning parts (−25 %); SIMD on the 4 corner blocks (2–3×); 1 kHz is acceptable because the integration is implicit.
- Phase 0 benchmark target: ≤ 250 µs per substep on the dev box.

**Determinism and Unity compatibility**
- Fixed ordering, doubles. Threads in the step are opt-in (dc_set_threads), with a fixed per-wheel / per-row partition and no cross-thread reductions, so results stay bit-identical for any thread count (docs/PERFORMANCE.md). The replay test must stay bit-identical.
- The C ABI is unchanged: car create/step, inputs, the column-torque FFB output, and the three-vector body output. It is extended only with joint-reaction telemetry.
- Unity P/Invoke structs gain one fixed array. The raylib demo and dist build are unaffected.

## 6. Phased migration with validations

Every phase keeps the old core runnable behind a parameter (`solver = reduced|multibody`) until Phase 7.

| Phase | Content | Validation (pass criteria) |
|---|---|---|
| 0 | Body, joint and force-element library; sparse LDL; projection | Pendulum, slider-crank, planar four-bar: reactions vs closed-form statics/dynamics < 1e-6 relative. Energy drift < 1e-6 per s with no damping. Zero allocations. Benchmark. |
| 1 | Single DWB corner on a fixed chassis (rig) | Camber, toe and travel curves equal dc_kin (< 1e-9). **Static equilibrium:** wheel load Fz → ball-joint, pivot and spring reactions match a hand free-body solution < 1e-6. Spring force = Fz/MR. |
| 2 | Full car, steering rigid to rack, tyres as today | **Static equilibrium:** corner loads match CG (52/48 ± 0.1 %). Σ reactions = m·g. Newton-3 per joint (sum of each joint's two sides = 0 to round-off). **Self-centring:** rolling at 5–20 km/h with the rim released at 90°, it returns. **Kingpin moment breakdown:** Mkp from multipliers vs the four-term formula (Fx, Fy, Fz, Mz) < 2 %. Ftie = Mkp/(Lsa·cos λ·cos δ). |
| 3 | Steering chain: column, U-joints, torsion bar, valve pressure states, rack housing | Boost curve (ΔP vs torsion torque) has the shape of the steady-state valve bridge. U-joint torque ripple = cos β law. Rim-torque frequency response to rack force shows the steering-gear mode at 10–15 Hz. Pressure lag of a few ms. |
| 4 | Tyre inside the implicit step; floors removed | Steady-state skidpad and Fy(α) unchanged vs today (< 1 %). **Step steer** (ISO 7401 style, 80 km/h): yaw-rate rise time and overshoot reported against the reduced core. 12 % slope hold at 0 km/h with no creep. Parking effort at standstill. |
| 5 | Bushings (arm pivots, rack mounts, subframe) | Virtual K&C rig: lateral compliance steer (deg/kN), braking caster wind-up, longitudinal compliance, reported against typical published ranges (NA-specific K&C data unknown). Kerb-strike peak load lower than rigid. |
| 6 | Driveline in the same solve: half-shafts, CV joints, viscous LSD | Anti-squat reaction through the arms. Launch, clutch kick and lift-off transients. Energy accounting closes. |
| 7 | Retire reduced core | **Hands-off catch:** fishcheck (rim free after a rear step-out → countersteer emerges; rim rate and peak angle reported). **Drift:** sustained drift test (existing drift harness) passes. Kickback and bump tests. Unity and raylib parity, determinism, perf. |

## 7. What stays

- Brush tyre with combined slip, falloff, carcass and relaxation states
- Multi-ray contact envelope and road module
- Implicit wheel-spin idea, which becomes a revolute coordinate in the big solve
- Capped friction for brakes and clutch, as box-bounded λ
- Engine map, gearbox ratios
- Driver/hands law in the control layer
- Telemetry, CSV/PNG tests, the C ABI, Unity P/Invoke, the raylib demo, `.ini` parameters

The `.ini` files are extended with:
- arm, knuckle and rack masses and inertias
- spring/damper mount points
- bushing rates
- column geometry (U-joint angles)
- valve orifice curves

## 8. Risks

1. **Data, not solver, limits fidelity.** NA bushing rates, arm masses and inertias, K&C curves and valve geometry are not public.
   - Without measurement, Phase 5 adds plausible but unverified compliance.
   - Mitigation: keep each compliance individually switchable; report the feel effect of each.
2. **Performance.** If 150–300 µs is exceeded: reduced-coordinate rotors, SIMD corner blocks, or 1 kHz.
3. **Near-singular geometry** (arms near toggle, full droop). Use proximal regularisation of the hard rows (LCABA, https://simple-robotics.github.io/publications/lcaba/static/paper/LCABA.pdf) and unilateral stops before toggle.
4. **Tyre linearisation at the limit.** The box-bounded force needs a second Newton iteration or a complementarity treatment, otherwise there is a small energy error at saturation.
5. **Numerical dissipation of implicit Euler** colours the feel (over-damped SAT). Track it, and use a θ-method or midpoint for the steering subsystem if it exceeds budget.
6. **Retuning.** Several current parameters were chosen against the reduced model (see the provenance table: μ, wheel rates, damping). The multibody will expose them; expect a re-calibration pass against road-test data.
7. **Schedule.** Phases 0–2 are the bulk of the infrastructure. Phases 3–7 are incremental and each one is independently testable.
