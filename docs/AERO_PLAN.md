# Aero Phase 1 — Geometry-derived aerodynamic loads (plan + prototype results)

Status: Phase 1 (research + prototypes). No core/demo source was changed.
Prototypes are in `tools/aero/`. The CFD scratch case lives outside the repo, in
`/workspace/aero_scratch/` (it holds the private mesh and must never be committed).

Labels used below: **[S]** = sourced (citation given), **[S2]** = secondary source
(forum, magazine quoted by a third party, or encyclopaedia; the primary is named where
known), **[M]** = measured or computed here by our own tools, **[U]** = unsourced
(engineering judgement, to be confirmed). All of the sources are engineering sources:
textbooks, SAE or journal papers, OEM data, and wind-tunnel or CFD tests. None of them
are game developers.

---

## 0. Summary

* **Recommended method: (d) hybrid.** Offline RANS CFD on the real car surface produces
  an aero map. The map is projected onto ~500–2000 runtime surface panels: per-panel
  force vectors per unit dynamic pressure, compressed with POD. At runtime each panel
  load is evaluated from the *local* relative air velocity, `v_point − w(x,t)`, and
  applied as a point load on the chassis body. Wheel panels load the unsprung bodies.
  This means:
  * pitch, roll and heave coupling and the aero-balance shift emerge from where the
    loads act;
  * wind enters only through the relative velocity;
  * nothing about the sign or size of lift is imposed.
* **Newtonian impact theory, option (c), is falsified on our mesh.** It gives
  *downforce* (CL = −0.26) where the real car makes lift. Tuning Cp_max to 1 makes
  Cd = 0.376 match the real car, yet lift stays wrong-signed (−0.13). This is the
  textbook example of a fitted coefficient hiding wrong physics.
* **A runtime panel/VLM method, option (b), is rejected.** It is attached-flow only: base
  pressure and separation cannot come out of it without prescribing separation lines,
  which would be an imposed gate. It is also too expensive (~70+ µs per sim-ms) once the
  ground-image influence matrix changes with ride height.
* **Runtime cost of (d), measured [M]:** 1024 panels takes ~36–40 µs per evaluation.
  At a 240 Hz aero tick that is **~9 µs per sim-ms, about 2% of the current MB core
  (~450 µs per sim-ms)**. 256 panels costs ~1.5–1.9 µs per sim-ms.
* **CFD does run on this box.** OpenFOAM v1912 (Debian) runs simpleFoam with k-ω SST,
  snappyHexMesh and a moving ground. The first coarse Miata case ran with 1.2 M cells
  (results in §5). **Geometry alone gave net lift, CL ≈ +0.11–0.12, with rear > front
  (CLR ≈ +0.15, CLF ≈ −0.04).** Pressure-drag Cd_p ≈ 0.47, against Mazda's 0.44 top
  down. The mesh is coarse with no layers: P2 has to converge this and validate the
  front split.

---

## 1. Engineering basis

### 1.1 Where road-car lift comes from
* A passenger car is a bluff lifting body in ground proximity. Its lift is set by the
  pressure difference between:
  * the upper surface, where flow accelerates over the bonnet leading edge, the
    windscreen header and the roof, giving suction (Cp < 0);
  * the underbody, where blockage, roughness and the cooling-air exit set the pressure;
  * the separated wake, which sets the base pressure.

  Hucho (ed.), *Aerodynamics of Road Vehicles*, 4th ed., SAE R-177, 1998, ch. 4 [S];
  Schütz (ed.), *Aerodynamik des Kraftfahrzeugs* / *Aerodynamics of Road Vehicles*,
  5th ed., SAE R-430, 2016 [S]; Barnard, *Road Vehicle Aerodynamic Design*, 3rd ed.,
  2009 [S].
* Most production cars of the 1970s–90s show **net positive lift**, usually with the rear
  axle lift larger for notchback and sports-car shapes. Howell, Windsor & Passmore,
  "Some Observations on Shape Factors Influencing Aerodynamic Lift on Passenger Cars",
  *Fluids* 6(1):44, 2021, doi:10.3390/fluids6010044 [S] reviews MIRA/Loughborough
  data. Its findings:
  * lift correlates with the rear upper-body shape, backlight angle and the
    underbody/ground-clearance condition;
  * classic camber-line ("airfoil") estimates (Carr; Morelli) do not reliably predict
    the lift of real cars;
  * lift rises with yaw angle.
* Ground effect, cooling flow and wheel rotation all change lift and drag measurably:
  * Katz, *Race Car Aerodynamics* (Bentley, 1995) and "Aerodynamics of Race Cars",
    *Annu. Rev. Fluid Mech.* 38:27–63, 2006 [S];
  * Wickern, SAE 970133 (rotating wheels) [S];
  * cooling-drag literature in Hucho ch. 11 [S].
* The force coefficient definitions follow SAE J1594 (Vehicle Aerodynamics Terminology)
  [S]:
  * Cd, CL and CS are referenced to q·A;
  * the pitching moment CPM is taken about the wheelbase centre at ground level;
  * `CLF = CL/2 + CPM`, `CLR = CL/2 − CPM`.

  `tools/aero/*` uses the equivalent load form `LF = Fz/2 − My/wb`, with My about the
  ground-level wheelbase midpoint, including the drag moment arm.
* Milliken & Milliken, *Race Car Vehicle Dynamics*, SAE R-146 [S] covers aero loads as
  axle load transfer and the aero balance.

### 1.2 Why the forces must be *point loads* on the body
* The existing MB core already applies forces at body points and accumulates the torque
  about the chassis CG (`dc_mbcar.c` ~515–527: drag at `aero_z`, lift at
  `aero_front_x/rear_x`). The problem is not the mechanism but the inputs:
  * `cla_front/rear` are hand-set coefficients (mx5_na.ini: −0.05 each, i.e. small
    lift);
  * there is no wind;
  * nothing depends on ride height, pitch, roll or yaw.
* The plan replaces the coefficients with a geometry-derived distributed load and keeps
  the point-load mechanism. The balance shift with pitch or yaw then *emerges*.
* Reaction: the equal and opposite force goes into the air as momentum, i.e. the wake.
  The phased plan (§8, P6) optionally exports the CFD wake deficit as a wind-field
  contribution for following cars (slipstream), so the reaction is not lost.

---

## 2. Method comparison

The budget is a small fraction of the ~100–300 µs per sim-ms total target. The MB core
is currently ~450 µs per sim-ms (docs/PERFORMANCE.md), so aero must be ≲ 10–20 µs per
sim-ms.

| | (a) Offline CFD maps → per-panel loads | (b) Runtime panel/VLM + ground image | (c) Newtonian impact | (d) Hybrid: (a) + POD + quasi-steady unsteady filter |
|---|---|---|---|---|
| Physics | Full RANS (later DES) incl. separation, base pressure, underbody, ground, cooling | Inviscid, attached flow; separation only if separation lines are *prescribed* (Maskew-type wake models) | Momentum of particles hitting windward faces; **no suction, no ground effect, no wake** (Anderson, *Hypersonic and High-Temperature Gas Dynamics*, ch. 3 [S]: valid for hypersonic flow) | As (a), plus a gust/yaw transient filter calibrated from transient CFD |
| Net lift from geometry? | Yes (subject to validation, §5) | Partial: gets upper-surface suction but not wake/base; separation input = imposed gate | **No: wrong sign** ([M] §4) | Yes |
| Wind/yaw/gusts | Yaw and ride states in the map; gust via quasi-steady | Natively (instantaneous) | Natively but wrong | Map + local v_rel per panel + admittance filter |
| Runtime cost | ~9 µs per sim-ms (1024 panels, [M]) | ≥ ~70 µs per sim-ms with a fixed LU; AIC must be rebuilt (N³) when ground-image geometry changes with heave/pitch → ms per update [M estimate] | ~2–5 µs per sim-ms (raster) | ~9 µs per sim-ms (POD K=16: 8.6 µs, 0.34 MB [M]) |
| Offline cost | High (CFD sweep: hours to days CPU) | None | None | High (same as a) |
| Verdict | Good, but static | Rejected (cost + needs imposed separation) | **Rejected (falsified)** | **Recommended** |

### 2.1 Why the 240 Hz aero tick is sufficient [U, with numbers]
* **Body motions:**
  * the aero inputs change at the rates of ride height, pitch, roll and yaw: body
    heave/pitch ~1–2 Hz, roll ~1.5–3 Hz, with transient content up to ~10 Hz;
  * wheel hop (~12–15 Hz) moves the unsprung bodies, and the wheel-panel loads are
    small;
  * Nyquist at 240 Hz is 120 Hz.
* **Gusts:**
  * the car integrates over its length, so the response to gusts falls off for reduced
    frequency `k = f·L/U ≳ 0.1–0.3` (aerodynamic admittance: Davenport 1961 [S];
    Passmore, Richardson & Imam, Proc. IMechE D 215, 2001 [S]; Sims-Williams,
    SAE 2011-01-0172 [S]);
  * for L = 4 m and U = 30 m/s, L/U = 0.13 s, so the admitted content is below a few Hz.
* **Hold delay:**
  * a zero-order hold over the mechanics substeps adds ½·(1/240) s ≈ 2.1 ms of delay;
  * at 3 Hz that is 2.3° of phase, which is negligible;
  * aero stiffness is far below the suspension stiffness [U], so an explicit hold is
    stable.
* **Energy audit:** to keep it exact, compute `Win += Σ F_i · v_i(t) dt` at every
  mechanics substep from the *held* forces and the *current* point velocities. This
  costs one dot product per panel per substep, or exactly with the existing
  force + torque on the CG: `F·v_cg + T·ω`.

---

## 3. Validation targets

### 3.1 1990 Mazda MX-5 / Miata (NA)

| Quantity | Value | Source |
|---|---|---|
| Cd, top up | **0.38** | Mazda 1990 MX-5 Miata specifications sheet [S] — https://www.miata.net/faq/First%2025%20Years/Pricing%20&%20Specs%20-%20Model%20Year/Specifications/1990%20MX-5%20Miata%20Specifications.pdf ; also docs/RESEARCH_FINDINGS.md:280 |
| Cd, top down | **0.44** | same Mazda spec sheet [S] |
| Frontal area | 1.644 m² (17.7 ft²) | Car and Driver road test, Sept 1989, as quoted on forum.miata.net (threads t=185634, t=395995) [S2] |
| Frontal area | 1.71 m² (18.42 ft²) | forum.miata.net t=5087, provenance unknown [S2] |
| Frontal area of our mesh (top down, incl. wheels) | **1.627 m²** | [M] `meshcheck.py` raster (1.624 m² from the `newtonian.py` raster) |
| CdA top up | 0.62–0.65 m² | derived; current ini 0.63 |
| Wheelbase | 2.265 m | Mazda spec [S]; mesh axle x: +1.099 / −1.167 m [M] |
| Lift, stock | **none published found** | [U] — see the trend data below |
| Lift trend (indirect) | Net lift, rear > front | Occam's Racers wind-tunnel results, https://occamsracers.com/results/ and /results/open-top-chop-top-hard-top-and-fastback/ [S2]: all modified cars (air dam, plywood boot, windows open, no wing). Hardtop + VGs: CLF 0.22 / CLR 0.38 / CL 0.59, Cd 0.49. Chop top: 0.11 / 0.20 / 0.31, Cd 0.48. Fastback: 0.16 / 0.18 / 0.35, Cd 0.38. The author's remark that stock CL is "high 0.30s" is unsourced [U]. |

**Acceptance for the Miata (top down, matching the mesh):**
* Cd within ±15% of 0.44;
* **CL > 0 from geometry alone**;
* CLR > CLF.

The closed-top split is backed by Occam's and by Howell 2021. The open-top split has no
sourced data, so open-top CLR > CLF is a provisional expectation [U].

### 3.2 Fiat Uno Mk1 (1983)

| Quantity | Value | Source |
|---|---|---|
| Cd | **0.34** (std) / 0.33 (ES) / 0.30 (Turbo i.e.) | Costelli, "Aerodynamic Characteristics of the Fiat UNO Car", **SAE 840297**, 1984 [S, primary not yet read]; values as quoted by Julian Edgar, ecomodder.com/forum/showthread.php/my-aerodynamic-car-today-39503.html [S2] |
| Frontal area | **1.81 m²** | same, via Edgar [S2] |
| CdA | ≈ 0.615 m² | derived |
| Wheelbase / length | 2362 mm / 3645 mm | Fiat data via encyclopaedic sources [S2] |
| Lift | **no numbers found**. Edgar paraphrases the paper as lift "good across yaw" | SAE 840297 almost certainly has CL/CLF/CLR vs yaw. **Obtaining it needs a purchase/user decision.** |
| Expected trend | net lift (the user's statement; typical of the era per Howell 2021) | Split for a squareback hatch: Howell 2021 indicates that a squareback shape gives low rear lift, so CLF ≥ CLR is plausible [U until 840297 is read] |

**Acceptance for the Uno:**
* Cd within ±15% of 0.34;
* CL > 0;
* the split as reported by SAE 840297.

There is **no Uno geometry on the box**; see the open questions.

### 3.3 Generic trend tests (both cars) [S: Howell 2021; Hucho ch. 4]
* CL increases with |yaw|.
* Nose-down pitch reduces front lift and shifts the balance forward.
* Side force CS has the correct sign: it pushes downwind.
* Yaw moment is destabilising for the yaw range under test (centre of pressure ahead of
  the CG for a typical car). Hucho ch. 4 [S] gives the typical sign; we have no number
  yet [U].

---

## 4. Newtonian falsification (option c) — [M]

Tool: `tools/aero/newtonian.py`.
* A visibility raster normal to U finds each cell's first-hit triangle.
* `Cp = Cp_max·sin²θ`, applied to the windward faces only (classical modified Newtonian).
* Lee-side Cp = 0.
* Same LOD A mesh as the CFD, top down, with wheels.

| Case | Cd | CL | CLF | CLR | CS | Verdict |
|---|---|---|---|---|---|---|
| β = 0, Cp_max = 2 | 0.753 | **−0.262** | −0.034 | −0.227 | 0 | FAIL (downforce) |
| β = 10°, Cp_max = 2 | 0.654 | −0.196 | — | — | −0.134 | FAIL |
| β = 0, Cp_max = 1 ("tuned") | **0.376** | **−0.131** | — | — | — | FAIL: Cd "matches", lift wrong sign |

* At β = 0, 69% of the impacted cells push *down*: the bonnet, windscreen and boot
  surfaces face partly upward-forward.
* Newtonian theory has no mechanism for the suction over these surfaces, which is the
  dominant source of lift on a real car. It also has no ground effect and no base
  pressure.
* The Cp_max = 1 row shows why tuned coefficients are unacceptable: a fitted constant
  reproduces the drag while the lift remains physically wrong.

---

## 5. CFD on the box — first coarse Miata run (top down)

**Case:**
* generated by `tools/aero/foam_case.py` into `/workspace/aero_scratch/miata_u30` (not in
  git);
* OpenFOAM v1912 (Debian package);
* steady simpleFoam with k-ω SST (Menter, AIAA J. 32(8), 1994 [S]).

**Setup:**
* U = 30 m/s; moving ground at road speed; slip far field.
* Domain x −16..+8 m, y ±5 m, height 5 m (blockage ≈ 1.627 / (10·5) = 3.3%).
* blockMesh 0.25 m. snappyHexMesh surface level 3–4 (31–16 mm), near box level 3, wake
  box level 2, **no prism layers**.
* 1,204,991 cells. snappy took 72 s on 4 cores.
* checkMesh: one failed check, 42 highly skewed faces (max 12.3), which is tolerable for
  a first look.

**Geometry:** LOD A source mesh, with glass, wheels and underbody.
* surfaceCheck: not closed, 200 illegal triangles.
* snappy closes sub-cell gaps. Flow can enter the cockpit, which is physically right for
  top down, and the engine bay through the grille *without radiator resistance*.

**Forces:** `tools/aero/foam_forces.py` integrates pressure only. The function-object
route aborts in this OpenFOAM build ("error in IOstream sha1", a Debian toolchain bug).
Viscous drag is excluded, typically ~5–10% of the total on a car per Hucho [S], so
`Cd_p < Cd`.

**Results [M]:**

| Snapshot | Cd_p (pressure only) | CL | CLF | CLR | CPM | CS |
|---|---|---|---|---|---|---|
| iteration 600 | 0.473 | **+0.120** | −0.036 | **+0.155** | −0.095 | −0.005 |
| iteration 700 | 0.476 | **+0.110** | −0.041 | **+0.152** | −0.096 | +0.005 |

A = 1.627 m², U = 30 m/s, ρ = 1.2. Forces: drag ≈ 416–418 N, lift ≈ +97–105 N.

* **Acceptance criterion met at first-look level.** The bare geometry gives **net lift
  (CL ≈ +0.11–0.12)** with **rear lift > front lift**. No sign or coefficient was
  imposed anywhere: the same solver and mesh pipeline just integrated the surface
  pressure.
* **Front is near zero / slightly negative.** CLF ≈ −0.04, i.e. a slight front
  downforce.
  * Occam's modified cars show a positive front lift, but those cars differ (air dam,
    hardtop/chop top).
  * This case has an open grille path with no radiator resistance and stationary
    wheels.
  * The front split is therefore *not yet validated*. It is a P2 item: porous radiator,
    rotating wheels and mesh convergence.
* **Drag.** Cd_p ≈ 0.47–0.48, against Mazda's 0.44 top down (total, including skin
  friction). The pressure-only value is already ~8% high. That is the expected direction
  for a coarse, layer-free mesh, which typically over-predicts separation. The Cd target
  is not yet met: within 15% if friction is small, but it must be confirmed in P2.
* **Lift is a small difference of large numbers.** The upward-looking surfaces contribute
  CL ≈ +2.44 (suction over the bonnet, screen and deck). The downward-looking surfaces
  contribute ≈ −2.32 (floor). So **CL is sensitive to the floor/underbody and to
  mesh resolution**: mesh convergence (P2) is mandatory before the map sweep.
* **x-station breakdown** (iteration 600): lift is generated mainly from the cockpit to
  the rear deck (x −1.6..−0.1 m: +0.38). The front overhang/bonnet region
  (x +0.4..+0.9) gives −0.17, due to floor pressure under the nose.
* **Convergence.** p residual ~4e-3, flat. The force change over the last 100 iterations
  was |ΔCL| = 0.01 and |ΔCd_p| = 0.003. A steady RANS solution on a bluff body keeps a
  small oscillation, so the uncertainty is ±0.01–0.02 in CL [M].

**Caveats:**
* coarse mesh, no wall layers (wall functions at y+ ≫ 30);
* stationary wheels;
* no radiator porous zone;
* pressure only;
* steady RANS on a bluff body; values averaged over the last iterations.

These are fidelity caveats, not imposed coefficients. Phase 2 does mesh convergence.

**Compute cost:**
* this coarse case: 73 s meshing plus **~1.2 s per iteration on 4 cores** when the CPU
  is uncontended (measured, iterations 600→700 in 120 s). It ran ~3–5 s per iteration
  while the perf-demo worker's wine process was busy;
* ≈ 15–25 min per state for 700–1000 iterations;
* production quality (8–20 M cells, layers, rotating wheels, porous radiator): est.
  10–30× more, i.e. ~6–30 h per case on 4 cores [U].
* Map sweep (§6.2) with a design of experiments of ~40–60 states:
  * at coarse level, ~15–25 h on the box;
  * at production level, a cloud/HPC allocation, or coarse sweep corrected by a few
    high-fidelity anchor cases.

---

## 6. Recommended runtime model (d)

### 6.1 Geometry → panels
1. Build the aero surface from the LOD A source mesh: exterior groups only (the
   `objload.py` DROP regex removes interior, damage and blur groups), glass included.
   The processed demo mesh `private/miata_mesh/body.obj` is **unsuitable**:
   * it has no glass;
   * its projected area is 1.361 m² [M], against the real ~1.64;
   * it has 2248 open and 102 non-manifold edges.
2. Decimate to N ≈ 500–2000 runtime panels, keeping the body/wheel tagging. Each CFD wall
   face maps to its nearest panel (area weighted).
3. Each panel stores its centroid `r_i` (body frame) and its owner body (chassis, or
   wheel/upright k).

### 6.2 Aero map (offline)
* **States:**
  * front ride height h_f and rear ride height h_r (or heave + pitch);
  * roll φ;
  * relative-air yaw β (0, 2.5, 5, 10, 15°, mirrored by symmetry);
  * Re via 2 speeds (20, 45 m/s), which captures the Re dependence of separation;
  * optionally a steer-angle state for the front wheels.
* **Stored data:**
  * per state s, per panel i: the force vector per unit reference dynamic pressure,
    `c_i(s)` (3 floats);
  * POD across states: `c_i(s) = c̄_i + Σ_k a_k(s) φ_ik`, with K ≈ 8–16 modes, and the
    `a_k(s)` multilinear-interpolated on the state grid.

  Measured: K = 16 gives 0.34 MB, against 27 MB for the direct 7×7×5×9 map.
* No sign or magnitude is imposed. Everything comes from the CFD of the geometry.

### 6.3 Runtime evaluation (every 240 Hz tick)
1. **Wind sample.** For each panel (or a coarse subset), the local air velocity is
   `w(r_i,t)` (§7).
2. **Relative velocity per panel:**
   `u_i = v_cg + ω × (r_i − r_cg) − w(r_i, t)` (world frame), transformed to body axes.
   For wheel panels, add the wheel's own body velocity.
3. **Reference state:**
   * `U_ref` = frontal-area-weighted mean of u_i;
   * `β = atan2(U_ref,y, U_ref,x)`;
   * `q_ref = ½ ρ |U_ref|²`;
   * ρ from the game's p and T (ρ = p/(R·T)); currently a constant 1.2 [U];
   * h_f, h_r and φ from the chassis pose against the ground under the axles.
4. **Unsteady correction.** β and q_ref pass through the admittance filter (§7.4) before
   the map lookup.
5. **Per-panel force:** `F_i = q_ref · R_body · c_i(state)`. An optional local-q
   correction, `F_i *= |u_i|²/|U_ref|²`, captures shear and yaw-rate effects to first
   order. It must be validated against CFD with a sheared inlet before use [U].
6. **Accumulation:** chassis panels add `F_i` and `(r_i − r_cg) × F_i` to the chassis
   wrench; wheel panels go to their wheel/upright bodies (unsprung).
   `DC_DBG_NO_AERO` is kept.
7. **Hold and audit.** The wrench is held over the 2 kHz mechanics substeps. `Win`
   accumulates `F·v_cg + T·ω` every substep (§2.1).

**Measured cost** (`tools/aero/bench_panel_map.c`, synthetic data,
`cc -O2 -march=native`), conservative because it includes synthetic wind with sinf per
panel:

| Panels | Direct map, µs per eval | POD K=16, µs per eval | µs per sim-ms at 240 Hz |
|---|---|---|---|
| 256 | ~6–8 | ~6–8 | ~1.5–1.9 |
| 1024 | 38–40 | 35.8 | 8.6–9.6 |
| 2048 | ~75–80 | ~75 | ~18–19 |

---

## 7. Wind interface

### 7.1 Field model `w(x, t)`
* **Mean profile.** Log law in the ground frame: `U(z) = U_10 · ln(z/z0) / ln(10/z0)`,
  with z clamped at ≥ max(10·z0, 0.3 m) near the road [U clamp].
  * Terrain roughness z0 per EN 1991-1-4 terrain categories [S]: 0.003 (sea), 0.01,
    0.05, 0.3, 1.0 m.
  * Mean direction θ_w.
* **Turbulence intensity.** `I_u = 1/ln(z/z0)` (EN 1991-1-4, k_I = c_0 = 1) [S], so
  `σ_u = I_u U(z)`. σ_v ≈ 0.75 σ_u and σ_w ≈ 0.5 σ_u (ESDU 85020 neutral-ABL ratios
  [S]; exact ratios to be confirmed against the ESDU item).
* **Length scale.** `L_u(z) = 300 (z/200)^α`, with `α = 0.67 + 0.05 ln z0` (EN 1991-1-4
  Annex B) [S].
* **Spectra:** von Kármán (von Kármán, PNAS 34, 1948; ESDU 85020) [S], with
  `n = f·L/U`:
  * `n S_u/σ_u² = 4n / (1 + 70.8 n²)^(5/6)`;
  * `n S_{v,w}/σ² = 4n (1 + 755.2 n²) / (1 + 283.2 n²)^(11/6)`.
* **Synthesis.** A frozen field (Taylor hypothesis) built from a sum of random-phase
  Fourier modes (Shinozuka & Deodatis, Appl. Mech. Rev. 44, 1991 [S]), or a Mann (1998)
  box for spatial coherence.
  * The car moving at V samples spatial wavenumbers, so the frequency it sees is
    `f = |V − U| κ / 2π`. This makes the experienced gust spectrum depend on the car
    speed automatically.
* **On-road reality.** Wordley & Saunders, "On-road Turbulence", SAE 2008-01-0475
  (SAE Int. J. Passenger Cars – Mech. Syst. 1(1):341–360), and Part 2,
  SAE 2009-01-0002 [S]:
  * near-road turbulence is shaped by terrain, roadside obstacles and traffic, and
    differs from the free ABL;
  * use their measured intensity and length-scale ranges as presets.

  Sims-Williams, SAE 2011-01-0172 [S] covers crosswind transients.
* **Discrete events** are added as deterministic field features with physical spatial
  extent, not as force gates:
  * gust fronts;
  * bridge and cutting exits;
  * the wake of an overtaken truck.

### 7.2 Coupling
* Sampling must be at the panel positions, not at the CG. A crosswind gust that reaches
  the nose first then produces the physical yaw moment transient.
* **The game's existing wind force on the car must be removed** when this model is
  active. This aero model *is* the wind force on the car, so keeping both double-counts
  it (open question Q1).

### 7.3 Ground-frame shear vs moving-ground CFD
* The CFD map assumes a uniform onflow over a moving ground (wind-tunnel convention).
* On the road, the oncoming flow is `V_car − U_w(z)`: skewed, with height-varying yaw.
  The plan uses the frontal-area-weighted `U_ref` plus the optional per-panel q
  correction.
* Validate with one CFD case using the sheared, yawed inlet (P5).

### 7.4 Unsteadiness (admittance)
* The quasi-steady model alone over-predicts the response at high reduced frequency.
* Apply a first-order (or measured-transfer-function) filter to β and q_ref, with a time
  constant of order L/U. Calibrate the filter per force/moment component from transient
  CFD: a pimpleFoam/DES yaw step and sinusoidal gusts.
* Measured admittance can deviate from the simple low-pass shape at some k (Passmore et
  al. 2001; Sims-Williams 2011 [S]; magnitudes to be extracted from the papers). The
  filter design therefore follows the CFD/literature result, not a fixed form.

---

## 8. Phased plan

| Phase | Work | Touches core? |
|---|---|---|
| **P1 (done)** | This plan; tools: `meshcheck.py`, `objload.py`, `newtonian.py`, `foam_case.py`, `foam_forces.py`, `bench_panel_map.c`; first coarse CFD | No |
| P2 | CFD validation of the setup on the Ahmed body (Ahmed, Ramm & Faltin, SAE 840300 [S]; LDA data Lienhart & Becker, SAE 2003-01-0656 [S]) at 25° and 35°; Miata mesh convergence (3 levels + prism layers, y+ report), rotating wheels (rotatingWallVelocity), radiator porous zone (pressure-drop data needed [U]); acceptance vs §3.1 | No |
| P3 | Aero-surface prep tool (panelisation, CFD → panel mapping); design-of-experiments map sweep; POD; map file format (`assets/aero/<car>.aeromap`, not private: it contains only loads, not geometry. Check with the user that derived data is OK to ship) | No |
| P4 | Runtime `dc_aero.c` in the core: panel evaluation, body/wheel loads, energy audit, `DC_DBG_NO_AERO`; per-car params; wind API; replaces `cda/cla_*`. **Needs user go-ahead after the perf/demo worker is done** | Yes |
| P5 | Unsteady: transient CFD for admittance; sheared-inlet check; crosswind step validation | small |
| P6 | Fiat Uno: geometry + same pipeline; validation vs SAE 840297; optional wake export for slipstream | small |

## 9. Tests (to implement with P4)
1. **Map reproduction.** Static pose, no wind: the runtime wrench equals the CFD integral
   to < 1%.
2. **Sign test.** CL > 0 and the CLF/CLR split match the CFD, with no tuning parameter
   present in the code path. This is the acceptance criterion.
3. **Galilean test.** Car at V with no wind equals a stationary car with wind −V (only
   the relative velocity matters).
4. **Energy audit closes** with aero on (`Win` from point velocities).
5. **Balance shift.** Imposed nose-down pitch reduces front lift, as in the CFD map.
6. **Crosswind step.** The yaw moment leads the side-force build-up as the gust passes
   nose to tail. The response settles to the quasi-steady map value.
7. **Symmetry.** ±β gives mirrored CS, yaw moment and roll moment, with identical CL.
8. **Cost.** < 15 µs per sim-ms at 1024 panels in the real core.

## 10. Open questions for the user
1. **Wind representation in the game today.** Is it a uniform vector, a field, or gusts?
   Where is its force on the car applied? It must be replaced by this model to avoid
   double counting.
2. **Air density.** Does the game have temperature, pressure or altitude? Otherwise
   ρ = 1.2 stays [U].
3. **Top-up geometry** for the Miata. The mesh is top down. Is a soft-top/hardtop model
   available, or should one be built from dimensions? It is needed to validate
   Cd = 0.38.
4. **Fiat Uno Mk1 geometry.** Is there a mesh?
5. **SAE 840297 (Costelli 1984).** May we buy it (SAE paper, ~USD 30–40 [U])? It is the
   only identified primary source for Uno lift.
6. **Map compute.** Is ~1 day of coarse CFD on the box OK, or should it go on a cloud
   machine for production-level maps? CFD shares the CPU with any demo/perf runs, and
   perf measurements taken during CFD are unreliable.
7. **Shipping derived data.** May the aero map (per-panel loads, no geometry) be
   distributed even though the mesh is private?
8. **Scope.** Should the reaction side (wake/slipstream for other cars) be in scope?

## 11. Tool reference
* `python3 tools/aero/meshcheck.py <obj> --offset=dx,dy,dz [--frame yup|phys]`: frontal
  area, topology, underbody clearance (aggregates only).
* `python3 tools/aero/newtonian.py <obj> --offset=... [--yaw deg] [--cpmax 2]`:
  Newtonian falsification.
* `python3 tools/aero/foam_case.py <obj> <case_dir outside repo> ...` then
  `cd case && ./Allrun` (or `./Allrun solve`).
* `mpirun -np 4 foamFormatConvert -parallel`, then
  `python3 tools/aero/foam_forces.py <case> --U 30 --A 1.627 --xf 1.099 --xr -1.167 --zg -0.4426`.
* `cc -O2 -march=native tools/aero/bench_panel_map.c -o /tmp/bpm -lm && /tmp/bpm`.
