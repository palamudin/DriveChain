# Research findings: steering, suspension and tyres as one coupled mechanism

Status: research and audit only. No solver code was changed for this document. The design that follows from it is in [MULTIBODY_PLAN.md](MULTIBODY_PLAN.md).

The question behind it: the core should behave like a cooperation of forces. Every force has a counter-force, the parts hold each other in a dependency chain, there are no gates, and each joint passes on only the reactions its design can carry. The suspension is part of the steering through its links, its inertia, the CG and the weight distribution.

Source policy:
- Ground truth means textbooks, SAE/ISO/journal papers, factory specifications and instrumented road tests.
- Game and simulator material is never treated as evidence. It appears only in the footnotes at the end.
- **[read]** means the page or PDF was opened and the cited content was seen.
- **[TOC]** means only the table of contents or the publisher page was seen.
- **[summary]** means the figure comes from a search-result summary and was not verified. Treat it as a lead, not a value.

---

## 1. Findings

### 1.1 Multibody formulation: joints carry only the reactions they can express

**Blundell & Harty, *The Multibody Systems Approach to Vehicle Dynamics*** (2nd ed., Elsevier) [TOC]
- https://shop.elsevier.com/books/the-multibody-systems-approach-to-vehicle-dynamics/blundell/978-0-08-099425-3
- This is the reference text for modelling a suspension as rigid parts joined by idealised joints or bushes, in the ADAMS style.
- Linkage models can include or omit bushings. The bushed model is what reproduces compliance steer and compliance camber.

**Blundell (1998), *Materials & Design* 19, "the influence of rubber bush compliance on vehicle suspension movement"** [read: abstract]
- https://doi.org/10.1016/s0261-3069(97)00101-5
- A model with rigid joints and one with bushings give measurably different suspension movement and loads.
- The bushes are what let the wheel steer and camber under force.

**Joint reaction components** (from rigid-body joint theory; the joint-type taxonomy is surveyed in *Mech. Mach. Theory* https://www.sciencedirect.com/science/article/abs/pii/S0094114X16300969 [read: abstract]):

| Joint | Relative DOF allowed | Reaction rows (force + moment) | Car part |
|---|---|---|---|
| Spherical (ball joint) | 3 rotations | 3 force + 0 moment = **3** | upper/lower ball joint, outer tie-rod end |
| Revolute (hinge, idealised pivot) | 1 rotation | 3 + 2 = **5** | arm pivots (rigid limit), wheel bearing |
| Universal (Hooke) | 2 rotations | 3 + 1 = **4** | column U-joints, inner tie-rod (rigid limit) |
| Cylindrical | 1 rotation + 1 translation | 2 + 2 = **4** | damper rod in tube |
| Prismatic | 1 translation | 2 + 3 = **5** | rack in housing |
| Gear / rack-pinion | ratio | **1** (tooth force) | pinion to rack |
| Distance (two-force link) | 5 | **1** (axial only) | ARB drop link, tie rod modelled as a rod |
| Bushing | 6 (compliant) | 6 compliant rows (stiffness + damping per axis) | arm pivots, rack mounts, subframe |

This table is the formal version of the user's statement that a joint "converts the forces it can express by design". A ball joint cannot pass a moment. So the steering moment reaches the knuckle only as the tie-rod axial force times its lever arm about the kingpin axis. The kingpin axis is the line through the two ball joints.

**Baraff (1996), "Linear-time dynamics using Lagrange multipliers"** (SIGGRAPH) [read]
- https://www.cs.cmu.edu/~baraff/papers/sig96.pdf
- Maximal coordinates with constraint rows J; the multipliers λ are the joint reaction forces.
- The KKT system [M Jᵀ; J 0] is solved in O(n) for tree-structured systems by sparse elimination in a fixed order. Loops are handled by a small extra set of rows.

**Featherstone, *Rigid Body Dynamics Algorithms*** (2008), ch. 8 "closed loop systems" [read]
- https://raw.githubusercontent.com/create3000/titania/master/Papers/%5B2008%20Featherstone%5D%20Rigid%20Body%20Dynamics%20Algorithms.pdf
- A loop is closed by a spanning tree plus loop-closure constraints, which gives the same KKT structure.
- Reduced coordinates are faster for trees, but loop closures need a constraint solve either way.
- Both double wishbones close a loop through the chassis, and the steering closes a loop through the rack.

**Constrained articulated-body algorithm (LCABA)** [read]
- https://simple-robotics.github.io/publications/lcaba/static/paper/LCABA.pdf
- A modern O(n) solver for loop-constrained systems, using proximal and augmented-Lagrangian handling of redundant or near-singular constraints.
- Relevant if the double-wishbone geometry approaches a toggle position.

### 1.2 Stabilisation and implicit integration

**Baumgarte stabilisation vs projection** (*Comput. Methods Appl. Mech. Eng.* 2011) [read: abstract]
- https://doi.org/10.1016/j.cma.2011.01.007
- Baumgarte terms (α·Ċ + β·C) bias the velocity solve and need tuned gains. Projection onto the constraint manifold removes drift without changing the dynamics.
- Recommended: velocity-level constraints plus a cheap position projection that reuses the factorisation.

**Catto (2011), "Soft constraints"** (GDC)
- https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf
- An earlier fetch was interrupted; the content is stated from the published derivation.
- A spring-damper written as a constraint with compliance and bias (CFM/ERP) is exactly an implicit-Euler spring-damper. This is the standard way to put stiff bushings into the same KKT solve without a stability limit.

**CarSim Math Models** (Mechanical Simulation; industrial reference) [read]
- https://www.carsim.com/users/pdf/release_notes/carsim/CarSim_Math_Models.pdf
- Industrial vehicle-dynamics models use reduced, table-driven kinematics and compliance (K&C) data from measurement rigs, not raw linkages.
- So the fidelity of any model is limited by having measured K&C and bushing data for the specific car. For a 1990 NA we do not have it (section 4).

### 1.3 Steering as coupled dynamics

**Kingpin moment, four terms** (MathWorks Vehicle Dynamics Blockset, Steering System; equations follow Gillespie ch. 8) [read]
- https://www.mathworks.com/help/vdynblks/ref/steeringsystem.html

```
Mkp   = Mkp,Fx + Mkp,Fy + Mkp,Fz + Mkp,Mz
Mkp,Fy = −Fy·(nT·cos ν)·cos δ          nT = Hl + rsl·tan ν     (caster trail + pneumatic)
Mkp,Fz = −Fz·sin λ·rs·sin δ + Fz·sin ν·rs·cos δ                 (KPI and caster jacking)
Mkp,Mz =  Mz·cos(√(λ² + ν²))
Ftie   =  Mkp / (Lsa·cos λ·cos δ)                                (tie-rod axial force)
```
- λ = KPI, ν = caster, rs = scrub radius, Lsa = steering-arm length.
- The last line is the "counter-force" relation: the knuckle moment exists in the chain only as a tie-rod axial force.

**Gillespie, *Fundamentals of Vehicle Dynamics*** (SAE R-114) [TOC]
- https://www.gbv.de/dms/tib-ub-hannover/1760320110.pdf
- Ch. 8, steering systems: steering geometry, the steering-system forces and moments (lateral, vertical, tractive, aligning, rolling-resistance and camber-thrust terms), steering-system models, and the effect of steering compliance on understeer.

**Milliken & Milliken, *Race Car Vehicle Dynamics*** (SAE R-146) [TOC]
- https://millikenresearch.com/rcvdbak.pdf
- Ch. 19, steering systems: kingpin geometry, the steering-torque breakdown, and compliance.
- Ch. 18, suspension: roll stiffness distribution and the geometric/elastic split of load transfer.

**Stanford steer-by-wire / power-steering model** [read]
- http://www-cdr.stanford.edu/dynamic/bywire/steering
- Minimum two-inertia model:
  ```
  Js·θ̈s = −cs·θ̇s + τs − τk
  Jw·θ̈w = −cw·θ̇w + τk·r + τp − τa
  ```
- Universal-joint torque law: τk = τs·cos β / (1 − sin²β·cos²θs).
- Orifice flow: Q = A·Cd·√(2ΔP/ρ); hydraulic pressure dynamics with bulk modulus 5.5·10⁸ Pa.
- Typical values: kt = 100 N·m/rad, Js = 0.0035 kg·m², cs = 0.1, ratio 16.1, pump flow 2·10⁻⁴ m³/s, piston area 10⁻³ m², Cd 0.6, ρ = 825 kg/m³.
- Driver torque is 0–2 N·m in normal driving and up to 15 N·m in an emergency.

**IJERA 2013, hydraulic power-steering modelling** [read]
- https://www.ijera.com/papers/Vol3_issue3/GZ3312301235.pdf
- Torsion-bar rates 1.37 → 1.13 N·m/deg (≈ 65–78 N·m/rad). The valve error angle is the torsion-bar twist.
- The steering-gear fundamental is about 10–15 Hz, and the steering and suspension are treated as one NVH system.

**Pfeffer, Harrer & Johnston (2008), *Vehicle System Dynamics* 46, on-centre handling** [read: abstract]
- https://doi.org/10.1080/00423110701416519
- Physical rack-force and steering model.
- Rack friction degrades response, column friction reduces on-centre feel, and a stiffer torsion bar improves on-centre feel.
- Steering feel depends on where the friction sits in the chain, which a single lumped DOF cannot represent.

**Hydraulic steering valve and assist** [read: abstracts]
- SAE 2005-01-2389: https://doi.org/10.4271/2005-01-2389
- JSME: https://doi.org/10.1299/jsmec.48.251
- UTS thesis: https://opus.lib.uts.edu.au/bitstream/10453/44030/7/02Whole.pdf
- Cardiff: https://orca.cardiff.ac.uk/id/eprint/2109/
- An open-centre rotary valve is a bridge of variable orifices driven by torsion-bar twist. Chamber pressures are states (compressibility, hose compliance). The boost curve is the steady-state result of that bridge, not an algebraic law.

### 1.4 Tyre facts that set steering feel

**Pacejka, *Tire and Vehicle Dynamics*** (3rd ed., Elsevier) [read]
- https://www.sciencedirect.com/book/9780080970165/tire-and-vehicle-dynamics
- Cornering stiffness Cα ≈ 6–30 × Fz per rad. Longitudinal slip stiffness ≈ 1.5 × Cα. Dry μ ≈ 0.9.
- Peak Fy/Fz falls with load.
- **Pneumatic trail of a real tyre at small slip ≈ ¼ of the contact length.** The parabolic-pressure brush model gives a/3, which is 1/6 of the length. So the brush model under-predicts the aligning moment by about 1.5× unless it is corrected (non-uniform pressure, or a trail shape factor).

**TU/e, MF-Swift tyre-pressure study** [read]
- https://pure.tue.nl/ws/files/3139488/677330157969510.pdf
- Lateral relaxation length σy = Kyα / cy (cornering stiffness ÷ lateral carcass stiffness).
- Measured σy ≈ 0.2–0.9 m, rising with load and falling with pressure.

**Alcázar Vargas et al. (2022), *Sensors*, 205/65R15** [read]
- https://pmc.ncbi.nlm.nih.gov/articles/PMC9459800/
- Cα = d1·sin(d2·atan(d3·Fz)), with d1 = 5.2·10⁴, d2 = 2.7, d3 = 1.1·10⁻⁴. This gives ≈ 26 kN/rad at 1.8 kN.
- Measured relaxation length 0.3–0.9 m.

**Locked-wheel friction on dry asphalt** (compiled reconstruction data) [read]
- http://mfes.com/friction.html
- Average 0.76 ± 0.06. Goudie et al., SAE 2000-01-1314: 0.828 ± 0.033.
- Sliding-to-peak ratio ≈ 0.75–0.85.

**Gillespie ch. 5 (course notes)** [read]
- https://bamason2.github.io/ttc066-module/notes/Section_5.html
- Passenger-car damping ratio 0.2–0.4, wheel hop 11–13 Hz, ζ = Cs / √(4·K·M).

### 1.5 How chassis inertia, CG and weight distribution reach the steering

1. **Through Fz on the steered tyres.** Load transfer m·ay·h/T, the front/rear roll-stiffness split, and the longitudinal transfer m·ax·h/L set Fz,FL and Fz,FR.
   - Fz sets the tyre's Fy capacity and its pneumatic trail (the contact length grows with Fz), so it sets the SAT.
   - Fz also enters Mkp directly through the jacking terms (sin λ, sin ν).
2. **Through the links, instantly.** The geometric (roll-centre and anti-dive) part of load transfer travels through the arm and joint reactions without spring lag. The elastic part travels through the springs with the roll mode's lag (≈ 1.5 Hz ride).
   - That timing difference is part of what a driver feels on turn-in. A raycast spring with a force line reproduces only the steady-state split.
3. **Through yaw inertia.** The yaw response sets how fast the front slip angle changes after a steer input, hence how fast SAT builds. NA Miata Izz ≈ 1340 kg·m² (chief engineer T. Hirai; secondary quote only, see section 4).
4. **Through the unsprung and steering inertia.** Knuckle and wheel inertia about the kingpin axis, together with the tie-rod and rack masses, form the "Jw" of the steering model. Wheel gyroscopics (I_spin·ω × δ̇) push on the kingpin during wheel hop and when steering quickly.
5. **Through the chassis frame itself.** The rack housing and arm pivots move with the body, so body roll and pitch move the rack relative to the arms (roll steer, bump steer). Body accelerations act on the rim and column inertia (the driver is in the body frame).

---

## 2. Audit of `core/`: where the chain is not one coupled solve

Starting point:
- The core is already **one coupled reduced-coordinate solve**: NQ = 12 (v, ω, 4 hub travels, rack, steering wheel), dense Cholesky on ΣJᵀMJ, symplectic Euler at 2 kHz, about 80 µs per substep.
- Kingpin moments reach the rack through the upright Jacobian. So the chain is not "detached".
- The gaps are in what the reduced model leaves out, and in what is solved outside that one system.

| # | Location | What is not in the coupled solve | Physical coupling lost | Felt symptom (expected) |
|---|---|---|---|---|
| A1 | `dc_kin.c` | Links are massless rigid constraints solved by Newton for kinematics only | Arm inertia; arm reactions as dynamic bodies | Small: unsprung inertia lumped at the hub, so wheel-hop frequency and damping are slightly off |
| A2 | whole core | **No bushings.** Every joint is rigid, and the rack housing is rigid to the body | Compliance steer (toe under Fy), caster wind-up under braking, longitudinal compliance, rack-mount compliance | **Steering too direct and too linear.** No progressive build-up and no lag between rim and tyre. Harsh kerb impacts. No toe change under braking or lateral load. Probably the largest feel gap. |
| A3 | `dc_car.c` `link_forces()` (≈ l.538) | Per-link forces are reconstructed after the step as telemetry, not solved for | Joint reactions are not unknowns, so Newton-3 per joint cannot be checked or enforced | None felt directly. Hardpoint loads cannot be validated. |
| A4 | `dc_car.c` internal forces | ARB = k·(sL − sR) on the travel coordinate | Bar twist from the real arm and drop-link geometry; bar reactions on the body at its bushes | Roll stiffness has a constant motion ratio. Minor. |
| A5 | `dc_car.c` | Bump/droop stops on the travel coordinate, with a toggle guard | Stop contact at real geometry | Artefacts only near full droop or bump |
| A6 | `dc_tyre.c` phase A, `dc_car.c` l.606–621 | **Tyre forces are explicit in the body DOFs.** Phase A uses start-of-step velocities. Carcass relaxation is implicit, and Fx is implicit only in wheel spin. | One-substep (0.5 ms) lag from body motion to tyre force. Tyre stiffness is not in the system matrix. Low-speed floors (`fmax(|Vcx|, 0.5)` at `dc_tyre.c:281–282`, Vref ≥ 1). The friction clamp is applied after linearisation. | Near-centre SAT is slightly over-damped. Low-speed and parking behaviour is shaped by the floors, not by the carcass. Small energy inconsistency at the limit. |
| A7 | `dc_car.c` "driveline PGS" (l.426ff, 606–621) | **Driveline, clutch, diff and brakes are a separate capped-impulse PGS before the body solve** | Wheel spin, drive torque and suspension/chassis reactions are not simultaneous. Half-shaft torque reaction does not pass through the uprights and links. | Splitting error in fast transients (clutch kick, lift-off, diff lock-up mid-drift). Rear axle "wind-up" is missing. |
| A8a | `dc_car.c:127` | Fixed column axis; **no intermediate shaft or U-joints** | U-joint torque ripple τout/τin = cos β / (1 − sin²β·cos²θ) | Uniform rim torque around the lap of the wheel. Real cars have a slight 2-per-rev variation. |
| A8b | `dc_car.c` | Constant pinion ratio | Variable-ratio racks are not representable. The NA rack is constant, so this is fine for the NA. | None for the NA |
| A8c | `dc_car.c` l.369–401 | **Hydraulic valve is algebraic**: boost P = Pmax·x⁴/(1+x⁴) on torsion-bar torque, with a flow limit and linear-implicit back-drive damping. No chamber pressure states. | Oil compressibility and hose compliance (a pressure lag of a few ms). Valve hysteresis. Pressure-dependent friction. | Assist responds instantly. Missing the slight "pressure build" lag and the steering-gear mode at 10–15 Hz (IJERA). Kickback is filtered by the back-drive law rather than by flow physics. |
| A8d | `dc_car.c:404` | Rack and column friction are tanh-regularised (5 mm/s, 0.05 rad/s) | True stick-slip. Pfeffer: where the friction sits shapes on-centre feel. | On-centre feels viscous rather than having a small friction "detent" |
| A8e | `dc_car.c:408ff` | Penalty rack stops (k 2·10⁶) | Stop as a unilateral constraint | Stop bounce at lock (minor) |
| A9 | `dc_car.c` l.716–742 | **Post-hoc momentum projection** (6×6 correction after the solve) | The consistent integrator itself should conserve momentum. The projection is a corrective step. | None while small. It is exactly the sort of correction the target design removes. |
| A10 | `dc_car.c` | Rigid chassis, **diagonal inertia** (no Ixz) | Roll–yaw product of inertia. Body torsion (low for an open roadster; value unknown). | Missing roll–yaw coupling in fast direction changes |
| A11 | `dc_car.c` | Spring/damper as a scalar on the travel coordinate with a fixed motion-ratio map | Spring/damper as a force element between real mounts (NA: lower arm) | Equivalent at first order. The motion ratio changes with travel. |
| A12 | control layer | Driver hands law | Correct to keep it outside the physics. Not a gap. | — |

Already sound and kept:
- the brush tyre with combined slip, carcass and relaxation states
- multi-ray contact envelope
- implicit wheel-spin update
- capped friction impulses for brakes
- energy and momentum telemetry
- determinism
- no allocation in the step

---

## 3. Deck corrections ("The Driving Chain", 19 slides)

Each entry gives the claim, the verdict, and the correction (with equation and source).

**01 Title / ground rules.** Claim: "every recommendation is closed-form or one small implicit update". **Partly wrong.**
- A coupled mechanism needs one implicit solve of the whole chain (a sparse KKT system), not independent closed forms per part.
- Correction: "every part contributes rows to one implicit system; reactions come out as multipliers" (Baraff 1996, https://www.cs.cmu.edu/~baraff/papers/sig96.pdf).

**02 Road surface.** Claims: Fz = kt·max(penetration); dry μ 0.8–1.1. **Correct, add measured data.**
- Locked-wheel dry asphalt 0.76 ± 0.06; peak/locked ≈ 1.2–1.3 (http://mfes.com/friction.html).
- Pacejka gives dry μ ≈ 0.9 (https://www.sciencedirect.com/book/9780080970165/tire-and-vehicle-dynamics).
- kt depends on pressure: TU/e, https://pure.tue.nl/ws/files/3139488/677330157969510.pdf.

**03 Brush tyre.** Claims: Fy = μFz[1−(1−σ)³]; Mz peaks at σ = ¼ with Fy at 58 %; trail a/3.
- **The maths is correct for the parabolic brush**: d/dσ[σ(1−σ)³] = (1−σ)²(1−4σ) → σ = ¼, and 1 − 0.75³ = 0.578.
- **Correction**: measured tyres have pneumatic trail ≈ ¼ of the contact length (= a/2), not 1/6 (Pacejka, URL above). The pure brush therefore under-predicts SAT by about 1.5× at small slip.
- State this, and either add a trail shape factor or use an asymmetric pressure distribution.

**04 Combined slip.** Claim: σ⃗ = (κ, tan α)/(1+κ)·C/(3μFz). **Correct** (Pacejka theoretical slip, ch. 3).
- Add that Cκ ≈ 1.5·Cα for real tyres (Pacejka). An isotropic C is a simplification.

**05 Relaxation.** Claims: σr = Cα/klat; 0.12–0.45 m; measured 0.3–0.9 m. **Correct.**
- Add the TU/e result σy = Kyα/cy, measured 0.2–0.9 m, varying with load and pressure (https://pure.tue.nl/ws/files/3139488/677330157969510.pdf).

**06 Wheel spin.** Claim: λ ≈ Cκ·r²/(I·|v|). **Correct for a rigid tyre.**
- Add: with a longitudinal carcass spring kx the mode is bounded at f = (1/2π)·√(kx·r²/I), about 30 Hz for kx = 3·10⁵ N/m, r = 0.29 m, I = 0.7 kg·m².
- So the 1/|v| singularity is an artefact of the rigid-tyre slip definition (Pacejka ch. 7, transient).

**07 Brakes.** Claim: Tb = μpad·p·A·n·reff, with a capped impulse. **Correct.**
- Add: the brake reaction goes into the knuckle (outboard calipers) and through the links. That is what makes anti-dive and caster wind-up, and it needs the multibody (Blundell & Harty).

**08 Steering geometry.** Claim: Mkp = k̂·[(p−a)×F] + (k̂·n̂)Mz.
- **Correct as vector mechanics.** Add the four-term breakdown (MathWorks/Gillespie, section 1.3) so each mechanism can be validated separately.
- **Add the missing link**: the knuckle receives Mkp only as tie-rod axial force, Ftie = Mkp/(Lsa·cos λ·cos δ). The ball joints carry 3 forces and no moment (https://www.mathworks.com/help/vdynblks/ref/steeringsystem.html).
- Correction to "the only centring that works at zero speed": at standstill the tyre's twist resists motion in both directions. Only the Fz jacking terms centre.

**09 "The steering chain is one degree of freedom".** **Wrong.**
- The physical chain has at least: rim and column inertia; a torsion bar (60–115 N·m/rad, much softer than the column); pinion; rack with hydraulic piston and pressure states; tie rods; knuckle inertia about the kingpin; all of it mounted on a moving body.
- Minimum correct model (Stanford, http://www-cdr.stanford.edu/dynamic/bywire/steering):
  ```
  Js·θ̈s = Th − kt·(θs − θp) − cs·θ̇s
  Jeq·θ̈p = kt·(θs − θp) + rp·(A·ΔP − Σ Ftie − Ffric)
  ```
  - plus pressure dynamics V/β·ΔṖ = Qvalve(θs − θp, ΔP) − A·ṙ
  - plus U-joint torque modulation.
- Friction placement matters (Pfeffer 2008, https://doi.org/10.1080/00423110701416519). The steering-gear mode is at 10–15 Hz (https://www.ijera.com/papers/Vol3_issue3/GZ3312301235.pdf).
- The DriveChain core already uses a 2-DOF version, so the slide lags the code.

**10 Pads and keyboards.** Claims: δ ≈ L·μg/v² + (αf − αr); tan αpeak = 3μFz/Cα. **Correct** (Gillespie steady-state cornering; brush peak).
- Note this is steady state. The physical answer for pads is a torque input into the full chain (slide 09), which this slide already recommends.

**11 Springs, dampers, ARBs.** Claim: "Keep the raycast spring-damper, apply along the linkage line". **Replace.**
- A force line through the instant centre reproduces only the steady geometric/elastic split. It does not reproduce the dynamic paths: link reactions are instantaneous, spring forces lag; motion ratio varies with travel; compliance steer; the brake and drive reaction through the arms.
- Correction: model the arms as bodies joined by joints and bushes, with spring and damper as force elements between their real mounts (Blundell & Harty; Blundell 1998 https://doi.org/10.1016/s0261-3069(97)00101-5).
- Roll stiffness split (Milliken RCVD ch. 18):
  ```
  ΔFz,f = (m·ay/Tf)·[ hrc,f·(Wf/W) + (h − hrc)·Kφf/(Kφf + Kφr) ] + unsprung term
  ```
- Damping: passenger cars run ζ ≈ 0.2–0.4 (https://bamason2.github.io/ttc066-module/notes/Section_5.html).

**12 Chassis.** Claims: ΔFz,long = m·ax·h/L; Izz = Σ mi·ri². **Correct.**
- Add a measured NA reference: 990 kg curb, 52/48 unladen, 50/50 laden (factory spec, https://www.miata.net/faq/First%2025%20Years/Pricing%20&%20Specs%20-%20Model%20Year/Specifications/1990%20MX-5%20Miata%20Specifications.pdf).
- Izz ≈ 1340 kg·m² (Hirai, secondary source).
- Add the product of inertia Ixz for roll–yaw coupling.

**13 Engine and clutch.** **Consistent with the literature.** No change.

**14 Driveline and diff.** Claim: generic clutch-pack LSD. **Correct the NA example.**
- The 1990 NA option was a **viscous** LSD (https://en.wikipedia.org/wiki/Mazda_MX-5_(NA)).
- Torque transfer is a function of speed difference, T ≈ c·Δω (rising with fluid temperature), not of input torque. A power/coast ramp model is the wrong type for this car.

**15 Aerodynamics.** Claim: Cd·A ≈ 0.30 × 2.2 (illustrative). **Fine as an illustration.** NA reference: Cd 0.38 with the top up (factory spec, URL above).

**16 Output: "merging everything into three vectors".** **Correct for a massless suspension, incomplete for a real one.**
- The chassis receives the sum of **joint and bushing reactions at the hardpoints**, which equals Σ tyre forces − Σ m_unsprung·a_unsprung − the steering-chain inertial terms.
- These differ during wheel hop, kerb strikes and fast steering. Per the user's principle, each joint outputs only its allowed reaction components (section 1.1 table), and the engine receives their sum (Baraff 1996).

**17 Order of operations and budget.** Claim: "~600 flops per substep". **Wrong by about two orders of magnitude for a coupled model.**
- The current 12-DOF coupled core costs ~80 µs per substep. A full multibody is estimated at 150–300 µs per substep at 2 kHz (MULTIBODY_PLAN.md §5).
- Sequential per-part stages are the splitting the user objects to. Replace with "assemble → one sparse implicit solve → project".

**18 Summary table.** **Update rows:**
- "Real CG and inertia, linkage-line suspension forces" → "multibody links and bushes; joint reactions as multipliers".
- "Column DOF with caster" → "column + torsion bar + valve pressure + rack + tie rods in one solve".

**19 Sources.** **Replace forum and game sources** with the engineering sources in section 1 (Pacejka, Gillespie, Milliken, Blundell & Harty, Baraff, Featherstone, SAE/JSME/VSD papers, the factory spec, instrumented road tests). Game and simulator sources move to a footnote.

---

## 4. Parameter provenance: public reference car `cars/mx5_na.ini` (1990 NA, 185/60R14)

Verdict key:
- ✅ matches a factory or literature value
- ⚠️ plausible but unverified, or at the edge of the literature range
- ❌ inconsistent with the literature or the factory spec
- 🔧 chosen value with no source; it needs measurement

**Reference data used:**
- Factory spec (1990 MX-5): https://www.miata.net/faq/First%2025%20Years/Pricing%20&%20Specs%20-%20Model%20Year/Specifications/1990%20MX-5%20Miata%20Specifications.pdf
- Car and Driver road test (Sept 1989): 0.82 g skidpad on 185/60HR14, 70–0 mph in 195 ft (≈ 0.84 g), 51/49 as tested. https://www.caranddriver.com/reviews/a15141519/1990-mazda-mx-5-miata-archived-test-review/ and https://www.caranddriver.com/features/a15106749/2016-mazda-mx-5-miata-grand-touring-vs-1990-mazda-mx-5-miata-feature/
- Motor Trend first test: 0.92 g. https://www.motortrend.com/features/1989-mazda-mx-5-miata
- Factory alignment ('91 shop-manual figures as posted): https://forum.miata.net/vb/showthread.php?t=24254 and https://miatatorque.com/spec/front-alignment-caster-na8/
- OEM spring rates (collected; 1990 manual: 164.17 lb/in front, 96.94 lb/in rear, linear): https://miatiniracing.wordpress.com/wp-content/uploads/2018/06/mx-5-federraten.pdf
- Motion ratios (suspension supplier calculator for the 1.6 NA: spring 0.66 F / 0.77 R; sway bar 0.548 F / 0.586 R): https://www.fatcatmotorsports.com/FCM_Ride_Harmony_1_6NA_Mazda_Miata_Online/FCM_Ride_Harmony_1_6NA_Mazda_Miata_Online.htm

| Parameter | Value in mx5_na | Source of the value | Literature / factory value | Verdict |
|---|---|---|---|---|
| Wheelbase, tracks | 2.265 m; 1.41 / 1.43 m | hardpoints | 2265 mm; 1410 / 1428 mm (factory) | ✅ |
| Mass | 1050 kg | chosen: curb + driver | 990 kg curb (factory); 2210 lb ≈ 1002 kg (C/D) | ✅ (curb + ~60 kg) |
| CG longitudinal | 52.0 % front | hardpoints | 52/48 unladen, 50/50 laden (factory); 51/49 (C/D) | ✅ |
| CG height | ≈ 0.45 m | chosen | not published for the NA. NHTSA inertia database has no NA entry (https://doi.org/10.4271/1999-01-1336) | 🔧 |
| Yaw inertia (total) | ≈ 1317 kg·m² | chosen sprung box + unsprung | 1340 kg·m² (Hirai, *Development of the MX-5/Miata*, quoted at http://weightreductionusha.blogspot.com/2015/06/weight-reduction-miata.html; book https://www.amazon.com/Mazda-Mx-5-Miata-Roadster-Development-ebook/dp/B0DHV48647). Secondary quote only. | ✅ / ⚠️ source |
| Roll/pitch inertia | 300 / 1000 kg·m² | chosen | no published NA value | 🔧 |
| Steering ratio, lock | 15:1, 504° (2.8 turns) | factory | power steering 15.0:1, 2.8 turns; manual 18.0:1, 3.3 turns | ✅ |
| Road-wheel lock | ≈ 33.6° | from ratio | inner 37°23′, outer 32°32′ (FSM) | ✅ (mean) |
| KPI, caster | ≈ 10.7°, ≈ 4.6° | hardpoints | 11°20′; 4°21′–5°16′ ± 45′ (FSM) | ✅ / ⚠️ KPI 0.6° low |
| Front camber | −0.5° | chosen | −0°20′ … +1°02′ | ✅ |
| Rear camber | −1.2° | chosen | −0°30′ … −1°08′ | ⚠️ 0.07° outside |
| Front toe | 0 | chosen | 3 ± 3 mm total toe-in (0°18′) | ❌ use ≈ +0.15° per side |
| Rear toe | 0.1° | chosen | in range | ✅ |
| Front wheel rate | 19.0 kN/m | chosen | spring 28.75 kN/m × MR² (0.66² → **12.5 kN/m**; geometric MR estimate 0.74 → 15.7) | ❌ 20–50 % stiff |
| Rear wheel rate | 17.5 kN/m | chosen | 16.98 kN/m × 0.77² → **10.1 kN/m** | ❌ ≈ 75 % stiff (also shifts roll balance rearward) |
| Ride frequency (front) | ≈ 1.41 Hz | derived | stock springs give ≈ 1.1–1.3 Hz | ⚠️ follows from the rate |
| Front ARB (wheel) | 9.5 kN/m | chosen | 19 mm bar, own estimate × 0.548² ≈ 8–10 kN/m | ✅ / ⚠️ estimate |
| Rear ARB (wheel) | 4.0 kN/m | chosen | 12 mm bar × 0.586² ≈ 2.5 kN/m (own estimate) | ⚠️ ≈ 60 % high (estimate) |
| Dampers (front) | bump 2200 / reb 6000 N·s/m (slow); rear 1700 / 5500 | chosen | c_crit ≈ 4.3 kN·s/m → ζ 0.51 bump, 1.4 rebound. Literature ζ 0.2–0.4 (Gillespie notes). | ❌ high for a stock car (sport valving at best) |
| Unsprung mass | 30 kg/corner | chosen | wheel hop ≈ 13.6 Hz vs 11–13 Hz typical | ⚠️ plausible |
| Tyre radius | 0.289 m | 185/60R14 geometry | 0.2888 m | ✅ |
| Tyre vertical stiffness | 200 kN/m | chosen | passenger tyres ≈ 150–250 kN/m; quarter-car notes use 160 kN/m | ✅ |
| Tyre μ (peak) | 1.12 / 1.15 | chosen | road tests 0.82 g (C/D) / 0.92 g (MT) give tyre μ ≈ 0.9–1.0. Pacejka dry ≈ 0.9. | ❌ ≈ 10–15 % high. Our simulated skidpad is 0.93 g. |
| Load sensitivity exponent | 0.80 / 0.75 | chosen | qualitative trend (Pacejka) | ⚠️ plausible |
| Cα (implied) | ≈ 41.5 kN/rad at 1.8 kN (≈ 23 × Fz) | from peak slip 8.5° and μ | Pacejka 6–30 × Fz; measured 205/65R15 ≈ 26 kN/rad at 1.8 kN | ⚠️ inside range, 1.6× the measured tyre |
| Peak slip | 8.5° | chosen | no data for this tyre | 🔧 |
| Slide/peak ratio | 0.85 | chosen | locked/peak ≈ 0.75–0.85 | ⚠️ upper edge |
| Falloff speed | 4 m/s | chosen shape parameter | no direct measurement | 🔧 |
| Carcass ky, kx | 150 / 300 kN/m | chosen | σy = Cα/ky ≈ 0.28 m vs measured 0.2–0.9 m (TU/e) | ⚠️ low end |
| Pneumatic trail (model) | a/3 | brush | ≈ ¼ contact length (Pacejka) | ❌ ≈ 1.5× low |
| Spin inertia | 0.7 kg·m² | chosen | 14" steel/alloy + 185 tyre ≈ 0.6–0.9 (estimate) | 🔧 |
| Rack mass, rim and column inertia | 3 kg; 0.035; 0.002 kg·m² | chosen | Js = 0.0035 (Stanford, column only); rim 0.03–0.04 typical | 🔧 |
| Torsion bar | 115 N·m/rad (assist) | chosen | 65–78 (IJERA), 100 (Stanford) | ✅ stiff end |
| Piston area | 8.8·10⁻⁴ m² | chosen | 10⁻³ (Stanford) | ✅ |
| Pump flow | 1·10⁻⁴ m³/s | chosen | 2·10⁻⁴ (Stanford) | ⚠️ |
| Relief pressure | 7 MPa | chosen | typical 7–10 MPa (unsourced) | 🔧 |
| Rack/column friction | 60 N / 0.15 N·m | chosen | no NA data; Pfeffer: placement matters | 🔧 |
| Gears, final drive | 3.136 / 1.888 / 1.330 / 1.000 / 0.814; 4.3 | factory | identical | ✅ |
| Diff model | ramp/preload (0.3 / 0.3 / 20 N·m) | chosen | NA option was **viscous** (Δω-driven) | ❌ wrong type |
| Engine inertia, clutch capacity | 0.12 kg·m², 260 N·m | chosen | B6 1.6: no source found | 🔧 |
| Brake torque | 950 / 450 N·m per wheel F/R | chosen | 0.84 g measured stop; our capacity ≈ 0.92 g | ✅ |
| Cd·A | 0.63 m² | chosen | Cd 0.38 × ≈ 1.65 m² | ✅ |
| Hunt–Crossley carcass reference (code constant) | 4 mm | chosen | no measurement | 🔧 |

Highest-impact corrections for feel:
1. Tyre μ about 10–15 % high, and pneumatic trail low.
2. Rear wheel rate about 75 % high (roll balance and drift behaviour).
3. Damping ζ about 2–3× the passenger-car range.
4. Diff type.
5. Front toe 0 instead of toe-in.

None of these were changed in this audit.

---

## Footnotes: game and simulator material (not evidence)
- BeamNG builds vehicles from node-beam networks (soft-body). That is a discretisation choice, not validation data: https://documentation.beamng.com/modding/vehicle/intro_jbeam/
- rF2 tyre and suspension description per a third-party write-up: https://motorlaps.com/car-physics-rfactor2.php
- Commercial MBS (ADAMS/Car) and CarSim are industrial tools. CarSim's own document is cited above for its modelling choices only.
