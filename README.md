# DriveChain: physically based steering, suspension and tyre core

DriveChain has a C core with a Unity frontend (P/Invoke) and a raylib test drive. The target feel is BeamNG-style physicality with the tarmac feel of dedicated circuit simulators.

- **Numerics:** double precision, deterministic, symplectic Euler at 2 kHz with a 12-DOF Cholesky solve. There is no heap use in the step.
- **Conservation:** Newton-3 and energy accounting are checked every run.

## Speed settings
- **Library:** the mechanics rate is `substep_hz`; the default stays 2000. Host calls to `dc_step(car, dt)` run as many substeps as `dt` covers.
  `dc_set_threads(car, n)` (1-4, default 1) spreads each step over n cores, and results are bit-identical for any n. With n > 1 the road callback must be thread-safe.
- **Test drive (demo.ini `[physics]`):**
  - `tick_hz = 960` (demo default): inputs, FFB and audio update once per tick. On a weaker PC use 480 or 240.
  - `model = fast` (demo default): the same coupled solve with a per-tick friction-solver sweep budget, a low-rank
    re-solve when a tyre leaves the friction circle, and exact closed forms in the tyre contact. It passes the
    fidelity suite (`make fastcheck`) and keeps the worst tick near 1.2 ms. `model = reference` (or
    `--set physics.model=reference`) selects the full reference numerics, unchanged bit for bit. Details: docs/PERF.md.
  - `substeps = 4`: mechanics steps per tick. Mechanics rate = tick_hz x substeps: the default 960 x 4 = 3840 Hz.
    This default is new and costs about 4x the CPU of the old 240 x 4; occasional slow ticks are being profiled.
    Lighter settings: 480 x 4 (1920 Hz), 240 x 4 (960 Hz, the old default), 240 x 2 (480 Hz).
  - `threads = 4`.
  - `perf_log = 1` prints the cost per simulated second.
  - Lower `tick_hz` or `substeps` is cheaper. 480 Hz mechanics is the lowest rate that matches the 1920 Hz handling results for the reference car.
- **Offload lane (demo.ini `[offload]`, fast model only):** `patch = auto` (demo default) traces the tyre-patch rays as
  float32 lookups in a per-wheel patch table and picks the fastest backend on the live batches: each wheel tries the
  candidates for its first 384 calls, then keeps the one with the lowest mean. Every backend gives the same bits, so the
  pick never changes a result. `cpu`, `cpu_simd` (AVX2 when the CPU has it) and `gpu` (OpenCL, loaded at run time; the
  closest CPU backend if none) force one; `ref` keeps the double rays. The F7 overlay shows the pick and the live
  cost per backend. Design and measurements: docs/BRICKS.md (offload lane).
- Costs and convergence tables: docs/PERFORMANCE.md.

## What is modelled
- **Tyre:** brush model with combined slip, load sensitivity, pneumatic trail, a relaxation length (σ = Cα/ky ≈ 0.23 m) and a carcass/rim puncture model.
- **Suspension:** double wishbone, MacPherson and multilink from hardpoints. Bump steer, camber gain and Ackermann all emerge from the geometry.
- **Steering chain:** rim → column → torsion bar → open-centre rotary valve (dP = pmax·x⁴/(1+x⁴), x = τ/τref) → rack → tie rods → uprights.
  - The valve uses the orifice back-drive law, gain 0.5 (see docs/FISHTAIL_KICKBACK.md).
  - Self-centring, countersteer and kickback are emergent.
  - FFB is the column torque, with no canned effects.
- **Driveline:** FWD, RWD and AWD; engine, clutch and gearbox (sequential or H-shifter); procedural audio.

## Layout
| Path | Contents |
|---|---|
| core/ | C core (`drivechain.h`, opaque C ABI). `dc_driver.c` holds the plug-in driver. |
| demo/ | raylib/SDL2 test drive: proving ground, FFB, gamepad arms layer, JSON-lines driver port. |
| tools/ | Test-harness and pipeline tools (see below). |
| unity/ | `DriveChain.cs` (P/Invoke and MonoBehaviour), `DriveChainDriverBridge.cs`, `AbiTest.cs`. |
| tests/ | `dc_tests.c` (physics test suite), `plot.py` |
| cars/ | `mx5_na.ini` (public NA Miata reference) and `hatch_fwd.ini` |
| results/ | Committed test CSV/PNG summaries |
| docs/ | `DRIVER_INTERFACE.md`, `FISHTAIL_KICKBACK.md`, `DRIFT.md` |

The tools are:
- `drivetest` (fishtail and kickback harness)
- `drvhost` (lockstep driver host)
- `dcdrive.py` (Python client)
- `plot_kickback.py`
- `build_windows.sh` and `package_win64.sh`

`private/` (privately calibrated car, mesh, private results), `dist/` and `results/shots/` are gitignored and are not part of the source.

## Build and test (Linux)
```
make                 # core lib + dc_tests
make test            # dc_tests + plots, fishcheck, driver selftest, Unity ABI/bridge test (if mono)
make demo            # raylib test drive (build/drivechain_demo)
make drivetest       # ./build/drivetest {fishtail|kickback|fishcheck|all} <car.ini> <outdir>
DC_CAR=other.ini DC_RESULTS=outdir ./build/dc_tests   # any car, results elsewhere
```
The Windows x64 build cross-compiles with mingw: `tools/build_windows.sh && tools/package_win64.sh` → `dist/drivechain_win64.zip`. See `tools/dist_README.md` for controls, FFB, wheel rotation, H-shifter and triple screens.

## Test suite (`make test`, all passing)
- **dc_tests:**
  - kinematics: camber gain, bump steer, Ackermann
  - static loads vs analytic
  - momentum, energy (flat and rough) and Newton-3 residuals (~1e-15 mg)
  - tyre rig: Mz before Fy, µ, peak slip, load sensitivity
  - skidpad (0.93 g, K = 0.70 deg/g)
  - step steer
  - hands-off countersteer and recovery
  - kerb envelope with FFB
  - puncture
  - assist effort
  - performance (≈52 µs/substep, ~9.6× real time)
- **fishcheck:** 5 fishtail scenarios × 4 drivers (+ RAW before), checking ζ, swings, settle and β.
- **drvtest:** 7 driver manoeuvres through the JSON protocol.
- **unitytest:** C# ABI test, TCP driver-bridge loopback, and a compile check of the Unity scripts against a UnityEngine stub.

Plots are in results/*.png, results/fishtail/, results/kickback/ and results/driver/.

## Proving ground
The demo's proving layout includes:
- a long pave straight with undulations (25 m ±90 mm, 9 m ±35 mm)
- single-side bumps and potholes
- crossfall/camber change, ruts and a bumpy corner
- a banked oval, a skidpad and a slalom

The demo spawns at the start hub on the dynamics pad (signposted; `[start] spot` in demo.ini). Keys 1–9 and 0 teleport to the spots: hub, start, skidpad, pave, undulations, oval, slalom, bumps, ruts, bumpy_corner. The list shows on the HUD for ~10 s at start, and F1 toggles it. F2 toggles the x-ray view, F3 cycles the HUD and F4 shows the force arrows. Screenshots are in results/shots/ (gitignored, because they render the private mesh).

**Perf overlay (F7, off by default).** It covers the last 10 s of physics ticks: mean, p99 and max per stage (the tyre phase is split into its wheel sub-stages), the share of ticks over the tick budget, the six worst spikes with their dominant stage and a cause tag (kerb = a rough or kerb surface under a tyre, load-jump = a wheel load changing faster than 200 kN/s, fast-steer = the wheel turning faster than 360°/s), the physics real-time factor (tick budget divided by the mean tick cost, and simulated time divided by wall time), tick rate × substeps, solver threads, the CPU model and thread count, the GPU name, render fps, the offload lane's table memory, and the backend for each stage. **F8** writes `perf_dump_<YYYYmmdd_HHMMSS>.txt` next to the executable: a `#`-commented summary followed by the per-tick CSV of the same window. Recording is always on in the demo build (`-DDC_PROF`, a fixed 16384-tick ring allocated at start). The overlay recomputes its statistics twice a second and allocates nothing while it runs. `DEMO_PERF=1` starts the demo with the overlay shown.

## Demo car setup (`[car] setup` in demo.ini)

- `setup = default` (shipped): the car file plus two proposal sets, each applied only where it was derived:
  - `propBR` on multibody-core cars (`solver = 1`): compliant arm-pivot bushings (kr 2e6, ka 5e5, kt 300) and an 8 kN/mm rack mount (docs/BUSHINGS.md).
  - `propD` on 185/60R14 cars: the TARMAC tyre study set (docs/TYRE_DATA.md).
  - The console prints which sets were applied. `hatch_fwd` (reduced core, 205 tyres) is unchanged.
- `setup = legacy`, or `--set car.setup=legacy`: the car file as-is, which is the earlier demo setup.
- An absent key means `legacy`. The headless harnesses (drivetest, drvhost, perf tests) therefore keep their baselines; pass `--set car.setup=default` to them for the demo setup.

## Proposal car variants (Windows build)

The Windows zip ships variants of each car next to the base files. Each variant is a complete setup, so drive it with
`setup = legacy`. With `default`, propBR/propD would be laid over it.
- `_tarmac` / `_tarmac_brush`: the project owner's TARMAC tyre (docs/TYRE_TARMAC.md).
- `_propF2` / `_propD`: tyre-study sets (docs/TYRE_DATA.md).
- Multibody-core cars only (`solver = 1`):
  - `_propV`: rotary-valve pressure states (docs/VALVE.md)
  - `_propB`: compliant arm-pivot bushings (docs/BUSHINGS.md)
  - `_propBR`: propB plus the rack mount (docs/BUSHINGS.md); the demo default on these cars
  - `_propC`: propshaft and diff-carrier inertia (docs/CARRIER.md)

## Driving it from code
See docs/DRIVER_INTERFACE.md. A bot, script or Python drives through finite-strength hands on the rim and pedals, over JSON lines (stdin lockstep host, demo TCP port or Unity bridge). The human can take over at any time.

## Fishtail, relaxation and kickback
See docs/FISHTAIL_KICKBACK.md for the root cause, the physical fixes, before/after metrics, the relaxation-length decision, and the kickback/undulation analysis with a source breakdown and the NA Miata check.

## Countersteer and sustained drift
See docs/DRIFT.md. In short:
- Zero input = hands off. The gamepad stick is linear to full lock at every speed, and the deadzone releases the hands. `column_inertia` (the steering shaft below the torsion bar) is 0.002 kg·m², down from 0.01.
- The front wheels track β_f to within about 3° rms (before: 4–29°). Plot: `results/countersteer/countersteer_before_after.png`.
- In the NA Miata, `tyre_falloff_speed` 1 → 4 (a value consistent with published passenger-tyre sliding-friction fits). A human-like driver (0.15 s delay, finite arms) now holds a 27° power drift for 12 s (before: 1.4 s). Plot: `results/drift/drift_mx5.png`.
- Commands:
  - `./build/drivetest counter [car] results/countersteer`
  - `./build/drivetest drift cars/mx5_na.ini results/drift`
  - `python3 tools/plot_drift_cs.py results`

