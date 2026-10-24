# DriveChain: physically based steering, suspension and tyre core

DriveChain has a C core with a Unity frontend (P/Invoke) and a raylib test drive. The target feel is BeamNG-style physicality with AC/LFS-style tarmac feel.

- **Numerics:** double precision, deterministic, symplectic Euler at 2 kHz with a 12-DOF Cholesky solve. There is no heap use in the step.
- **Conservation:** Newton-3 and energy accounting are checked every run.

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
- `ac_import.py`

`private/` (AC-calibrated car, mesh, private results), `dist/` and `results/shots/` are gitignored and are not part of the source.

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

The demo spawns at the start hub on the dynamics pad (signposted; `[start] spot` in demo.ini). Keys 1–9 and 0 teleport to the spots: hub, start, skidpad, pave, undulations, oval, slalom, bumps, ruts, bumpy_corner. The list shows on the HUD for ~10 s at start, and F1 toggles it. Screenshots are in results/shots/ (gitignored, because they render the private mesh).

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

