# Driver interface plug-in

This plug-in lets a bot, a script or Python drive the car the way a human does. It applies **torque to the steering rim through finite-strength "arms"** and works the pedals; it never sets wheel angles or body states. This means the self-aligning torque, kickback and the valve still act on the bot's hands, and the physics under test is never bypassed. A human can take over at any instant.

## Core: `dc_driver` (core/src/dc_driver.c, C ABI)
- **Hands:** a PD on the rim angle, `T = clamp(k·(θ_cmd−θ) − d·ω, ±tmax)`. Defaults: k 25 N·m/rad, d 1 N·m·s/rad, tmax 20 N·m. The stiffness is the measured driver–wheel arm stiffness, 10–46 N·m/rad, up to ~80 with co-contraction (Xing et al. 2018, IEEE/ASME Trans. Mechatronics, doi:10.1109/TMECH.2018.2812643). The old 10 N·m/rad sat at the bottom of that range and let the path driver hunt ±0.7 m against on-centre joint friction (docs/JOINT_FRICTION.md).
- **Command shaping:** human-like reaction delay (0.18 s), bandwidth (3 Hz) and rim rate limit.
- **Steer modes:**
  - `off`
  - `hands_off`: zero torque
  - `torque`: raw rim torque, for "directional force" tests
  - `angle`: rim target, optionally with a sine `wave_*`
  - `path`: pure pursuit plus predictive aim, yaw damping, a rear-slip catch with a 0.03 deadband, and a conditional lateral trim
  - `locked`: a very stiff hold
- **Pedals:**
  - `off`
  - `direct`: throttle/brake/handbrake
  - `speed`: speed PI
- **Determinism and allocation:** deterministic, with no heap use in the step.

## Protocol: JSON lines
One JSON object per line goes in, and one reply line comes out. There are three transports:

| Transport | How | Clock |
|---|---|---|
| `build/drvhost` (Windows: `drvhost.exe`) | stdin/stdout | Lockstep: time only advances on `step` (1 ms ticks). Deterministic. |
| Demo `--drive-port N` or `[driver] port` | TCP, bound to 127.0.0.1 by default | Real time. Press G to toggle; the HUD shows DRIVER ENGAGED. |
| Unity `DriveChainDriverBridge` (port 7878) | TCP | Unity FixedUpdate |

Keep the bind on loopback: any process that can reach the port can drive the car. Over TCP, `log` only accepts relative file names (no `..`, `\`, `:` or absolute paths).

| cmd | fields | reply |
|---|---|---|
| `hello` / `info` | | version, lockstep flag, named spots |
| `reset` | `spot` (skidpad, slalom, …) or `x,y,yaw_deg` or `s,d`; `speed_kmh`, `gear`, `keep_driver` | ok |
| `path` | `type`: `track` / `circle` (`cx,cy,r,dir`) / `line` (`x,y,yaw_deg,length`) / `points` (`pts:[x0,y0,x1,y1,…]`, `closed`) | ok, n |
| `drive` | `steer`, `pedals`, `target_deg`, `torque`, `wave_amp_deg/freq/delay`, `offset`, `throttle`, `brake`, `handbrake`, `speed_kmh`; tuning `arm_k, arm_d, arm_tmax, delay, bandwidth_hz, rate_max_dps, preview_time, preview_min, yaw_damp, catch_gain, lock_frac, lat_ki, predict, speed_kp/ki, brake_kp`; `engage` (default 1) | ok |
| `engage` | `on` | engaged |
| `gear` | `gear` | |
| `step` | `n` ticks or `seconds` (drvhost only) | telemetry |
| `get` | | telemetry: t, x, y, z, yaw, v, yaw rate, β, ax/ay, sw angle/rate, column/hand torque, path error, engaged…; per wheel [FL,FR,RL,RR]: fz, fx, fy, mz, alpha_deg, kappa, travel_mm, bumpstop_n, deflection_mm, rim_n, travel_vel, sliding, contact_z, omega |
| `metrics`, `metrics_reset` | | rms/max lateral error, peak β, saturation |
| `stream` | `hz` | Pushes telemetry lines (TCP). |
| `log` | `file`, `every` | CSV trace |
| `set` | `key: "car.assist"`, `value` | The car is rebuilt in place. |
| `quit` | | |

Example:
```
{"cmd":"reset","spot":"skidpad","speed_kmh":40}
{"cmd":"path","type":"circle","cx":0,"cy":30,"r":30,"dir":1}
{"cmd":"drive","steer":"path","pedals":"speed","speed_kmh":40}
{"cmd":"step","seconds":10}
{"cmd":"drive","steer":"torque","torque":4}     # push the rim with 4 N·m
```

## Takeover
- With `takeover = 1`, any deliberate human input releases the bot immediately: stick above 0.25, wheel moved more than 45° from the bot's rim, or pedal above 0.15. The wheel threshold is `takeover_wheel_deg`: the physical wheel held more than that far from the sim rim.
- While engaged, the human's hands are off the rim: the bot's torque is the only driver input.
- The bot also disengages when the last TCP client disconnects.

## Python client: `tools/dcdrive.py`
```
python3 tools/dcdrive.py list
python3 tools/dcdrive.py run skidpad --speed 50 --out results/driver
python3 tools/dcdrive.py selftest --out results/driver          # make drvtest
python3 tools/dcdrive.py run lane_change --tcp 127.0.0.1:7878   # live demo or Unity
```
`selftest` runs these manoeuvres with pass bounds: skidpad, lane_change, slalom, track_lap, step_steer (angle mode), lift_off and bump_hold (locked hands over a bump). Results go to results/driver/driver_summary.csv, driver_paths.png and driver_events.png.

`lift_off` enters the 40 m skidpad circle at ≤ 30 km/h and ramps the speed target to the test speed (default 55 km/h,
0.60 g) over 8 s, holds 4 s, then lifts. It reports `pre_lift_beta_deg`, `pre_lift_sw_range_deg` and `pre_lift_ok`
(|β| < 5° and SW range < 20° over the last second before the lift). An earlier version reset the car directly at
speed on the circle: the path driver was still oscillating at the lift, and at 58–60 km/h that turn-in transient alone
spun the car, in both cores and with both tyres. With the ramp-in entry, 50–60 km/h runs show post-lift β 1.0–2.0° and
no spin for MB and legacy cores and for the brush and TARMAC tyres.

## Unity
- Add `DriveChainDriverBridge` next to `DriveChain` (unity/DriveChainDriverBridge.cs). It listens on 7878, G toggles it, and it speaks the same protocol (`hello/info, drive, engage, path, get, stream, quit`).
- The engine-independent `DriverBridge` class is tested by `unity/AbiTest.cs` (TCP loopback: drives a 2 m line change, checks takeover and rejects bad modes) under `make unitytest`.
