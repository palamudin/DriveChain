# DriveChain test drive — Windows x64

A physically based steering, suspension and tyre core (C, double precision, 2 kHz) driven by a raylib 3D test drive.
Physics runs on its own 1 kHz thread (2 kHz core substeps), separate from rendering. Input and force feedback go through SDL2.

> PRIVATE BUILD: `assets/miata_mesh` and `cars/miata_ac.ini` come from a third-party mod and are only for your own testing.
> Do not redistribute this zip.

## Run
Unzip anywhere and double-click `drivechain_demo.exe`. Run it from its own folder, because paths are relative to the working directory.
- `drivechain_demo.exe --config demo.ini` (default `demo.ini`)
- `--set key=value` overrides any ini key (repeatable), e.g. `--set camera.mode=cockpit --set ffb.gain=0.7`
- `--audio-wav out.wav --time 30` records the procedural audio offline.

The console window prints the joysticks it found, which one it picked, and whether constant-force FFB opened (`[ffb] constant force OK`).

## Controls
| Action | Keyboard | Wheel / pedals (SDL joystick) | Gamepad |
|---|---|---|---|
| Steer | ← → / A D (let go = hands off, so caster and SAT centre the wheel) | wheel axis, 1:1 with the car's steering wheel | left stick |
| Throttle / brake | ↑ / W, ↓ / S | throttle / brake axes | RT / LT |
| Clutch | Left Shift | clutch axis | X |
| Handbrake | Space | `handbrake_axis` or `handbrake_button` | B |
| Gear up / down | E or PgUp / Q, Z or PgDn | H-shifter buttons, or paddles `shift_up/down_button` | RB / LB |
| Reset to track | R | | |
| Puncture (front-left) | P | | |
| Hydraulic assist on/off | H | | |
| Autopilot | O | | |
| Pause | F5 / Pause | | |
| Camera (chase, cockpit, wheel, orbit, tv) | C (orbit: drag with the mouse, wheel to zoom) | | |
| X-ray suspension / HUD / force arrows | F2 / F3 / F4 | | |

## Gamepad steering (`[gamepad]`)
- By default the stick does not set the rim angle. It sets where your **hands** want the rim, and finite-strength arms push the rim there.
  - The arms are a PD of `arms_k` 10 N·m/rad and `arms_d` 1, capped at `arms_tmax` 20 N·m.
  - Self-aligning torque can therefore still catch a slide, as a real driver's relaxed grip allows. This removes the left-right fishtail.
- `range_highspeed` / `range_speed` limit how much lock the stick reaches at speed (25 % at high speed, halfway point at 12 m/s). Other keys are `deadzone` 0.04 and `expo` 0.5.
- `mode = raw` restores the old direct mapping, kept for comparison. FFB is unaffected either way: it is always the physical column torque.

## Letting a bot or script drive (`[driver]`)
- Set `port = 7878` (or run with `--drive-port 7878`). Press **G** to toggle the plug-in driver; the HUD shows DRIVER ENGAGED and its path.
- A script then sends JSON lines over TCP, e.g. with `python tools/dcdrive.py run lane_change --tcp 127.0.0.1:7878`.
- The bot only applies torque to the rim through finite-strength hands, and works the pedals. The physics is never bypassed.
- Any deliberate input from you (stick > 0.25, wheel > 45° from the bot's rim, pedal > 0.15) takes over instantly.
- `drvhost.exe` is the headless lockstep host used by `dcdrive.py`'s default spawn mode. The protocol is in `docs/DRIVER_INTERFACE.md`.
- Keep `bind = 127.0.0.1`.

## Proving ground spots
Keys **1–9** teleport to: start, skidpad, pave, undulations, oval, slalom, bumps (single-wheel bumps/potholes and camber change), ruts, bumpy_corner.

## Gearbox and clutch
- **H-shifter:** set `gear1_button` … `gear7_button` and `reverse_button`. If no gear button is held, the box is in neutral.
  - With an H-shifter configured, `auto_clutch` defaults to off: use the clutch pedal, or the engine stalls or the gears crunch, just like the car.
  - Defaults are for a Logitech G29/G920 shifter (buttons 12–18).
- **Sequential (paddles/keys):** `auto_clutch=1` blips the clutch for you.
- **Mapping help:** with a wheel connected, the HUD's second input line shows live axes (`0:+0.12 …`), pressed button numbers, the H gear and the FFB output level.
  - Move each pedal and shifter position and read the numbers off.
  - Pedals that read +1 at rest need `*_invert = 1`, which is the default.

## Steering rotation
- `input.rotation_deg` (default 900) must equal the rotation set in your wheel's driver, e.g. Logitech G HUB, Fanatec, Simucube or Moza. The game then maps the wheel 1:1 to the car's steering wheel.
  - Wheel angle = axis × rotation / 2.
  - The car keeps its own lock. The MX-5's rack runs into its stops at the car's `steer_lock_deg` (504° lock-to-lock for the bundled Miata).
- For the most realistic feel, set both the driver rotation and `rotation_deg` to the car's lock, e.g. 504.
  - At 900°, the rack stop arrives before your wheel's mechanical stop, and FFB pushes back at the car's lock exactly as the rack end-stop does.

## Force feedback (constant force)
- The FFB signal is the real torque in the physical steering column, not a canned effect. It is made up of:
  - tyre self-aligning torque
  - mechanical trail
  - jacking
  - kerb and bump impacts
  - rack friction
  - hydraulic assist (when on)
  - the rack end-stops
- `[ffb]` options:

| Key | Meaning |
|---|---|
| `enabled` | 1/0 |
| `gain` | Overall scale. Output = gain × column torque / `max_torque_nm`, clipped to ±100 %. |
| `max_torque_nm` | Column torque (N·m) that equals 100 % of your base. Lower = stronger. Around 8 suits a G29/T300 with assist on. Use about 12–20 for direct-drive bases, or with assist off (H): a manual-rack MX-5 has much heavier column torque. |
| `min_force` | Deadband compensation (0–0.2). It lifts small forces past your wheel's internal friction (gear/belt wheels about 0.03–0.08; direct drive 0). |
| `invert` | Flip the force direction if the wheel pushes away from centre. |
| `smoothing_hz` | Low-pass on the output (0 = off). 80–150 removes 2 kHz hash on gear wheels. Direct drive can run 0 or 300+. |

- Set your wheel driver to linear, 100 % strength, with the driver's own centring spring and damper **off**. Self-centring is physical and comes from caster and SAT.
- Gamepads get rumble from tyre slip and kerbs.

## Display
- `[display]` options:
  - `width`, `height`, `fullscreen`
  - `vsync` (default 0; physics is decoupled from frame rate either way)
  - `fps_cap`
  - `msaa`
  - `fov_v_deg` (vertical FOV, so ultrawide 21:9 / 32:9 just widens the view)
- **Triple screens:** span the window across all three monitors (`width = 3×panel`) and set `triple = 1`.
  - Each third renders its own frustum, yawed by the panel's horizontal FOV plus `bezel_deg`, or by an explicit `triple_side_deg`.
  - Applies to the cockpit camera.
- **Cockpit eye:** `[camera] cockpit_dx/dy/dz` (metres from the steering-wheel centre) and `cockpit_pitch_deg`.

## Files
- `drivechain_demo.exe`: the test drive. Needs `SDL2.dll` beside it.
- `drvhost.exe`, `tools/dcdrive.py`, `unity/*.cs` and `docs/`: the driver plug-in host, the Python client, the Unity scripts and the docs.
- `drivechain.dll`: the physics core as a C ABI DLL with an opaque handle, for the Unity P/Invoke binding in the repo.
- `demo.ini`: all settings above.
- `cars/`:
  - `miata_ac.ini`: hardpoints, springs, dampers, tyres, steering and engine, calibrated from the AC data.
  - `mx5_na.ini` and `hatch_fwd.ini`: reference cars. Edit `car.ini` to switch.
- `assets/miata_mesh/`: the car mesh (LOD B), with wheels, calipers/hubs and the steering wheel split out so each is posed from the physics.
