# DriveChain showcase (proof of concept)

Tarmac physics running outside Unity. The core is the native C++ vehicle solver behind a small C API (`tmcore/api/tms.h`).
It drives a car on a glTF world in two renderers:

- `web/`: three.js r187, WebGPURenderer with a WebGL2 fallback. The physics is compiled to wasm64 with Emscripten and runs in a worker at a fixed 240 Hz. It is a static folder: serve it with any HTTP server.
- `native/`: Rust + ash (Vulkan 1.3). It links the same C++ core through `cc`, and has an optional VK_KHR_ray_query path for RTX cards.

## What is NOT in this repository (kept local and private)
- The Tarmac-Lab-Beta solver sources (`tmcore/upstream/`), the stand-in headers (`tmcore/shim/`) and the compiled wasm (`web/wasm/`).
- The Ušće world mesh and textures, and the miata_ac car mesh (`private-assets/`, linked as `web/private`). These are mod-derived.

Without them the web app falls back to its own procedural track and coupe (`web/js/proc-world.js`, `car.js`), but it still needs `web/wasm/tarmac.{mjs,wasm}`.
To build the wasm and native core, put the solver at `tmcore/upstream` (or set `TARMAC_SRC`) and run `tmcore/build_wasm.sh`.

## Car: stock 1990 Mazda MX-5 (NA6)
The C API ships one car, `TMS_CAR_MIATA_NA` (3). Its gearbox, engine curve, open diff, spring, damper and ARB rates, brakes and hydraulic clutch come from public sources, cited in `tmcore/api/tms.cpp`:
- Moss Miata 1990 spec sheet
- Mazda's 1990 press specifications (hosted by miata.net)
- topofsecond.com
- DriveChain's own `cars/mx5_na.ini`

Limits of the swap:
- The core's chassis mass, inertia, tyres and suspension geometry are its frozen baseline (1200 kg), and this ABI cannot set them.
- The core requires springs of at least 18 kN/m and dampers of at least 1.2 kN·s/m. The softer stock Miata rates (12.5/10.1 kN/m) are therefore clamped to that floor.

## World loader (swappable)
Any glTF/GLB works: `?world=path/to/world.glb`.
- Meshes named `COL_*` are collision only and `VIS_*` are visual only. Everything else is both.
- Optional `extras.tarmac` carries the surface type, spawns and racing line.
- `tools/dcmesh_to_glb.py` converts the old DriveChain dcmesh format.

## Web: run
```
cd web && python3 -m http.server 8765   # open http://localhost:8765/
```

Controls: keyboard WASD or arrows, Space handbrake, Q/E gears. Gamepads and wheels: see **Input and mapping** below.

Keys:
- 1–4: tier (RAW / BASE / HIGH / SHOWCASE)
- V: camera
- P: autopilot
- K: shadow cache on/off (A/B)
- I: far impostors
- T: sun
- G: wet
- R: reset
- H: help

URL params: `tier, webgl, world, auto, sunel, sunaz, exp, gi, fog, wet, cam, dpr, still, warm`.
Vegetation toggles: `trees=0, shrubs=0, grass=0, grassdensity=24 (blades/m², same as native), grassdist=45 (m), grassmax=160000, treeshadows=0, impdist=70 (m, mesh→impostor), vegplan=<url>, freeze (stops grass wind)`.

Tiers:
- RAW: unlit look, as in the old version.
- BASE: PBR materials from the generated textures, physical sky, a cached VSM sun shadow plus a per-frame car-only shadow, ACES, bloom and SMAA.
- HIGH: BASE plus half-res SSGI and the far-field impostor cube cache.
- SHOWCASE: full-res SSGI plus god rays.

Design rules followed (Threat Interactive handbook):
- No TAA dependence: SMAA only, and SSGI temporal filtering is off.
- Static work is cached: the sun shadow and the far ring of the city in a cube impostor, refreshed one face per frame.
- GI runs at half resolution.
- Frame timings are shown in the HUD (GPU ms via timestamp queries, physics µs/tick).

## Known limits (headless verification)
- Verified headless only in Chrome + SwiftShader via WebGL2. The WebGPU path initialises there, but SwiftShader loses the device. It falls back to WebGL2 automatically.
- God rays hang the SwiftShader WebGL2 main thread, so the stills use `godrays=0`. God rays are unverified on a real GPU.

## Native: build
- Linux: `cd native && cargo build --release`
- Windows cross-build from Linux: `native/scripts/build_windows.sh`
- On Windows itself (MSVC + Vulkan SDK): `native/scripts/build_windows.ps1 [-Dlss]`

Run modes:
- `drivechain-native [world.glb]` opens a window (default world `../private-assets/usce/usce_v2.glb`, falls back to `usce.glb`).
- `--still out.png --size 1920x1080` renders a headless still; `--cam x,y,z --look x,y,z`, `--cam-rel back,up,side`, `--fov`, `--seq N [--dt s] [--cam-step x,y,z]` (frame sequences for flicker tests), `--grain`, `--sun-el/--sun-az`.
- `--physics-bench 20` benchmarks the physics.
- `--input-test 30` (no GPU) lists controllers and prints raw axes/buttons and the mapped controls for 30 s.

Vegetation toggles (native): `--no-trees --no-shrubs --no-grass --grass-density 24 --grass-dist 70 --max-blades 1500000 --no-tree-shadows --imp-dist 90 --imp-band 0.2 --old-foliage`.
Image-stability toggles: `--taa on|off` (default on), `--taa-sharpen 0.1`, `--aniso 16` (clamped to the GPU limit), `--spec-aa on|off`
(default on), `--present fifo|mailbox|immediate|relaxed` (default fifo = vsync; falls back to FIFO when unsupported). In the window:
**F5** TAA on/off, **F8** spec-AA on/off.
At startup the log (`drivechain.log` next to the exe, and stderr) prints the GPU, driver version, swapchain format, present mode (and the
supported ones), the material switches and the TAA/spec-AA/aniso state, and the vegetation setup; every 600 frames it logs the frame time
(mean / p99 / max: frame pacing) and the GPU ms per pass (also in the window title), including `grass compute` and `foliage+grass draw`.

### Material groups (render only what you need)
Groups: `leaves bark impostors shrubs grass markings asphalt kerbs terrain props car`. Each group has a mode, `on`, `flat` (unlit,
textured), `debug` (unlit solid colour) or `off` (culled from every pass, shadows included), plus overrides `opaque` (no alpha test),
`nomips` (mip 0 only), `noshadow` (casts no shadow) and `nospec` (no specular).
- CLI: `--mat leaves=off,grass=flat,impostors=debug+nomips` (`all=` sets every group; `+flag` adds an override).
- `--isolate leaves[,bark,...]` renders only those groups. Everything else is culled, and GI, volumetrics and bloom are skipped with a cheap sky.
  Add `--crop x,y,w,h` (a window of the `--size` frame; same pixels, same projection) for still/sequence tests. On lavapipe a
  12-frame tree sequence takes 8 s instead of 70 s for the full scene (GPU 208 ms vs 2302 ms per frame).
- `render.ini` next to the exe: `[materials]` with `leaves = flat+opaque`, `isolate = leaves,bark`, ... (the CLI wins).
- Keys: **F3** next group, **F4** next mode (on → flat → debug → off), **F6** pick an override, **F7** toggle it on the group, **F9** reset all.
  The current state is shown on screen and logged to `drivechain.log` on every change.

Foliage-flicker bisect recipe (Jakov's rig, in the window, standing still then driving slowly past trees):
1. F5 (TAA off): if the boiling is clearly worse, TAA was working and the rest is non-temporal aliasing; F5 back on.
2. F3 to `leaves`, F4 → `flat`. If the boiling stops, it is lighting (shadows/GI/spec) on the leaves; if it stays, it is coverage (alpha).
3. Still on `leaves`: F6 to `opaque`, F7. Solid cards = no alpha test. If the boiling stops, it is alpha-test aliasing.
   F7 again, F6 to `nomips`, F7: much worse = the mips are doing their job.
4. F6 to `noshadow`, F7 on `leaves` and on `impostors`: if it stops, it is leaf self-shadowing (cascade/RT shadow noise).
5. Same for `impostors` (far trees) and `grass`. `debug` mode shows which group a flickering pixel belongs to.
6. Note the steps that change it (the log has every state) and send `drivechain.log`.
Headless, the same on a soft GPU: `tools/flicker_native.sh <name> [args]` (12-frame sub-pixel dolly, foliage metrics),
`tools/flicker_mask.sh <name> <groups>` (pixel mask of groups from the debug mode), `MASK=scratch/flk/mask_<name>.png tools/flicker_native.sh ...`.

Passes, each timed with GPU timestamps:
1. G-buffer
2. Shadow cascades (near per-frame, far cached)
3. Ray-query shadow mask (RT only)
4. Direct light
5. Screen-probe GI (SSRT3-style, half-res, ray query when available; not used on foliage, which gets a world-locked canopy AO instead)
6. Composite with height fog and physical sky
7. TAA (Halton 2,3 jitter, motion vectors incl. wind, closest-depth velocity, Catmull-Rom history, YCoCg variance clip, reset on camera cuts)
8. Bloom
9. Tonemap (+ light TAA sharpen)
10. AA, lens and grain

## Vegetation (web + native)
All old Ušće foliage geometry is gone: `tools/usce_veg_prep.py` writes `usce_v2.glb` (private) without `VIS_foliage` (35.5k tris of tree/hedge
blocks) and without the 11,023 grass/bush "cards" inside `VIS_other` (44k tris: 2-8 triangle crossed quads, authored for an alpha shader
we don't have, so they drew as black/flat slabs). Props (tents, boards, barriers) stay. It also writes `veg_plan.json`: 2936 trees + 131
shrubs, seed 1990, at the original foliage positions, trees >= 3.5 m from tarmac/kerb/line, shrubs never on the road.
- Kinds (same presets + seeds on both renderers, `web/js/trees.js` KINDS): poplar ×2 (aspen structure with green ash leaves: the aspen leaf
  texture in ez-tree is autumn yellow), ash ×2, oak, pine; shrubs = ez-tree bush_1/2/3 (short trunk, dense branching).
- Near: instanced ez-tree meshes (web 70 m, native 90 m; shrubs 60 %), shadow casting, wind in the vertex shader (native).
- Far: hemi-octahedral impostors (8×8 views, albedo+alpha and normal atlases), lit at runtime. Native: alpha mips are
  coverage-preserving (Castaño), so canopies don't thin out or shimmer; far trees cast shadows as sun-facing impostors; foliage gets its own
  slope-scaled depth bias in the shadow cascades.
- Native assets: `tools/bake_vegetation.mjs` runs `web/bake_veg.html` headless and writes `native/assets/vegetation/` (meshes .bin,
  bark/leaf textures, impostor atlases, `vegetation.json`), generated from ez-tree (MIT). `DCS_VEG` / `DCS_VEGPLAN` override the paths.
- Grass: blades only on grass collision surfaces (COL_grass) and never where a road surface lies above the grass mesh. Look after
  Velorexe/unity-geometry-grass-shader (Unlicense: dense, tall blades, segmented bend, per-blade random facing/height/width/bend, wind,
  base→tip gradient), rebuilt without geometry shaders: native `grass_gen.comp` generates the blade buffer per frame for 8 m cells
  around the camera (hash-stable density falloff, frustum culled) and draws it with one indirect instanced draw (`grass.vert` builds a
  5-segment blade from `gl_VertexIndex`, widened to >= 1.2 px at distance against shimmer); web `js/grass.js` is the same blade model as
  an instanced mesh with vertex-shader wind.
- The RAW tier (web) shows no vegetation (the old look minus the broken blocks).

## Input and mapping
Native (Windows): XInput for Xbox-style pads (polled every frame, works without window focus, hotplug checked every 2 s) and
DirectInput 8 for wheels, pedal boxes, shifters and other joysticks (re-enumerated every 2 s). gilrs is no longer used on Windows: it
only read the first device (on a wheel rig usually the wheel) through Windows.Gaming.Input, which is why the pad did nothing.
Linux uses gilrs (evdev). Every device is logged at startup and on hotplug to `drivechain.log` (name, GUID, backend, axes, buttons;
DirectInput also reports its own axis/button/POV counts and force-feedback flag).

- **F1** (or pad Back/View) toggles the input overlay: every device with live raw axis values and held buttons, plus the mapped
  steer/throttle/brake/clutch. **F2** reloads `input.ini`. `C` cycles the camera (chase, far chase, bonnet), `M` toggles auto/manual gearbox.
- Default pad mapping = the original DriveChain (`demo/input.c`): left stick steer (deadzone 0.04, linear), RT throttle, LT brake
  (0..1, linear, no deadzone), B handbrake, X clutch, RB/LB gear up/down (switches to manual), Y reset, right stick click camera, Back overlay.
  An old `input.ini` (no `version = 2`) is replaced by the new defaults on start; the old file is kept as `input.ini.v1.bak`.
- Xbox pads are always read through XInput, never as a DirectInput "wheel": DirectInput devices whose HID path carries `IG_`
  (= XInput device) are skipped and logged. Pads that only exist in DirectInput (DualShock/DualSense, generic USB pads, third-party
  Xbox clones) get a pad profile with the triggers split correctly (Xbox-style DI pads report both triggers on one combined Z axis:
  LT = Z > 0, RT = Z < 0; Sony: L2/R2 on Rx/Ry). Wheel axes that the device doesn't have read as unbound instead of half throttle.
  Tests: `cargo test --release input` (simulated XInput/DirectInput states) and `node tools/input_unit.mjs` (web).
- `input.ini` is written next to the exe on first run. `[pad]` uses names (LX LY RX RY LT RT, A B X Y LB RB Back Start LS RS DUp DDown
  DLeft DRight, `-` prefix inverts an axis). `[wheel]` uses raw DirectInput numbers: `axis0..7` = X, Y, Z, Rx, Ry, Rz, Slider0, Slider1,
  `button0..31`, POV hat = `button32..35` (up, right, down, left). Prefix a binding with part of another device's name to read it from
  that device, for separate pedals or shifters, e.g. `throttle = CSL Pedals/axis1`. `device =` picks the wheel by name or GUID
  substring (empty = first DirectInput device). Pedals: `*_invert = 1` when they read +1 released (most do), `pedal_deadzone`.
  Steering: `steer_invert`, `steer_deadzone`, `steer_gain` (e.g. 1.5 when the wheel is set to 900° and the car's rack is about 540°).
- How to map a new wheel: run `drivechain-native.exe --input-test 60` (or press F1 in game), move each control and note which
  `aN` changes and in which direction, then write those numbers into `[wheel]`. Defaults are the Logitech G25/G27/G29/G920 layout
  (steer axis0, throttle axis1, brake axis5, clutch axis6, paddles button4/5). Thrustmaster T300 combined pedals: throttle axis5, brake axis1.
- Force feedback is not implemented yet (the wheel is read, not driven).

Web: Gamepad API with the same model. Standard-mapped pads use the named mapping; wheels and pedals (non-standard mapping) use raw
`axisN`/`buttonN` with the same `[wheel]` keys. F1 or pad Back toggles the overlay. Override the mapping with
`localStorage.setItem('dcs.inputMap', JSON.stringify({wheel: {throttle: 'axis2', brake: 'axis3'}}))` (or `window.__dcs.input.saveMap({...})`,
or `?inputmap=<url-encoded JSON>`); unset keys keep the defaults. Browsers hide pads until the first button press.

## Credits and references
- ez-tree by Daniel Greenheck (MIT): https://github.com/dgreenheck/ez-tree, vendored at `web/vendor/ez-tree` with its LICENSE; the native vegetation assets in `native/assets/vegetation` are generated from it (bark/leaf textures are ez-tree's own, MIT).
- Grass look: Velorexe/unity-geometry-grass-shader (Unlicense): https://github.com/Velorexe/unity-geometry-grass-shader. Technique reproduced (compute + instancing instead of geometry/tessellation shaders), no code copied.
- Coverage-preserving alpha mips: I. Castaño, "Computing Alpha Mipmaps" (2010).
- SSRT3 (screen-space GI idea): https://github.com/cdrinmatane/SSRT3 — no code copied.
- SmartImposter (impostor caching idea, Rust/ash reference): https://github.com/SamG-Coder/SmartImposter — no code copied.
- dlss5-visual-enhancer (https://github.com/Merserk/dlss5-visual-enhancer): evaluated. It is an offline image/video enhancer, not a real-time pass, so it is not used at runtime.
- Textures: generated, see `web/assets/textures/CREDITS.md`.
