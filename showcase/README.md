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

## World loader (swappable)
Any glTF/GLB works: `?world=path/to/world.glb`.
- Meshes named `COL_*` are collision only and `VIS_*` are visual only. Everything else is both.
- Optional `extras.tarmac` carries the surface type, spawns and racing line.
- `tools/dcmesh_to_glb.py` converts the old DriveChain dcmesh format.

## Web: run
```
cd web && python3 -m http.server 8765   # open http://localhost:8765/
```

Controls:
- Keyboard: WASD or arrows, Space for the handbrake, Q/E to shift gears.
- Gamepad: standard mapping.
- Steering wheel: Gamepad API. Press C to calibrate.

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

## Native: build
- Linux: `cd native && cargo build --release`
- Windows cross-build from Linux: `native/scripts/build_windows.sh`
- On Windows itself (MSVC + Vulkan SDK): `native/scripts/build_windows.ps1 [-Dlss]`

Run modes:
- `drivechain-native [world.glb]` opens a window.
- `--still out.png --size 1920x1080` renders a headless still.
- `--physics-bench 20` benchmarks the physics.

Passes, each timed with GPU timestamps:
1. G-buffer
2. Shadow cascades (near per-frame, far cached)
3. Ray-query shadow mask (RT only)
4. Direct light
5. Screen-probe GI (SSRT3-style, half-res, ray query when available)
6. Composite with height fog and physical sky
7. Bloom
8. Tonemap
9. AA, lens and grain

## Credits and references
- SSRT3 (screen-space GI idea): https://github.com/cdrinmatane/SSRT3 — no code copied.
- SmartImposter (impostor caching idea, Rust/ash reference): https://github.com/SamG-Coder/SmartImposter — no code copied.
- dlss5-visual-enhancer (https://github.com/Merserk/dlss5-visual-enhancer): evaluated. It is an offline image/video enhancer, not a real-time pass, so it is not used at runtime.
- Textures: generated, see `web/assets/textures/CREDITS.md`.
