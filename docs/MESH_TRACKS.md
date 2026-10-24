# Mesh maps (drivable triangle-mesh tracks)

Besides the procedural proving ground and circuit, the demo, `drvhost` and `drivetest` can drive on any
triangle-mesh map: a collider used **as is** (its own triangles, no smoothing, no remeshing) plus optional
render-only scenery. Code: `demo/meshtrack.[ch]` (loader + height query), `demo/track.c`
(`track_load_mesh`, `track_select`), and `tools/meshdiag.c` (mesh checks).

## Layout of a map directory

```
<map>/track.cfg      name, files, spots, optional grip overrides
<map>/collider.bin   DCMESH01: the ground the tyres drive on (+ wall triangles, render only)
<map>/scenery.bin    DCMESH01: render-only scenery (optional)
<map>/line.csv       closed centre/racing line: "x,y,z" per row, '#' comments, metres
```

`track.cfg` keys:

| key | meaning |
|---|---|
| `name` | display name (HUD, help, F6 list) |
| `line`, `collider`, `scenery` | file names inside the map directory |
| `half_width` | nominal half-width of the road around the line, metres (autopilot recovery, off-track stats) |
| `spot0` .. `spot9` | `name x y yaw_deg`: teleport spots (keys 1-9, 0). A spot called `start` is the default spawn |
| `grip.<id>` | friction multiplier override for surface id `<id>` |

## DCMESH01

Little endian: `char magic[8] = "DCMESH01"; int32 nv, nt, reserved; double v[nv][3]; int32 tri[nt][3]; uint8 surface[nt]`.
Frame: x/y horizontal, z up, metres, right-handed (the same frame as the core). A converter from a y-up tool
frame must use a proper rotation, e.g. (x, y_up, z) -> (x, -z, y), never a mirror.

Surface ids are the `SURF_*` values of `demo/track.h`:

| id | surface | grip | colour |
|---|---|---|---|
| 0 | tarmac | 1.00 | dark grey |
| 11 | concrete (kerbstones, slabs) | 1.00 | light grey |
| 1 | kerb | 0.92 | red |
| 6 | pave / setts | 0.85 | brown-grey |
| 3 | rough | 0.75 | |
| 8 | gravel / sand | 0.70 | sand |
| 2 | grass | 0.62 | green |
| 9 | wall | (not ground) | pale grey |
| 10 | other | 1.00 | stone |

Grip multipliers are relative to dry dense asphalt; sources in `demo/meshtrack.c` (Wong, *Theory of Ground
Vehicles*, Table 1.4; Bosch *Automotive Handbook* tyre-road friction table; EN 1436 skid-resistance classes for
painted surfaces). The core's only surface property is the grip multiplier; surface roughness comes from the
geometry itself. Scenery may use render-only colour classes >= 32 (foliage, tyre stacks, fences).

## Height query

The tyre's lattice sampler asks for the height at (x, y). The answer is the **highest** non-wall triangle whose
XY projection contains the point: the exact plane height and exact face normal (downward-wound faces count as
ground with the normal flipped up). Stacked geometry (a kerbstone above the tarmac or grass it was modelled
over) therefore resolves to the top surface. Points in a crack between triangles of up to 5 cm (seams,
T-junctions) take the nearest triangle's closest point, so seams never open a hole. Anything further from every
triangle is off the map: `zmin - 20 m`, surface `other`, counted as a miss.

Walls are render only: the core has no body or wall contact yet, so a car that leaves the ground area drives
through walls. Vertical faces of the ground itself (ledges, kerbstone faces) are seen by the tyre as a height
step.

## Selecting a map

* demo: `--set track.layout=<proving|circuit|map dir|name under tracks/>`; **F6** cycles proving, circuit and
  every `<track.maps_dir>/<name>/track.cfg` (default `tracks/`) in place. The HUD shows the current map.
* `drvhost --layout <...>` and the driver protocol (`tools/dcdrive.py`, `spawn(..., layout=...)`) take the
  same names. `info` reports `"layout": "mesh"` and `"map": "<name>"`.

## Checking a mesh

`build/meshdiag <map dir> [band.csv]` reports per-surface triangle counts, degenerate, downward-wound and
vertical faces, welded open / non-manifold edges (total and inside the driving band), and scans the band
(`|d| <= half_width + 3 m`) at 5 cm for holes, bridged cracks, stacked layers and height steps.
