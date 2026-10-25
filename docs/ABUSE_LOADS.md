# Abuse loads: kerb strikes, wall strikes, drops (`make crashtest`)

The car must survive what a driver does to it off the racing line, on both cores, without clamps: a square kerb
taken head-on, a wheel or the body driven into a wall, a ledge drop. Faults found driving and their physical fixes:

| Fault | Root cause | Fix |
|---|---|---|
| Body sank into walls / pushed up them | body contact penalised only vertical penetration | **minimum-translation depth**: 16 horizontal directions plus vertical, bisection to the free surface; normal = that direction (`dc_body_depth`) |
| Body bounced off walls elastically | linear penalty stored the whole crash energy and gave it back | **elastic-plastic crush** (`dc_body_normal`): elastic `body_k` up to a yield force `body_fy`, linear hardening `body_kh`, permanent set per contact point (crush-zone force-deflection after Campbell's linear crush model; ~40 kN yield per point) |
| Tyre climbed vertical faces taller than itself | column potential kept growing with face height | **chord clip**: a column filled to the disc top adds no area or force |
| Wall strike created energy (up to ~12 kJ) | rim force applied along the tyre normal, and the face gain capped at 4, so the forces were not the gradient of the stored energy | rim force along its own area gradient with its own station moment, gain uncapped: forces are again the exact gradient of `t->pe` |
| Linkage driven to toggle on a 1 m drop, then blow-up | bump stop modelled as an unbounded progressive spring | **solid stop** `bumpstop_solid` (40 mm past bump-stop contact, kept ≥ 20 mm inside the valid linkage range): rigid inelastic speculative unilateral row (closing speed ≤ gap/h) |
| Rack overran its compliant end stop | no rigid limit | **rack end stop** `rack_stop_solid` (3 mm past the compliant stop, steel on steel), same rigid row; linkage scanned to that travel |

Both rigid limits act through the full mass matrix (wheel or rack AND body react: Newton 3); the kinetic energy
they remove is counted as dissipation.

## Energy audit for contacts
- Reduced core: body-contact loss = work the contact forces did on the chassis over the step (with the velocities
  the integrator actually used) minus the change of their elastic store. Exact for the discrete step, so a contact
  that returned more than it took would show up as negative loss. Note: this core's `dissipated` does not include
  tyre slip losses, so KE+PE+dissipated drifts down slowly while rolling.
- Multibody core: body-contact work is no longer booked as external work; its elastic store is in the potential
  and its loss appears in the audited residual (dissipation).

## Suite (tests/crash_suite.py, tool build/kerbstrike)
- kerb 135 mm square, head-on, 25 / 40 km/h: peak single-tyre load bands 8–30 / 15–45 kN; 50 mm at 5°, 60 km/h: < 8 kN.
- wall: heights 0.3/0.5/1.0/1.3 m × 10/30/90° × 30/60 km/h, coasting in neutral (no energy source): all finite,
  KE+PE never above its starting value by more than 1 % of the initial KE (an energy-creating contact fails this).
- drop: 0.5/1.0/1.5 m × 20/50/80 km/h: finite, KE gain ≤ 1.05 m g h, travel bounded by the solid stop.

## Kerb peak loads
A 135 mm square step taken head-on is a rim clash, not a tyre event: the rim pinches the carcass on the step edge.
Measured here: ~19–21 kN at 25 km/h, ~31–35 kN at 40 km/h (both cores), against ~2.4 kN static. Published cleat /
pothole-style tests of passenger tyres show the same regime once the rim bottoms (peaks of several times static
load, rising with speed), while a tyre that envelopes the obstacle without bottoming stays at a few kN, which is
what a 50 mm kerb gives here (3.9 kN at 60 km/h). Rim yield (a bent rim absorbing the strike) is not modelled.

## Cost
Lane columns (4 per tread row) roughly quadrupled contact integration; with the lookup caching and block culling the
step is ~100 µs on the reference box, 5x real time at 2 kHz. The perf test budget is 4x real time (< 125 µs).
