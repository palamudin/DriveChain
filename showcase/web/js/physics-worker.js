// physics-worker.js - runs Jakov's Tarmac core (wasm64) at a fixed 240 Hz off the main thread.
// Main thread posts {type:'step', dt, input}; the worker advances floor(acc*240) ticks with that input and
// replies with the packed 80-float state (see tms.h). One message per rendered frame, no shared memory needed.
import { Tarmac } from './tarmac-core.js';
let T = null, world = 0, car = 0, acc = 0, spawn = null, carId = 1, tune = 0, busyMs = 0;
const HZ = 240, MAX_TICKS_PER_FRAME = 24;
self.onmessage = async (e) => {
  const m = e.data;
  try {
    if (m.type === 'init') {
      const t0 = performance.now();
      T = await Tarmac.load();
      world = T.createWorld(m.positions, m.indices, m.surf);
      const tWorld = performance.now() - t0;
      spawn = m.spawn; carId = m.carId ?? 1; tune = m.tune ?? 0;
      const y = T.height(world, spawn.x, 500, spawn.z);
      car = T.createCar(world, carId, tune, spawn.x, spawn.z, spawn.yaw, y + 1.0);
      self.postMessage({ type: 'ready', version: T.version(), worldMs: tWorld, tris: m.indices.length / 3, state: T.readState(car).slice() });
    } else if (m.type === 'step' && car) {
      acc = Math.min(acc + Math.min(m.dt, 0.1), MAX_TICKS_PER_FRAME / HZ);
      let n = Math.floor(acc * HZ + 1e-6); acc -= n / HZ;
      const t0 = performance.now();
      let r = 0; if (n > 0) r = T.step(car, m.input, n);
      busyMs = performance.now() - t0;
      if (r < 0) self.postMessage({ type: 'error', error: 'step: ' + T.error() });
      self.postMessage({ type: 'state', state: T.readState(car).slice(), ticks: n, busyMs, usPerTick: n ? 1000 * busyMs / n : 0 });
    } else if (m.type === 'reset' && car) {
      const s = m.spawn || spawn; const y = T.height(world, s.x, 500, s.z);
      if (m.recreate) { T.destroyCar(car); car = T.createCar(world, m.carId ?? carId, m.tune ?? tune, s.x, s.z, s.yaw, y + 1.0); carId = m.carId ?? carId; tune = m.tune ?? tune; }
      else if (T.reset(car, s.x, s.z, s.yaw, y + 1.0) !== 0) { T.destroyCar(car); car = T.createCar(world, carId, tune, s.x, s.z, s.yaw, y + 1.0); }
      acc = 0; self.postMessage({ type: 'state', state: T.readState(car).slice(), ticks: 0, busyMs: 0, usPerTick: 0 });
    } else if (m.type === 'assists' && car) { T.setAssists(car, m.abs, m.esp); }
    else if (m.type === 'steerOwner' && car) { T.setSteerOwner(car, m.flags); }
  } catch (err) { self.postMessage({ type: 'error', error: String(err && err.stack || err) }); }
};
