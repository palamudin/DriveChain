// tarmac-core.js - JS facade over the wasm64 build of the Tarmac core (tms.h C API).
// wasm64 (memory64) keeps Jakov's 64-bit ABI struct layouts byte-identical; pointers cross as BigInt.
import TarmacCore from '../wasm/tarmac.mjs';
const B = (p) => BigInt(p), N = (p) => Number(p);
export class Tarmac {
  static async load() { const t = new Tarmac(); t.M = await TarmacCore(); t.state = new Float32Array(80); t.pState = N(t.M._malloc(B(80 * 4))); return t; }
  str(p) { return this.M.UTF8ToString(N(p)); }
  version() { return this.str(this.M._tms_version()); }
  error() { return this.str(this.M._tms_last_error()); }
  alloc(bytes) { return N(this.M._malloc(B(bytes))); }
  free(p) { this.M._free(B(p)); }
  // positions: Float32Array xyz (Tarmac frame), indices: Uint32Array, surf: Uint8Array per triangle
  createWorld(positions, indices, surf) {
    const M = this.M, pP = this.alloc(positions.byteLength), pI = this.alloc(indices.byteLength), pS = this.alloc(Math.max(1, surf.byteLength));
    new Float32Array(M.HEAPF32.buffer, pP, positions.length).set(positions);
    new Uint32Array(M.HEAPU32.buffer, pI, indices.length).set(indices);
    new Uint8Array(M.HEAPU8.buffer, pS, surf.length).set(surf);
    const w = N(M._tms_world_create(B(pP), positions.length / 3, B(pI), indices.length / 3, B(pS)));
    this.free(pP); this.free(pI); this.free(pS); // core deep-copies
    if (!w) throw new Error('tms_world_create: ' + this.error());
    return w;
  }
  height(w, x, yFrom, z) { return this.M._tms_world_height(B(w), x, yFrom, z); }
  createCar(w, carId, tune, x, z, yaw, yHint) {
    const c = N(this.M._tms_car_create(B(w), carId, tune, x, z, yaw, yHint));
    if (!c) throw new Error('tms_car_create: ' + this.error()); return c;
  }
  destroyCar(c) { this.M._tms_car_destroy(B(c)); }
  reset(c, x, z, yaw, yHint) { return this.M._tms_car_reset(B(c), x, z, yaw, yHint); }
  step(c, i, ticks) { return this.M._tms_car_step7(B(c), i.throttle, i.brake, i.steer, i.handbrake, i.clutch, i.gear | 0, i.starter | 0, ticks | 0); }
  readState(c) { this.M._tms_car_state(B(c), B(this.pState)); this.state.set(new Float32Array(this.M.HEAPF32.buffer, this.pState, 80)); return this.state; }
  setAssists(c, abs, esp) { return this.M._tms_car_set_assists(B(c), abs ? 1 : 0, esp ? 1 : 0); }
  setSteerOwner(c, f) { return this.M._tms_car_set_steer_owner(B(c), f >>> 0); }
}
