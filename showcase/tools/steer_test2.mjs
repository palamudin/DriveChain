import { Tarmac } from '../web/js/tarmac-core.js';
import { loadColliders } from './glb_colliders.mjs';
const W = loadColliders(new URL('../private-assets/usce/usce.glb', import.meta.url).pathname);
const T = await Tarmac.load(); const w = T.createWorld(W.positions, W.indices, W.surf);
const sp = W.meta.spawns[0]; const y = T.height(w, sp.x, 500, sp.z); const c = T.createCar(w, 1, 0, sp.x, sp.z, sp.yaw, y + 1);
const ang = (s) => Math.atan2((s[18] + s[34]) / 2 - (s[50] + s[66]) / 2, (s[16] + s[32]) / 2 - (s[48] + s[64]) / 2);
const yawOfQ = (x, y, z, w) => Math.atan2(2 * (w * y + x * z), 1 - 2 * (y * y + x * x));
let gear = 1;
for (let t = 0; t < 240 * 12; t++) {
  const sec = t / 240; const steer = sec > 4 ? (sec > 8 ? -1 : 1) : 0;
  const s0 = T.readState(c); if (s0[11] > 5500 && gear < 4) gear++;
  T.step(c, { throttle: sec < 4 ? 0.8 : 0.3, brake: 0, steer, handbrake: 0, clutch: 0, gear, starter: 0 }, 1);
  if (t % 120 === 0) { const s = T.readState(c); const wy = yawOfQ(s[19], s[20], s[21], s[22]) - yawOfQ(s[3], s[4], s[5], s[6]);
    console.log(sec.toFixed(1), 'steerIn', steer, 'spd', s[10].toFixed(1), 'g', s[12], 'rpm', s[11].toFixed(0), 'head', ang(s).toFixed(3), 'w0yawRel', wy.toFixed(3), 'contact', s[30], s[46], 'running', s[13]); }
}
