import { Tarmac } from '../web/js/tarmac-core.js';
import { loadColliders } from './glb_colliders.mjs';
const W = loadColliders(new URL('../private-assets/usce/usce.glb', import.meta.url).pathname);
const T = await Tarmac.load(); const w = T.createWorld(W.positions, W.indices, W.surf);
const sp = W.meta.spawns[0]; const y = T.height(w, sp.x, 500, sp.z); const c = T.createCar(w, 1, 0, sp.x, sp.z, sp.yaw, y + 1);
const head = () => { const s = T.readState(c); const fx = (s[16] + s[32]) / 2 - (s[48] + s[64]) / 2, fz = (s[18] + s[34]) / 2 - (s[50] + s[66]) / 2; return { s, ang: Math.atan2(fz, fx), fx, fz }; };
let h0 = head(); console.log('spawn yaw', sp.yaw, 'fwd', h0.fx.toFixed(2), h0.fz.toFixed(2), 'w0', [...h0.s.slice(16, 19)].map(v => v.toFixed(2)), 'w1', [...h0.s.slice(32, 35)].map(v => v.toFixed(2)));
for (let i = 0; i < 240 * 3; i++) T.step(c, { throttle: 0.5, brake: 0, steer: 0, handbrake: 0, clutch: 0, gear: 1 }, 1);
let a = head(); console.log('after 3s speed', a.s[10].toFixed(1), 'gear', a.s[12], 'rpm', a.s[11].toFixed(0));
for (let i = 0; i < 240; i++) T.step(c, { throttle: 0.3, brake: 0, steer: 0.5, handbrake: 0, clutch: 0, gear: 1 }, 1);
let b = head(); let d = b.ang - a.ang; d = Math.atan2(Math.sin(d), Math.cos(d));
// in three (Y up), left turn = counter-clockwise when seen from +Y: heading angle atan2(z,x) DEcreases for CCW-from-above
console.log('steer +0.5 for 1s: dAng(atan2(z,x))', d.toFixed(3), d < 0 ? '=> +steer turns LEFT (as seen from above, Y up)' : '=> +steer turns RIGHT');
// which side is wheel 0 ? left vector L = U x F, U=(0,1,0): L = (F.z*1? ) compute
const F = [b.fx, 0, b.fz], L = [F[2], 0, -F[0]]; const s = b.s; const dx = s[16] - s[32], dz = s[18] - s[34];
console.log('wheel0 is', (dx * L[0] + dz * L[2]) > 0 ? 'LEFT' : 'RIGHT');
