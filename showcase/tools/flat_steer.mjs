import { Tarmac } from '../web/js/tarmac-core.js';
const N = 40, S = 25; const pos = [], idx = [], surf = [];
for (let j = 0; j <= N; j++) for (let i = 0; i <= N; i++) pos.push((i - N / 2) * S, (i === 0 && j === 0) ? -1 : 0, (j - N / 2) * S);
for (let j = 0; j < N; j++) for (let i = 0; i < N; i++) { const a = j * (N + 1) + i, b = a + 1, c = a + N + 1, d = c + 1; idx.push(a, c, b, b, c, d); surf.push(0, 0); }
const T = await Tarmac.load(); const w = T.createWorld(new Float32Array(pos), new Uint32Array(idx), new Uint8Array(surf));
if (!w) { console.log('world err', T.error()); process.exit(1); }
const c = T.createCar(w, +(process.argv[2] || 1), 0, 0, 0, 0, 1); if (!c) { console.log('car err', T.error()); process.exit(1); }
const ang = (s) => Math.atan2((s[18] + s[34]) / 2 - (s[50] + s[66]) / 2, (s[16] + s[32]) / 2 - (s[48] + s[64]) / 2);
let gear = 1; const hz = 240;
for (let t = 0; t < hz * 16; t++) {
  const sec = t / hz; const SA = +(process.argv[3] || 0.3); const steer = sec < 6 ? 0 : sec < 10 ? SA : -SA;
  const s0 = T.readState(c); if (s0[11] > 5800 && gear < 4) gear++;
  const th = sec < 6 ? 1 : 0.35;
  T.step(c, { throttle: th, brake: 0, steer, handbrake: 0, clutch: 0, gear, starter: 0 }, 1);
  if (t % 120 === 0) { const s = T.readState(c);
    const yq=(o)=>Math.atan2(2*(s[o+6]*s[o+4]+s[o+3]*s[o+5]),1-2*(s[o+4]**2+s[o+3]**2)); console.log('w0rel',(yq(16)-yq(0)).toFixed(3),'w1rel',(yq(32)-yq(0)).toFixed(3),'contact',s[30],s[46],s[62],s[78],'load',s[26].toFixed(0)); console.log(sec.toFixed(1), 'st', steer, 'spd', s[10].toFixed(1), 'g', s[12], 'rpm', s[11].toFixed(0), 'head', ang(s).toFixed(3), 'pos', s[0].toFixed(1), s[2].toFixed(1)); }
}
