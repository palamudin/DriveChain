// input.js - keyboard, gamepads and steering wheels / pedals through the Gamepad API, same mapping model as the native
// input.ini: standard-mapped pads use named axes/buttons, everything else (wheels, pedal boxes, shifters) raw axisN/buttonN,
// optionally prefixed with a device-name substring ("CSL Pedals/axis1").  Mapping is stored in localStorage 'dcs.inputMap'
// (edit with window.__dcs.inputMap / setInputMap, or ?inputmap=<url-encoded JSON>); F1 or pad Back toggles the overlay.
// Produces {throttle, brake, steer(+left), handbrake, clutch, gearUp, gearDown, reset, camera} in physics units.
const PAD_AXES = ['LX', 'LY', 'RX', 'RY', 'LT', 'RT'];
const PAD_BUTTONS = ['A', 'B', 'X', 'Y', 'LB', 'RB', 'LT2', 'RT2', 'Back', 'Start', 'LS', 'RS', 'DUp', 'DDown', 'DLeft', 'DRight', 'Home']; // W3C standard order
export const DEFAULT_MAP = {
  // pad defaults mirror the original C/raylib DriveChain (demo/input.c, SDL GameController): RT gas, LT brake, linear 0..1 triggers without
  // deadzone, linear stick with a 0.04 deadzone, B handbrake, X clutch, RB/LB shift.
  pad: { deadzone: 0.04, steer_linearity: 1, trigger_deadzone: 0, steer: 'LX', throttle: 'RT', brake: 'LT', clutch: 'X', gear_up: 'RB', gear_down: 'LB', handbrake: 'B', reset: 'Y', camera: 'RS', overlay: 'Back' },
  wheel: { device: '', steer: 'axis0', steer_invert: 0, steer_deadzone: 0, steer_gain: 1, throttle: 'axis1', throttle_invert: 1, brake: 'axis5', brake_invert: 1, clutch: 'axis6', clutch_invert: 1, pedal_deadzone: 0.02,
    gear_up: 'button4', gear_down: 'button5', handbrake: '', reset: 'button23', camera: 'button3', overlay: 'button7' },
};

const WHEELISH = /wheel|pedal|shifter|g29|g920|g923|g27|g25|t300|t150|tmx|t248|fanatec|csl|moza|simucube|thrustmaster|handbrake|button box/i;
const PADISH = /xinput|xbox|045e|054c|057e|2dc8|dualshock|dualsense|wireless controller|gamepad|joypad|controller/i;
/** Profile for a pad the browser did not map to the W3C 'standard' layout (Firefox on Linux, some Bluetooth pads):
 *  axis/button indices of LX LY RX RY LT RT and the face/shoulder/menu buttons in standard order; tAx = triggers reported as -1..1 axes. */
export function padProfile(id) {
  const s = id.toLowerCase(); if (WHEELISH.test(s) || !PADISH.test(s)) return null;
  if (/054c|dualshock|dualsense|wireless controller/.test(s) && !/xbox/.test(s)) return { ax: [0, 1, 2, 5], lt: 3, rt: 4, tAx: true, btn: [1, 2, 0, 3, 4, 5, null, null, 8, 9, 10, 11] };
  if (/xinput|xbox|045e/.test(s)) return { ax: [0, 1, 3, 4], lt: 2, rt: 5, tAx: true, btn: [0, 1, 2, 3, 4, 5, null, null, 6, 7, 9, 10] };
  return { ax: [0, 1, 2, 3], lt: 6, rt: 7, tAx: false, btn: [1, 2, 0, 3, 4, 5, 6, 7, 8, 9, 10, 11] }; // generic HID (e.g. F310 D-mode): triggers are buttons
}
/** Non-standard pad -> {axes, buttons} in standard layout. A trigger axis reads 0 until it has reported anything but exactly 0: browsers report an
 *  untouched -1..1 trigger as 0, which mapped as (0+1)/2 = HALF throttle/brake at rest (one of the "triggers aren't gas/brake" causes). */
export function padView(p, prof, seen) {
  const a = (i) => p.axes[i] ?? 0, B = (i) => i == null ? { value: 0, pressed: false } : (p.buttons[i] || { value: 0, pressed: false });
  const trig = (i) => { if (!prof.tAx) { const b = B(i); return b.value || (b.pressed ? 1 : 0); } const v = p.axes[i]; if (v == null) return 0; if (v !== 0) seen.add(i); return seen.has(i) ? Math.min(1, Math.max(0, (v + 1) / 2)) : 0; };
  const lt = trig(prof.lt), rt = trig(prof.rt), btn = prof.btn.map(B);
  btn[6] = { value: lt, pressed: lt > 0.5 }; btn[7] = { value: rt, pressed: rt > 0.5 };
  return { index: p.index, id: p.id, mapping: 'standard', virtual: true, axes: prof.ax.map(a), buttons: [...btn, { value: 0, pressed: false }, { value: 0, pressed: false }, { value: 0, pressed: false }, { value: 0, pressed: false }] };
}
function bind(v, pad) {
  v = String(v ?? '').trim(); if (!v || v === 'none') return null; let dev = null; const k = v.lastIndexOf('/'); if (k >= 0) { dev = v.slice(0, k).trim().toLowerCase(); v = v.slice(k + 1).trim(); }
  let neg = false; if (v[0] === '-') { neg = true; v = v.slice(1); }
  if (pad) { const a = PAD_AXES.findIndex(n => n.toLowerCase() === v.toLowerCase()); if (a >= 0) return { dev, axis: a, neg, pad: true }; const b = PAD_BUTTONS.findIndex(n => n.toLowerCase() === v.toLowerCase()); if (b >= 0) return { dev, button: b }; }
  let m = /^axis(\d+)$/i.exec(v); if (m) return { dev, axis: +m[1], neg }; m = /^button(\d+)$/i.exec(v); if (m) return { dev, button: +m[1] }; console.warn('input map: cannot parse', v); return null;
}
export class Input {
  constructor() {
    this.keys = new Set(); this.kSteer = 0; this.kThr = 0; this.kBrk = 0; this.edges = {}; this.mode = 'keyboard'; this.prev = new Map(); this.overlay = false;
    const q = new URLSearchParams(location.search); let user = null; try { user = JSON.parse(q.get('inputmap') || localStorage.getItem('dcs.inputMap') || 'null'); } catch (e) { console.warn('bad input map', e); }
    this.setMap(user);
    addEventListener('keydown', (e) => { if (!this.keys.has(e.code)) this.edges[e.code] = true; this.keys.add(e.code); if (e.code === 'F1') { this.overlay = !this.overlay; e.preventDefault(); } if (['ArrowUp','ArrowDown','ArrowLeft','ArrowRight','Space'].includes(e.code)) e.preventDefault(); });
    addEventListener('keyup', (e) => this.keys.delete(e.code)); addEventListener('blur', () => this.keys.clear());
    addEventListener('gamepadconnected', (e) => { const p = e.gamepad; console.log(`input: connected #${p.index} "${p.id}" mapping=${p.mapping || 'none'} ${p.axes.length} axes ${p.buttons.length} buttons`); });
    addEventListener('gamepaddisconnected', (e) => console.log(`input: disconnected #${e.gamepad.index} "${e.gamepad.id}"`));
    this.el = document.createElement('pre'); Object.assign(this.el.style, { position: 'fixed', left: '12px', top: '64px', margin: 0, padding: '8px 10px', background: 'rgba(10,12,16,.82)', color: '#e8e8e8', font: '12px/1.35 ui-monospace,monospace', zIndex: 20, display: 'none', pointerEvents: 'none', whiteSpace: 'pre' });
    document.body.appendChild(this.el);
  }
  setMap(user) { const M = { pad: { ...DEFAULT_MAP.pad, ...(user?.pad || {}) }, wheel: { ...DEFAULT_MAP.wheel, ...(user?.wheel || {}) } }; this.map = M;
    this.pb = {}; this.wb = {}; for (const a of ['steer', 'throttle', 'brake', 'clutch', 'gear_up', 'gear_down', 'handbrake', 'reset', 'camera', 'overlay']) { this.pb[a] = bind(M.pad[a], true); this.wb[a] = bind(M.wheel[a], false); } }
  saveMap(user) { localStorage.setItem('dcs.inputMap', JSON.stringify(user)); this.setMap(user); }
  pressed(code) { const v = !!this.edges[code]; delete this.edges[code]; return v; }
  pads() { const raw = navigator.getGamepads ? [...navigator.getGamepads()].filter(Boolean) : []; this.seen ??= new Map();
    return raw.map(p => { if (p.mapping === 'standard') return p; const prof = padProfile(p.id); if (!prof) return p; if (!this.seen.has(p.index)) this.seen.set(p.index, new Set()); return padView(p, prof, this.seen.get(p.index)); }); }
  startCalib() { return 'use the F1 overlay to read raw axis numbers, then window.__dcs.input.saveMap({wheel:{steer:"axis0",throttle:"axis2",...}})'; }
  read(dt) {
    const k = this.keys, out = { throttle: 0, brake: 0, steer: 0, handbrake: 0, clutch: 0, gearUp: false, gearDown: false, reset: false, camera: false, device: 'keyboard' };
    const left = k.has('ArrowLeft') || k.has('KeyA'), right = k.has('ArrowRight') || k.has('KeyD');
    const tgt = (left ? 1 : 0) - (right ? 1 : 0), rate = tgt === 0 ? 3.5 : (Math.sign(tgt) !== Math.sign(this.kSteer) ? 4.5 : 2.2);
    this.kSteer += Math.max(-rate * dt, Math.min(rate * dt, tgt - this.kSteer));
    this.kThr += Math.max(-4 * dt, Math.min(2.5 * dt, ((k.has('ArrowUp') || k.has('KeyW')) ? 1 : 0) - this.kThr));
    this.kBrk += Math.max(-6 * dt, Math.min(4 * dt, ((k.has('ArrowDown') || k.has('KeyS')) ? 1 : 0) - this.kBrk));
    out.steer = this.kSteer; out.throttle = this.kThr; out.brake = this.kBrk; out.handbrake = k.has('Space') ? 1 : 0;
    out.gearUp = this.pressed('KeyE') || this.pressed('ShiftLeft'); out.gearDown = this.pressed('KeyQ') || this.pressed('ControlLeft');
    const pads = this.pads(); const P = this.map.pad, W = this.map.wheel; let padSteer = null, wheelSteer = null;
    const prevOf = (p) => this.prev.get(p.index) || [];
    for (const p of pads.filter(p => p.mapping === 'standard')) {
      const v = (b) => !b ? 0 : b.axis != null ? (b.axis === 4 ? p.buttons[6]?.value || 0 : b.axis === 5 ? p.buttons[7]?.value || 0 : (p.axes[b.axis] || 0) * (b.neg ? -1 : 1)) : (p.buttons[b.button]?.value || 0);
      const edge = (b) => b && b.button != null && p.buttons[b.button]?.pressed && !prevOf(p)[b.button];
      const sx = v(this.pb.steer), mag = Math.abs(sx), s = mag < P.deadzone ? 0 : Math.pow(Math.min(1, (mag - P.deadzone) / (1 - P.deadzone)), P.steer_linearity) * Math.sign(sx);
      const trig = (x) => !(x > P.trigger_deadzone) ? 0 : Math.min(1, (x - P.trigger_deadzone) / (1 - P.trigger_deadzone));
      const th = trig(v(this.pb.throttle)), br = trig(v(this.pb.brake));
      if (s !== 0 || th > 0 || br > 0) { padSteer = padSteer ?? -s; out.device = 'gamepad'; }
      out.throttle = Math.max(out.throttle, th); out.brake = Math.max(out.brake, br); out.clutch = Math.max(out.clutch, trig(v(this.pb.clutch)));
      if (v(this.pb.handbrake) > 0.5) out.handbrake = 1;
      out.gearUp ||= edge(this.pb.gear_up); out.gearDown ||= edge(this.pb.gear_down); out.reset ||= edge(this.pb.reset); out.camera ||= edge(this.pb.camera); if (edge(this.pb.overlay)) this.overlay = !this.overlay;
    }
    const raw = pads.filter(p => p.mapping !== 'standard');
    if (raw.length) {
      const pick = (dev) => { const key = dev ?? String(W.device || '').toLowerCase(); return key ? raw.find(p => p.id.toLowerCase().includes(key)) : raw[0]; };
      const val = (a) => { const b = this.wb[a]; if (!b) return null; const p = pick(b.dev); if (!p) return null; let x = b.axis != null ? p.axes[b.axis] : (p.buttons[b.button]?.pressed ? 1 : -1); if (x == null || !Number.isFinite(x)) return null; /* absent axis = unbound, not half pedal */ if (b.neg) x = -x; if (+W[a + '_invert']) x = -x; return x; };
      const edge = (a) => { const b = this.wb[a]; if (!b || b.button == null) return false; const p = pick(b.dev); return !!(p && p.buttons[b.button]?.pressed && !prevOf(p)[b.button]); };
      const pedal = (a) => { const x = val(a); if (x == null) return null; const q = Math.min(1, Math.max(0, (x + 1) / 2)); return q < W.pedal_deadzone ? 0 : (q - W.pedal_deadzone) / (1 - W.pedal_deadzone); };
      let s = val('steer'); if (s != null) { if (Math.abs(s) < W.steer_deadzone) s = 0; s = Math.max(-1, Math.min(1, s * W.steer_gain)); wheelSteer = -s; out.device = 'wheel'; }
      const t = pedal('throttle'), b = pedal('brake'), c = pedal('clutch'); if (t != null) out.throttle = Math.max(out.throttle, t); if (b != null) out.brake = Math.max(out.brake, b); if (c != null) out.clutch = Math.max(out.clutch, c);
      if ((val('handbrake') ?? -1) > 0) out.handbrake = 1;
      out.gearUp ||= edge('gear_up'); out.gearDown ||= edge('gear_down'); out.reset ||= edge('reset'); out.camera ||= edge('camera'); if (edge('overlay')) this.overlay = !this.overlay;
    }
    // wheel wins when it is turned; a centred wheel does not mask the pad
    if (wheelSteer != null && (Math.abs(wheelSteer) > 0.02 || padSteer == null)) out.steer = wheelSteer; else if (padSteer != null) out.steer = padSteer;
    for (const p of pads) this.prev.set(p.index, p.buttons.map(b => b.pressed));
    if (out.device !== 'keyboard') this.mode = out.device;
    if (this.overlay) this.drawOverlay(pads, out); else if (this.el.style.display !== 'none') this.el.style.display = 'none';
    return out;
  }
  drawOverlay(pads, o) {
    const L = ['INPUT  (F1 / pad Back toggles; mapping: localStorage dcs.inputMap, see README)', `mapped: steer ${o.steer >= 0 ? '+' : ''}${o.steer.toFixed(2)} thr ${o.throttle.toFixed(2)} brk ${o.brake.toFixed(2)} clu ${o.clutch.toFixed(2)} hb ${o.handbrake}  [${o.device}]`];
    if (!pads.length) L.push('no gamepads visible (press any button on the device: browsers hide pads until first input)');
    for (const p of pads) { const std = p.mapping === 'standard'; L.push(`${p.index} ${p.id.slice(0, 60)} [${std ? 'standard pad' : 'raw'}]`);
      const ax = p.axes.map((a, i) => `${(std ? PAD_AXES[i] : 'a' + i).padStart(4)} ${a >= 0 ? '+' : ''}${a.toFixed(3)}`); if (std) ax.push(`  LT ${p.buttons[6]?.value.toFixed(3)}`, `  RT ${p.buttons[7]?.value.toFixed(3)}`);
      for (let i = 0; i < ax.length; i += 4) L.push('   ' + ax.slice(i, i + 4).join('  '));
      const held = p.buttons.map((b, i) => b.pressed ? (std ? PAD_BUTTONS[i] || i : i) : null).filter(x => x != null); L.push('   buttons held: ' + (held.length ? held.join(' ') : '-')); }
    this.el.textContent = L.join('\n'); this.el.style.display = 'block';
  }
}
