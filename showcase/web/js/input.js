// input.js - keyboard, standard gamepad and steering wheels through the Gamepad API.
// Produces {throttle, brake, steer(+left), handbrake, clutch, gearUp, gearDown} in physics units.
const WHEEL_RE = /wheel|g29|g920|g923|t300|t150|tmx|t248|fanatec|csl|moza|simucube|thrustmaster|logitech.*racing|driving force/i;
export class Input {
  constructor() {
    this.keys = new Set(); this.kSteer = 0; this.kThr = 0; this.kBrk = 0; this.edges = {};
    this.mode = 'keyboard'; this.wheelMap = JSON.parse(localStorage.getItem('dcs.wheelMap') || 'null');
    this.calib = null; this.deadzone = 0.06;
    addEventListener('keydown', (e) => { if (!this.keys.has(e.code)) this.edges[e.code] = true; this.keys.add(e.code); if (['ArrowUp','ArrowDown','ArrowLeft','ArrowRight','Space'].includes(e.code)) e.preventDefault(); });
    addEventListener('keyup', (e) => this.keys.delete(e.code));
    addEventListener('blur', () => this.keys.clear());
  }
  pressed(code) { const v = !!this.edges[code]; delete this.edges[code]; return v; }
  pad() { const ps = navigator.getGamepads ? [...navigator.getGamepads()].filter(Boolean) : []; return ps.find(p => WHEEL_RE.test(p.id)) || ps[0] || null; }
  // Wheel calibration: call startCalib(); then follow prompts; axes that move away from rest get assigned.
  startCalib() { const p = this.pad(); if (!p) return 'no device'; this.calib = { step: 0, rest: [...p.axes], map: { id: p.id } }; return this.calibPrompt(); }
  calibPrompt() { return ['Turn the wheel fully LEFT', 'Press THROTTLE fully', 'Press BRAKE fully', 'Press CLUTCH (or any button to skip)'][this.calib.step] || 'done'; }
  calibUpdate(p) {
    const c = this.calib, key = ['steer', 'throttle', 'brake', 'clutch'][c.step];
    let best = -1, bd = 0.5; p.axes.forEach((v, i) => { const d = Math.abs(v - c.rest[i]); if (d > bd && !Object.values(c.map).some(m => m && m.axis === i)) { bd = d; best = i; } });
    if (best >= 0) { c.map[key] = { axis: best, rest: c.rest[best], full: p.axes[best] }; c.step++; c.rest = [...p.axes]; }
    else if (c.step === 3 && p.buttons.some(b => b.pressed)) c.step++;
    if (c.step > 3) { this.wheelMap = c.map; localStorage.setItem('dcs.wheelMap', JSON.stringify(c.map)); this.calib = null; return 'done'; }
    return this.calibPrompt();
  }
  read(dt) {
    const k = this.keys, out = { throttle: 0, brake: 0, steer: 0, handbrake: 0, clutch: 0, gearUp: false, gearDown: false, device: 'keyboard' };
    // keyboard with rate-limited ramps (digital keys would otherwise slam the rack)
    const left = k.has('ArrowLeft') || k.has('KeyA'), right = k.has('ArrowRight') || k.has('KeyD');
    const tgt = (left ? 1 : 0) - (right ? 1 : 0), rate = tgt === 0 ? 3.5 : (Math.sign(tgt) !== Math.sign(this.kSteer) ? 4.5 : 2.2);
    this.kSteer += Math.max(-rate * dt, Math.min(rate * dt, tgt - this.kSteer));
    this.kThr += Math.max(-4 * dt, Math.min(2.5 * dt, ((k.has('ArrowUp') || k.has('KeyW')) ? 1 : 0) - this.kThr));
    this.kBrk += Math.max(-6 * dt, Math.min(4 * dt, ((k.has('ArrowDown') || k.has('KeyS')) ? 1 : 0) - this.kBrk));
    out.steer = this.kSteer; out.throttle = this.kThr; out.brake = this.kBrk; out.handbrake = k.has('Space') ? 1 : 0;
    out.gearUp = this.pressed('KeyE') || this.pressed('ShiftLeft'); out.gearDown = this.pressed('KeyQ') || this.pressed('ControlLeft');
    const p = this.pad();
    if (p) {
      if (this.calib) { this.calibMsg = this.calibUpdate(p); return out; }
      const isWheel = WHEEL_RE.test(p.id), dz = (v) => Math.abs(v) < this.deadzone ? 0 : (v - Math.sign(v) * this.deadzone) / (1 - this.deadzone);
      let used = false;
      if (isWheel && this.wheelMap && this.wheelMap.id === p.id) {
        const m = this.wheelMap, ax = (e) => e ? Math.min(1, Math.max(0, (p.axes[e.axis] - e.rest) / (e.full - e.rest))) : 0;
        const s = m.steer ? p.axes[m.steer.axis] * Math.sign(m.steer.full - m.steer.rest) : 0;
        out.steer = s; out.throttle = ax(m.throttle); out.brake = ax(m.brake); out.clutch = ax(m.clutch); used = true; out.device = 'wheel';
      } else if (p.mapping === 'standard') {
        const st = -dz(p.axes[0]), th = p.buttons[7]?.value || 0, br = p.buttons[6]?.value || 0;
        if (Math.abs(st) > 0 || th > 0.02 || br > 0.02) { out.steer = Math.sign(st) * Math.pow(Math.abs(st), 1.6); out.throttle = th; out.brake = br; used = true; out.device = 'gamepad'; }
        if (p.buttons[2]?.pressed) out.handbrake = 1;
      } else if (isWheel) { out.steer = -p.axes[0]; out.device = 'wheel (uncalibrated: press C)'; used = true; }
      const btn = (i) => !!p.buttons[i]?.pressed, prev = this.prevBtn || [];
      if (btn(5) && !prev[5]) out.gearUp = true; if (btn(4) && !prev[4]) out.gearDown = true;
      if (btn(0) && !prev[0]) out.gearUp = true; if (btn(1) && !prev[1]) out.gearDown = true;
      this.prevBtn = p.buttons.map(b => b.pressed);
      if (used) this.mode = out.device;
    }
    return out;
  }
}
