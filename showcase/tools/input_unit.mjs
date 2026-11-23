// Node unit test of web/js/input.js with simulated Gamepad API states (no browser): node tools/input_unit.mjs
// Covers Jakov's report "RT/LT should be gas/brake like the original drivechain" for standard pads, non-standard pads
// (Firefox/Linux Xbox and DS4 layouts with -1..1 trigger axes that read 0 until touched), and wheel + pad together.
globalThis.addEventListener = () => {}; globalThis.location = { search: '' }; globalThis.localStorage = { getItem: () => null, setItem() {} };
globalThis.document = { createElement: () => ({ style: {} }), body: { appendChild() {} } };
let pads = []; Object.defineProperty(globalThis, "navigator", { value: { getGamepads: () => pads }, configurable: true });
const { Input } = await import('../web/js/input.js');
const B = (v) => ({ pressed: v > 0.5, touched: v > 0, value: v }); let fails = 0;
const ok = (c, m) => { console.log((c ? 'PASS ' : 'FAIL ') + m); if (!c) fails++; };
const near = (a, b) => Math.abs(a - b) < 1e-6;
const std = (lx, lt, rt, btn = []) => ({ index: 0, id: 'Xbox Wireless Controller (STANDARD GAMEPAD Vendor: 045e)', mapping: 'standard', axes: [lx, 0, 0, 0], buttons: Array.from({ length: 17 }, (_, i) => B(i === 6 ? lt : i === 7 ? rt : btn.includes(i) ? 1 : 0)) });
let I = new Input(); let o;
pads = [std(0, 0, 0)]; o = I.read(1 / 60); ok(o.throttle === 0 && o.brake === 0, 'standard pad at rest: no gas/brake');
pads = [std(0, 0.5, 1)]; o = I.read(1 / 60); ok(near(o.throttle, 1) && near(o.brake, 0.5), `standard pad RT=gas LT=brake (thr ${o.throttle} brk ${o.brake})`);
pads = [std(0, 0, 0.2)]; o = I.read(1 / 60); ok(near(o.throttle, 0.2), 'no trigger deadzone (original: linear 0..1)');
pads = [std(-0.52, 0, 0)]; o = I.read(1 / 60); ok(near(o.steer, (0.52 - 0.04) / 0.96), `stick deadzone 0.04 linear (steer ${o.steer.toFixed(4)})`);
pads = [std(0, 0, 0, [1, 2])]; o = I.read(1 / 60); ok(o.handbrake === 1 && o.clutch === 1, 'B handbrake, X clutch (original)');
// Firefox/Linux xpad layout, mapping '' : axes LX LY LT RX RY RT, triggers -1..1 but 0 before first touch
I = new Input(); const ff = (lt, rt) => ({ index: 0, id: '045e-028e-Microsoft X-Box 360 pad', mapping: '', axes: [0, 0, lt, 0, 0, rt, 0, 0], buttons: Array.from({ length: 11 }, () => B(0)) });
pads = [ff(0, 0)]; o = I.read(1 / 60); ok(o.throttle === 0 && o.brake === 0, `non-standard Xbox pad, untouched triggers (axis 0) are NOT half pedal (thr ${o.throttle} brk ${o.brake})`);
pads = [ff(-1, 1)]; o = I.read(1 / 60); ok(near(o.throttle, 1) && o.brake === 0, 'non-standard Xbox pad RT axis = gas');
pads = [ff(0, -1)]; o = I.read(1 / 60); ok(near(o.brake, 0.5) && o.throttle === 0, 'non-standard Xbox pad LT axis = brake');
I = new Input(); const ds = (l2, r2) => ({ index: 0, id: '054c-09cc-Sony Interactive Entertainment Wireless Controller', mapping: '', axes: [0, 0, 0, l2, r2, 0], buttons: Array.from({ length: 14 }, () => B(0)) });
pads = [ds(-1, -1)]; I.read(1 / 60); pads = [ds(-1, 1)]; o = I.read(1 / 60); ok(near(o.throttle, 1) && o.brake === 0, 'non-standard DS4 R2 = gas');
// old bug: pad was treated as a wheel -> throttle = (stickY+1)/2 = 0.5 at rest
ok(!I.read(1 / 60).device.includes('wheel'), 'pad never used through the wheel path');
// pad + idle G29: pad triggers drive, wheel pedals released (+1 inverted -> 0), centred wheel does not steal steering
I = new Input(); const g29 = (st) => ({ index: 1, id: 'G29 Driving Force Racing Wheel (Vendor: 046d Product: c24f)', mapping: '', axes: [st, 1, 1, 1, 1, 1, 1, 0, 0, 0], buttons: Array.from({ length: 25 }, () => B(0)) });
pads = [std(-0.5, 0.3, 0.8), g29(0)]; o = I.read(1 / 60); ok(near(o.throttle, 0.8) && near(o.brake, 0.3) && o.steer > 0.4, `pad + idle wheel (thr ${o.throttle} brk ${o.brake} steer ${o.steer.toFixed(3)})`);
pads = [std(-0.5, 0, 0), g29(0.25)]; o = I.read(1 / 60); ok(near(o.steer, -0.25), 'turned wheel wins steering');
console.log(fails ? `${fails} FAILED` : 'all passed'); process.exit(fails ? 1 : 0);
