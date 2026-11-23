// Headless check of the web input mapping with fake devices (standard pad + raw "G29" wheel). Prints mapped input + overlay text.
import { createRequire } from 'module'; const require = createRequire('/workspace/sc-tools/'); const puppeteer = require('puppeteer-core');
const out = process.argv[2] || '/tmp/input_overlay.png';
const browser = await puppeteer.launch({ executablePath: '/opt/google/chrome/chrome', headless: 'new', protocolTimeout: 600000, args: ['--no-sandbox', '--disable-dev-shm-usage', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--window-size=1280,720'] });
const page = await browser.newPage(); await page.setViewport({ width: 1280, height: 720 });
await page.evaluateOnNewDocument(() => {
  const B = (v) => ({ pressed: v > 0.5, touched: v > 0, value: v });
  window.__fake = { pad: { index: 0, id: 'Xbox Wireless Controller (STANDARD GAMEPAD Vendor: 045e Product: 0b13)', mapping: 'standard', connected: true, axes: [0.6, 0, 0, 0], buttons: Array.from({ length: 17 }, () => B(0)) },
    wheel: { index: 1, id: 'G29 Driving Force Racing Wheel (Vendor: 046d Product: c24f)', mapping: '', connected: true, axes: [-0.25, -1, 1, 1, 1, 1, 1, 0, 0, 0], buttons: Array.from({ length: 25 }, () => B(0)) } };
  window.__fake.pad.buttons[7] = B(0.8); window.__fake.pad.buttons[2] = B(1);
  navigator.getGamepads = () => [window.__fake.pad, window.__fake.wheel];
});
const logs = []; page.on('console', m => logs.push(m.text()));
await page.goto('http://localhost:8765/index.html?tier=base&webgl&trees=0&grass=0&imp=0', { waitUntil: 'load', timeout: 300000 });
await page.waitForFunction('window.__dcs && window.__dcs.ready && window.__dcs.frames > 20', { timeout: 300000, polling: 500 });
await page.keyboard.press('F1'); await new Promise(r => setTimeout(r, 1500));
const a = await page.evaluate(() => ({ inp: window.__dcs.lastInput, overlay: document.querySelector('pre')?.textContent }));
// wheel only: release the pad stick/trigger, push throttle pedal half (axis1 0 = half), steer wheel right
await page.evaluate(() => { const f = window.__fake; f.pad.axes[0] = 0; f.pad.buttons[7] = { pressed: false, touched: false, value: 0 }; f.wheel.axes[0] = 0.5; f.wheel.axes[1] = 0; });
await new Promise(r => setTimeout(r, 1500));
const b = await page.evaluate(() => window.__dcs.lastInput);
await page.screenshot({ path: out });
console.log(JSON.stringify({ padAndWheel: a.inp, wheelOnly: b }, null, 1)); console.log(a.overlay);
await browser.close();
