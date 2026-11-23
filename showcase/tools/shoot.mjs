// Headless capture: node tools/shoot.mjs "<query>" out.png [frames] [w] [h] [js-to-eval-after-ready]
import { createRequire } from 'module'; const require = createRequire('/workspace/sc-tools/'); const puppeteer = require('puppeteer-core');
const [q = '', out = '/tmp/shot.png', frames = '60', W = '1600', H = '900', script = ''] = process.argv.slice(2);
const browser = await puppeteer.launch({ executablePath: '/opt/google/chrome/chrome', headless: 'new', protocolTimeout: 900000,
  args: ['--no-sandbox', '--disable-dev-shm-usage', '--js-flags=--max-old-space-size=4096', '--enable-unsafe-webgpu', '--enable-features=Vulkan,WebGPU', '--use-angle=swiftshader', '--use-webgpu-adapter=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist', `--window-size=${W},${H}`] });
const page = await browser.newPage(); await page.setViewport({ width: +W, height: +H });
const logs = []; page.on('console', m => logs.push('[' + m.type() + '] ' + m.text())); page.on('pageerror', e => logs.push('[pageerror] ' + e.message)); page.on('error', e => { console.error('[PAGE CRASH]', e.message); }); if (process.env.LIVE) page.on('console', m => console.error('[live]', m.text().slice(0, 300)));
const t0 = Date.now();
await page.goto('http://localhost:8765/index.html?' + q, { waitUntil: 'load', timeout: 600000 });
try { await page.waitForFunction(`window.__dcs && window.__dcs.ready && window.__dcs.frames >= ${+frames}`, { timeout: 600000, polling: 1000 }); }
catch (e) { logs.push('[timeout] ' + e.message); }
if (script) { await page.evaluate(script); await page.waitForFunction(`window.__dcs.frames >= ${+frames} + 20`, { timeout: 600000, polling: 1000 }).catch(() => {}); }
const info = await page.evaluate(() => ({ backend: window.__dcs.backend, frames: window.__dcs.frames, sim: window.__dcs.simTime, world: window.__dcs.worldKind, car: window.__dcs.carKind && window.__dcs.carKind(), state: window.__dcs.state, errors: window.__dcs.errors, gpu: window.__dcs.gpuMs, phys: window.__dcs.physVersion && window.__dcs.physVersion(), stats: document.getElementById('stats')?.textContent })).catch(e => ({ err: String(e) }));
await page.screenshot({ path: out });
console.log(JSON.stringify({ ...info, wallS: (Date.now() - t0) / 1000 }, null, 0));
console.log(logs.slice(-25).join('\n'));
await browser.close();
