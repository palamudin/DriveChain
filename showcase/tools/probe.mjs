// probe frame rate of a tier: node tools/probe.mjs "<query>" W H seconds
import { createRequire } from 'module'; const require = createRequire('/workspace/sc-tools/'); const puppeteer = require('puppeteer-core');
const [q, W = '800', H = '450', S = '120'] = process.argv.slice(2);
const b = await puppeteer.launch({ executablePath: '/opt/google/chrome/chrome', headless: 'new', protocolTimeout: 600000, args: ['--no-sandbox', '--disable-dev-shm-usage', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
const p = await b.newPage(); await p.setViewport({ width: +W, height: +H }); p.on('pageerror', e => console.log('pageerror', e.message)); p.on('error', e => console.log('CRASH', e.message));
await p.goto('http://localhost:8765/index.html?' + q);
const t0 = Date.now(); while (Date.now() - t0 < +S * 1000) { await new Promise(r => setTimeout(r, 10000)); const f = await p.evaluate(() => [window.__dcs.frames, window.__dcs.paused, window.__dcs.errors.slice(-2)]).catch(e => String(e)); console.log(((Date.now() - t0) / 1000).toFixed(0) + 's', JSON.stringify(f)); }
await b.close();
