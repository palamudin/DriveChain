// node tools/flicker_web.mjs "<query>" outPrefix N  -> N screenshots of a static camera (car parked, no autopilot)
import { createRequire } from 'module'; const require = createRequire('/workspace/sc-tools/'); const puppeteer = require('puppeteer-core');
const [q, pre = '/tmp/flkw/web', n = '8'] = process.argv.slice(2);
const browser = await puppeteer.launch({ executablePath: '/opt/google/chrome/chrome', headless: 'new', protocolTimeout: 900000, args: ['--no-sandbox', '--disable-dev-shm-usage', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--window-size=960,540'] });
const page = await browser.newPage(); await page.setViewport({ width: 960, height: 540 });
await page.goto('http://localhost:8765/index.html?' + q, { waitUntil: 'load', timeout: 600000 });
await page.waitForFunction('window.__dcs && window.__dcs.ready && window.__dcs.frames > 40', { timeout: 600000, polling: 1000 });
for (let i = 0; i < +n; i++) { const f0 = await page.evaluate(() => window.__dcs.frames); await page.waitForFunction(`window.__dcs.frames >= ${f0 + 2}`, { timeout: 120000, polling: 200 }); await page.screenshot({ path: `${pre}_${String(i).padStart(3, '0')}.png` }); }
console.log(await page.evaluate(() => document.getElementById('stats')?.textContent)); await browser.close();
