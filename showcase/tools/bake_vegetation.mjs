// node tools/bake_vegetation.mjs [outDir]  -> native/assets/vegetation/<kind>.bin + vegetation.json + textures + impostor atlases.
// Runs web/bake_veg.html headless (needs the http server on :8765). Assets are generated from ez-tree (MIT, Daniel Greenheck).
// bin layout: u32 nb_verts, u32 nb_idx (branches), u32 nl_verts, u32 nl_idx (leaves); then per part: vertices (8 f32: pos, nrm, uv), u32 indices.
import { createRequire } from 'module'; import fs from 'fs'; import path from 'path'; const require = createRequire('/workspace/sc-tools/'); const puppeteer = require('puppeteer-core');
const dir = process.argv[2] || new URL('../native/assets/vegetation/', import.meta.url).pathname; fs.mkdirSync(dir, { recursive: true });
const browser = await puppeteer.launch({ executablePath: '/opt/google/chrome/chrome', headless: 'new', protocolTimeout: 900000, args: ['--no-sandbox', '--disable-dev-shm-usage', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
const page = await browser.newPage(); page.on('console', m => { if (!/^THREE/.test(m.text())) console.error('[page]', m.text().slice(0, 300)); }); page.on('pageerror', e => console.error('[pageerror]', e.message));
await page.goto('http://localhost:8765/bake_veg.html', { waitUntil: 'load' }); await page.waitForFunction('window.__bake', { timeout: 700000, polling: 1000 });
const names = await page.evaluate(() => Object.keys(window.__bake)); if (names.includes('error')) { console.error('bake error'); process.exit(1); } const index = {};
for (const n of names) {
  const k = await page.evaluate((n) => window.__bake[n], n);
  const pack = (g) => { const nv = g.pos.length / 3, idx = g.idx || [...Array(nv).keys()]; const v = new Float32Array(nv * 8);
    for (let i = 0; i < nv; i++) { v.set(g.pos.slice(i * 3, i * 3 + 3), i * 8); v.set(g.nrm ? g.nrm.slice(i * 3, i * 3 + 3) : [0, 1, 0], i * 8 + 3); v.set(g.uv ? g.uv.slice(i * 2, i * 2 + 2) : [0, 0], i * 8 + 6); }
    return [nv, idx.length, Buffer.from(v.buffer), Buffer.from(new Uint32Array(idx).buffer)]; };
  const [bv, bi, bvb, bib] = pack(k.branches), [lv, li, lvb, lib] = pack(k.leaves);
  fs.writeFileSync(path.join(dir, n + '.bin'), Buffer.concat([Buffer.from(new Uint32Array([bv, bi, lv, li]).buffer), bvb, bib, lvb, lib]));
  const save = (url, f) => { if (!url) return null; fs.writeFileSync(path.join(dir, f), Buffer.from(url.split(',')[1], 'base64')); return f; };
  const files = { bark: save(k.bark, `${k.kind.file}_bark.png`), barkNormal: save(k.barkNormal, `${k.kind.file}_bark_n.png`), leaf: save(k.leaf, `${n}_leaf.png`), atlasAlbedo: save(k.atlasAlbedo, `${n}_imp_albedo.png`), atlasNormal: save(k.atlasNormal, `${n}_imp_normal.png`) };
  index[n] = { R: k.R, cy: k.cy, h: k.h, base: k.base, N: k.N, CELL: k.CELL, shrub: !!k.kind.shrub, barkTint: k.barkTint, leafTint: k.leafTint, alphaTest: k.alphaTest, branchTris: bi / 3, leafTris: li / 3, files, preset: k.kind };
  console.log(n, 'branches', bi / 3, 'tris, leaves', li / 3, 'tris, R', k.R.toFixed(2), 'h', k.h.toFixed(2));
}
fs.writeFileSync(path.join(dir, 'vegetation.json'), JSON.stringify({ generator: 'ez-tree (MIT, (c) 2024 Daniel Greenheck) via web/bake_veg.html', atlas: '8x8 hemi-octahedral, albedo rgb+alpha / normal', kinds: index }, null, 1));
await browser.close();
