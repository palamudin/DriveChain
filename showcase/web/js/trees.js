// Vegetation: ez-tree (MIT, Daniel Greenheck, https://github.com/dgreenheck/ez-tree) procedural trees replace the stand-in
// blocky foliage. Near trees are real instanced ez-tree meshes. Mid/far trees are hemi-octahedral impostors
// (SmartImposter-style: an 8x8 atlas of albedo+normal views baked once per preset at load, lit at runtime by the real sun).
import * as THREE from 'three';
import { Fn, attribute, vec2, vec3, vec4, float, cameraPosition, cameraViewMatrix, normalize, cross, abs, floor, clamp, texture, uniform, varying, select, max, normalView, positionLocal, distance } from 'three/tsl';
import { Tree } from '../vendor/ez-tree/ez-tree.es.js';

const N = 8, CELL = 192; // atlas: 8x8 views, 160 px each (1280^2)
// Belgrade / Usce parkland mix: poplar (aspen preset as Populus stand-in), ash, oak, pine; two seeds for the common ones.
const PRESETS = [
  { name: 'aspen', file: 'aspen_medium', seed: 11, w: 0.22 }, { name: 'aspen2', file: 'aspen_medium', seed: 4711, w: 0.13 },
  { name: 'ash', file: 'ash_medium', seed: 23, w: 0.2 }, { name: 'ash2', file: 'ash_medium', seed: 977, w: 0.1 },
  { name: 'oak', file: 'oak_medium', seed: 5, w: 0.2 }, { name: 'pine', file: 'pine_medium', seed: 31, w: 0.15 },
];

function rng(seed) { let s = seed >>> 0 || 1; return () => ((s = (s * 1664525 + 1013904223) >>> 0) / 4294967296); }

// ---------- placement from the stand-in foliage mesh: clusters on a 4 m grid, then seeded points away from the road
export function planTrees(entries, opts = {}) {
  const cell = 4, clear = opts.clearance ?? 3.5, minGap = opts.minGap ?? 4.5, rnd = rng(opts.seed ?? 1990);
  const fol = entries.filter(e => e.cls === 32 || /foliage/i.test(e.name || ''));
  const road = entries.filter(e => e.collide && (e.surf === 0 || e.surf === 1 || e.surf === 5));
  const cells = new Map(); const v = new THREE.Vector3(); const key = (i, j) => i * 100003 + j;
  for (const e of fol) { const g = e.mesh.geometry, p = g.attributes.position, m = e.mesh.matrixWorld;
    for (let k = 0; k < p.count; k++) { v.fromBufferAttribute(p, k).applyMatrix4(m); const i = Math.floor(v.x / cell), j = Math.floor(v.z / cell), K = key(i, j);
      let c = cells.get(K); if (!c) cells.set(K, c = { i, j, min: Infinity, max: -Infinity }); c.min = Math.min(c.min, v.y); c.max = Math.max(c.max, v.y); } }
  // road raster (1 m) dilated by the clearance
  const rc = 1, roadSet = new Set(); const rk = (i, j) => i * 100003 + j; const D = Math.ceil(clear / rc);
  const a = new THREE.Vector3(), b = new THREE.Vector3(), c3 = new THREE.Vector3();
  for (const e of road) { const g = e.mesh.geometry, p = g.attributes.position, idx = g.index, m = e.mesh.matrixWorld; const n = idx ? idx.count : p.count;
    for (let t = 0; t < n; t += 3) { a.fromBufferAttribute(p, idx ? idx.getX(t) : t).applyMatrix4(m); b.fromBufferAttribute(p, idx ? idx.getX(t + 1) : t + 1).applyMatrix4(m); c3.fromBufferAttribute(p, idx ? idx.getX(t + 2) : t + 2).applyMatrix4(m);
      const x0 = Math.floor(Math.min(a.x, b.x, c3.x) / rc), x1 = Math.floor(Math.max(a.x, b.x, c3.x) / rc), z0 = Math.floor(Math.min(a.z, b.z, c3.z) / rc), z1 = Math.floor(Math.max(a.z, b.z, c3.z) / rc);
      if ((x1 - x0) * (z1 - z0) > 400) continue; for (let i = x0; i <= x1; i++) for (let j = z0; j <= z1; j++) roadSet.add(rk(i, j)); } }
  const nearRoad = (x, z) => { const i0 = Math.floor(x / rc), j0 = Math.floor(z / rc); for (let i = -D; i <= D; i++) for (let j = -D; j <= D; j++) if (i * i + j * j <= D * D && roadSet.has(rk(i0 + i, j0 + j))) return true; return false; };
  // connected clusters
  const seen = new Set(), clusters = [];
  for (const [K, c] of cells) { if (seen.has(K)) continue; const st = [c], cl = []; seen.add(K);
    while (st.length) { const q = st.pop(); cl.push(q); for (const [di, dj] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) { const K2 = key(q.i + di, q.j + dj); const n2 = cells.get(K2); if (n2 && !seen.has(K2)) { seen.add(K2); st.push(n2); } } }
    clusters.push(cl); }
  const trees = [], grid = new Map(); const gk = (x, z) => key(Math.floor(x / minGap), Math.floor(z / minGap));
  const tooClose = (x, z) => { const i0 = Math.floor(x / minGap), j0 = Math.floor(z / minGap); for (let i = -1; i <= 1; i++) for (let j = -1; j <= 1; j++) for (const t of grid.get(key(i0 + i, j0 + j)) || []) if ((t.x - x) ** 2 + (t.z - z) ** 2 < minGap * minGap) return true; return false; };
  let rejected = 0;
  for (const cl of clusters) { const area = cl.length * cell * cell; const want = Math.max(1, Math.min(40, Math.round(area / 55)));
    const hMax = Math.max(...cl.map(c => c.max - c.min));
    for (let n = 0, tries = 0; n < want && tries < want * 8; tries++) { const c = cl[Math.floor(rnd() * cl.length)]; const x = (c.i + rnd()) * cell, z = (c.j + rnd()) * cell;
      if (nearRoad(x, z)) { rejected++; continue; } if (tooClose(x, z)) continue;
      let r = rnd(), p = 0; for (; p < PRESETS.length - 1; p++) { r -= PRESETS[p].w; if (r < 0) break; }
      const t = { x, z, y: c.min, h: THREE.MathUtils.clamp(hMax, 6, 16) * (0.8 + 0.4 * rnd()), yaw: rnd() * Math.PI * 2, p };
      trees.push(t); const G = gk(x, z); (grid.get(G) || grid.set(G, []).get(G)).push(t); n++; } }
  return { trees, clusters: clusters.length, rejected, foliage: fol };
}

// ---------- presets + impostor bake
async function loadPreset(file) { return (await fetch(new URL(`../vendor/ez-tree/presets/${file}.json`, import.meta.url))).json(); }
function bakeMats(orig, normalPass) {
  const m = new THREE.MeshBasicNodeMaterial({ side: THREE.DoubleSide, alphaTest: orig.alphaTest || 0 });
  const tex = orig.map ? texture(orig.map).level(0) : null; // LOD 0: minified leaf alpha would otherwise fall under the test and thin the canopy
  if (normalPass) { m.colorNode = normalView.mul(0.5).add(0.5); if (tex && orig.alphaTest) m.opacityNode = tex.a; }
  else { m.colorNode = tex ? tex.rgb.mul(vec3(orig.color.r, orig.color.g, orig.color.b)) : vec3(orig.color.r, orig.color.g, orig.color.b); if (tex && orig.alphaTest) m.opacityNode = tex.a; }
  m.alphaTest = orig.alphaTest ? 0.3 : 0; return m;
}
function octDir(i, j) { const ex = (i + 0.5) / N * 2 - 1, ey = (j + 0.5) / N * 2 - 1; const ox = (ex + ey) / 2, oz = (ex - ey) / 2; return new THREE.Vector3(ox, Math.max(0, 1 - Math.abs(ox) - Math.abs(oz)), oz).normalize(); }
function bake(renderer, tree) {
  tree.updateMatrixWorld(true); const box = new THREE.Box3().setFromObject(tree); const sph = box.getBoundingSphere(new THREE.Sphere()); const sz = box.getSize(new THREE.Vector3()); const R = 0.5 * 1.05 * Math.max(sz.y, Math.hypot(sz.x, sz.z)); sph.center.copy(box.getCenter(new THREE.Vector3()));
  const out = {}; const W = N * 2 * R;
  const cam = new THREE.OrthographicCamera(-W / 2, W / 2, W / 2, -W / 2, 0.01, R * 8); cam.position.set(0, 0, R * 4); cam.lookAt(0, 0, 0);
  for (const pass of ['albedo', 'normal']) {
    const sc = new THREE.Scene(); const mats = [bakeMats(tree.branchesMesh.material, pass === 'normal'), bakeMats(tree.leavesMesh.material, pass === 'normal')];
    for (let i = 0; i < N; i++) for (let j = 0; j < N; j++) { const v = octDir(i, j); let up = new THREE.Vector3(0, 1, 0);
      const right = new THREE.Vector3().crossVectors(up, v); if (right.lengthSq() < 1e-6) right.set(1, 0, 0); right.normalize(); up = new THREE.Vector3().crossVectors(v, right);
      const Rm = new THREE.Matrix4().makeBasis(right, up, v).transpose();
      const M = new THREE.Matrix4().makeTranslation((i + 0.5) * 2 * R - W / 2, (j + 0.5) * 2 * R - W / 2, 0).multiply(Rm).multiply(new THREE.Matrix4().makeTranslation(-sph.center.x, -sph.center.y, -sph.center.z));
      for (const [k, src] of [[0, tree.branchesMesh], [1, tree.leavesMesh]]) { const m = new THREE.Mesh(src.geometry, mats[k]); m.matrixAutoUpdate = false; m.matrix.copy(M); m.frustumCulled = false; sc.add(m); } }
    const rt = new THREE.RenderTarget(N * CELL, N * CELL, { type: THREE.UnsignedByteType, generateMipmaps: true, minFilter: THREE.LinearMipmapLinearFilter, magFilter: THREE.LinearFilter });
    const prevTM = renderer.toneMapping, prevC = new THREE.Color(), prevA = renderer.getClearAlpha(); renderer.getClearColor(prevC);
    renderer.toneMapping = THREE.NoToneMapping; renderer.setClearColor(pass === 'normal' ? 0x8080ff : 0x2a3a1a, 0); renderer.setRenderTarget(rt); renderer.clear(); renderer.render(sc, cam); renderer.setRenderTarget(null);
    renderer.toneMapping = prevTM; renderer.setClearColor(prevC, prevA); out[pass] = rt.texture; mats.forEach(m => m.dispose());
  }
  return { ...out, R, cy: sph.center.y, h: box.max.y - box.min.y, base: box.min.y };
}

export class Vegetation {
  constructor(renderer, scene, plan, opts = {}) { this.renderer = renderer; this.scene = scene; this.plan = plan; this.nearR = opts.nearR ?? 70; this.group = new THREE.Group(); scene.add(this.group); this.enabled = true; this.last = new THREE.Vector3(1e9, 0, 0); this.nearCount = 0; this.tris = 0; }
  async build() {
    const t0 = performance.now(); this.kinds = [];
    for (const P of PRESETS) { const tree = new Tree(); const json = await loadPreset(P.file); tree.options.copy ? tree.options.copy(json) : Object.assign(tree.options, json); tree.options.seed = P.seed; tree.generate();
      const ib = bake(this.renderer, tree); const list = this.plan.trees.filter(t => t.p === this.kinds.length);
      const mk = (src) => { const im = new THREE.InstancedMesh(src.geometry, src.material, Math.max(1, list.length)); im.count = 0; im.castShadow = true; im.receiveShadow = true; im.frustumCulled = false; this.group.add(im); return im; };
      this.kinds.push({ P, tree, ib, list, br: mk(tree.branchesMesh), lv: mk(tree.leavesMesh), tris: tree.triangleCount, imp: this.makeImpostors(list, ib) }); }
    this.bakeMs = performance.now() - t0; return this;
  }
  makeImpostors(list, ib) {
    const n = list.length; if (!n) return null; const pos = new Float32Array(n * 12), corner = new Float32Array(n * 8), ctr = new Float32Array(n * 12), sz = new Float32Array(n * 4), yaw = new Float32Array(n * 4), idx = [];
    list.forEach((t, k) => { const s = t.h / ib.h; const cx = t.x, cy = t.y + (ib.cy - ib.base) * s, cz = t.z; const C = [[-1, -1], [1, -1], [1, 1], [-1, 1]];
      for (let q = 0; q < 4; q++) { const o = k * 4 + q; pos.set([cx, cy, cz], o * 3); ctr.set([cx, cy, cz], o * 3); corner.set(C[q], o * 2); sz[o] = ib.R * s; yaw[o] = t.yaw; }
      idx.push(k * 4, k * 4 + 1, k * 4 + 2, k * 4, k * 4 + 2, k * 4 + 3); });
    const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.BufferAttribute(pos, 3)); g.setAttribute('corner', new THREE.BufferAttribute(corner, 2));
    g.setAttribute('center', new THREE.BufferAttribute(ctr, 3)); g.setAttribute('isize', new THREE.BufferAttribute(sz, 1)); g.setAttribute('iyaw', new THREE.BufferAttribute(yaw, 1)); g.setIndex(idx);
    const m = new THREE.MeshStandardNodeMaterial({ roughness: 0.85, metalness: 0, side: THREE.DoubleSide }); const nearR = uniform(this.nearR); this.nearRU = nearR;
    const center = attribute('center', 'vec3'), cornerA = attribute('corner', 'vec2'), isize = attribute('isize', 'float'), iyaw = attribute('iyaw', 'float');
    const toCam = normalize(cameraPosition.sub(center)); const right = normalize(cross(vec3(0, 1, 0), toCam)); const up = cross(toCam, right);
    const s = select(distance(center, cameraPosition).lessThan(nearR), float(0), isize);
    m.positionNode = center.add(right.mul(cornerA.x).add(up.mul(cornerA.y)).mul(s));
    // local view dir (undo the tree's yaw), hemi-octahedral cell
    const cy = iyaw.cos(), sy = iyaw.sin(); const lv = normalize(vec3(cy.mul(toCam.x).sub(sy.mul(toCam.z)), max(toCam.y, 0.0), sy.mul(toCam.x).add(cy.mul(toCam.z))));
    const vLocal = varying(lv), vRight = varying(right), vUp = varying(up), vToCam = varying(toCam), vUV = varying(cornerA.mul(0.5).add(0.5));
    const auv = Fn(() => { const d = abs(vLocal.x).add(abs(vLocal.y)).add(abs(vLocal.z)); const o = vec2(vLocal.x, vLocal.z).div(d); const e = vec2(o.x.add(o.y), o.x.sub(o.y)).mul(0.5).add(0.5);
      const cell = clamp(floor(e.mul(N)), 0, N - 1); return cell.add(vUV).div(N); })();
    const alb = texture(ib.albedo, auv), nrm = texture(ib.normal, auv).xyz.mul(2).sub(1);
    m.colorNode = alb.rgb; m.opacityNode = alb.a; m.alphaTest = 0.5;
    const nW = normalize(vRight.mul(nrm.x).add(vUp.mul(nrm.y)).add(vToCam.mul(nrm.z))); m.normalNode = normalize(cameraViewMatrix.mul(vec4(nW, 0)).xyz);
    const mesh = new THREE.Mesh(g, m); mesh.frustumCulled = false; mesh.receiveShadow = true; this.group.add(mesh); return mesh;
  }
  update(camPos, force) {
    if (!this.kinds || (!force && camPos.distanceToSquared(this.last) < 25)) return; this.last.copy(camPos);
    const M = new THREE.Matrix4(), q = new THREE.Quaternion(), S = new THREE.Vector3(), T = new THREE.Vector3(), Y = new THREE.Vector3(0, 1, 0); let near = 0, tris = 0; const r2 = this.nearR * this.nearR;
    for (const k of this.kinds) { let c = 0; for (const t of k.list) { if ((t.x - camPos.x) ** 2 + (t.z - camPos.z) ** 2 > r2) continue; const s = t.h / k.ib.h; q.setFromAxisAngle(Y, t.yaw); S.setScalar(s); T.set(t.x, t.y - k.ib.base * s, t.z); M.compose(T, q, S); k.br.setMatrixAt(c, M); k.lv.setMatrixAt(c, M); c++; }
      k.br.count = k.lv.count = c; k.br.instanceMatrix.needsUpdate = k.lv.instanceMatrix.needsUpdate = true; near += c; tris += c * k.tris; }
    this.nearCount = near; this.tris = tris;
  }
  setEnabled(on) { this.enabled = on; this.group.visible = on; for (const f of this.plan.foliage) f.mesh.visible = !on; }
}
