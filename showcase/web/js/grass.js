// Shader grass (web): procedural segmented blades on grass surfaces only (COL_grass / surf 2), instanced, vertex-shader wind.
// Technique after Velorexe/unity-geometry-grass-shader (Unlicense) and Roystan's grass tutorial: per-blade random facing,
// height, width and forward bend, segmented curvature, wind displacement, base-to-tip colour gradient, ground-colour blend.
// No geometry shaders (not in WebGPU): blades are instances of one strip mesh; the CPU scatters roots on grass triangles in
// 8 m cells around the camera with distance-based thinning (Ghost of Tsushima-style density falloff + height fade).
import * as THREE from 'three';
import { attribute, vec3, vec4, float, uniform, mix, sin, cos, pow, normalize, positionLocal, smoothstep, cameraPosition, distance, clamp } from 'three/tsl';

const SEG = 5;
function bladeGeometry() { // x in [-0.5,0.5] across, y in [0,1] along; tip is a single vertex
  const p = [], idx = [];
  for (let s = 0; s < SEG; s++) { const t = s / SEG; p.push(-0.5, t, 0, 0.5, t, 0); }
  p.push(0, 1, 0); const tip = SEG * 2;
  for (let s = 0; s < SEG - 1; s++) { const a = s * 2; idx.push(a, a + 1, a + 3, a, a + 3, a + 2); }
  const a = (SEG - 1) * 2; idx.push(a, a + 1, tip);
  const g = new THREE.InstancedBufferGeometry(); g.setAttribute('position', new THREE.Float32BufferAttribute(p, 3)); g.setIndex(idx); return g;
}
function hash(x, z, k) { let h = Math.imul((x * 73856093) ^ (z * 19349663) ^ (k * 83492791), 2654435761) >>> 0; h ^= h >>> 15; h = Math.imul(h, 2246822519) >>> 0; h ^= h >>> 13; return (h >>> 0) / 4294967296; }

export class Grass {
  constructor(scene, entries, opts = {}) {
    this.density = opts.density ?? 14; this.radius = opts.radius ?? 45; this.max = opts.max ?? 90000; this.cell = 8; this.enabled = true;
    // grass triangles (world space) binned into cells
    this.cells = new Map(); const a = new THREE.Vector3(), b = new THREE.Vector3(), c = new THREE.Vector3(); let area = 0, ntri = 0;
    for (const e of entries.filter(e => e.collide && [0, 1, 2, 5, 6, 8, 11].includes(e.surf))) { const road = e.surf !== 2; e.mesh.updateWorldMatrix(true, false); const g = e.mesh.geometry, P = g.attributes.position, I = g.index, m = e.mesh.matrixWorld, n = I ? I.count : P.count;
      for (let t = 0; t < n; t += 3) { a.fromBufferAttribute(P, I ? I.getX(t) : t).applyMatrix4(m); b.fromBufferAttribute(P, I ? I.getX(t + 1) : t + 1).applyMatrix4(m); c.fromBufferAttribute(P, I ? I.getX(t + 2) : t + 2).applyMatrix4(m);
        const tri = [a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z]; const ar = Math.abs((b.x - a.x) * (c.z - a.z) - (c.x - a.x) * (b.z - a.z)) / 2; if (ar < 1e-4) continue; if (!road) { area += ar; ntri++; }
        const x0 = Math.floor(Math.min(a.x, b.x, c.x) / this.cell), x1 = Math.floor(Math.max(a.x, b.x, c.x) / this.cell), z0 = Math.floor(Math.min(a.z, b.z, c.z) / this.cell), z1 = Math.floor(Math.max(a.z, b.z, c.z) / this.cell);
        for (let i = x0; i <= x1; i++) for (let k = z0; k <= z1; k++) { const K = i * 65536 + k; (this.cells.get(K) || this.cells.set(K, { i, k, tris: [], road: [], blades: null }).get(K))[road ? 'road' : 'tris'].push(tri); } } }
    this.area = area; this.ntri = ntri;
    const g = bladeGeometry(); this.root = new THREE.InstancedBufferAttribute(new Float32Array(this.max * 4), 4); this.param = new THREE.InstancedBufferAttribute(new Float32Array(this.max * 4), 4);
    this.root.setUsage(THREE.DynamicDrawUsage); this.param.setUsage(THREE.DynamicDrawUsage); g.setAttribute('iRoot', this.root); g.setAttribute('iParam', this.param); g.instanceCount = 0; this.geo = g;
    this.time = uniform(0); this.wind = uniform(opts.wind ?? 1); this.fadeR = uniform(this.radius);
    const root = attribute('iRoot', 'vec4'), prm = attribute('iParam', 'vec4'); // root xyz + hash, (height, width, yaw, bend)
    const t = positionLocal.y, side = positionLocal.x; const dist = distance(root.xyz, cameraPosition);
    const fade = float(1).sub(smoothstep(this.fadeR.mul(0.7), this.fadeR, dist)); // shrink out at the ring edge (no popping)
    const h = prm.x.mul(fade), w = prm.y.mul(float(1).sub(t.mul(0.85))); const cy = cos(prm.z), sy = sin(prm.z);
    const fwd = vec3(sy, 0, cy), right = vec3(cy, 0, sy.negate());
    const gust = sin(this.time.mul(1.3).add(root.x.mul(0.21)).add(root.z.mul(0.17))).mul(0.6).add(sin(this.time.mul(2.9).add(root.x.mul(0.53)).sub(root.z.mul(0.41))).mul(0.25)).add(0.35);
    const windDir = vec3(0.8, 0, 0.6); const bend = pow(t, 2).mul(h).mul(prm.w.add(gust.mul(this.wind).mul(0.35)));
    const lean = fwd.mul(bend.mul(1 - 0)).add(windDir.mul(pow(t, 2).mul(h).mul(gust).mul(this.wind).mul(0.25)));
    const pos = root.xyz.add(right.mul(side.mul(w))).add(vec3(0, t.mul(h).mul(float(1).sub(pow(t, 2).mul(prm.w).mul(0.35))), 0)).add(lean);
    const m = new THREE.MeshStandardNodeMaterial({ side: THREE.DoubleSide, roughness: 0.6, metalness: 0 });
    m.positionNode = pos;
    m.normalNode = normalize(cameraViewNormal(fwd, t));
    const hv = root.w; const base = vec3(0.035, 0.07, 0.018), tip = mix(vec3(0.24, 0.42, 0.08), vec3(0.38, 0.46, 0.12), hv), ground = vec3(0.07, 0.09, 0.035);
    m.colorNode = mix(mix(ground, base, smoothstep(0.0, 0.15, t)), tip, pow(t, 1.4));
    this.mesh = new THREE.Mesh(g, m); this.mesh.frustumCulled = false; this.mesh.castShadow = false; this.mesh.receiveShadow = true; scene.add(this.mesh);
    this.last = new THREE.Vector3(1e9, 0, 0); this.count = 0;
    function cameraViewNormal(f, tt) { return mix(f.negate(), vec3(0, 1, 0), clamp(tt.mul(0.6).add(0.35), 0, 1)); } // soft, view-independent blade normal (stable lighting)
  }
  cellBlades(c) { // jittered grid candidates; keep only those inside a grass triangle (never on tarmac)
    if (c.blades) return c.blades; const out = []; const step = 1 / Math.sqrt(this.density); const n = Math.ceil(this.cell / step);
    for (let u = 0; u < n; u++) for (let v = 0; v < n; v++) { const k = u * n + v; const x = (c.i + (u + hash(c.i, c.k, k * 2)) / n) * this.cell, z = (c.k + (v + hash(c.i, c.k, k * 2 + 1)) / n) * this.cell;
      const bary = (T, m) => { const [ax, ay, az, bx, by, bz, cx, cy, cz] = T; const d = (bz - cz) * (ax - cx) + (cx - bx) * (az - cz); if (Math.abs(d) < 1e-9) return null;
        const l1 = ((bz - cz) * (x - cx) + (cx - bx) * (z - cz)) / d, l2 = ((cz - az) * (x - cx) + (ax - cx) * (z - cz)) / d, l3 = 1 - l1 - l2; return (l1 < -m || l2 < -m || l3 < -m) ? null : l1 * ay + l2 * by + l3 * cy; };
      let roadY = -1e9; for (const T of c.road) { const y = bary(T, 0.02); if (y != null) roadY = Math.max(roadY, y); } // road over the grass mesh: no blade
      for (const T of c.tris) { const y = bary(T, 0); if (y == null) continue; if (roadY > y - 0.6) break; const r = hash(c.i, c.k, k * 7 + 3);
        out.push(x, y, z, r, 0.32 + 0.45 * hash(c.i, c.k, k * 5 + 1), 0.035 + 0.04 * hash(c.i, c.k, k * 3 + 2), hash(c.i, c.k, k * 11 + 4) * 6.2832, 0.15 + 0.5 * hash(c.i, c.k, k * 13 + 5)); break; } }
    return (c.blades = new Float32Array(out));
  }
  update(cam, simTime, force) {
    this.time.value = simTime;
    if (!this.enabled || (!force && cam.distanceToSquared(this.last) < 16)) return; this.last.copy(cam);
    const R = this.radius, ci = Math.floor(cam.x / this.cell), ck = Math.floor(cam.z / this.cell), rc = Math.ceil(R / this.cell); const list = [];
    for (let i = -rc; i <= rc; i++) for (let k = -rc; k <= rc; k++) { const c = this.cells.get((ci + i) * 65536 + (ck + k)); if (!c) continue; const dx = (c.i + 0.5) * this.cell - cam.x, dz = (c.k + 0.5) * this.cell - cam.z; const d = Math.hypot(dx, dz); if (d < R + this.cell) list.push([d, c]); }
    list.sort((a, b) => a[0] - b[0]); let n = 0; const RA = this.root.array, PA = this.param.array;
    for (const [, c] of list) { const B = this.cellBlades(c); for (let j = 0; j < B.length && n < this.max; j += 8) { const d = Math.hypot(B[j] - cam.x, B[j + 2] - cam.z); if (d > R) continue;
        const keep = d < 12 ? 1 : Math.max(0.12, 1 - (d - 12) / (R - 12)); if (B[j + 3] > keep) continue; // density falloff (hash-stable: no shimmer when moving)
        RA.set(B.subarray(j, j + 4), n * 4); PA.set(B.subarray(j + 4, j + 8), n * 4); PA[n * 4 + 1] *= 1 + (1 - keep) * 1.5; n++; } if (n >= this.max) break; }
    this.count = n; this.geo.instanceCount = n; this.root.needsUpdate = true; this.param.needsUpdate = true; this.root.addUpdateRange?.(0, n * 4); this.param.addUpdateRange?.(0, n * 4);
  }
  setEnabled(on) { this.enabled = on; this.mesh.visible = on; }
}
