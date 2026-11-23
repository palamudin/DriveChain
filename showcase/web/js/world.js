// world.js - swappable glTF world loader + materials.
//
// World convention (any glTF 2.0 file, so Jakov's world-builder can export straight into it):
//   * Frame: Y up, metres; the physics uses the same coordinates as the render.
//   * Collision: meshes whose node (or an ancestor) has extras.tarmac.collide === true, with extras.tarmac.surf = DriveChain
//     surface id (0 tarmac,1 kerb,2 grass,3 gravel,4 sausage,5 line,6 pave,7 rough,8 pad,9 wall,10 other,11 concrete).
//     Fallback when no extras: node names "COL_<surfname>" are collision; everything else is visual only.
//   * Visual only: all other meshes (extras.tarmac.cls is an optional visual class: 10 building, 32 foliage, 33 tyres, 34 fence).
//   * Materials: author your own PBR materials and they are kept, unless named "surf_<name>", which get the showcase
//     surface shaders below. TEXCOORD_0 on roads may carry (arc length m, lateral m from racing line) for kerb stripes/rubber.
//   * Scene extras.tarmac: { name, spawns:[{name,x,z,yaw}], line:[[x,z],...] (optional racing line) }.
import * as THREE from 'three/webgpu';
import { texture, positionWorld, vec2, vec3, float, uv, mix, smoothstep, fract, abs, step, sin, normalMap, triplanarTexture, uniform, clamp, max, min, Fn, normalWorld, dot, color } from 'three/tsl';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { MeshoptDecoder } from 'three/addons/libs/meshopt_decoder.module.js';

export const SURF = { tarmac: 0, kerb: 1, grass: 2, gravel: 3, sausage: 4, line: 5, pave: 6, rough: 7, pad: 8, wall: 9, other: 10, concrete: 11 };
const SURF_NAME = Object.fromEntries(Object.entries(SURF).map(([k, v]) => [v, k]));

export async function loadWorld(url, onProgress) {
  const loader = new GLTFLoader(); loader.setMeshoptDecoder(MeshoptDecoder);
  const gltf = await loader.loadAsync(url, (e) => onProgress && e.total && onProgress(e.loaded / e.total));
  const root = gltf.scene; root.updateMatrixWorld(true);
  const meta = (gltf.scene.userData && gltf.scene.userData.tarmac) || (gltf.scenes[0].userData || {}).tarmac || {};
  const colliders = [], visuals = [];
  root.traverse((o) => {
    if (!o.isMesh) return;
    let t = null; for (let p = o; p && !t; p = p.parent) t = p.userData && p.userData.tarmac;
    let name = o.name; for (let p = o; p; p = p.parent) if (/^(COL|VIS)_/.test(p.name)) { name = p.name; break; }
    const collide = t ? !!t.collide : /^COL_/.test(name);
    const surf = t && t.surf != null ? t.surf : (SURF[name.replace(/^COL_/, '')] ?? 0);
    const cls = t && t.cls != null ? t.cls : null;
    (collide ? colliders : visuals).push({ mesh: o, surf, cls, name });
  });
  return { root, meta, colliders, visuals, spawns: meta.spawns || [{ name: 'origin', x: 0, z: 0, yaw: 0 }] };
}

// Packs all collision meshes into the arrays tms_world_create wants (world-space float32 xyz, uint32 tris, uint8 surf per tri).
export function buildCollision(world) {
  let nv = 0, nt = 0;
  for (const c of world.colliders) { const g = c.mesh.geometry; nv += g.attributes.position.count; nt += (g.index ? g.index.count : g.attributes.position.count) / 3; }
  const pos = new Float32Array(nv * 3), idx = new Uint32Array(nt * 3), surf = new Uint8Array(nt);
  let vo = 0, to = 0; const v = new THREE.Vector3();
  for (const c of world.colliders) {
    const g = c.mesh.geometry, p = g.attributes.position, m = c.mesh.matrixWorld;
    for (let i = 0; i < p.count; i++) { v.fromBufferAttribute(p, i).applyMatrix4(m); pos.set([v.x, v.y, v.z], (vo + i) * 3); }
    const n = g.index ? g.index.count : p.count;
    for (let i = 0; i < n; i++) idx[to * 3 + i] = vo + (g.index ? g.index.getX(i) : i);
    surf.fill(c.surf, to, to + n / 3); vo += p.count; to += n / 3;
  }
  return { positions: pos, indices: idx, surf };
}

// ---------------------------------------------------------------- materials
const texCache = new Map();
function tex(path, srgb) {
  if (texCache.has(path)) return texCache.get(path);
  const t = new THREE.TextureLoader().load(path); t.wrapS = t.wrapT = THREE.RepeatWrapping; t.anisotropy = 8;
  t.colorSpace = srgb ? THREE.SRGBColorSpace : THREE.NoColorSpace; texCache.set(path, t); return t;
}
const TEX = 'assets/textures/';
// two-scale sampling with a rotated macro layer to break tiling (no extra textures needed)
function antiTile(map, uvw, scale, macro = 0.137) {
  const a = texture(map, uvw.mul(scale));
  const r = vec2(uvw.x.mul(0.8).sub(uvw.y.mul(0.6)), uvw.x.mul(0.6).add(uvw.y.mul(0.8)));
  const b = texture(map, r.mul(scale * macro));
  return { a, b };
}
export const look = { wet: uniform(0), rubber: uniform(1), sunDir: uniform(new THREE.Vector3(0.4, 0.8, 0.3)) };

function pbr(name, opts = {}) {
  const set = { albedo: tex(TEX + name + '_albedo.jpg', true), normal: tex(TEX + name + '_normal.jpg'), rough: tex(TEX + name + '_rough.jpg') };
  const m = new THREE.MeshStandardNodeMaterial({ roughness: 1, metalness: 0 });
  const worldUV = opts.triplanar ? null : positionWorld.xz;
  const s = 1 / (opts.tile || 4);
  if (opts.triplanar) {
    m.colorNode = triplanarTexture(texture(set.albedo), null, null, float(s)).rgb.mul(opts.tint || vec3(1));
    m.roughnessNode = triplanarTexture(texture(set.rough), null, null, float(s)).r.mul(opts.roughMul ?? 1);
  } else {
    const A = antiTile(set.albedo, worldUV, s), R = texture(set.rough, worldUV.mul(s));
    const macroV = A.b.rgb.dot(vec3(0.33)).mul(1.6).add(0.2).clamp(0.6, 1.25);
    m.colorNode = A.a.rgb.mul(macroV).mul(opts.tint || vec3(1));
    m.roughnessNode = R.r.mul(opts.roughMul ?? 1);
    m.normalNode = normalMap(texture(set.normal, worldUV.mul(s)), vec2(opts.normalScale ?? 1));
  }
  return { m, set, worldUV, s };
}

export function makeShowcaseMaterials() {
  const M = {};
  // asphalt: anti-tiled albedo, rubbered racing line from TEXCOORD_0.y (lateral metres), wetness darkens + smooths
  {
    const { m, worldUV, s } = pbr('asphalt', { tile: 5, roughMul: 1.0, normalScale: 0.8, tint: vec3(1.7) });
    const lat = abs(uv().y), rub = smoothstep(2.4, 0.4, lat).mul(look.rubber).mul(0.45);
    const base = m.colorNode;
    m.colorNode = base.mul(float(1).sub(rub)).mul(mix(float(1), float(0.55), look.wet));
    m.roughnessNode = m.roughnessNode.mul(float(1).sub(rub.mul(0.5))).mul(mix(float(1), float(0.18), look.wet)).clamp(0.04, 1);
    M.tarmac = m;
  }
  { // kerb: red/white stripes every 1.5 m of arc length (procedural), painted roughness
    const m = new THREE.MeshStandardNodeMaterial({ metalness: 0 });
    const st = step(0.5, fract(uv().x.div(3.0)));
    const grime = texture(tex(TEX + 'concrete_albedo.jpg', true), positionWorld.xz.mul(0.5)).rgb;
    m.colorNode = mix(color(0xb5170f), color(0xe8e4dc), st).mul(grime.mul(0.6).add(0.55));
    m.roughnessNode = float(0.55).mul(mix(float(1), float(0.3), look.wet));
    M.kerb = m;
  }
  M.grass = pbr('grass', { tile: 3, normalScale: 1.2, tint: vec3(0.95, 1.0, 0.9) }).m;
  M.pave = pbr('cobble', { tile: 2.5 }).m;
  M.concrete = pbr('concrete', { tile: 4, tint: vec3(0.6) }).m;
  M.wall = pbr('concrete', { triplanar: true, tile: 3, tint: vec3(0.42) }).m;
  M.other = pbr('facade', { triplanar: true, tile: 9, tint: vec3(0.62, 0.6, 0.57) }).m; M.other.flatShading = true;
  { // foliage: no UVs in this stand-in mesh -> world-noise tinted, slight subsurface-ish wrap via emissive from sun side
    const m = new THREE.MeshStandardNodeMaterial({ roughness: 0.8, metalness: 0, side: THREE.DoubleSide });
    const n = texture(tex(TEX + 'grass_albedo.jpg', true), positionWorld.xz.mul(0.11).add(positionWorld.y.mul(0.07))).rgb;
    m.colorNode = n.mul(vec3(0.55, 0.7, 0.45));
    M.foliage = m;
  }
  M.tyres = new THREE.MeshStandardNodeMaterial({ color: 0x1a1a1c, roughness: 0.92, metalness: 0, flatShading: false });
  M.fence = new THREE.MeshStandardNodeMaterial({ color: 0x8c9196, roughness: 0.45, metalness: 0.7, flatShading: true });
  M.gravel = pbr('cobble', { tile: 1.2, tint: vec3(0.9, 0.85, 0.75) }).m;
  return M;
}

// "Raw" look = the old raylib DriveChain demo: flat lambert colours, no textures, no shadows, no post.
export function makeRawMaterials() {
  const c = (hex, flat = true) => new THREE.MeshLambertNodeMaterial({ color: hex, flatShading: flat });
  return { tarmac: c(0x505055), kerb: c(0xc83232), grass: c(0x3c8c3c), pave: c(0x9a9a92), concrete: c(0x8a8a8a), wall: c(0xa0a0a0),
    other: c(0xb4aa96), foliage: c(0x2f7a33), tyres: c(0x222222), fence: c(0x9a9aa0), gravel: c(0xa08c64) };
}

const CLS_NAME = { 10: 'other', 32: 'foliage', 33: 'tyres', 34: 'fence' };
export function materialKey(entry) {
  const mn = (entry.mesh.material && entry.mesh.material.name) || '';
  if (mn.startsWith('surf_')) return mn.slice(5);
  if (entry.cls != null) return CLS_NAME[entry.cls] || 'other';
  return null; // authored material: keep
}
export function applyMaterials(world, set) {
  for (const e of [...world.colliders, ...world.visuals]) {
    if (!e.original) e.original = e.mesh.material;
    const k = e.key ?? (e.key = materialKey(e));
    e.mesh.material = (k && set[k]) || e.original;
  }
}
