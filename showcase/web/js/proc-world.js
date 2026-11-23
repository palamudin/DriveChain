// proc-world.js - own procedural test track used when no private world is present (public builds / CI).
// Emits the same structure as the glTF loader (COL_/VIS_ meshes with userData.tarmac), so the pipeline is identical.
import * as THREE from 'three/webgpu';
export function makeProceduralWorld() {
  const root = new THREE.Group(); root.name = 'procedural-test-track';
  // closed spline track (~1.6 km) with varied corners
  const P = [[0, 0], [220, -20], [330, 60], [300, 190], [180, 230], [120, 160], [20, 200], [-120, 240], [-230, 150], [-210, 20], [-110, -40]];
  const curve = new THREE.CatmullRomCurve3(P.map(([x, z]) => new THREE.Vector3(x, 0, z)), true, 'centripetal');
  const N = 900, W = 6.5, K = 1.2, G = 40; const pts = curve.getSpacedPoints(N);
  const strip = (inner, outer, surf, name, height = 0) => {
    const pos = [], uv = [], nor = [], idx = []; let arc = 0;
    for (let i = 0; i <= N; i++) { const p = pts[i % N], q = pts[(i + 1) % N]; const t = q.clone().sub(p).normalize(); const l = new THREE.Vector3(-t.z, 0, t.x);
      if (i > 0) arc += pts[i % N].distanceTo(pts[i - 1]);
      for (const off of [inner, outer]) { const v = p.clone().addScaledVector(l, off); v.y = height + (Math.abs(off) > W ? -0.02 * (Math.abs(off) - W) : 0); pos.push(v.x, v.y, v.z); uv.push(arc, off); nor.push(0, 1, 0); } }
    for (let i = 0; i < N; i++) { const a = i * 2; idx.push(a, a + 2, a + 1, a + 1, a + 2, a + 3); }
    const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3)); g.setAttribute('normal', new THREE.Float32BufferAttribute(nor, 3)); g.setAttribute('uv', new THREE.Float32BufferAttribute(uv, 2)); g.setIndex(idx);
    const m = new THREE.Mesh(g, new THREE.MeshStandardMaterial({ name: 'surf_' + name })); m.name = 'COL_' + name; m.userData.tarmac = { collide: true, surf }; return m;
  };
  root.add(strip(-W, W, 0, 'tarmac')); root.add(strip(W, W + K, 1, 'kerb', 0.03)); root.add(strip(-W - K, -W, 1, 'kerb', 0.03));
  root.add(strip(W + K, W + G, 2, 'grass', -0.02)); root.add(strip(-W - G, -W - K, 2, 'grass', -0.02));
  const ground = new THREE.Mesh(new THREE.PlaneGeometry(1600, 1600, 1, 1).rotateX(-Math.PI / 2).translate(50, -0.6, 100), new THREE.MeshStandardMaterial({ name: 'surf_grass' }));
  ground.name = 'COL_grass_far'; ground.userData.tarmac = { collide: true, surf: 2 }; root.add(ground);
  // some own blocky buildings + tyre walls for scale
  const bm = new THREE.MeshStandardMaterial({ name: 'surf_other' }); let s = 3; const r = () => (s = (s * 16807) % 2147483647) / 2147483647;
  for (let i = 0; i < 70; i++) { const t = i / 70, p = curve.getPointAt(t), tg = curve.getTangentAt(t); const l = new THREE.Vector3(-tg.z, 0, tg.x); const side = i % 2 ? 1 : -1;
    const h = 8 + r() * 40, w = 12 + r() * 20; const b = new THREE.Mesh(new THREE.BoxGeometry(w, h, 12 + r() * 16), bm); b.position.copy(p).addScaledVector(l, side * (70 + r() * 90)); b.position.y = h / 2 - 0.6; b.rotation.y = Math.atan2(tg.x, tg.z); b.name = 'VIS_other_' + i; b.userData.tarmac = { collide: false, cls: 10 }; root.add(b); }
  root.updateMatrixWorld(true);
  const p0 = pts[0], p1 = pts[3]; const yaw0 = Math.atan2(p1.x - p0.x, p1.z - p0.z);
  const line = pts.map(p => [p.x, 0, p.z]);
  return { root, meta: { name: 'Procedural test track (own, public)', spawns: [{ name: 'start', x: p0.x, z: p0.z, yaw: yaw0 }], line, trackLength: curve.getLength() }, yawConvention: 'atan2x_z' };
}
