// chunks.js - split huge merged meshes into grid cells so frustum culling (and distance tiers) work.
// Index-only split: all chunks share the source vertex buffers (uploaded once).
import * as THREE from 'three/webgpu';
export function chunkMesh(mesh, cell = 96) {
  const g = mesh.geometry, pos = g.attributes.position, idx = g.index;
  if (!idx || idx.count < 30000) return [mesh];
  const n = idx.count / 3, buckets = new Map();
  for (let t = 0; t < n; t++) {
    const a = idx.getX(t * 3), b = idx.getX(t * 3 + 1), c = idx.getX(t * 3 + 2);
    const cx = (pos.getX(a) + pos.getX(b) + pos.getX(c)) / 3, cz = (pos.getZ(a) + pos.getZ(b) + pos.getZ(c)) / 3;
    if (!isFinite(cx) || !isFinite(cz)) continue;
    const k = Math.floor(cx / cell) + ',' + Math.floor(cz / cell);
    let arr = buckets.get(k); if (!arr) buckets.set(k, arr = []); arr.push(a, b, c);
  }
  const out = [];
  for (const [k, arr] of buckets) {
    const cg = new THREE.BufferGeometry();
    for (const name in g.attributes) cg.setAttribute(name, g.attributes[name]);
    const I = pos.count > 65535 ? new Uint32Array(arr) : new Uint16Array(arr); cg.setIndex(new THREE.BufferAttribute(I, 1));
    const box = new THREE.Box3(), v = new THREE.Vector3(); for (let i = 0; i < arr.length; i++) box.expandByPoint(v.fromBufferAttribute(pos, arr[i]));
    cg.boundingBox = box; cg.boundingSphere = box.getBoundingSphere(new THREE.Sphere());
    const m = new THREE.Mesh(cg, mesh.material); m.name = mesh.name + '@' + k; m.matrixAutoUpdate = false; m.matrix.copy(mesh.matrix);
    m.castShadow = mesh.castShadow; m.receiveShadow = mesh.receiveShadow; m.userData = { ...mesh.userData, chunkOf: mesh.name, cell: k, center: cg.boundingSphere.center.clone().applyMatrix4(mesh.matrixWorld) };
    out.push(m);
  }
  return out;
}
