// Minimal GLB collider extractor for node tests (uncompressed GLB, node transforms assumed identity).
import fs from 'fs';
export function loadColliders(file) {
  const b = fs.readFileSync(file); const jl = b.readUInt32LE(12); const j = JSON.parse(b.slice(20, 20 + jl).toString());
  const bin = b.slice(20 + jl + 8);
  const acc = (i) => { const a = j.accessors[i], bv = j.bufferViews[a.bufferView]; const off = (bv.byteOffset || 0) + (a.byteOffset || 0);
    const n = a.count * ({ SCALAR: 1, VEC2: 2, VEC3: 3, VEC4: 4 }[a.type]); const C = { 5126: Float32Array, 5125: Uint32Array, 5123: Uint16Array }[a.componentType];
    return new C(bin.buffer.slice(bin.byteOffset + off, bin.byteOffset + off + n * C.BYTES_PER_ELEMENT)); };
  const P = [], I = [], S = []; let vo = 0;
  for (const n of j.nodes) { const t = n.extras && n.extras.tarmac; if (!t || !t.collide || n.mesh == null) continue;
    for (const pr of j.meshes[n.mesh].primitives) { const p = acc(pr.attributes.POSITION), ix = acc(pr.indices); P.push(p); I.push(Array.from(ix, v => v + vo)); S.push(new Array(ix.length / 3).fill(t.surf)); vo += p.length / 3; } }
  const pos = new Float32Array(vo * 3); let o = 0; for (const p of P) { pos.set(p, o); o += p.length; }
  return { positions: pos, indices: Uint32Array.from(I.flat()), surf: Uint8Array.from(S.flat()), meta: j.scenes[0].extras.tarmac };
}
