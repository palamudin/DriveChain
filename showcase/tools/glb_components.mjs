// Connected-component analysis of a GLB mesh node (private world audit): node tools/glb_components.mjs file.glb NODE
import fs from 'fs';
const [file, node = 'VIS_other'] = process.argv.slice(2);
const b = fs.readFileSync(file); const jl = b.readUInt32LE(12); const j = JSON.parse(b.slice(20, 20 + jl)); const bin = b.slice(20 + jl + 8);
const acc = (i) => { const a = j.accessors[i], v = j.bufferViews[a.bufferView]; const off = (v.byteOffset || 0) + (a.byteOffset || 0); const n = a.count * ({ SCALAR: 1, VEC2: 2, VEC3: 3 }[a.type]);
  return a.componentType === 5126 ? new Float32Array(bin.buffer, bin.byteOffset + off, n) : a.componentType === 5125 ? new Uint32Array(bin.buffer, bin.byteOffset + off, n) : new Uint16Array(bin.buffer, bin.byteOffset + off, n); };
const nd = j.nodes.find(n => n.name === node); const prim = j.meshes[nd.mesh].primitives[0]; const P = acc(prim.attributes.POSITION), I = acc(prim.indices);
// weld by position (components share positions even if split verts)
const key = (k) => `${Math.round(P[3 * k] * 200)},${Math.round(P[3 * k + 1] * 200)},${Math.round(P[3 * k + 2] * 200)}`; const id = new Map(); const vid = new Int32Array(P.length / 3);
for (let k = 0; k < vid.length; k++) { const s = key(k); let x = id.get(s); if (x === undefined) id.set(s, x = id.size); vid[k] = x; }
const par = new Int32Array(id.size).map((_, i) => i); const f = (x) => { while (par[x] !== x) x = par[x] = par[par[x]]; return x; };
for (let t = 0; t < I.length; t += 3) { const a = f(vid[I[t]]), b2 = f(vid[I[t + 1]]), c = f(vid[I[t + 2]]); par[b2] = a; par[f(c)] = a; }
const comps = new Map();
for (let t = 0; t < I.length; t += 3) { const r = f(vid[I[t]]); let c = comps.get(r); if (!c) comps.set(r, c = { tris: 0, mn: [1e9, 1e9, 1e9], mx: [-1e9, -1e9, -1e9] }); c.tris++;
  for (let q = 0; q < 3; q++) { const k = I[t + q]; for (let d = 0; d < 3; d++) { c.mn[d] = Math.min(c.mn[d], P[3 * k + d]); c.mx[d] = Math.max(c.mx[d], P[3 * k + d]); } } }
const list = [...comps.values()].map(c => ({ ...c, sx: c.mx[0] - c.mn[0], sy: c.mx[1] - c.mn[1], sz: c.mx[2] - c.mn[2] }));
const cat = {}; for (const c of list) { const h = c.sy, w = Math.max(c.sx, c.sz), t = Math.min(c.sx, c.sz);
  const k = h < 0.35 && w > 0.6 ? 'flat-slab' : (h > 2 && t < 0.4 && w > 1.5) ? 'thin-vertical-panel' : h > 2 ? 'tall' : 'small'; (cat[k] ||= { n: 0, tris: 0, ex: [] }); cat[k].n++; cat[k].tris += c.tris; if (cat[k].ex.length < 5) cat[k].ex.push([c.mn.map(v => v.toFixed(1)).join(','), c.sx.toFixed(2), c.sy.toFixed(2), c.sz.toFixed(2), c.tris]); }
console.log(node, 'components', list.length); console.log(JSON.stringify(cat, null, 1));
if (process.argv[4]) fs.writeFileSync(process.argv[4], JSON.stringify(list.map(c => ({ mn: c.mn, mx: c.mx, tris: c.tris }))));
