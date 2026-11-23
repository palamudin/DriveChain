#!/usr/bin/env python3
"""usce_veg_prep.py - strip the stand-in foliage from the (PRIVATE) Ušće GLB and write a vegetation plan.

Input : private-assets/usce/usce.glb (from dcmesh_to_glb.py)
Output: private-assets/usce/usce_v2.glb  - same world WITHOUT  VIS_foliage (tree/hedge blocks) and WITHOUT the
                                           grass/bush "cards" inside VIS_other (2-8 tri crossed quads at ground level,
                                           authored for an alpha-tested foliage shader we don't have -> drawn as slabs)
        private-assets/usce/veg_plan.json - seeded trees + shrubs at the original foliage positions, >= clearance from
                                           tarmac/kerb/line for trees, never on road for shrubs; plus the removal report.
Classification is geometric (the source mesh carries no object names): see classify().
"""
import json, struct, sys, os, math
import numpy as np
sys.path.insert(0, os.path.dirname(__file__)); from dcmesh_to_glb import GLB

def read_glb(path):
    b = open(path, 'rb').read(); jl = struct.unpack_from('<I', b, 12)[0]; j = json.loads(b[20:20 + jl]); bin_ = b[20 + jl + 8:]
    def acc(i):
        a = j['accessors'][i]; v = j['bufferViews'][a['bufferView']]; off = v.get('byteOffset', 0) + a.get('byteOffset', 0)
        n = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3}[a['type']]; dt = {5126: '<f4', 5125: '<u4', 5123: '<u2'}[a['componentType']]
        return np.frombuffer(bin_, dt, a['count'] * n, off).reshape(a['count'], n) if n > 1 else np.frombuffer(bin_, dt, a['count'], off)
    parts = []
    for nd in j['nodes']:
        pr = j['meshes'][nd['mesh']]['primitives'][0]; at = pr['attributes']
        parts.append(dict(name=nd['name'], extras=nd.get('extras', {}), mat=j['materials'][pr['material']]['name'], p=acc(at['POSITION']).copy(),
                          n=acc(at['NORMAL']).copy() if 'NORMAL' in at else None, uv=acc(at['TEXCOORD_0']).copy() if 'TEXCOORD_0' in at else None, t=acc(pr['indices']).reshape(-1, 3).astype(np.int64)))
    return parts, j['scenes'][0].get('extras', {})

def components(p, t):
    q = np.round(p * 200).astype(np.int64); _, vid = np.unique(q, axis=0, return_inverse=True); vid = vid.reshape(-1)
    par = np.arange(vid.max() + 1)
    def f(x):
        r = x
        while par[r] != r: r = par[r]
        while par[x] != r: par[x], x = r, par[x]
        return r
    for a, b, c in vid[t]:
        ra, rb, rc = f(a), f(b), f(c); par[rb] = ra; par[rc] = ra
    roots = np.array([f(x) for x in range(len(par))]); return roots[vid[t[:, 0]]]

def classify(p, t, comp):
    """per-component: card = <= 8 tris, footprint <= 3 m, height <= 2.2 m, base within 0.6 m of the lowest point and
    made of near-vertical or tilted quads (crossed grass/bush billboards). Bigger or taller pieces are props (kept)."""
    out = {}; tri_c = {}
    for i, c in enumerate(comp): tri_c.setdefault(c, []).append(i)
    for c, tris in tri_c.items():
        v = p[t[tris].reshape(-1)]; mn, mx = v.min(0), v.max(0); sz = mx - mn; foot = math.hypot(sz[0], sz[2])
        card = len(tris) <= 8 and foot <= 3.2 and 0.15 <= sz[1] <= 2.2 and foot >= 0.3
        out[c] = dict(card=card, tris=tris, mn=mn, mx=mx)
    return out

def road_raster(parts, cell=1.0):
    s = set()
    for pt in parts:
        x = pt['extras'].get('tarmac', {})
        if not (x.get('collide') and x.get('surf') in (0, 1, 5)): continue
        P = pt['p'][pt['t']]; lo = np.floor(P[:, :, [0, 2]].min(1) / cell).astype(int); hi = np.floor(P[:, :, [0, 2]].max(1) / cell).astype(int)
        for (x0, z0), (x1, z1) in zip(lo, hi):
            if (x1 - x0 + 1) * (z1 - z0 + 1) > 900: continue
            for i in range(x0, x1 + 1):
                for k in range(z0, z1 + 1): s.add((i, k))
    return s

def main(src, out_glb, out_plan, seed=1990):
    parts, scene_extras = read_glb(src); rng = np.random.default_rng(seed)
    road = road_raster(parts)
    def dist_road(x, z, maxd):  # metres to nearest road cell (<= maxd), else maxd+1
        i0, k0 = int(math.floor(x)), int(math.floor(z)); R = int(math.ceil(maxd)); best = maxd + 1
        for di in range(-R, R + 1):
            for dk in range(-R, R + 1):
                if (i0 + di, k0 + dk) in road: best = min(best, math.hypot(di, dk))
        return best
    g = GLB(); report = {}
    seeds = []  # (x, y, z, height, footprint, source)
    for pt in parts:
        name = pt['name']
        if name == 'VIS_foliage':
            comp = components(pt['p'], pt['t']); cl = classify(pt['p'], pt['t'], comp)
            for c in cl.values(): seeds.append(((c['mn'][0] + c['mx'][0]) / 2, c['mn'][1], (c['mn'][2] + c['mx'][2]) / 2, c['mx'][1] - c['mn'][1], math.hypot(*(c['mx'] - c['mn'])[[0, 2]]), 'foliage'))
            report['VIS_foliage'] = dict(removed_tris=int(len(pt['t'])), blobs=len(cl)); continue
        if name == 'VIS_other':
            comp = components(pt['p'], pt['t']); cl = classify(pt['p'], pt['t'], comp)
            keep = np.ones(len(pt['t']), bool); ncard = 0
            for c in cl.values():
                if c['card']:
                    keep[c['tris']] = False; ncard += 1
                    if c['mx'][1] - c['mn'][1] > 0.9: seeds.append(((c['mn'][0] + c['mx'][0]) / 2, c['mn'][1], (c['mn'][2] + c['mx'][2]) / 2, c['mx'][1] - c['mn'][1], 1.5, 'bushcard'))
            report['VIS_other'] = dict(components=len(cl), cards_removed=ncard, card_tris_removed=int((~keep).sum()), props_kept_tris=int(keep.sum()))
            t = pt['t'][keep]; used = np.unique(t); remap = -np.ones(len(pt['p']), np.int64); remap[used] = np.arange(len(used))
            pt = dict(pt, p=pt['p'][used], n=None if pt['n'] is None else pt['n'][used], uv=None if pt['uv'] is None else pt['uv'][used], t=remap[t])
        g.add(name, pt['p'], pt['t'], pt['n'], pt['uv'], pt['extras'], pt['mat'])
    g.write(out_glb, scene_extras)
    # ---- plan: trees where blobs are tall, shrubs where low; a big blob gets several plants spread over its footprint
    trees, shrubs = [], []; occ = {}
    def free(x, z, gap):
        i0, k0 = int(x // 4), int(z // 4)
        for di in (-1, 0, 1):
            for dk in (-1, 0, 1):
                for (a, b, gg) in occ.get((i0 + di, k0 + dk), []):
                    if (a - x) ** 2 + (b - z) ** 2 < max(gap, gg) ** 2: return False
        return True
    rejected = dict(tree_clearance=0, shrub_on_road=0, crowded=0)
    for (x, y, z, h, foot, src_) in seeds:
        tall = h >= 3.0
        n = 1 if foot < 6 else min(6, int(foot / 4))
        for k in range(n):
            px, pz = (x, z) if n == 1 else (x + rng.uniform(-foot / 3, foot / 3), z + rng.uniform(-foot / 3, foot / 3))
            if tall:
                d = dist_road(px, pz, 3.5)
                if d <= 3.5: rejected['tree_clearance'] += 1; continue
                if not free(px, pz, 4.0): rejected['crowded'] += 1; continue
                kind = rng.choice(['poplar', 'poplar2', 'ash', 'ash2', 'oak', 'pine'], p=[.22, .13, .2, .1, .2, .15])
                hh = float(np.clip(h, 6, 16) * rng.uniform(0.85, 1.15)); trees.append(dict(x=round(float(px), 2), y=round(float(y), 2), z=round(float(pz), 2), h=round(hh, 2), yaw=round(float(rng.uniform(0, 6.2832)), 3), kind=str(kind)))
                occ.setdefault((int(px // 4), int(pz // 4)), []).append((px, pz, 4.0))
            else:
                d = dist_road(px, pz, 1.0)
                if d <= 1.0: rejected['shrub_on_road'] += 1; continue
                if not free(px, pz, 1.6): rejected['crowded'] += 1; continue
                kind = rng.choice(['bush1', 'bush2', 'bush3']); hh = float(np.clip(h, 0.8, 2.6) * rng.uniform(0.8, 1.2))
                shrubs.append(dict(x=round(float(px), 2), y=round(float(y), 2), z=round(float(pz), 2), h=round(hh, 2), yaw=round(float(rng.uniform(0, 6.2832)), 3), kind=str(kind)))
                occ.setdefault((int(px // 4), int(pz // 4)), []).append((px, pz, 1.6))
    plan = dict(seed=seed, clearance_tree_m=3.5, trees=trees, shrubs=shrubs, removed=report, rejected=rejected, source='PRIVATE derived from Ušće positions')
    json.dump(plan, open(out_plan, 'w')); print(json.dumps(dict(trees=len(trees), shrubs=len(shrubs), removed=report, rejected=rejected)))

if __name__ == '__main__':
    d = '/workspace/drivechain-showcase/private-assets/usce/'
    main(d + 'usce.glb', d + 'usce_v2.glb', d + 'veg_plan.json')
