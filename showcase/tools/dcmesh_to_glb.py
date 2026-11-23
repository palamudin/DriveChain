#!/usr/bin/env python3
"""dcmesh_to_glb.py - convert a DriveChain mesh map (DCMESH01 collider.bin + scenery.bin + line.csv) into a
glTF 2.0 binary the showcase world loader understands. Output stays in a PRIVATE folder when the source is
mod-derived (Ušće). Any glTF that follows the same convention can replace it (e.g. Jakov's world builder export):

  mesh/node name  COL_<surf>  -> physics collider (also rendered), extras.tarmac = {"collide": true, "surf": <id>}
  mesh/node name  VIS_<class> -> render only,                      extras.tarmac = {"collide": false, "cls": <id>}
  material name   surf_<name> -> picks the showcase material preset
  TEXCOORD_0 (optional)       -> metres (u along track, v lateral) for road-like surfaces; triplanar otherwise
  extras on the scene: {"tarmac": {"spawns": [[x,y,z,yaw],...], "line": [[x,y,z],...]}}  (Tarmac frame: Y up)

Frames: DriveChain (x east, y north, z up) -> Tarmac/three.js (X, Y up, Z) = (x, z, -y)  (proper rotation).
"""
import json, struct, sys, math, os
import numpy as np

SURF = {0:'tarmac',1:'kerb',2:'grass',3:'rough',4:'sausage',5:'line',6:'pave',7:'pad',8:'gravel',9:'wall',10:'other',11:'concrete',
        32:'foliage',33:'tyres',34:'fence'}
ROADLIKE = {0,1,4,5,6,7,11}

def read_dc(path):
    b = open(path,'rb').read(); assert b[:8]==b'DCMESH01', path
    nv, nt, _ = struct.unpack('<3i', b[8:20]); o = 20
    v = np.frombuffer(b[o:o+nv*24],'<f8').reshape(-1,3); o += nv*24
    t = np.frombuffer(b[o:o+nt*12],'<i4').reshape(-1,3).astype(np.int64); o += nt*12
    s = np.frombuffer(b[o:o+nt],'u1').copy()
    return v, t, s

def to_tm(v): return np.stack([v[:,0], v[:,2], -v[:,1]], 1)

def vnormals(p, t):
    fn = np.cross(p[t[:,1]]-p[t[:,0]], p[t[:,2]]-p[t[:,0]])
    n = np.zeros_like(p); 
    for k in range(3): np.add.at(n, t[:,k], fn)
    l = np.linalg.norm(n, axis=1, keepdims=True); l[l<1e-12] = 1
    n = n/l; n[np.linalg.norm(n,axis=1)<.5] = [0,1,0]; return n

def track_uv(p, line):
    """u = arc length (m) of nearest AI-line point (+ projection), v = signed lateral offset (m, + = left)."""
    L = line[:, [0,2]]; seg = np.roll(L,-1,0) - L; sl = np.linalg.norm(seg,axis=1); s0 = np.concatenate([[0],np.cumsum(sl)[:-1]])
    # grid-accelerated nearest segment
    q = p[:, [0,2]]; best = np.full(len(q), 1e18); u = np.zeros(len(q)); v = np.zeros(len(q))
    for i in range(len(L)):
        a = L[i]; d = seg[i]; ll = max(sl[i]**2, 1e-9)
        tt = np.clip(((q-a)@d)/ll, 0, 1); c = a + tt[:,None]*d; e = q-c; dist = (e**2).sum(1)
        m = dist < best
        if m.any():
            best[m] = dist[m]; u[m] = s0[i] + tt[m]*sl[i]
            nrm = np.array([-d[1], d[0]])/max(sl[i],1e-9)   # left normal in (X,Z): forward (dx,dz) -> left = (dz,-dx)? computed below
            v[m] = (e[m] @ np.array([d[1], -d[0]])) / max(sl[i],1e-9)
    return np.stack([u, v], 1).astype(np.float32), float(s0[-1]+sl[-1])

class GLB:
    def __init__(s): s.bin = bytearray(); s.views=[]; s.acc=[]; s.meshes=[]; s.nodes=[]; s.mats={}; s.matlist=[]
    def view(s, data, target=None):
        while len(s.bin)%4: s.bin.append(0)
        off = len(s.bin); s.bin += data; v = {'buffer':0,'byteOffset':off,'byteLength':len(data)}
        if target: v['target']=target
        s.views.append(v); return len(s.views)-1
    def accessor(s, arr, ctype, typ, target, minmax=False, normalized=False):
        bv = s.view(arr.tobytes(), target); a = {'bufferView':bv,'componentType':ctype,'count':int(arr.shape[0]),'type':typ}
        if normalized: a['normalized']=True
        if minmax: a['min']=arr.min(0).tolist(); a['max']=arr.max(0).tolist()
        s.acc.append(a); return len(s.acc)-1
    def mat(s, name):
        if name not in s.mats: s.mats[name]=len(s.matlist); s.matlist.append({'name':name,'pbrMetallicRoughness':{'baseColorFactor':[.5,.5,.5,1],'metallicFactor':0,'roughnessFactor':.9}})
        return s.mats[name]
    def add(s, name, p, t, n=None, uv=None, extras=None, matname='x'):
        attrs = {'POSITION': s.accessor(p.astype(np.float32),5126,'VEC3',34962,True)}
        if n is not None: attrs['NORMAL'] = s.accessor((np.clip(n,-1,1)*127).round().astype(np.int8).repeat(1,0).reshape(-1,3).copy(),5120,'VEC3',34962,normalized=True) if False else s.accessor(n.astype(np.float32),5126,'VEC3',34962)
        if uv is not None: attrs['TEXCOORD_0'] = s.accessor(uv.astype(np.float32),5126,'VEC2',34962)
        idx = s.accessor(t.astype(np.uint32).reshape(-1),5125,'SCALAR',34963)
        s.meshes.append({'name':name,'primitives':[{'attributes':attrs,'indices':idx,'material':s.mat(matname)}],'extras':extras or {}})
        s.nodes.append({'name':name,'mesh':len(s.meshes)-1,'extras':extras or {}})
    def write(s, path, scene_extras):
        js = {'asset':{'version':'2.0','generator':'drivechain-showcase dcmesh_to_glb.py'},'scene':0,
              'scenes':[{'nodes':list(range(len(s.nodes))),'extras':scene_extras}],'nodes':s.nodes,'meshes':s.meshes,
              'materials':s.matlist,'accessors':s.acc,'bufferViews':s.views,'buffers':[{'byteLength':len(s.bin)}]}
        j = json.dumps(js,separators=(',',':')).encode(); j += b' '*((4-len(j)%4)%4)
        while len(s.bin)%4: s.bin.append(0)
        with open(path,'wb') as f:
            f.write(struct.pack('<III',0x46546C67,2,12+8+len(j)+8+len(s.bin))); f.write(struct.pack('<II',len(j),0x4E4F534A)); f.write(j)
            f.write(struct.pack('<II',len(s.bin),0x004E4942)); f.write(s.bin)

def compact(p, t):
    used = np.unique(t); remap = -np.ones(len(p), np.int64); remap[used] = np.arange(len(used)); return p[used], remap[t]

def main(src, out):
    cfg = dict(l.split('=',1) for l in open(os.path.join(src,'track.cfg')) if '=' in l and not l.startswith('#'))
    cfg = {k.strip(): v.split(';')[0].strip() for k,v in cfg.items()}
    line = np.loadtxt(os.path.join(src,'line.csv'), delimiter=',', comments='#'); lineT = to_tm(line)
    g = GLB()
    v, t, s = read_dc(os.path.join(src,'collider.bin')); p = to_tm(v)
    fn = np.cross(p[t[:,1]]-p[t[:,0]], p[t[:,2]]-p[t[:,0]]); flip = (fn[:,1] < 0) & (s != 9); t[flip] = t[flip][:, [0,2,1]]
    uvAll, L = track_uv(p, lineT)
    for sid in sorted(set(s.tolist())):
        pp, tt = compact(p, t[s==sid]); n = vnormals(pp, tt)
        used = np.unique(t[s==sid]); uv = uvAll[used] if sid in ROADLIKE else None
        g.add(f'COL_{SURF[sid]}', pp, tt, n, uv, {'tarmac':{'collide':True,'surf':int(sid)}}, f'surf_{SURF[sid]}')
        print('COL', SURF[sid], len(tt), 'tris', len(pp), 'verts')
    sc = os.path.join(src, cfg.get('scenery','scenery.bin'))
    if os.path.exists(sc):
        v, t, s = read_dc(sc); p = to_tm(v)
        ok = np.isfinite(p[t]).all(axis=(1,2)) & (np.abs(p[t]).max(axis=(1,2)) < 1e5); t, s = t[ok], s[ok]
        for sid in sorted(set(s.tolist())):
            pp, tt = compact(p, t[s==sid]); n = vnormals(pp, tt) if sid in (32,33) else None
            g.add(f'VIS_{SURF.get(sid,"other")}', pp, tt, n, None, {'tarmac':{'collide':False,'cls':int(sid)}}, f'surf_{SURF.get(sid,"other")}')
            print('VIS', SURF.get(sid,'other'), len(tt), 'tris')
    spawns = []
    for k in range(8):
        if f'spot{k}' in cfg:
            name, x, y, h = cfg[f'spot{k}'].split()[:4]; x, y, h = float(x), float(y), math.radians(float(h))
            # DC heading h (CCW from +x east) -> Tarmac forward (cos h, 0, -sin h) -> yaw = atan2(fx, fz)
            spawns.append({'name':name,'x':x,'z':-y,'yaw':math.atan2(math.cos(h), -math.sin(h))})
    extras = {'tarmac':{'name':cfg.get('name','map'),'frame':'tarmac-yup-zfwd','spawns':spawns,'trackLength':L,
              'line':np.round(lineT,3).tolist(),'source':'PRIVATE mod-derived (Ušće) - never publish'}}
    g.write(out, extras); print('wrote', out, os.path.getsize(out)//1024, 'KiB')

if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv)>1 else '/workspace/drivechain/private/tracks/usce',
         sys.argv[2] if len(sys.argv)>2 else '/workspace/drivechain-showcase/private-assets/usce/usce.glb')
