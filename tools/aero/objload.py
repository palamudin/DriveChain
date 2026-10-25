"""objload.py - minimal OBJ loader for the aero prototypes (numpy only). Geometry stays in memory; nothing is written out."""
import re
import numpy as np

DROP = re.compile(r"damage|blur|light_beam|int_|_int|cockpit|steer|gauge|arrow_|needle|cinture|belt|seat|carpet|dash|gear|handbrake|"
                  r"interior|button|white_circle|mirror_middle|bulb|szelepsapka|collider", re.I)

def load(path, offset=(0, 0, 0), frame="ac", exterior=True):
    V, F, G, g = [], [], [], "default"
    for ln in open(path, errors="replace"):
        if ln.startswith("v "): V.append([float(t) for t in ln.split()[1:4]])
        elif ln.startswith(("g ", "o ")): g = ln.split(None, 1)[1].strip() if len(ln.split()) > 1 else "default"
        elif ln.startswith("f "):
            idx = [int(t.split("/")[0]) for t in ln.split()[1:]]
            idx = [i - 1 if i > 0 else len(V) + i for i in idx]
            for k in range(1, len(idx) - 1): F.append((idx[0], idx[k], idx[k + 1])); G.append(g)
    V = np.array(V); F = np.array(F); G = np.array(G)
    if frame == "ac": V = V[:, [2, 0, 1]]     # AC model (x left, y up, z fwd) -> physics (x fwd, y left, z up)
    V = V + np.asarray(offset, float)
    if exterior:
        k = np.array([not DROP.search(x) for x in G]); F, G = F[k], G[k]
    return V, F, G

def wheel_x(V, F, G):
    """front/rear axle x and ground z from the tyre groups."""
    xs = {}
    for g in set(G):
        if g.lower().startswith("tyre"):
            P = V[np.unique(F[G == g])]; xs[g] = (P[:, 0].mean(), P[:, 2].min())
    xf = np.mean([v[0] for v in xs.values() if v[0] > 0]); xr = np.mean([v[0] for v in xs.values() if v[0] < 0])
    return xf, xr, min(v[1] for v in xs.values())
