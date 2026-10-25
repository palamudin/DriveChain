"""objload.py - minimal OBJ loader for the aero prototypes (numpy only). Geometry stays in memory; nothing is written out."""
import re
import numpy as np

import os
# Groups that are not part of the exterior aero surface. Generic names by default; a mesh-specific
# regex (one line) can be supplied via $AERO_DROP_FILE or private/aero/drop_groups.txt (kept private).
_DEFAULT_DROP = r"interior|cockpit|steer|gauge|needle|belt|seat|carpet|dash|gear|handbrake|button|bulb|collider"
def _drop_regex():
    here = os.path.dirname(os.path.abspath(__file__))
    for f in (os.environ.get("AERO_DROP_FILE"), os.path.join(here, "..", "..", "private", "aero", "drop_groups.txt")):
        if f and os.path.isfile(f):
            return open(f).read().strip()
    return _DEFAULT_DROP
DROP = re.compile(_drop_regex(), re.I)

def load(path, offset=(0, 0, 0), frame="yup", exterior=True):
    V, F, G, g = [], [], [], "default"
    for ln in open(path, errors="replace"):
        if ln.startswith("v "): V.append([float(t) for t in ln.split()[1:4]])
        elif ln.startswith(("g ", "o ")): g = ln.split(None, 1)[1].strip() if len(ln.split()) > 1 else "default"
        elif ln.startswith("f "):
            idx = [int(t.split("/")[0]) for t in ln.split()[1:]]
            idx = [i - 1 if i > 0 else len(V) + i for i in idx]
            for k in range(1, len(idx) - 1): F.append((idx[0], idx[k], idx[k + 1])); G.append(g)
    V = np.array(V); F = np.array(F); G = np.array(G)
    if frame == "yup": V = V[:, [2, 0, 1]]     # y-up model (x left, y up, z fwd) -> physics (x fwd, y left, z up)
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
