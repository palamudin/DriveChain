#!/usr/bin/env python3
"""dcdrive.py - Python client for the drivechain JSON-lines driver protocol (docs/DRIVER_INTERFACE.md).

Drives the car through the plug-in driver (finite-strength hands on the rim + pedals), exactly like a human
would, either on the headless deterministic host (build/drvhost, lockstep) or on the live demo (--drive-port,
real time, human can take over at any moment).

    from dcdrive import DriveChain
    with DriveChain.spawn() as dc:                    # headless lockstep host (default car cars/mx5_na.ini)
        dc.reset(spot="skidpad", speed_kmh=40)
        dc.path_circle()                              # skidpad circle
        dc.drive(steer="path", pedals="speed", speed_kmh=40)
        dc.run(10.0, log="skid.csv")
        print(dc.metrics())

    python3 tools/dcdrive.py list
    python3 tools/dcdrive.py run skidpad --speed 50 --out results/driver
    python3 tools/dcdrive.py selftest --out results/driver      (make drvtest)
    python3 tools/dcdrive.py run lane_change --tcp 127.0.0.1:7878   (live demo)
"""
import argparse, json, math, os, socket, subprocess, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


class DriveChain:
    def __init__(self, rfile, wfile, closer=None, lockstep=True):
        self._r, self._w, self._closer = rfile, wfile, closer
        self.info = self.cmd("hello")
        self.lockstep = bool(self.info.get("lockstep", lockstep))

    # ---- transports ----
    @classmethod
    def spawn(cls, car=None, sets=(), host=None, layout="proving"):
        name = "drvhost.exe" if os.name == "nt" else "drvhost"
        exe = host or next((c for c in (os.path.join(ROOT, "build", name), os.path.join(ROOT, name)) if os.path.exists(c)), os.path.join(ROOT, "build", name))
        args = [exe, "--layout", layout]
        if car: args += ["--car", car]
        for s in sets: args += ["--set", s]
        p = subprocess.Popen(args, cwd=ROOT, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        return cls(p.stdout, p.stdin, closer=lambda: (p.stdin.close(), p.wait(5)))

    @classmethod
    def connect(cls, addr="127.0.0.1:7878", timeout=10.0):
        h, _, p = addr.rpartition(":")
        s = socket.create_connection((h or "127.0.0.1", int(p)), timeout=timeout)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        f = s.makefile("rw", buffering=1, newline="\n")
        return cls(f, f, closer=lambda: (f.close(), s.close()), lockstep=False)

    def close(self):
        try: self.cmd("quit") if self.lockstep else None
        except Exception: pass
        if self._closer: self._closer()
        self._closer = None

    def __enter__(self): return self
    def __exit__(self, *a): self.close()

    # ---- protocol ----
    def cmd(self, name, **kw):
        kw["cmd"] = name
        self._w.write(json.dumps(kw) + "\n"); self._w.flush()
        while True:
            line = self._r.readline()
            if not line: raise ConnectionError("driver host closed the connection")
            r = json.loads(line)
            if "t" in r and name not in ("get", "step", "reset") and "duration" not in r and "protocol" not in r and "engaged" not in r and "rebuilt" not in r and "points" not in r and "logging" not in r and "hz" not in r and "gear" not in r and "bye" not in r:
                continue   # a streamed telemetry frame (live demo) - skip until our reply arrives
            if not r.get("ok", False): raise RuntimeError(f"{name}: {r.get('error')}")
            return r

    def reset(self, **kw): return self.cmd("reset", **kw)
    def drive(self, **kw): return self.cmd("drive", **kw)
    def get(self): return self.cmd("get")
    def metrics(self): return self.cmd("metrics")
    def metrics_reset(self): return self.cmd("metrics_reset")
    def set(self, key, value): return self.cmd("set", key=key, value=str(value))
    def log(self, file=None, every=1): return self.cmd("log", file=file or "", every=every)
    def path_track(self): return self.cmd("path", type="track")
    def path_circle(self, cx=None, cy=None, r=None, dir=1):
        kw = {k: v for k, v in dict(cx=cx, cy=cy, r=r).items() if v is not None}
        return self.cmd("path", type="circle", dir=dir, **kw)
    def path_line(self, x, y, yaw_deg, length=1000): return self.cmd("path", type="line", x=x, y=y, yaw_deg=yaw_deg, length=length)
    def path_points(self, pts, closed=False):
        flat = [round(c, 4) for p in pts for c in p]
        return self.cmd("path", type="points", pts=flat, closed=int(closed))
    def spot(self, name):
        for s in self.info["spots"]:
            if s["name"] == name: return s
        raise KeyError(name)

    def step(self, seconds=None, n=None):
        if self.lockstep:
            return self.cmd("step", seconds=seconds) if seconds is not None else self.cmd("step", n=n or 1)
        time.sleep(seconds if seconds is not None else (n or 1) * 1e-3)
        return self.get()

    def run(self, seconds, every=0.01, log=None, until=None):
        """advance `seconds`, sampling telemetry every `every` s; returns the list of frames.
        until(frame) -> True stops early."""
        if log: self.log(log)
        frames, t_end = [], None
        k = max(1, int(round(every / 1e-3)))
        n = int(round(seconds / (k * 1e-3)))
        for _ in range(n):
            f = self.step(n=k) if self.lockstep else (time.sleep(every), self.get())[1]
            frames.append(f)
            if until and until(f): break
        if log: self.log(None)
        return frames


# ---------------- manoeuvres ----------------
def _cl(pts_s_d, dc):
    """centreline offset points -> world xy using the reset spot frame (start straight is along +x at y=0)."""
    return pts_s_d

def m_skidpad(dc, speed_kmh=40, seconds=14.0, **_):
    sp = dc.spot("skidpad")
    dc.reset(spot="skidpad", speed_kmh=speed_kmh)
    dc.path_circle(dir=1)
    dc.drive(steer="path", pedals="speed", speed_kmh=speed_kmh)
    return seconds

def m_step_steer(dc, speed_kmh=80, target_deg=45, seconds=6.0, **_):
    dc.reset(spot="start", speed_kmh=speed_kmh)
    dc.drive(steer="angle", target_deg=0, pedals="speed", speed_kmh=speed_kmh)
    dc.step(seconds=1.0) if dc.lockstep else time.sleep(1.0)
    dc.metrics_reset()
    dc.drive(steer="angle", target_deg=target_deg, rate_max_dps=500)
    return seconds

def m_sine(dc, speed_kmh=80, amp_deg=40, freq=0.5, seconds=8.0, **_):
    dc.reset(spot="start", speed_kmh=speed_kmh)
    dc.drive(steer="angle", target_deg=0, wave_amp_deg=amp_deg, wave_freq=freq, wave_delay=0.5, pedals="speed", speed_kmh=speed_kmh)
    return seconds

def m_lane_change(dc, speed_kmh=60, seconds=8.0, **_):
    """ISO 3888-1 style double lane change on the start straight (offset 3.5 m), built as a points path."""
    sp = dc.spot("start"); x0, y0 = sp["x"], sp["y"]
    pts, L1, T1, L2, T2, L3 = [], 15 + 15, 30, 25, 25, 40   # entry+approach, shift, hold, shift back, exit
    x = 0.0
    def seg(xa, xb, ya, yb):
        n = int((xb - xa) / 1.0)
        for i in range(n):
            u = i / n; s = 0.5 - 0.5 * math.cos(math.pi * u)
            pts.append((x0 + xa + (xb - xa) * u, y0 + ya + (yb - ya) * s))
    seg(0, L1, 0, 0); seg(L1, L1 + T1, 0, 3.5); seg(L1 + T1, L1 + T1 + L2, 3.5, 3.5)
    seg(L1 + T1 + L2, L1 + T1 + L2 + T2, 3.5, 0); seg(L1 + T1 + L2 + T2, L1 + T1 + L2 + T2 + L3 + 150, 0, 0)
    dc.reset(spot="start", speed_kmh=speed_kmh)
    dc.path_points(pts)
    dc.drive(steer="path", pedals="speed", speed_kmh=speed_kmh)
    return seconds

def m_slalom(dc, speed_kmh=50, seconds=12.0, **_):
    cones = dc.info["cones"]
    if not cones: raise RuntimeError("layout has no slalom cones")
    sp = dc.spot("slalom"); x0, y0 = sp["x"], sp["y"]
    cs = sorted(cones, key=lambda c: c[0]); amp = 2.2
    pts = [(x0 + i, y0) for i in range(0, int(cs[0][0] - x0 - 9), 1)]
    for i, (cx, cy) in enumerate(cs):
        side = 1 if i % 2 == 0 else -1
        pts.append((cx, cy + side * amp))
    last = cs[-1][0]
    pts += [(last + 9 + i, y0) for i in range(0, 120, 3)]
    # resample a smooth path (Catmull-Rom) every ~1 m
    sm = []
    P = [pts[0]] + pts + [pts[-1]]
    for i in range(1, len(P) - 2):
        p0, p1, p2, p3 = P[i - 1], P[i], P[i + 1], P[i + 2]
        seglen = math.hypot(p2[0] - p1[0], p2[1] - p1[1]); n = max(1, int(seglen))
        for k in range(n):
            t = k / n; t2, t3 = t * t, t * t * t
            sm.append(tuple(0.5 * ((2 * p1[j]) + (-p0[j] + p2[j]) * t + (2 * p0[j] - 5 * p1[j] + 4 * p2[j] - p3[j]) * t2 + (-p0[j] + 3 * p1[j] - 3 * p2[j] + p3[j]) * t3) for j in (0, 1)))
    dc.reset(spot="slalom", speed_kmh=speed_kmh)
    dc.path_points(sm)
    dc.drive(steer="path", pedals="speed", speed_kmh=speed_kmh)
    return seconds

def m_lift_off(dc, speed_kmh=55, seconds=8.0, ramp_s=8.0, hold_s=4.0, **_):
    """lift-off oversteer on the skidpad: path-hold at speed, then throttle snapped shut (catch ability).
    Entry: start at 30 km/h on the circle and ramp the speed target to speed_kmh over ramp_s, then hold hold_s, so the
    lift happens from a settled steady state (a reset at full speed straight onto the circle starts a turn-in transient
    the path driver is still oscillating from 4 s later; at 58-60 km/h that transient alone spun the car before the lift,
    in both cores and with both tyres). The circle is R = 40 m: 55 km/h is 0.60 g, 60 km/h 0.71 g, against the ~0.78 g
    skidpad limit of mx5_na.
    pre_lift_* record the last second of the hold (|beta| max, sw range); pre_lift_ok = 0 flags an unsettled entry so a
    pre-lift loss is never read as a lift-off spin."""
    v0 = min(30.0, speed_kmh)
    dc.reset(spot="skidpad", speed_kmh=v0)
    dc.path_circle(dir=1)
    dc.drive(steer="path", pedals="speed", speed_kmh=v0)
    if not dc.lockstep:
        dc.drive(steer="path", pedals="speed", speed_kmh=speed_kmh); time.sleep(ramp_s + hold_s); dc.extra = {}
    else:
        n = max(1, int(round(ramp_s / 0.25)))
        for k in range(1, n + 1):
            dc.drive(steer="path", pedals="speed", speed_kmh=v0 + (speed_kmh - v0) * k / n); dc.step(seconds=0.25)
        dc.step(seconds=max(0.0, hold_s - 1.0))
        b, sw = [], []
        for _ in range(10):
            dc.step(seconds=0.1); g = dc.get(); b.append(abs(g["beta_deg"])); sw.append(g["sw_deg"])
        dc.extra = dict(pre_lift_beta_deg=round(max(b), 3), pre_lift_sw_range_deg=round(max(sw) - min(sw), 2),
                        pre_lift_ok=int(max(b) < 5.0 and max(sw) - min(sw) < 20.0))
    dc.metrics_reset()
    dc.drive(pedals="direct", throttle=0.0, brake=0.0)
    return seconds

def m_bump_hold(dc, speed_kmh=60, seconds=6.0, steer="angle", **_):
    """single-wheel bump (straight 2, left bump at s_str2+120) with the hands holding straight ahead: kickback"""
    s = dc.info["s_str2"] + 120 - 45
    dc.reset(s=s, d=0, speed_kmh=speed_kmh)
    dc.path_track()
    dc.drive(steer=steer, target_deg=0, pedals="speed", speed_kmh=speed_kmh)
    return seconds

def m_track_lap(dc, speed_kmh=70, seconds=40.0, **_):
    dc.reset(spot="start", speed_kmh=speed_kmh)
    dc.path_track()
    dc.drive(steer="path", pedals="speed", speed_kmh=speed_kmh)
    return seconds

MANOEUVRES = dict(skidpad=m_skidpad, step_steer=m_step_steer, sine=m_sine, lane_change=m_lane_change,
                  slalom=m_slalom, lift_off=m_lift_off, bump_hold=m_bump_hold, track_lap=m_track_lap)


def run_manoeuvre(dc, name, out=None, tag=None, **kw):
    seconds = MANOEUVRES[name](dc, **kw)
    dc.metrics_reset() if name not in ("step_steer", "lift_off") else None
    tag = tag or name
    csv = os.path.join(out, f"drv_{tag}.csv") if out else None
    if csv: os.makedirs(out, exist_ok=True)
    frames = dc.run(seconds, every=0.01, log=os.path.abspath(csv) if (csv and dc.lockstep) else None)
    m = dc.metrics(); m.update(getattr(dc, "extra", None) or {}); dc.extra = None; m["manoeuvre"] = name; m["tag"] = tag; m.update({k: v for k, v in kw.items() if isinstance(v, (int, float, str))})
    if out:
        with open(os.path.join(out, f"drv_{tag}_metrics.json"), "w") as f: json.dump(m, f, indent=1)
        if not dc.lockstep:   # live: log the sampled frames ourselves
            keys = ["t", "x", "y", "yaw_deg", "speed_kmh", "yaw_rate_dps", "beta_deg", "ay", "sw_deg", "column_nm", "hand_nm", "lat_err", "takeover"]
            with open(os.path.join(out, f"drv_{tag}.csv"), "w") as f:
                f.write(",".join(keys) + "\n")
                for fr in frames: f.write(",".join(str(fr.get(k, "")) for k in keys) + "\n")
    return m, frames


def plot(out, tags, title, png):
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    except ImportError:
        return False
    import csv
    fig, ax = plt.subplots(2, 2, figsize=(12, 8))
    for tag in tags:
        p = os.path.join(out, f"drv_{tag}.csv")
        if not os.path.exists(p): continue
        rows = list(csv.DictReader(open(p)))[::10]
        g = lambda k: [float(r[k]) for r in rows]
        ax[0, 0].plot(g("x"), g("y"), label=tag)
        ax[0, 1].plot(g("t"), g("lat_err"), label=tag)
        ax[1, 0].plot(g("t"), g("sw_deg"), label=tag + " rim"); ax[1, 0].plot(g("t"), g("rim_cmd_delayed_deg"), "--", lw=0.8)
        ax[1, 1].plot(g("t"), g("hand_nm"), label=tag + " hand"); ax[1, 1].plot(g("t"), g("column_nm"), ":", lw=0.8)
    ax[0, 0].set_title("path (m)"); ax[0, 0].axis("equal"); ax[0, 1].set_title("lateral error to path (m)")
    ax[1, 0].set_title("rim angle (deg) / delayed command (--)"); ax[1, 1].set_title("hand torque (N m) / column torque (:)")
    for a in ax.flat: a.grid(alpha=0.3); a.legend(fontsize=7)
    fig.suptitle(title); fig.tight_layout(); fig.savefig(png, dpi=110); plt.close(fig)
    return True


SELFTEST = [  # (manoeuvre, kwargs, checks: metric -> (op, limit))
    ("skidpad", dict(speed_kmh=40), {"rms_lat_err": ("<", 0.5), "max_beta_deg": ("<", 6), "spin": ("==", 0), "offtrack_frac": ("==", 0)}),
    ("lane_change", dict(speed_kmh=60), {"max_lat_err": ("<", 1.2), "spin": ("==", 0), "offtrack_frac": ("==", 0)}),
    ("slalom", dict(speed_kmh=40), {"max_lat_err": ("<", 1.3), "spin": ("==", 0)}),
    ("track_lap", dict(speed_kmh=60, seconds=55.0), {"rms_lat_err": ("<", 0.3), "offtrack_frac": ("==", 0), "spin": ("==", 0)}),
    ("step_steer", dict(speed_kmh=80, target_deg=45), {"spin": ("==", 0), "max_hand_nm": ("<", 20.5)}),
    # lift-off AT the 0.9 g limit of the r=40 pad: the pass condition is the catch (no spin, rear slip contained, stays on the pad), not line tracking
    ("lift_off", dict(speed_kmh=55), {"spin": ("==", 0), "max_beta_deg": ("<", 8), "max_lat_err": ("<", 3.0), "offtrack_frac": ("==", 0)}),
    ("bump_hold", dict(speed_kmh=60), {"max_sw_deg": ("<", 6), "spin": ("==", 0)}),
]


def selftest(out, car=None):
    os.makedirs(out, exist_ok=True)
    ok_all, rows = True, []
    ops = {"<": lambda a, b: a < b, "==": lambda a, b: a == b, ">": lambda a, b: a > b}
    with DriveChain.spawn(car=car) as dc:
        for name, kw, checks in SELFTEST:
            t0 = time.time(); m, _ = run_manoeuvre(dc, name, out=out, **kw); el = time.time() - t0
            bad = [f"{k}={m[k]:.3g} !{op} {lim}" for k, (op, lim) in checks.items() if not ops[op](m[k], lim)]
            ok_all &= not bad
            rows.append(m)
            print(f"{'PASS' if not bad else 'FAIL'} {name:12s} rms_lat {m['rms_lat_err']:.3f} max_lat {m['max_lat_err']:.3f} m  beta {m['max_beta_deg']:.1f} deg  "
                  f"r {m['max_yaw_rate_dps']:.1f} dps  hand {m['max_hand_nm']:.1f} col {m['max_column_nm']:.1f} N m  sat {m['saturated_frac']:.3f}  "
                  f"sw {m['max_sw_deg']:.0f} deg  ({el:.1f} s wall) {' '.join(bad)}", flush=True)
    keys = ["tag", "duration", "rms_lat_err", "max_lat_err", "max_head_err_deg", "max_column_nm", "max_hand_nm", "saturated_frac", "max_beta_deg",
            "max_yaw_rate_dps", "max_sw_deg", "max_sw_rate_dps", "max_ay", "distance", "offtrack_frac", "spin"]
    with open(os.path.join(out, "driver_summary.csv"), "w") as f:
        f.write(",".join(keys) + "\n")
        for m in rows: f.write(",".join(str(m.get(k, "")) for k in keys) + "\n")
    plot(out, ["skidpad", "lane_change", "slalom"], "drivechain plug-in driver: closed-loop path following (finite-strength hands)", os.path.join(out, "driver_paths.png"))
    plot(out, ["step_steer", "lift_off", "bump_hold"], "drivechain plug-in driver: open-loop / disturbance manoeuvres", os.path.join(out, "driver_events.png"))
    print("DRIVER SELFTEST", "PASS" if ok_all else "FAIL")
    return 0 if ok_all else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", choices=["list", "run", "selftest", "shell"])
    ap.add_argument("manoeuvre", nargs="?")
    ap.add_argument("--tcp", help="connect to a live demo/drvhost at HOST:PORT instead of spawning build/drvhost")
    ap.add_argument("--car"); ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--speed", type=float); ap.add_argument("--out", default="results/driver"); ap.add_argument("--tag")
    a = ap.parse_args()
    if a.what == "list":
        for k, f in MANOEUVRES.items(): print(f"{k:12s} {(f.__doc__ or '').strip()}")
        return 0
    if a.what == "selftest": return selftest(a.out, a.car)
    dc = DriveChain.connect(a.tcp) if a.tcp else DriveChain.spawn(car=a.car, sets=a.set)
    with dc:
        if a.what == "shell":   # raw JSON lines from stdin, replies to stdout
            for line in sys.stdin:
                line = line.strip()
                if not line: continue
                o = json.loads(line); name = o.pop("cmd")
                try: print(json.dumps(dc.cmd(name, **o)))
                except Exception as e: print(json.dumps({"ok": False, "error": str(e)}))
            return 0
        kw = {}
        if a.speed: kw["speed_kmh"] = a.speed
        m, _ = run_manoeuvre(dc, a.manoeuvre, out=a.out, tag=a.tag, **kw)
        print(json.dumps(m, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
