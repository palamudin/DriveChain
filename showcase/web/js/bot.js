// bot.js - pure-pursuit attract-mode driver on the world's racing line (scene extras.tarmac.line). Used for demo + stills.
export class Bot {
  constructor(line) { this.line = line; this.i = 0; this.speedCap = 30; }
  nearest(x, z) { let best = 1e18, bi = this.i; const N = this.line.length;
    for (let k = -30; k < 60; k++) { const j = (this.i + k + N) % N, p = this.line[j]; const d = (p[0] - x) ** 2 + (p[2] - z) ** 2; if (d < best) { best = d; bi = j; } }
    if (best > 400) for (let j = 0; j < N; j++) { const p = this.line[j]; const d = (p[0] - x) ** 2 + (p[2] - z) ** 2; if (d < best) { best = d; bi = j; } }
    this.i = bi; return bi; }
  drive(car) { // car: {pos:[x,z], fwd:[fx,fz], speed}
    const N = this.line.length, i = this.nearest(car.x, car.z), look = Math.max(6, car.speed * 0.9);
    let j = i, acc = 0; while (acc < look) { const a = this.line[j], b = this.line[(j + 1) % N]; acc += Math.hypot(b[0] - a[0], b[2] - a[2]); j = (j + 1) % N; }
    const t = this.line[j], dx = t[0] - car.x, dz = t[2] - car.z;
    const fwd = car.fx * dx + car.fz * dz, left = car.lx * dx + car.lz * dz; // left vector supplied by caller
    const ang = Math.atan2(left, fwd), steer = Math.max(-1, Math.min(1, 2.2 * ang)); // +left
    // curvature ahead -> target speed
    let kmax = 0; for (let s = 0, q = i; s < 40; s++) { const a = this.line[q], b = this.line[(q + 4) % N], c = this.line[(q + 8) % N];
      const h1 = Math.atan2(b[2] - a[2], b[0] - a[0]), h2 = Math.atan2(c[2] - b[2], c[0] - b[0]); let d = Math.abs(Math.atan2(Math.sin(h2 - h1), Math.cos(h2 - h1)));
      const len = Math.hypot(c[0] - a[0], c[2] - a[2]) || 1; kmax = Math.max(kmax, d / len * (1 - s / 60)); q = (q + 2) % N; }
    const vT = Math.min(this.speedCap, Math.sqrt(7.0 / Math.max(kmax, 1e-3)));
    const e = vT - car.speed; return { steer, throttle: e > 0 ? Math.min(1, e * 0.35) : 0, brake: e < -1.5 ? Math.min(1, -e * 0.2) : 0 };
  }
}
