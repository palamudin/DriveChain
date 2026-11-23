//! Simulation glue shared by all modes: physics world/car, bot driver, car visual transforms, chase camera.
use crate::tms::{Car, Input, State, World as PWorld};
use crate::world::World;
use glam::{Mat4, Vec3};

pub struct Sim { pub pw: PWorld, pub car: Car, pub state: State, pub gear: i32, pub shift_cd: f32, pub line: Vec<Vec3>, pub li: usize, pub spawn: crate::world::Spawn }
impl Sim {
    pub fn new(w: &World, car_id: i32, spawn_idx: usize) -> anyhow::Result<Sim> {
        let (p, i, s) = w.collision();
        let t0 = std::time::Instant::now(); let pw = PWorld::new(&p, &i, &s)?;
        eprintln!("physics world: {} tris, built in {:.0} ms", s.len(), t0.elapsed().as_secs_f64() * 1e3);
        let sp = w.spawns[spawn_idx.min(w.spawns.len() - 1)];
        let car = Car::new(&pw, car_id, 0, sp.x, sp.z, sp.yaw)?; let state = car.state();
        Ok(Sim { pw, car, state, gear: 1, shift_cd: 0.0, line: w.line.clone(), li: 0, spawn: sp })
    }
    pub fn basis(&self) -> (Vec3, Vec3, Vec3, Vec3) { // centre, fwd, left, up (from wheel centres; handedness-agnostic)
        let s = &self.state; let w: Vec<Vec3> = (0..4).map(|i| s.wheel(i)).collect();
        let f = (w[0] + w[1]) * 0.5 - (w[2] + w[3]) * 0.5; let centre = (w[0] + w[1] + w[2] + w[3]) * 0.25; let fwd = f.normalize_or(Vec3::Z);
        let up = s.quat() * Vec3::Y; let up = if up.y < 0.3 { Vec3::Y } else { up }; let left = up.cross(fwd).normalize(); (centre, fwd, left, fwd.cross(left).normalize())
    }
    /// pure pursuit on the racing line (same as web/js/bot.js). Returns input with +left steer convention.
    pub fn bot(&mut self) -> Input {
        let (c, f, l, _) = self.basis(); let n = self.line.len(); if n < 3 { return Input { throttle: 0.3, ..Default::default() } }
        let mut best = f32::MAX; let mut bi = self.li;
        for k in 0..90 { let j = (self.li + n + k - 30) % n; let d = (self.line[j] - c).length_squared(); if d < best { best = d; bi = j; } }
        if best > 400.0 { for j in 0..n { let d = (self.line[j] - c).length_squared(); if d < best { best = d; bi = j; } } }
        self.li = bi; let look = (self.state.speed() * 0.9).max(6.0); let (mut j, mut acc) = (bi, 0.0);
        while acc < look { acc += (self.line[(j + 1) % n] - self.line[j]).length(); j = (j + 1) % n; }
        let d = self.line[j] - c; let ang = d.dot(l).atan2(d.dot(f)); let steer = (2.2 * ang).clamp(-1.0, 1.0);
        let mut kmax: f32 = 0.0; let mut q = bi;
        for s in 0..40 { let (a, b, cc) = (self.line[q], self.line[(q + 4) % n], self.line[(q + 8) % n]);
            let h1 = (b.z - a.z).atan2(b.x - a.x); let h2 = (cc.z - b.z).atan2(cc.x - b.x); let dd = (h2 - h1).sin().atan2((h2 - h1).cos()).abs();
            let len = (cc - a).length().max(1.0); kmax = kmax.max(dd / len * (1.0 - s as f32 / 60.0)); q = (q + 2) % n; }
        let vt = (7.0 / kmax.max(1e-3)).sqrt().min(30.0); let e = vt - self.state.speed();
        Input { steer: steer as f64, throttle: if e > 0.0 { (e * 0.35).min(1.0) as f64 } else { 0.0 }, brake: if e < -1.5 { (-e * 0.2).min(1.0) as f64 } else { 0.0 }, ..Default::default() }
    }
    /// Advance by dt with input (+left steer). Auto gearbox like the web build.
    pub fn step(&mut self, mut i: Input, ticks: i32) -> anyhow::Result<()> {
        self.shift_cd -= ticks as f32 / 240.0; let rpm = self.state.rpm();
        if self.shift_cd <= 0.0 { if rpm > 6200.0 && self.gear < 5 { self.gear += 1; self.shift_cd = 0.6; } else if rpm < 2400.0 && self.gear > 1 { self.gear -= 1; self.shift_cd = 0.6; } }
        i.steer = -i.steer; // solver frame is left-handed (Unity): + = right
        i.gear = self.gear; i.starter = (!self.state.running()) as i32;
        self.car.step(&i, ticks)?; self.state = self.car.state(); Ok(())
    }
    pub fn body_matrix(&self) -> Mat4 { let (c, f, l, u) = self.basis(); Mat4::from_cols(f.extend(0.0), l.extend(0.0), u.extend(0.0), c.extend(1.0)) }
    pub fn wheel_matrix(&self, i: usize) -> Mat4 {
        let (_, f, l, u) = self.basis(); let ang = self.state.wheel_angle(i);
        let wq = self.state.wheel_quat(i); let cq = self.state.quat(); let fw = wq * Vec3::Z; let fc = cq * Vec3::Z;
        let mut steer = fc.cross(fw).dot(u).atan2(fc.dot(fw)); if !steer.is_finite() || steer.abs() > 0.8 { steer = 0.0; }
        Mat4::from_cols(f.extend(0.0), l.extend(0.0), u.extend(0.0), self.state.wheel(i).extend(1.0)) * Mat4::from_rotation_z(steer) * Mat4::from_rotation_y(-ang)
    }
    pub fn chase_camera(&self, prev: Option<(Vec3, Vec3)>, dt: f32) -> (Vec3, Vec3) {
        let (c, f, _, _) = self.basis(); let flat = Vec3::new(f.x, 0.0, f.z).normalize_or(Vec3::Z);
        let want = c - flat * 6.2 + Vec3::Y * 2.0; let tgt = c + flat * 2.5 + Vec3::Y * 0.7;
        match prev { Some((p, t)) => { let k = 1.0 - (-dt * 6.0).exp(); (p.lerp(want, k), t.lerp(tgt, (k * 1.6).min(1.0))) } None => (want, tgt) }
    }
}

/// Own procedural car mesh (publishable): body + cabin boxes, 4 wheel cylinders. Car frame: x fwd, y left, z up.
pub fn car_meshes() -> Vec<(Vec<crate::render::Vertex>, Vec<u32>, u32, glam::Vec4, i32)> {
    use crate::render::Vertex;
    fn bx(min: Vec3, max: Vec3) -> (Vec<Vertex>, Vec<u32>) {
        let mut v = vec![]; let mut idx = vec![];
        let faces = [(Vec3::X, Vec3::Y, Vec3::Z), (-Vec3::X, Vec3::Z, Vec3::Y), (Vec3::Y, Vec3::Z, Vec3::X), (-Vec3::Y, Vec3::X, Vec3::Z), (Vec3::Z, Vec3::X, Vec3::Y), (-Vec3::Z, Vec3::Y, Vec3::X)];
        let c = (min + max) * 0.5; let h = (max - min) * 0.5;
        for (n, a, b) in faces { let base = v.len() as u32; for (s, t) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] { let p = c + (n + a * s + b * t) * h; v.push(Vertex { pos: p.to_array(), nrm: n.to_array(), uv: [0.0, 0.0] }); }
            idx.extend([base, base + 1, base + 2, base, base + 2, base + 3]); }
        (v, idx)
    }
    fn cyl(r: f32, w: f32, seg: u32) -> (Vec<Vertex>, Vec<u32>) {
        let mut v = vec![]; let mut idx = vec![];
        for i in 0..=seg { let a = i as f32 / seg as f32 * std::f32::consts::TAU; let (x, z) = (a.cos() * r, a.sin() * r); let n = [a.cos(), 0.0, a.sin()];
            v.push(Vertex { pos: [x, -w / 2.0, z], nrm: n, uv: [0.0, 0.0] }); v.push(Vertex { pos: [x, w / 2.0, z], nrm: n, uv: [0.0, 0.0] }); }
        for i in 0..seg { let b = i * 2; idx.extend([b, b + 1, b + 2, b + 1, b + 3, b + 2]); }
        for side in [-1.0f32, 1.0] { let c0 = v.len() as u32; v.push(Vertex { pos: [0.0, side * w / 2.0, 0.0], nrm: [0.0, side, 0.0], uv: [0.0, 0.0] });
            for i in 0..=seg { let a = i as f32 / seg as f32 * std::f32::consts::TAU; v.push(Vertex { pos: [a.cos() * r * 0.98, side * w / 2.0, a.sin() * r * 0.98], nrm: [0.0, side, 0.0], uv: [0.0, 0.0] }); }
            for i in 0..seg { idx.extend([c0, c0 + 1 + i, c0 + 2 + i]); } }
        (v, idx)
    }
    let paint = glam::Vec4::new(0.45, 0.02, 0.015, 0.3); let glass = glam::Vec4::new(0.01, 0.012, 0.015, 0.05); let tyre = glam::Vec4::new(0.015, 0.015, 0.015, 0.9);
    let (bv, bi) = bx(Vec3::new(-2.15, -0.78, -0.25), Vec3::new(2.15, 0.78, 0.22));
    let (cv, ci) = bx(Vec3::new(-1.2, -0.68, 0.22), Vec3::new(0.45, 0.68, 0.78));
    let (wv, wi) = cyl(0.31, 0.2, 28);
    // -1 = body, 0..3 = wheel index
    vec![(bv, bi, 100, paint, -1), (cv, ci, 102, glass, -1), (wv.clone(), wi.clone(), 101, tyre, 0), (wv.clone(), wi.clone(), 101, tyre, 1), (wv.clone(), wi.clone(), 101, tyre, 2), (wv, wi, 101, tyre, 3)]
}
