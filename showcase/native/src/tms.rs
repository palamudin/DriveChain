//! FFI to the Tarmac core through our C API (tmcore/api/tms.h). Same API the wasm build uses.
#![allow(dead_code)]
use std::ffi::{c_char, c_int, c_void, CStr};

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct Input { pub throttle: f64, pub brake: f64, pub steer: f64, pub handbrake: f64, pub clutch: f64, pub gear: i32, pub starter: i32 }

extern "C" {
    fn tms_version() -> *const c_char;
    fn tms_last_error() -> *const c_char;
    fn tms_world_create(p: *const f32, nv: u32, idx: *const u32, nt: u32, surf: *const u8) -> *mut c_void;
    fn tms_world_destroy(w: *mut c_void);
    fn tms_world_height(w: *mut c_void, x: f64, y: f64, z: f64) -> f64;
    fn tms_car_create(w: *mut c_void, car: c_int, tune: c_int, x: f64, z: f64, yaw: f64, y: f64) -> *mut c_void;
    fn tms_car_destroy(c: *mut c_void);
    fn tms_car_reset(c: *mut c_void, x: f64, z: f64, yaw: f64, y: f64) -> i32;
    fn tms_car_step(c: *mut c_void, i: *const Input, ticks: i32) -> i32;
    fn tms_car_state(c: *mut c_void, out: *mut f32) -> i32;
    fn tms_tick_hz() -> f64;
    fn tms_car_set_assists(c: *mut c_void, abs: c_int, esp: c_int) -> i32;
}
fn err() -> String { unsafe { CStr::from_ptr(tms_last_error()).to_string_lossy().into_owned() } }
pub fn version() -> String { unsafe { CStr::from_ptr(tms_version()).to_string_lossy().into_owned() } }
pub fn tick_hz() -> f64 { unsafe { tms_tick_hz() } }

pub struct World(*mut c_void);
unsafe impl Send for World {}
impl World {
    pub fn new(pos: &[f32], idx: &[u32], surf: &[u8]) -> anyhow::Result<World> {
        let w = unsafe { tms_world_create(pos.as_ptr(), (pos.len() / 3) as u32, idx.as_ptr(), (idx.len() / 3) as u32, surf.as_ptr()) };
        if w.is_null() { anyhow::bail!("tms_world_create: {}", err()) } Ok(World(w))
    }
    pub fn height(&self, x: f64, y: f64, z: f64) -> f64 { unsafe { tms_world_height(self.0, x, y, z) } }
}
impl Drop for World { fn drop(&mut self) { unsafe { tms_world_destroy(self.0) } } }

/// Packed state, see tms.h: [0..3] pos, [3..7] quat xyzw, [7..10] vel, 10 speed, 11 rpm, 12 gear, 13 running, 14 t, 15 tick,
/// then per wheel (16 + 16*w): centre xyz, quat xyzw, angle, omega, compression, load, slip, fx, fy, contact, semantic role.
#[derive(Clone, Copy)]
pub struct State(pub [f32; 80]);
impl Default for State { fn default() -> Self { State([0.0; 80]) } }
impl State {
    pub fn pos(&self) -> glam::Vec3 { glam::Vec3::new(self.0[0], self.0[1], self.0[2]) }
    pub fn quat(&self) -> glam::Quat { glam::Quat::from_xyzw(self.0[3], self.0[4], self.0[5], self.0[6]) }
    pub fn speed(&self) -> f32 { self.0[10] }
    pub fn rpm(&self) -> f32 { self.0[11] }
    pub fn gear(&self) -> i32 { self.0[12] as i32 }
    pub fn running(&self) -> bool { self.0[13] > 0.5 }
    pub fn wheel(&self, w: usize) -> glam::Vec3 { let b = 16 + w * 16; glam::Vec3::new(self.0[b], self.0[b + 1], self.0[b + 2]) }
    pub fn wheel_quat(&self, w: usize) -> glam::Quat { let b = 16 + w * 16; glam::Quat::from_xyzw(self.0[b + 3], self.0[b + 4], self.0[b + 5], self.0[b + 6]) }
    pub fn wheel_angle(&self, w: usize) -> f32 { self.0[16 + w * 16 + 7] }
    pub fn wheel_contact(&self, w: usize) -> bool { self.0[16 + w * 16 + 14] > 0.5 }
}

pub struct Car(*mut c_void);
unsafe impl Send for Car {}
impl Car {
    pub fn new(w: &World, car_id: i32, tune: i32, x: f64, z: f64, yaw: f64) -> anyhow::Result<Car> {
        let y = w.height(x, 500.0, z);
        let c = unsafe { tms_car_create(w.0, car_id, tune, x, z, yaw, y + 1.0) };
        if c.is_null() { anyhow::bail!("tms_car_create: {}", err()) } Ok(Car(c))
    }
    pub fn step(&mut self, i: &Input, ticks: i32) -> anyhow::Result<i32> { let r = unsafe { tms_car_step(self.0, i, ticks) }; if r < 0 { anyhow::bail!("step: {}", err()) } Ok(r) }
    pub fn state(&self) -> State { let mut s = State::default(); unsafe { tms_car_state(self.0, s.0.as_mut_ptr()) }; s }
    pub fn reset(&mut self, w: &World, x: f64, z: f64, yaw: f64) -> i32 { let y = w.height(x, 500.0, z); unsafe { tms_car_reset(self.0, x, z, yaw, y + 1.0) } }
    pub fn set_assists(&mut self, abs: bool, esp: bool) { unsafe { tms_car_set_assists(self.0, abs as c_int, esp as c_int); } }
}
impl Drop for Car { fn drop(&mut self) { unsafe { tms_car_destroy(self.0) } } }
