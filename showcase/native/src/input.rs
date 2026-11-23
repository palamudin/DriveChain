//! Input: keyboard (winit), gamepads and DirectInput wheels/pedals, mapped through `input.ini`.
//!
//! Windows: XInput (xinput1_4, polled every frame: works without focus, hotplug by re-polling empty slots every 2 s)
//!          + DirectInput 8 for wheels, pedals, shifters and other joysticks (re-enumerated every 2 s for hotplug).
//!          gilrs (Windows.Gaming.Input backend) is NOT used on Windows: it only reported the first device, which on a
//!          wheel rig is usually the wheel, and its trigger/axis layout for non-Xbox devices was unmapped -> "joypad ain't working".
//! Linux:   gilrs (evdev) for both: SDL-mapped gamepads become "pad" devices, everything else is exposed as raw axes.
//!
//! Every device is exposed as raw normalised axes (-1..1; pad triggers 0..1) + buttons, so the overlay (F1 / pad Back)
//! shows exactly what the ini mapping sees.
use std::collections::HashMap;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Class { Pad, Raw }
pub struct Device { pub name: String, pub guid: String, pub backend: &'static str, pub class: Class, pub axes: Vec<f32>, pub buttons: Vec<bool>, pub prev: Vec<bool>, pub alive: bool }
/// Canonical pad layout (XInput order). Axes: LX LY RX RY LT RT. Buttons below.
pub const PAD_AXES: [&str; 6] = ["LX", "LY", "RX", "RY", "LT", "RT"];
pub const PAD_BUTTONS: [&str; 14] = ["A", "B", "X", "Y", "LB", "RB", "Back", "Start", "LS", "RS", "DUp", "DDown", "DLeft", "DRight"];

#[derive(Default, Clone, Copy, Debug)]
pub struct Controls { pub steer: f64, pub throttle: f64, pub brake: f64, pub clutch: f64, pub handbrake: f64, pub gear_up: bool, pub gear_down: bool, pub reset: bool, pub camera: bool, pub overlay: bool, pub active: bool }

pub const DEFAULT_INI: &str = r#"; DriveChain native input mapping. Edit, then restart or press F2 to reload.
; Every connected device is listed in drivechain.log at startup and on hotplug (name, GUID, axes, buttons).
; Press F1 (or the pad's Back/View button) for the live overlay showing raw axis/button values.

[pad]          ; Xbox / XInput pads (Windows) and SDL-mapped pads (Linux)
deadzone = 0.08            ; left-stick radial deadzone (0..1)
steer_linearity = 1.5      ; 1 = linear, 2 = much finer around centre
trigger_deadzone = 0.03
steer = LX                 ; axes: LX LY RX RY LT RT (prefix '-' to invert)
throttle = RT
brake = LT
gear_up = RB               ; buttons: A B X Y LB RB Back Start LS RS DUp DDown DLeft DRight
gear_down = LB
handbrake = X
reset = Y
camera = B
overlay = Back

[wheel]        ; DirectInput wheels, pedals, shifters (any non-XInput controller)
device =                   ; substring of the device name or GUID from drivechain.log; empty = first DirectInput device
; Bindings: [<device substring>/]axisN or buttonN. A prefix selects another device (separate pedal or shifter boxes),
; e.g.  throttle = CSL Pedals/axis1   or   gear_up = Shifter/button0.   Leave empty or 'none' to disable.
; DirectInput axis numbers: 0 X, 1 Y, 2 Z, 3 Rx, 4 Ry, 5 Rz, 6 Slider0, 7 Slider1. POV hat = buttons 32 up 33 right 34 down 35 left.
steer = axis0
steer_invert = 0
steer_deadzone = 0.0
steer_gain = 1.0           ; 900-degree wheel driving a ~540-degree rack: try 1.5
throttle = axis1           ; Logitech G25/G27/G29/G920 defaults; Thrustmaster T300 combined: throttle axis5 brake axis1
throttle_invert = 1        ; most pedals read +1 released, -1 pressed
brake = axis5
brake_invert = 1
clutch = axis6
clutch_invert = 1
pedal_deadzone = 0.02
gear_up = button4          ; paddles (G29: 4 right, 5 left)
gear_down = button5
handbrake =
reset = button23
camera = button3
overlay = button7
"#;

struct Ini(HashMap<String, String>);
impl Ini {
    fn parse(s: &str) -> Ini {
        let mut m = HashMap::new(); let mut sec = String::new();
        for l in s.lines() { let l = l.split(';').next().unwrap_or("").trim(); if l.is_empty() { continue; }
            if l.starts_with('[') { sec = l.trim_matches(|c| c == '[' || c == ']').trim().to_lowercase(); continue; }
            if let Some((k, v)) = l.split_once('=') { m.insert(format!("{}.{}", sec, k.trim().to_lowercase()), v.trim().to_string()); } }
        Ini(m)
    }
    fn s(&self, k: &str) -> String { self.0.get(k).cloned().unwrap_or_default() }
    fn f(&self, k: &str, d: f32) -> f32 { self.0.get(k).and_then(|v| v.parse().ok()).unwrap_or(d) }
    fn b(&self, k: &str) -> bool { matches!(self.s(k).as_str(), "1" | "true" | "yes") }
}

#[derive(Clone, Debug)]
enum Bind { None, Axis { dev: Option<String>, i: usize, neg: bool }, Button { dev: Option<String>, i: usize } }
fn parse_bind(v: &str, pad: bool) -> Bind {
    let v = v.trim(); if v.is_empty() || v.eq_ignore_ascii_case("none") { return Bind::None; }
    let (dev, b) = match v.rsplit_once('/') { Some((d, b)) => (Some(d.trim().to_lowercase()), b.trim()), None => (None, v) };
    let (neg, b) = if let Some(r) = b.strip_prefix('-') { (true, r) } else { (false, b) };
    let bl = b.to_lowercase();
    if pad {
        if let Some(i) = PAD_AXES.iter().position(|n| n.eq_ignore_ascii_case(b)) { return Bind::Axis { dev, i, neg }; }
        if let Some(i) = PAD_BUTTONS.iter().position(|n| n.eq_ignore_ascii_case(b)) { return Bind::Button { dev, i }; }
    }
    if let Some(n) = bl.strip_prefix("axis") { if let Ok(i) = n.parse() { return Bind::Axis { dev, i, neg }; } }
    if let Some(n) = bl.strip_prefix("button") { if let Ok(i) = n.parse() { return Bind::Button { dev, i }; } }
    eprintln!("input.ini: cannot parse binding '{v}'"); Bind::None
}

pub struct Mapping { pad: HashMap<&'static str, Bind>, wheel: HashMap<&'static str, Bind>, dz: f32, lin: f32, tdz: f32, wdev: String, winv: HashMap<&'static str, bool>, sdz: f32, sgain: f32, pdz: f32 }
const ACTIONS: [&str; 10] = ["steer", "throttle", "brake", "clutch", "gear_up", "gear_down", "handbrake", "reset", "camera", "overlay"];
impl Mapping {
    pub fn load(path: &std::path::Path) -> Mapping {
        if !path.exists() { let _ = std::fs::write(path, DEFAULT_INI); crate::logln(&format!("input: wrote default mapping {}", path.display())); }
        let txt = std::fs::read_to_string(path).unwrap_or_else(|_| DEFAULT_INI.into()); let ini = Ini::parse(&txt); let def = Ini::parse(DEFAULT_INI);
        let get = |k: &str| if ini.0.contains_key(k) { ini.s(k) } else { def.s(k) };
        let mut pad = HashMap::new(); let mut wheel = HashMap::new(); let mut winv = HashMap::new();
        for a in ACTIONS { pad.insert(a, parse_bind(&get(&format!("pad.{a}")), true)); wheel.insert(a, parse_bind(&get(&format!("wheel.{a}")), false));
            let ik = format!("wheel.{a}_invert"); winv.insert(a, if ini.0.contains_key(&ik) { ini.b(&ik) } else { def.b(&ik) }); }
        let f = |k: &str, d: f32| if ini.0.contains_key(k) { ini.f(k, d) } else { def.f(k, d) };
        Mapping { pad, wheel, winv, dz: f("pad.deadzone", 0.08), lin: f("pad.steer_linearity", 1.5).max(0.2), tdz: f("pad.trigger_deadzone", 0.03), wdev: get("wheel.device").to_lowercase(),
            sdz: f("wheel.steer_deadzone", 0.0), sgain: f("wheel.steer_gain", 1.0), pdz: f("wheel.pedal_deadzone", 0.02) }
    }
}

pub struct Inputs { pub devs: Vec<Device>, pub map: Mapping, pub ini_path: std::path::PathBuf, pub overlay: bool, last_scan: std::time::Instant, #[allow(dead_code)] plat: plat::Plat }
impl Inputs {
    pub fn new(ini_path: std::path::PathBuf, hwnd: isize) -> Inputs {
        let map = Mapping::load(&ini_path);
        let mut s = Inputs { devs: vec![], map, ini_path, overlay: std::env::args().any(|a| a == "--input-overlay"), last_scan: std::time::Instant::now(), plat: plat::Plat::new(hwnd) };
        s.scan(true); if s.devs.is_empty() { crate::logln("input: no controllers detected (keyboard only); hotplug is checked every 2 s"); } s
    }
    pub fn reload(&mut self) { self.map = Mapping::load(&self.ini_path); crate::logln(&format!("input: reloaded {}", self.ini_path.display())); }
    fn scan(&mut self, first: bool) {
        let before: Vec<String> = self.devs.iter().map(|d| d.guid.clone()).collect();
        self.plat.scan(&mut self.devs);
        for d in &self.devs { if !before.contains(&d.guid) { crate::logln(&format!("input: {} device '{}' guid {} [{}] {} axes, {} buttons{}", if first { "found" } else { "connected" }, d.name, d.guid, d.backend, d.axes.len(), d.buttons.len(), if d.class == Class::Pad { " (gamepad layout)" } else { "" })); } }
        for g in before { if !self.devs.iter().any(|d| d.guid == g) { crate::logln(&format!("input: disconnected {g}")); } }
    }
    /// Poll all devices and map them. Returns combined controls (pad + wheel; wheel wins on steer when it is used).
    pub fn poll(&mut self) -> Controls {
        if self.last_scan.elapsed().as_secs_f32() > 2.0 { self.last_scan = std::time::Instant::now(); self.scan(false); }
        for d in self.devs.iter_mut() { d.prev = d.buttons.clone(); }
        self.plat.poll(&mut self.devs); self.devs.retain(|d| d.alive);
        let mut c = map_controls(&self.map, &self.devs);
        if c.overlay { self.overlay = !self.overlay; }
        c
    }
    /// Text lines for the overlay.
    pub fn overlay_lines(&self, c: &Controls) -> Vec<String> {
        let mut v = vec![format!("INPUT  (F1 toggle, F2 reload {})", self.ini_path.file_name().map(|f| f.to_string_lossy().to_string()).unwrap_or_default()),
            format!("mapped: steer {:+.2} thr {:.2} brk {:.2} clu {:.2} hb {:.0}", c.steer, c.throttle, c.brake, c.clutch, c.handbrake)];
        if self.devs.is_empty() { v.push("no controllers detected".into()); }
        for (k, d) in self.devs.iter().enumerate() {
            v.push(format!("{k} {} [{}{}]", d.name.chars().take(40).collect::<String>(), d.backend, if d.class == Class::Pad { " pad" } else { "" }));
            let names = |i: usize| if d.class == Class::Pad { PAD_AXES.get(i).map(|s| s.to_string()).unwrap_or(format!("a{i}")) } else { format!("a{i}") };
            for ch in d.axes.chunks(4).enumerate() { v.push(format!("   {}", ch.1.iter().enumerate().map(|(j, a)| format!("{:>4} {:+.3}", names(ch.0 * 4 + j), a)).collect::<Vec<_>>().join("  "))); }
            let pressed: Vec<String> = d.buttons.iter().enumerate().filter(|b| *b.1).map(|(i, _)| if d.class == Class::Pad { PAD_BUTTONS.get(i).map(|s| s.to_string()).unwrap_or(i.to_string()) } else { i.to_string() }).collect();
            v.push(format!("   buttons held: {}", if pressed.is_empty() { "-".into() } else { pressed.join(" ") }));
        }
        v
    }
}


/// Pure mapping step (unit-tested): devices + ini -> controls.
pub fn map_controls(m: &Mapping, devs: &[Device]) -> Controls {
    let mut c = Controls::default();
        // pads
        for d in devs.iter().filter(|d| d.class == Class::Pad) {
            let ax = |b: &Bind| match b { Bind::Axis { i, neg, .. } => d.axes.get(*i).map(|v| if *neg { -*v } else { *v }).unwrap_or(0.0), Bind::Button { i, .. } => d.buttons.get(*i).map(|&p| p as i32 as f32).unwrap_or(0.0), Bind::None => 0.0 };
            let edge = |b: &Bind| matches!(b, Bind::Button { i, .. } if d.buttons.get(*i) == Some(&true) && d.prev.get(*i) != Some(&true));
            let sx = ax(&m.pad["steer"]); let mag = sx.abs(); let s = if mag < m.dz { 0.0 } else { ((mag - m.dz) / (1.0 - m.dz)).min(1.0).powf(m.lin) * sx.signum() };
            let trig = |v: f32| if v < m.tdz { 0.0 } else { ((v - m.tdz) / (1.0 - m.tdz)).min(1.0) };
            let th = trig(ax(&m.pad["throttle"])); let br = trig(ax(&m.pad["brake"]));
            if s != 0.0 || th > 0.0 || br > 0.0 { c.active = true; }
            if c.steer == 0.0 { c.steer = -s as f64; } // stick right = steer right; Input convention is +left
            c.throttle = c.throttle.max(th as f64); c.brake = c.brake.max(br as f64); c.clutch = c.clutch.max(trig(ax(&m.pad["clutch"])) as f64);
            if ax(&m.pad["handbrake"]) > 0.5 { c.handbrake = 1.0; }
            c.gear_up |= edge(&m.pad["gear_up"]); c.gear_down |= edge(&m.pad["gear_down"]); c.reset |= edge(&m.pad["reset"]); c.camera |= edge(&m.pad["camera"]); c.overlay |= edge(&m.pad["overlay"]);
        }
        // wheels / pedals (raw DirectInput or evdev): bindings may name other devices
        let raw: Vec<&Device> = devs.iter().filter(|d| d.class == Class::Raw).collect();
        if !raw.is_empty() {
            let pick = |sel: &Option<String>| -> Option<&Device> { let key = sel.clone().unwrap_or_else(|| m.wdev.clone());
                if key.is_empty() { raw.first().copied() } else { raw.iter().copied().find(|d| d.name.to_lowercase().contains(&key) || d.guid.to_lowercase().contains(&key)) } };
            let val = |a: &str| -> Option<f32> { match &m.wheel[a] { Bind::Axis { dev, i, neg } => pick(dev).and_then(|d| d.axes.get(*i).copied()).map(|v| { let v = if *neg { -v } else { v }; if m.winv[a] { -v } else { v } }),
                Bind::Button { dev, i } => pick(dev).and_then(|d| d.buttons.get(*i).map(|&p| if p { 1.0 } else { -1.0 })), Bind::None => None } };
            let edge = |a: &str| match &m.wheel[a] { Bind::Button { dev, i } => pick(dev).map(|d| d.buttons.get(*i) == Some(&true) && d.prev.get(*i) != Some(&true)).unwrap_or(false), _ => false };
            let pedal = |a: &str| val(a).map(|v| { let p = ((v + 1.0) * 0.5).clamp(0.0, 1.0); if p < m.pdz { 0.0 } else { ((p - m.pdz) / (1.0 - m.pdz)) as f64 } });
            if let Some(s) = val("steer") { let s = if s.abs() < m.sdz { 0.0 } else { s }; let s = (s * m.sgain).clamp(-1.0, 1.0); if s.abs() > 0.02 || c.steer == 0.0 { c.steer = -s as f64; } if s.abs() > 0.02 { c.active = true; } } // a turned wheel wins; a centred wheel doesn't mask the pad
            if let Some(t) = pedal("throttle") { c.throttle = c.throttle.max(t); if t > 0.0 { c.active = true; } } if let Some(b) = pedal("brake") { c.brake = c.brake.max(b); if b > 0.0 { c.active = true; } } if let Some(k) = pedal("clutch") { c.clutch = c.clutch.max(k); }
            if val("handbrake").unwrap_or(-1.0) > 0.0 { c.handbrake = 1.0; }
            c.gear_up |= edge("gear_up"); c.gear_down |= edge("gear_down"); c.reset |= edge("reset"); c.camera |= edge("camera"); c.overlay |= edge("overlay");
        }
    c
}

#[cfg(test)]
mod tests {
    use super::*;
    fn m() -> Mapping { let p = std::env::temp_dir().join("dcs_input_test.ini"); let _ = std::fs::remove_file(&p); Mapping::load(&p) }
    fn pad(lx: f32, lt: f32, rt: f32, btn: &[usize]) -> Device { let mut b = vec![false; 14]; for &i in btn { b[i] = true; } Device { name: "XInput controller 1".into(), guid: "xinput-0".into(), backend: "XInput", class: Class::Pad, axes: vec![lx, 0.0, 0.0, 0.0, lt, rt], buttons: b, prev: vec![false; 14], alive: true } }
    #[test] fn pad_mapping() {
        let m = m(); let c = map_controls(&m, &[pad(0.05, 0.0, 0.0, &[])]); assert_eq!(c.steer, 0.0); assert!(!c.active); // inside deadzone
        let c = map_controls(&m, &[pad(1.0, 0.25, 1.0, &[5, 2])]); assert!((c.steer + 1.0).abs() < 1e-6, "full right stick = full right (-)"); assert!((c.throttle - 1.0).abs() < 1e-6); assert!(c.brake > 0.2 && c.brake < 0.25);
        assert!(c.gear_up && c.handbrake == 1.0 && !c.gear_down);
        let c = map_controls(&m, &[pad(-0.5, 0.0, 0.0, &[])]); let lin = ((0.5f32 - 0.08) / 0.92).powf(1.5) as f64; assert!((c.steer - lin).abs() < 1e-5, "{} vs {}", c.steer, lin);
    }
    #[test] fn wheel_mapping() {
        let m = m(); let mut ax = vec![0.0f32; 8]; ax[0] = -0.5; ax[1] = -1.0; ax[5] = 1.0; ax[6] = 0.0; // half left, throttle floored, brake released, clutch half
        let mut b = vec![false; 36]; b[4] = true;
        let w = Device { name: "Logitech G29 Driving Force Racing Wheel".into(), guid: "ABC".into(), backend: "DirectInput", class: Class::Raw, axes: ax, buttons: b, prev: vec![false; 36], alive: true };
        let c = map_controls(&m, &[w]); assert!((c.steer - 0.5).abs() < 1e-6); assert!((c.throttle - 1.0).abs() < 1e-6); assert_eq!(c.brake, 0.0); assert!((c.clutch - 0.4898).abs() < 1e-3); assert!(c.gear_up);
    }
    #[test] fn bind_parse() { assert!(matches!(parse_bind("CSL Pedals/axis1", false), Bind::Axis { dev: Some(_), i: 1, neg: false })); assert!(matches!(parse_bind("-RX", true), Bind::Axis { i: 2, neg: true, .. })); assert!(matches!(parse_bind("Start", true), Bind::Button { i: 7, .. })); }
}

#[cfg(windows)]
mod plat {
    //! XInput + DirectInput 8 (windows crate).
    use super::{Class, Device};
    use std::ffi::c_void;
    use windows::core::{GUID, Interface};
    use windows::Win32::Devices::HumanInterfaceDevice::*;
    use windows::Win32::Foundation::{HWND, HINSTANCE}; use windows::core::BOOL;
    use windows::Win32::System::LibraryLoader::GetModuleHandleW;
    use windows::Win32::UI::Input::XboxController::*;

    const DIDFT_AXIS_: u32 = 0x3; const DIDFT_BUTTON_: u32 = 0xC; const DIDFT_POV_: u32 = 0x10; const DIDFT_ANYINSTANCE_: u32 = 0x00FF_FF00; const DIDFT_OPTIONAL_: u32 = 0x8000_0000;
    const DIDF_ABSAXIS_: u32 = 1; const DISCL_NONEXCLUSIVE_: u32 = 2; const DISCL_BACKGROUND_: u32 = 8; const DI8DEVCLASS_GAMECTRL_: u32 = 4; const DIEDFL_ATTACHEDONLY_: u32 = 1;
    #[repr(C)] #[derive(Clone, Copy, Default)] struct JoyState { ax: [i32; 8], pov: [u32; 4], btn: [u8; 32] } // = DIJOYSTATE layout (80 bytes)

    struct Di { dev: IDirectInputDevice8W, guid: GUID }
    pub struct Plat { di: Option<IDirectInput8W>, hwnd: HWND, dis: Vec<Di>, x_conn: [bool; 4], fmt_objs: Vec<DIOBJECTDATAFORMAT>, fmt: DIDATAFORMAT }
    static AXIS_GUIDS: [GUID; 8] = [GUID_XAxis, GUID_YAxis, GUID_ZAxis, GUID_RxAxis, GUID_RyAxis, GUID_RzAxis, GUID_Slider, GUID_Slider];
    fn guid_str(g: &GUID) -> String { format!("{:08X}-{:04X}-{:04X}-{}", g.data1, g.data2, g.data3, g.data4.iter().map(|b| format!("{b:02X}")).collect::<String>()) }
    impl Plat {
        pub fn new(hwnd: isize) -> Plat {
            let di = unsafe { let mut p: *mut c_void = std::ptr::null_mut(); let h: HINSTANCE = GetModuleHandleW(None).map(|m| m.into()).unwrap_or_default();
                match DirectInput8Create(h, 0x0800, &IDirectInput8W::IID, &mut p, None) { Ok(()) if !p.is_null() => Some(IDirectInput8W::from_raw(p)), r => { crate::logln(&format!("input: DirectInput8Create failed {r:?}")); None } } };
            let mut objs = vec![];
            for (k, g) in AXIS_GUIDS.iter().enumerate() { objs.push(DIOBJECTDATAFORMAT { pguid: g, dwOfs: (k * 4) as u32, dwType: DIDFT_AXIS_ | DIDFT_ANYINSTANCE_ | DIDFT_OPTIONAL_, dwFlags: 0 }); }
            for k in 0..4 { objs.push(DIOBJECTDATAFORMAT { pguid: &GUID_POV, dwOfs: 32 + k * 4, dwType: DIDFT_POV_ | DIDFT_ANYINSTANCE_ | DIDFT_OPTIONAL_, dwFlags: 0 }); }
            for k in 0..32 { objs.push(DIOBJECTDATAFORMAT { pguid: std::ptr::null(), dwOfs: 48 + k, dwType: DIDFT_BUTTON_ | DIDFT_ANYINSTANCE_ | DIDFT_OPTIONAL_, dwFlags: 0 }); }
            let mut p = Plat { di, hwnd: HWND(hwnd as *mut c_void), dis: vec![], x_conn: [false; 4], fmt_objs: objs, fmt: unsafe { std::mem::zeroed() } };
            p.fmt = DIDATAFORMAT { dwSize: std::mem::size_of::<DIDATAFORMAT>() as u32, dwObjSize: std::mem::size_of::<DIOBJECTDATAFORMAT>() as u32, dwFlags: DIDF_ABSAXIS_, dwDataSize: std::mem::size_of::<JoyState>() as u32, dwNumObjs: p.fmt_objs.len() as u32, rgodf: p.fmt_objs.as_mut_ptr() };
            p
        }
        pub fn scan(&mut self, devs: &mut Vec<Device>) {
            // XInput slots
            for i in 0..4u32 { let mut st = XINPUT_STATE::default(); let ok = unsafe { XInputGetState(i, &mut st) } == 0; let g = format!("xinput-{i}");
                if ok && !devs.iter().any(|d| d.guid == g) { devs.push(Device { name: format!("XInput controller {}", i + 1), guid: g, backend: "XInput", class: Class::Pad, axes: vec![0.0; 6], buttons: vec![false; 14], prev: vec![false; 14], alive: true }); }
                self.x_conn[i as usize] = ok; }
            // DirectInput game controllers
            let Some(di) = self.di.clone() else { return };
            unsafe extern "system" fn cb(ddi: *mut DIDEVICEINSTANCEW, ctx: *mut c_void) -> BOOL { let v = &mut *(ctx as *mut Vec<(GUID, String, GUID)>); let d = &*ddi;
                let n = String::from_utf16_lossy(&d.tszProductName); v.push((d.guidInstance, n.trim_end_matches('\0').to_string(), d.guidProduct)); BOOL(1) }
            let mut found: Vec<(GUID, String, GUID)> = vec![];
            if unsafe { di.EnumDevices(DI8DEVCLASS_GAMECTRL_, Some(cb), &mut found as *mut _ as *mut c_void, DIEDFL_ATTACHEDONLY_) }.is_err() { return; }
            let any_x = self.x_conn.iter().any(|&c| c);
            for (gi, name, _gp) in found {
                let g = guid_str(&gi); if devs.iter().any(|d| d.guid == g) { continue; }
                // XInput pads also show up in DirectInput (with a merged trigger axis); skip those duplicates
                let lname = name.to_lowercase(); if any_x && (lname.contains("xbox") || lname.contains("xinput") || lname.contains("x-box")) { continue; }
                let mut dev: Option<IDirectInputDevice8W> = None;
                if unsafe { di.CreateDevice(&gi, &mut dev, None) }.is_err() { continue; } let Some(dev) = dev else { continue };
                unsafe { if let Err(e) = dev.SetDataFormat(std::ptr::addr_of_mut!(self.fmt)) { crate::logln(&format!("input: SetDataFormat({name}) failed {e:?}")); continue; }
                    if !self.hwnd.0.is_null() { let _ = dev.SetCooperativeLevel(self.hwnd, DISCL_BACKGROUND_ | DISCL_NONEXCLUSIVE_); } /* default (no call) is background/non-exclusive too */ let _ = dev.Acquire(); }
                let mut caps = DIDEVCAPS { dwSize: std::mem::size_of::<DIDEVCAPS>() as u32, ..Default::default() }; unsafe { let _ = dev.GetCapabilities(&mut caps); }
                crate::logln(&format!("input: DirectInput '{name}' reports {} axes, {} buttons, {} POVs, ff={}", caps.dwAxes, caps.dwButtons, caps.dwPOVs, (caps.dwFlags & 0x100) != 0));
                devs.push(Device { name, guid: g, backend: "DirectInput", class: Class::Raw, axes: vec![0.0; 8], buttons: vec![false; 36], prev: vec![false; 36], alive: true });
                self.dis.push(Di { dev, guid: gi });
            }
        }
        pub fn poll(&mut self, devs: &mut Vec<Device>) {
            for d in devs.iter_mut() {
                if let Some(i) = d.guid.strip_prefix("xinput-").and_then(|s| s.parse::<u32>().ok()) {
                    let mut st = XINPUT_STATE::default(); if unsafe { XInputGetState(i, &mut st) } != 0 { d.alive = false; continue; }
                    let g = st.Gamepad; let n = |v: i16| (v as f32 / 32767.0).clamp(-1.0, 1.0);
                    d.axes = vec![n(g.sThumbLX), n(g.sThumbLY), n(g.sThumbRX), n(g.sThumbRY), g.bLeftTrigger as f32 / 255.0, g.bRightTrigger as f32 / 255.0];
                    let b = g.wButtons.0 as u16; let bit = |m: u16| b & m != 0;
                    d.buttons = vec![bit(0x1000), bit(0x2000), bit(0x4000), bit(0x8000), bit(0x0100), bit(0x0200), bit(0x0020), bit(0x0010), bit(0x0040), bit(0x0080), bit(0x0001), bit(0x0002), bit(0x0004), bit(0x0008)];
                }
            }
            let mut dead = vec![];
            for (k, di) in self.dis.iter().enumerate() {
                let g = guid_str(&di.guid); let Some(d) = devs.iter_mut().find(|d| d.guid == g) else { continue };
                let mut js = JoyState::default();
                let ok = unsafe { let _ = di.dev.Poll(); di.dev.GetDeviceState(std::mem::size_of::<JoyState>() as u32, &mut js as *mut _ as *mut c_void).is_ok()
                    || (di.dev.Acquire().is_ok() && di.dev.GetDeviceState(std::mem::size_of::<JoyState>() as u32, &mut js as *mut _ as *mut c_void).is_ok()) };
                if !ok { d.alive = false; dead.push(k); continue; }
                d.axes = js.ax.iter().map(|&v| ((v as f32) / 32767.5 - 1.0).clamp(-1.0, 1.0)).collect(); // default DI range 0..65535
                let mut b: Vec<bool> = js.btn.iter().map(|&x| x & 0x80 != 0).collect(); let pov = js.pov[0];
                let (u, r, dn, l) = if pov == 0xFFFF_FFFF || (pov & 0xFFFF) == 0xFFFF { (false, false, false, false) } else { let a = pov / 100; (a >= 315 || a <= 45, (45..=135).contains(&a), (135..=225).contains(&a), (225..=315).contains(&a)) };
                b.extend([u, r, dn, l]); d.buttons = b;
            }
            for k in dead.into_iter().rev() { self.dis.remove(k); }
        }
    }
}

#[cfg(not(windows))]
mod plat {
    //! gilrs (evdev on Linux): mapped pads -> Pad class, everything else -> raw axes/buttons by event code order.
    use super::{Class, Device};
    use gilrs::{Axis, Button, Gilrs};
    pub struct Plat { g: Option<Gilrs> }
    const AX: [Axis; 6] = [Axis::LeftStickX, Axis::LeftStickY, Axis::RightStickX, Axis::RightStickY, Axis::LeftZ, Axis::RightZ];
    const BT: [Button; 14] = [Button::South, Button::East, Button::West, Button::North, Button::LeftTrigger, Button::RightTrigger, Button::Select, Button::Start, Button::LeftThumb, Button::RightThumb, Button::DPadUp, Button::DPadDown, Button::DPadLeft, Button::DPadRight];
    impl Plat {
        pub fn new(_hwnd: isize) -> Plat { Plat { g: Gilrs::new().map_err(|e| crate::logln(&format!("input: gilrs init failed {e}"))).ok() } }
        pub fn scan(&mut self, devs: &mut Vec<Device>) {
            let Some(g) = self.g.as_mut() else { return }; while g.next_event().is_some() {}
            for (id, p) in g.gamepads() { let guid = format!("{}-{}", p.uuid().iter().map(|b| format!("{b:02x}")).collect::<String>(), usize::from(id));
                if devs.iter().any(|d| d.guid == guid) { continue; }
                let pad = p.mapping_source() != gilrs::MappingSource::None;
                devs.push(Device { name: p.name().to_string(), guid, backend: "gilrs", class: if pad { Class::Pad } else { Class::Raw }, axes: vec![0.0; if pad { 6 } else { 8 }], buttons: vec![false; if pad { 14 } else { 36 }], prev: vec![], alive: true }); }
        }
        pub fn poll(&mut self, devs: &mut Vec<Device>) {
            let Some(g) = self.g.as_mut() else { return }; while g.next_event().is_some() {}
            for d in devs.iter_mut() { let Some((_, p)) = g.gamepads().find(|(id, p)| d.guid.ends_with(&format!("-{}", usize::from(*id))) && d.name == p.name()) else { d.alive = false; continue };
                if d.class == Class::Pad {
                    for (k, a) in AX.iter().enumerate().take(4) { d.axes[k] = p.value(*a); }
                    d.axes[4] = p.button_data(Button::LeftTrigger2).map(|b| b.value()).unwrap_or(0.0); d.axes[5] = p.button_data(Button::RightTrigger2).map(|b| b.value()).unwrap_or(0.0);
                    for (k, b) in BT.iter().enumerate() { d.buttons[k] = p.is_pressed(*b); }
                } else {
                    let st = p.state(); let mut ax: Vec<(u32, f32)> = st.axes().map(|(c, a)| (c.into_u32(), a.value())).collect(); ax.sort_by_key(|x| x.0);
                    d.axes = ax.iter().map(|x| x.1).collect(); let mut bt: Vec<(u32, bool)> = st.buttons().map(|(c, b)| (c.into_u32(), b.is_pressed())).collect(); bt.sort_by_key(|x| x.0); d.buttons = bt.iter().map(|x| x.1).collect();
                }
            }
        }
    }
}
