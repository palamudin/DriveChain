//! drivechain-native - Tarmac physics (Jakov's NativeCurrent287 core via tms C API) + Vulkan 1.3 renderer.
//! Modes:
//!   drivechain-native [world.glb]                     windowed; keyboard, XInput pads, DirectInput wheels (input.ini); F1 input overlay
//!   drivechain-native --input-test 30                 no GPU: list controllers and print raw axes/buttons for 30 s
//!   drivechain-native --physics-bench 20 [world.glb]  no GPU: bot drives 20 s of sim, prints us/tick
//!   drivechain-native --still out.png [--frames 90] [--size 1600x900] [--gi 0|1|2] [world.glb]   offscreen render to PNG
mod accel; mod dlss; mod input; mod mat; mod render; mod veg; mod sim; mod tms; mod vk; mod world;
use anyhow::{Context, Result};
use glam::{Mat4, Vec3, Vec4};

/// Log to stderr and to drivechain.log next to the exe (the Windows build has no console when started from Explorer).
pub fn logln(m: &str) {
    use std::io::Write; eprintln!("{m}");
    static LOG: std::sync::OnceLock<std::sync::Mutex<Option<std::fs::File>>> = std::sync::OnceLock::new();
    let l = LOG.get_or_init(|| std::sync::Mutex::new(std::env::current_exe().ok().and_then(|e| std::fs::File::create(e.with_file_name("drivechain.log")).ok())));
    if let Ok(mut g) = l.lock() { if let Some(f) = g.as_mut() { let _ = writeln!(f, "{m}"); let _ = f.flush(); } }
}
fn exe_dir_file(name: &str) -> std::path::PathBuf { std::env::current_exe().ok().map(|e| e.with_file_name(name)).filter(|p| p.parent().map(|d| d.exists()).unwrap_or(false)).unwrap_or_else(|| name.into()) }
fn arg(name: &str) -> Option<String> { let a: Vec<String> = std::env::args().collect(); a.iter().position(|x| x == name).and_then(|i| a.get(i + 1).cloned()) }
fn world_path() -> String {
    let a: Vec<String> = std::env::args().skip(1).collect(); let mut skip = false;
    for x in &a { if skip { skip = false; continue; } if x.starts_with("--") { skip = !matches!(x.as_str(), "--rt-off"); continue; } if x.ends_with(".glb") || x.ends_with(".gltf") { return x.clone(); } }
    std::env::var("DCS_WORLD").unwrap_or_else(|_| if std::path::Path::new("../private-assets/usce/usce_v2.glb").exists() { "../private-assets/usce/usce_v2.glb".into() } else { "../private-assets/usce/usce.glb".into() }) // private, never in git (EXT_meshopt not supported here: use the plain GLB)
}
fn load_textures() -> Vec<(Vec<u8>, Vec<u8>)> {
    let dir = std::env::var("DCS_TEXTURES").unwrap_or_else(|_| "../web/assets/textures".into());
    ["asphalt", "grass", "concrete", "cobble", "facade"].iter().filter_map(|n| {
        let l = |s: &str| image::open(format!("{dir}/{n}_{s}.jpg")).ok().map(|i| i.resize_exact(1024, 1024, image::imageops::FilterType::Triangle).to_rgba8().into_raw());
        Some((l("albedo")?, l("rough")?)) }).collect()
}
fn attach_veg(r: &mut render::Renderer, c: &mut vk::Ctx, w: &world::World) {
    let o = veg::VegOpts::from_args(); if !(o.trees || o.shrubs || o.grass) { logln("vegetation: off"); return; }
    match veg::Veg::new(c, r.ubo_info(), w, &world_path(), o) { Ok(v) => r.veg = Some(v), Err(e) => logln(&format!("vegetation disabled: {e:#}")) }
}
/// Flat triangles of the visual "other" class that lie on the carriageway (0..12 cm above a tarmac/kerb/line collision triangle)
/// are paint/markings decals: they are rendered as their own material group (surf 12, lifted 2 cm against z-fighting).
fn split_decals(w: &world::World, p: &world::MeshPart) -> (Vec<u32>, Vec<u32>) {
    const C: f32 = 4.0; let mut grid: std::collections::HashMap<(i32, i32), Vec<[[f32; 3]; 3]>> = Default::default();
    for q in w.parts.iter().filter(|q| q.collide && matches!(q.surf, 0 | 1 | 5 | 8)) { for t in q.indices.chunks(3).filter(|t| t.len() == 3) {
        let v = [q.positions[t[0] as usize], q.positions[t[1] as usize], q.positions[t[2] as usize]];
        let (x0, x1) = ((v.iter().map(|a| a[0]).fold(f32::MAX, f32::min) / C).floor() as i32, (v.iter().map(|a| a[0]).fold(f32::MIN, f32::max) / C).floor() as i32);
        let (z0, z1) = ((v.iter().map(|a| a[2]).fold(f32::MAX, f32::min) / C).floor() as i32, (v.iter().map(|a| a[2]).fold(f32::MIN, f32::max) / C).floor() as i32);
        if (x1 - x0 + 1) * (z1 - z0 + 1) > 256 { continue; } for i in x0..=x1 { for k in z0..=z1 { grid.entry((i, k)).or_default().push(v); } } } }
    let height = |x: f32, z: f32| -> Option<f32> { grid.get(&((x / C).floor() as i32, (z / C).floor() as i32))?.iter().find_map(|t| {
        let (a, b, c) = (t[0], t[1], t[2]); let d = (b[2] - c[2]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[2] - c[2]); if d.abs() < 1e-9 { return None; }
        let l1 = ((b[2] - c[2]) * (x - c[0]) + (c[0] - b[0]) * (z - c[2])) / d; let l2 = ((c[2] - a[2]) * (x - c[0]) + (a[0] - c[0]) * (z - c[2])) / d; let l3 = 1.0 - l1 - l2;
        (l1 >= -1e-3 && l2 >= -1e-3 && l3 >= -1e-3).then(|| l1 * a[1] + l2 * b[1] + l3 * c[1]) }) };
    let (mut keep, mut decal) = (vec![], vec![]);
    for t in p.indices.chunks(3).filter(|t| t.len() == 3) { let v: Vec<glam::Vec3> = t.iter().map(|&k| glam::Vec3::from(p.positions[k as usize])).collect();
        let n = (v[1] - v[0]).cross(v[2] - v[0]); let ctr = (v[0] + v[1] + v[2]) / 3.0;
        let on_road = n.length() > 1e-8 && n.normalize().y.abs() > 0.97 && height(ctr.x, ctr.z).is_some_and(|h| (-0.03..0.12).contains(&(ctr.y - h)));
        if on_road { decal.extend_from_slice(t) } else { keep.extend_from_slice(t) } }
    (keep, decal)
}
fn upload_world(r: &mut render::Renderer, c: &mut vk::Ctx, w: &world::World) -> Result<()> {
    let mut tris = 0;
    for p in &w.parts {
        if !p.collide && p.cls == 32 && !std::env::args().any(|a| a == "--old-foliage") { continue; } // stand-in foliage blocks: replaced by ez-tree vegetation
        let surf = if p.collide { p.surf as u32 } else if p.cls >= 0 { p.cls as u32 } else { 10 };
        let v: Vec<render::Vertex> = (0..p.positions.len()).map(|i| render::Vertex { pos: p.positions[i], nrm: p.normals.get(i).copied().unwrap_or([0.0; 3]), uv: p.uvs.get(i).copied().unwrap_or([0.0; 2]) }).collect();
        // drop NaN triangles (seen in converted scenery)
        let idx: Vec<u32> = p.indices.chunks(3).filter(|t| t.len() == 3 && t.iter().all(|&k| p.positions[k as usize].iter().all(|x| x.is_finite()))).flatten().copied().collect();
        if !p.collide && p.cls == 10 { let pp = world::MeshPart { indices: idx.clone(), ..Default::default() }; let pp = world::MeshPart { positions: p.positions.clone(), ..pp };
            let (keep, decal) = split_decals(w, &pp); logln(&format!("world: {} paint/marking decal tris split from '{}' (material group 'markings')", decal.len() / 3, p.name));
            if !decal.is_empty() { tris += decal.len() / 3; r.add_mesh(c, &v, &decal, 12, Vec4::ZERO, false, true)?; }
            if !keep.is_empty() { tris += keep.len() / 3; r.add_mesh(c, &v, &keep, surf, Vec4::ZERO, false, true)?; } continue; }
        tris += idx.len() / 3; r.add_mesh(c, &v, &idx, surf, Vec4::ZERO, false, true)?;
    }
    eprintln!("render world: {} meshes, {} tris", w.parts.len(), tris); Ok(())
}
struct CarIdx { body: Vec<usize>, wheels: Vec<(usize, usize)> }
fn upload_car(r: &mut render::Renderer, c: &mut vk::Ctx, s: &sim::Sim) -> Result<CarIdx> {
    let mut ci = CarIdx { body: vec![], wheels: vec![] };
    // local private OBJ car (same set as the web build) if present, else the own procedural placeholder
    let st = &s.state; let w: Vec<glam::Vec3> = (0..4).map(|i| st.wheel(i)).collect();
    let wb = ((w[0] + w[1]) * 0.5 - (w[2] + w[3]) * 0.5).length(); let tr = ((w[0] - w[1]).length() + (w[2] - w[3]).length()) * 0.5;
    let dir = std::path::PathBuf::from(std::env::var("DCS_CARMESH").unwrap_or_else(|_| "../private-assets/miata_mesh".into()));
    let meshes = match sim::car_meshes_obj(&dir, wb, tr) { Some(m) if !m.is_empty() => { eprintln!("car: OBJ set {} ({} parts, physics wheelbase {:.3} m track {:.3} m)", dir.display(), m.len(), wb, tr); m } _ => { eprintln!("car: procedural placeholder (no {})", dir.display()); sim::car_meshes() } };
    for (v, i, surf, tint, wheel) in meshes { let k = r.add_mesh(c, &v, &i, surf, tint, true, false)?; if wheel < 0 { ci.body.push(k) } else { ci.wheels.push((k, wheel as usize)) } }
    Ok(ci)
}
fn place_car(r: &mut render::Renderer, s: &sim::Sim, ci: &CarIdx) {
    let b = s.body_matrix() * Mat4::from_translation(Vec3::new(0.0, 0.0, 0.02)); for &k in &ci.body { r.meshes[k].model = b; }
    for &(k, w) in &ci.wheels { r.meshes[k].model = s.wheel_matrix(w); }
}
fn settings_from_args() -> render::Settings {
    let mut s = render::Settings::default();
    if let Some(g) = arg("--gi") { s.gi = g.parse().unwrap_or(1); }
    if let Some(v) = arg("--sun-el") { s.sun_el = v.parse().unwrap_or(s.sun_el); } if let Some(v) = arg("--exposure") { s.exposure = v.parse().unwrap_or(s.exposure); }
    if let Some(v) = arg("--wet") { s.wet = v.parse().unwrap_or(0.0); }
    if let Some(v) = arg("--grain") { s.grain = v.parse().unwrap_or(s.grain); } if let Some(v) = arg("--sun-az") { s.sun_az = v.parse().unwrap_or(s.sun_az); }
    let onoff = |v: String| !matches!(v.as_str(), "off" | "0" | "false" | "no");
    if let Some(v) = arg("--taa") { s.taa = onoff(v); } if let Some(v) = arg("--spec-aa") { s.spec_aa = onoff(v); }
    if let Some(v) = arg("--taa-sharpen") { s.taa_sharpen = v.parse().unwrap_or(s.taa_sharpen); } if let Some(v) = arg("--aniso") { s.aniso = v.parse().unwrap_or(s.aniso); }
    if let Some(v) = arg("--bloom") { s.bloom = v.parse().unwrap_or(s.bloom); } if let Some(v) = arg("--volumetric") { s.volumetric = v.parse().unwrap_or(s.volumetric); }
    if std::env::args().any(|a| a == "--cheap-sky") { s.cheap_sky = true; } s
}
/// Material groups (render.ini [materials] + --mat/--isolate). Isolate runs skip GI, volumetrics and bloom and use a cheap sky:
/// only the group under test is rendered (other groups culled from every pass incl. shadows).
fn mats_setup(r: &mut render::Renderer) {
    let m = mat::Mats::from_args(Some(&exe_dir_file("render.ini"))); r.mats = m;
    if m.isolate { r.settings.gi = 0; r.settings.volumetric = 0.0; r.settings.bloom = 0.0; r.settings.cheap_sky = true; }
    logln(&format!("materials: {}{} | TAA {} (sharpen {:.2}) | spec-AA {} | aniso {}x", m.describe(), if m.isolate { " (isolate: GI/volumetrics/bloom off, cheap sky)" } else { "" },
        if r.settings.taa { "on" } else { "off" }, r.settings.taa_sharpen, if r.settings.spec_aa { "on" } else { "off" }, r.aniso));
}

fn main() -> Result<()> {
    if let Some(secs) = arg("--input-test") { // no GPU: list controllers, then print raw axes/buttons + mapped controls while they change
        let mut ii = input::Inputs::new(exe_dir_file("input.ini"), 0); let t0 = std::time::Instant::now(); let mut last = String::new();
        while t0.elapsed().as_secs_f32() < secs.parse::<f32>().unwrap_or(30.0) { let c = ii.poll(); let l = ii.overlay_lines(&c).join("\n"); if l != last { println!("---- t={:.1}s\n{l}", t0.elapsed().as_secs_f32()); last = l; } std::thread::sleep(std::time::Duration::from_millis(50)); }
        return Ok(());
    }
    eprintln!("drivechain-native | {}", tms::version());
    let wp = world_path(); let w = world::load(&wp).with_context(|| "world (set DCS_WORLD or pass a .glb)")?;
    eprintln!("world '{}': {} parts, {} spawns, line {} pts", w.name, w.parts.len(), w.spawns.len(), w.line.len());
    if let Some(secs) = arg("--physics-bench") {
        let secs: f32 = secs.parse().unwrap_or(20.0); let mut s = sim::Sim::new(&w, 3, 0)?; let n = (secs * 240.0) as i32; let t0 = std::time::Instant::now();
        for k in 0..n / 4 { let i = s.bot(); s.step(i, 4)?; if k % 240 == 0 { let p = s.state.pos(); eprintln!("t={:5.1}s v={:5.1} m/s gear {} rpm {:5.0} pos ({:.1},{:.1},{:.1})", k as f32 * 4.0 / 240.0, s.state.speed(), s.state.gear(), s.state.rpm(), p.x, p.y, p.z); } }
        let el = t0.elapsed().as_secs_f64(); println!("physics bench: {} ticks in {:.2} s = {:.0} us/tick ({:.0}% of one core at 240 Hz)", n, el, el * 1e6 / n as f64, el * 1e6 / n as f64 / 4166.7 * 100.0);
        return Ok(());
    }
    if let Some(out) = arg("--still") { return still(&w, &out); }
    window::run(w)
}

fn still(w: &world::World, out: &str) -> Result<()> {
    let (wd, ht) = arg("--size").and_then(|s| { let (a, b) = s.split_once('x')?; Some((a.parse().ok()?, b.parse().ok()?)) }).unwrap_or((1600u32, 900u32));
    let frames: u32 = arg("--frames").and_then(|f| f.parse().ok()).unwrap_or(90);
    let mut c = vk::Ctx::new(&[], std::env::args().all(|a| a != "--rt-off"))?;
    eprintln!("Vulkan device: {} | hardware RT (ray query): {}", c.name, c.rt);
    // --crop x,y,w,h: render only that window of the --size frame (same pixels as the full render; for fast soft-GPU tests)
    let crop = arg("--crop").and_then(|v| { let p: Vec<u32> = v.split(',').filter_map(|x| x.trim().parse().ok()).collect(); (p.len() == 4).then(|| (p[0], p[1], p[2], p[3])) });
    let mut set = settings_from_args(); let (rw, rh) = if let Some((x, y, cw, ch)) = crop { set.crop = Some([wd as f32, ht as f32, x as f32, y as f32]); (cw, ch) } else { (wd, ht) };
    let mut r = render::Renderer::new(&mut c, rw, rh, &load_textures(), set)?; let (wd, ht) = (rw, rh);
    mats_setup(&mut r); let t_setup = std::time::Instant::now();
    let mut s = sim::Sim::new(w, 3, 0)?; upload_world(&mut r, &mut c, w)?; let ci = upload_car(&mut r, &mut c, &s)?;
    if c.rt { let a = accel::Accel::new(&mut c, &r.meshes)?; r.bind_tlas(&c.device, a.tlas); r.tlas = Some(a); }
    attach_veg(&mut r, &mut c, w);
    let mut cam = None;
    for f in 0..frames { let i = s.bot(); s.step(i, 8)?; cam = Some(s.chase_camera(cam, 1.0 / 30.0)); let _ = f; }
    place_car(&mut r, &s, &ci); let (mut cp, mut ct) = cam.unwrap();
    // --cam x,y,z --look x,y,z (absolute) or --cam-rel back,up,side relative to the car; --fov deg
    let v3 = |n: &str| arg(n).and_then(|v| { let p: Vec<f32> = v.split(',').filter_map(|x| x.trim().parse().ok()).collect(); (p.len() == 3).then(|| Vec3::new(p[0], p[1], p[2])) });
    if let Some(p) = v3("--cam") { cp = p; } if let Some(p) = v3("--look") { ct = p; }
    if let Some(p) = v3("--cam-rel") { let (c0, f, l, _) = s.basis(); let fl = Vec3::new(f.x, 0.0, f.z).normalize_or(Vec3::Z); cp = c0 - fl * p.x + Vec3::Y * p.y + l * p.z; ct = c0 + Vec3::Y * 0.5; }
    let fov = arg("--fov").and_then(|v| v.parse::<f32>().ok()).unwrap_or(62.0).to_radians();
    let view = Mat4::look_at_rh(cp, ct, Vec3::Y);
    let (centre, ..) = s.basis();
    if let Some(v) = r.veg.as_mut() { v.update(cp); }
    // --seq N: N more frames at the same camera, each saved as <out>_NNN.png (temporal-stability / flicker measurement); --dt s per frame (wind time)
    let dt: f32 = arg("--dt").and_then(|v| v.parse().ok()).unwrap_or(1.0 / 60.0); r.dt = dt;
    // warm-up frames at the start camera: caches, velocity, and (TAA) a converged history; --warm N overrides
    let warm: u32 = arg("--warm").and_then(|v| v.parse().ok()).unwrap_or(if r.settings.taa { 16 } else { 3 });
    for k in 0..warm { r.render(&c, view, cp, fov, centre, k as f32 * dt)?; r.wait(&c)?; }
    if let Some(n) = arg("--seq").and_then(|v| v.parse::<u32>().ok()) {
        let step = v3("--cam-step").unwrap_or(Vec3::ZERO); // per-frame camera dolly (motion shimmer test)
        for k in 0..n { let (p, t) = (cp + step * k as f32, ct + step * k as f32); if step != Vec3::ZERO { if let Some(v) = r.veg.as_mut() { v.update(p); } }
            r.render(&c, Mat4::look_at_rh(p, t, Vec3::Y), p, fov, centre, (warm + k) as f32 * dt)?; r.wait(&c)?; let px = r.readback(&mut c)?;
            let f = std::fs::File::create(format!("{}_{k:03}.png", out.trim_end_matches(".png")))?; let mut e = png::Encoder::new(std::io::BufWriter::new(f), wd, ht); e.set_color(png::ColorType::Rgba); e.set_depth(png::BitDepth::Eight); e.write_header()?.write_image_data(&px)?; }
    }
    r.read_timings(&c);
    let px = r.readback(&mut c)?;
    let f = std::fs::File::create(out)?; let mut e = png::Encoder::new(std::io::BufWriter::new(f), wd, ht); e.set_color(png::ColorType::Rgba); e.set_depth(png::BitDepth::Eight); e.write_header()?.write_image_data(&px)?;
    for (n, t) in render::PASS_NAMES.iter().zip(r.timings.iter()) { eprintln!("  {:18} {:8.2} ms", n, t); }
    eprintln!("  {:18} {:8.2} ms (sum of passes)   wall {:.1} s since world upload", "GPU frame", r.timings.iter().sum::<f32>(), t_setup.elapsed().as_secs_f32());
    if let Some(v) = &r.veg { eprintln!("  {:18} {:8.2} ms\n  {:18} {:8.2} ms   (inside gbuffer)\n  vegetation: {}", "grass compute", r.veg_ms[0], "foliage+grass draw", r.veg_ms[1], v.stats()); }
    eprintln!("wrote {out} ({}x{}), far-cascade renders {}", wd, ht, r.far_renders); Ok(())
}

mod window {
    //! Windowed mode: winit 0.30 + swapchain; the renderer's final rgba8 image is blitted into the swapchain image.
    use super::*;
    use ash::vk as avk;
    use raw_window_handle::{HasDisplayHandle, HasWindowHandle};
    use winit::{application::ApplicationHandler, event::{ElementState, WindowEvent}, event_loop::{ActiveEventLoop, EventLoop}, keyboard::{KeyCode, PhysicalKey}, window::{Window, WindowId}};
    struct App { w: world::World, st: Option<State>, keys: std::collections::HashSet<KeyCode>, inp: Option<input::Inputs>, pressed: Vec<KeyCode> }
    struct State { win: Window, c: vk::Ctx, r: render::Renderer, s: sim::Sim, ci: CarIdx, surface: avk::SurfaceKHR, sc_d: ash::khr::swapchain::Device, sc: avk::SwapchainKHR, imgs: Vec<avk::Image>, sem_acq: avk::Semaphore, sem_done: avk::Semaphore, last: std::time::Instant, acc: f64, cam: Option<(Vec3, Vec3)>, frames: u32, steer: f32, auto: bool, cam_mode: u32, t0: std::time::Instant, bgra: bool, ft: Vec<f32>, mat_ov: bool, ov: Option<vk::Buffer>, ov_wh: (u32, u32), ov_last: Vec<String> }
    pub fn run(w: world::World) -> Result<()> {
        let el = EventLoop::new()?; let mut app = App { w, st: None, keys: Default::default(), inp: None, pressed: vec![] }; el.run_app(&mut app)?; Ok(())
    }
    impl ApplicationHandler for App {
        fn resumed(&mut self, el: &ActiveEventLoop) {
            if self.st.is_some() { return; }
            let win = el.create_window(Window::default_attributes().with_title("DriveChain Showcase (native)").with_inner_size(winit::dpi::PhysicalSize::new(1920, 1080))).unwrap();
            let exts = ash_window::enumerate_required_extensions(win.display_handle().unwrap().as_raw()).unwrap().to_vec();
            let mut c = vk::Ctx::new(&exts, true).expect("vulkan");
            let surface = unsafe { ash_window::create_surface(&c.entry, &c.instance, win.display_handle().unwrap().as_raw(), win.window_handle().unwrap().as_raw(), None).unwrap() };
            let size = win.inner_size(); let (wd, ht) = (size.width.max(64), size.height.max(64));
            let si = c.surface_i.clone().unwrap(); let caps = unsafe { si.get_physical_device_surface_capabilities(c.pdev, surface).unwrap() };
            let fmts = unsafe { si.get_physical_device_surface_formats(c.pdev, surface).unwrap() };
            let fmt = fmts.iter().find(|f| f.format == avk::Format::B8G8R8A8_UNORM || f.format == avk::Format::R8G8B8A8_UNORM).copied().unwrap_or(fmts[0]);
            let sc_d = ash::khr::swapchain::Device::new(&c.instance, &c.device);
            // --present fifo|mailbox|immediate (default fifo = vsync); unsupported modes fall back to FIFO
            let pms = unsafe { si.get_physical_device_surface_present_modes(c.pdev, surface).unwrap_or_default() };
            let want = match arg("--present").as_deref() { Some("mailbox") => avk::PresentModeKHR::MAILBOX, Some("immediate") => avk::PresentModeKHR::IMMEDIATE, Some("relaxed") => avk::PresentModeKHR::FIFO_RELAXED, _ => avk::PresentModeKHR::FIFO };
            let pm = if pms.contains(&want) { want } else { avk::PresentModeKHR::FIFO };
            let sc = unsafe { sc_d.create_swapchain(&avk::SwapchainCreateInfoKHR::default().surface(surface).min_image_count(caps.min_image_count.max(2) + 1).image_format(fmt.format).image_color_space(fmt.color_space)
                .image_extent(avk::Extent2D { width: wd, height: ht }).image_array_layers(1).image_usage(avk::ImageUsageFlags::TRANSFER_DST | avk::ImageUsageFlags::COLOR_ATTACHMENT)
                .pre_transform(caps.current_transform).composite_alpha(avk::CompositeAlphaFlagsKHR::OPAQUE).present_mode(pm).clipped(true), None).unwrap() };
            let imgs = unsafe { sc_d.get_swapchain_images(sc).unwrap() };
            logln(&format!("Vulkan device: {} | driver {} | hardware RT (ray query): {} | swapchain {}x{} {:?} | present mode {:?} (supported {:?}; --present fifo|mailbox|immediate|relaxed)", c.name, c.driver, c.rt, wd, ht, fmt.format, pm, pms));
            let hwnd: isize = match win.window_handle().map(|h| h.as_raw()) { #[cfg(windows)] Ok(raw_window_handle::RawWindowHandle::Win32(h)) => h.hwnd.get(), _ => 0 };
            self.inp = Some(input::Inputs::new(exe_dir_file("input.ini"), hwnd));
            let mut r = render::Renderer::new(&mut c, wd, ht, &load_textures(), settings_from_args()).unwrap();
            let s = sim::Sim::new(&self.w, 3, 0).unwrap(); upload_world(&mut r, &mut c, &self.w).unwrap(); let ci = upload_car(&mut r, &mut c, &s).unwrap();
            if c.rt { let a = accel::Accel::new(&mut c, &r.meshes).unwrap(); r.bind_tlas(&c.device, a.tlas); r.tlas = Some(a); }
            mats_setup(&mut r);
            attach_veg(&mut r, &mut c, &self.w);
            let sem = || unsafe { c.device.create_semaphore(&avk::SemaphoreCreateInfo::default(), None).unwrap() };
            let (sem_acq, sem_done) = (sem(), sem());
            self.st = Some(State { win, c, r, s, ci, surface, sc_d, sc, imgs, sem_acq, sem_done, last: std::time::Instant::now(), acc: 0.0, cam: None, frames: 0, steer: 0.0, auto: false, cam_mode: 0, t0: std::time::Instant::now(), bgra: fmt.format == avk::Format::B8G8R8A8_UNORM, ov: None, ov_wh: (0, 0), ov_last: vec![], ft: vec![], mat_ov: false });
        }
        fn window_event(&mut self, el: &ActiveEventLoop, _: WindowId, ev: WindowEvent) {
            match ev {
                WindowEvent::CloseRequested => el.exit(),
                WindowEvent::KeyboardInput { event, .. } => if let PhysicalKey::Code(k) = event.physical_key {
                    if event.state == ElementState::Pressed { if !event.repeat { self.pressed.push(k); } self.keys.insert(k); if let Some(st) = self.st.as_mut() { match k {
                        KeyCode::KeyP => st.auto = !st.auto, KeyCode::KeyR => { let sp = st.s.spawn; st.s.car.reset(&st.s.pw, sp.x, sp.z, sp.yaw); }
                        KeyCode::Digit1 => st.r.settings.gi = 0, KeyCode::Digit2 => st.r.settings.gi = 1, KeyCode::Escape => el.exit(), _ => {} } } } else { self.keys.remove(&k); } },
                WindowEvent::RedrawRequested => { if let Err(e) = self.frame() { eprintln!("frame error: {e:?}"); el.exit(); } }
                _ => {}
            }
        }
        fn about_to_wait(&mut self, _: &ActiveEventLoop) { if let Some(st) = &self.st { st.win.request_redraw(); } }
    }
    /// Debug text panel (font8x8, 2x scale) rendered on the CPU into a host-visible buffer, copied onto the swapchain image.
    fn draw_overlay(st: &mut State, max_w: u32, max_h: u32) -> Option<avk::Buffer> {
        let lines = &st.ov_last; let sc = 2u32; let lh = 9 * sc; let cols = lines.iter().map(|l| l.chars().count()).max().unwrap_or(1) as u32;
        let ow = ((cols * 8 * sc) + 16).min(max_w).max(8); let oh = (lines.len() as u32 * lh + 12).min(max_h).max(8);
        if st.ov.is_none() || st.ov_wh.0 * st.ov_wh.1 < ow * oh { if let Some(b) = st.ov.take() { st.c.free_buffer(b); } st.ov = st.c.buffer("overlay", (ow * oh * 4) as u64, avk::BufferUsageFlags::TRANSFER_SRC, gpu_allocator::MemoryLocation::CpuToGpu).ok(); }
        st.ov_wh = (ow, oh); let bgra = st.bgra; let b = st.ov.as_mut()?; let px = b.alloc.as_mut()?.mapped_slice_mut()?;
        for p in px[..(ow * oh * 4) as usize].chunks_mut(4) { p.copy_from_slice(&[12, 14, 18, 255]); }
        use font8x8::UnicodeFonts;
        for (li, l) in lines.iter().enumerate() { let col = if li == 0 { [255u8, 210, 90] } else if l.starts_with("   ") { [190, 230, 255] } else { [235, 235, 235] };
            for (ci, ch) in l.chars().enumerate() { let Some(g) = font8x8::BASIC_FONTS.get(ch) else { continue };
                for (gy, row) in g.iter().enumerate() { for gx in 0..8u32 { if row >> gx & 1 == 0 { continue; }
                    for sy in 0..sc { for sx in 0..sc { let x = 8 + ci as u32 * 8 * sc + gx * sc + sx; let y = 6 + li as u32 * lh + gy as u32 * sc + sy; if x >= ow || y >= oh { continue; }
                        let o = ((y * ow + x) * 4) as usize; let c = if bgra { [col[2], col[1], col[0], 255] } else { [col[0], col[1], col[2], 255] }; px[o..o + 4].copy_from_slice(&c); } } } } } }
        Some(b.buf)
    }
    impl App {
        fn frame(&mut self) -> Result<()> {
            let k = &self.keys; let Some(st) = self.st.as_mut() else { return Ok(()) };
            let now = std::time::Instant::now(); let dt = (now - st.last).as_secs_f64().min(0.1); st.last = now; st.acc += dt;
            let mut inp = tms::Input::default(); let tgt = (k.contains(&KeyCode::KeyA) || k.contains(&KeyCode::ArrowLeft)) as i32 as f32 - (k.contains(&KeyCode::KeyD) || k.contains(&KeyCode::ArrowRight)) as i32 as f32;
            st.steer += (tgt - st.steer).clamp(-3.0 * dt as f32, 3.0 * dt as f32); inp.steer = st.steer as f64;
            inp.throttle = (k.contains(&KeyCode::KeyW) || k.contains(&KeyCode::ArrowUp)) as i32 as f64; inp.brake = (k.contains(&KeyCode::KeyS) || k.contains(&KeyCode::ArrowDown)) as i32 as f64; inp.handbrake = k.contains(&KeyCode::Space) as i32 as f64;
            let pk = std::mem::take(&mut self.pressed);
            if let Some(ii) = self.inp.as_mut() {
                if pk.contains(&KeyCode::F1) { ii.overlay = !ii.overlay; } if pk.contains(&KeyCode::F2) { ii.reload(); }
                // material groups: F3 group, F4 mode, F6 pick feature override, F7 toggle it, F9 reset all; F5 TAA on/off, F8 spec-AA on/off
                let mk = [KeyCode::F3, KeyCode::F4, KeyCode::F6, KeyCode::F7, KeyCode::F9, KeyCode::F5, KeyCode::F8];
                if mk.iter().any(|k| pk.contains(k)) { let m = &mut st.r.mats;
                    if pk.contains(&KeyCode::F3) { m.key_group(); } if pk.contains(&KeyCode::F4) { m.key_mode(); } if pk.contains(&KeyCode::F6) { m.key_flag_pick(); }
                    if pk.contains(&KeyCode::F7) { m.key_flag(); } if pk.contains(&KeyCode::F9) { m.reset(); } if pk.contains(&KeyCode::F5) { st.r.settings.taa = !st.r.settings.taa; st.r.taa_reset = true; }
                    if pk.contains(&KeyCode::F8) { st.r.settings.spec_aa = !st.r.settings.spec_aa; }
                    st.mat_ov = true; logln(&format!("materials (keys): {} | selected {} / override {} | TAA {} | spec-AA {}", st.r.mats.describe(), mat::GROUPS[st.r.mats.sel], mat::FLAGS[st.r.mats.sel_flag], if st.r.settings.taa { "on" } else { "off" }, if st.r.settings.spec_aa { "on" } else { "off" })); }
                let c = ii.poll();
                if c.active { inp.steer = c.steer; inp.throttle = inp.throttle.max(c.throttle); inp.brake = inp.brake.max(c.brake); }
                inp.clutch = c.clutch; if c.handbrake > 0.5 { inp.handbrake = 1.0; }
                if c.gear_up || pk.contains(&KeyCode::KeyE) { st.s.shift(1); } if c.gear_down || pk.contains(&KeyCode::KeyQ) { st.s.shift(-1); }
                if pk.contains(&KeyCode::KeyM) { st.s.manual = !st.s.manual; logln(&format!("gearbox: {}", if st.s.manual { "manual" } else { "auto" })); }
                if c.reset { let sp = st.s.spawn; st.s.car.reset(&st.s.pw, sp.x, sp.z, sp.yaw); }
                if c.camera || pk.contains(&KeyCode::KeyC) { st.cam_mode = (st.cam_mode + 1) % 3; st.cam = None; st.r.taa_reset = true; }
                if c.reset { st.r.taa_reset = true; }
                if st.mat_ov || !st.r.mats.is_default() { /* shown below */ }
                if ii.overlay { st.ov_last = ii.overlay_lines(&c); st.ov_last.push(format!("car: {:.0} km/h  gear {}{}  rpm {:.0}", st.s.state.speed() * 3.6, st.s.gear, if st.s.manual { " (manual)" } else { " (auto)" }, st.s.state.rpm())); } else { st.ov_last.clear(); }
            }
            if st.mat_ov || !st.r.mats.is_default() { let mut l = st.r.mats.overlay_lines(); l.push(format!("TAA {} (F5) specAA {} (F8) {}x", if st.r.settings.taa { "on" } else { "off" }, if st.r.settings.spec_aa { "on" } else { "off" }, st.r.aniso)); st.ov_last.extend(l); }
            if st.auto { inp = st.s.bot(); }
            let ticks = (st.acc * 240.0).floor() as i32; let ticks = ticks.min(24); st.acc -= ticks as f64 / 240.0; if st.acc > 0.1 { st.acc = 0.0; }
            if ticks > 0 { st.s.step(inp, ticks)?; }
            st.cam = Some(st.s.camera(st.cam_mode, st.cam, dt as f32)); let (cp, ct) = st.cam.unwrap(); place_car(&mut st.r, &st.s, &st.ci); if let Some(v) = st.r.veg.as_mut() { v.update(cp); } let (centre, ..) = st.s.basis();
            st.r.dt = dt as f32; st.ft.push(dt as f32 * 1000.0);
            st.r.render(&st.c, Mat4::look_at_rh(cp, ct, Vec3::Y), cp, 62f32.to_radians(), centre, st.t0.elapsed().as_secs_f32())?; st.r.wait(&st.c)?;
            let (idx, _) = unsafe { st.sc_d.acquire_next_image(st.sc, u64::MAX, st.sem_acq, avk::Fence::null())? }; let img = st.imgs[idx as usize];
            let (d, out, w, h) = (st.c.device.clone(), st.r.out_image(), st.r.w, st.r.h);
            let ov_buf = if st.ov_last.is_empty() { None } else { draw_overlay(st, w.saturating_sub(24).min(760), h.saturating_sub(24)) }; let ov_wh = st.ov_wh;
            st.c.one_shot(|cb| unsafe {
                vk::barrier(&d, cb, img, avk::ImageLayout::UNDEFINED, avk::ImageLayout::TRANSFER_DST_OPTIMAL, false, 1);
                let sub = avk::ImageSubresourceLayers { aspect_mask: avk::ImageAspectFlags::COLOR, mip_level: 0, base_array_layer: 0, layer_count: 1 };
                let o = [avk::Offset3D::default(), avk::Offset3D { x: w as i32, y: h as i32, z: 1 }];
                d.cmd_blit_image(cb, out, avk::ImageLayout::GENERAL, img, avk::ImageLayout::TRANSFER_DST_OPTIMAL, &[avk::ImageBlit { src_subresource: sub, src_offsets: o, dst_subresource: sub, dst_offsets: o }], avk::Filter::NEAREST);
                if let Some(ob) = ov_buf { let (ow, oh) = ov_wh; d.cmd_copy_buffer_to_image(cb, ob, img, avk::ImageLayout::TRANSFER_DST_OPTIMAL, &[avk::BufferImageCopy { buffer_offset: 0, buffer_row_length: 0, buffer_image_height: 0,
                    image_subresource: sub, image_offset: avk::Offset3D { x: 12, y: 12, z: 0 }, image_extent: avk::Extent3D { width: ow, height: oh, depth: 1 } }]); }
                vk::barrier(&d, cb, img, avk::ImageLayout::TRANSFER_DST_OPTIMAL, avk::ImageLayout::PRESENT_SRC_KHR, false, 1);
            })?; // NOTE: simple path - waits on acquire implicitly via queue idle; a production loop would chain semaphores
            let scs = [st.sc]; let ids = [idx]; let waits = [st.sem_acq];
            // acquire semaphore must be consumed: submit an empty batch waiting on it, then present
            let stages = [avk::PipelineStageFlags::ALL_COMMANDS]; let sigs = [st.sem_done];
            unsafe { st.c.device.queue_submit(st.c.queue, &[avk::SubmitInfo::default().wait_semaphores(&waits).wait_dst_stage_mask(&stages).signal_semaphores(&sigs)], avk::Fence::null())?;
                st.sc_d.queue_present(st.c.queue, &avk::PresentInfoKHR::default().wait_semaphores(&sigs).swapchains(&scs).image_indices(&ids))?; }
            st.frames += 1;
            if st.frames % 60 == 0 { let t: Vec<String> = render::PASS_NAMES.iter().zip(st.r.timings.iter()).map(|(n, t)| format!("{n} {t:.2}")).collect();
                let vs = st.r.veg.as_ref().map(|v| format!(" | grass {:.2} veg {:.2} | {}", st.r.veg_ms[0], st.r.veg_ms[1], v.stats())).unwrap_or_default();
                st.win.set_title(&format!("DriveChain native | {:.0} km/h | GPU ms: {}{}", st.s.state.speed() * 3.6, t.join(" | "), vs));
                if st.frames % 600 == 0 { let mut f = std::mem::take(&mut st.ft); f.sort_by(|a, b| a.partial_cmp(b).unwrap()); let n = f.len().max(1);
                    let (mean, p99, mx) = (f.iter().sum::<f32>() / n as f32, f[(n * 99 / 100).min(n - 1)], *f.last().unwrap_or(&0.0));
                    logln(&format!("frame {}: frame time mean {mean:.2} ms p99 {p99:.2} ms max {mx:.2} ms (pacing) | GPU ms {}{}", st.frames, t.join(" | "), vs)); } }
            Ok(())
        }
    }
}
