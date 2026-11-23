//! drivechain-native - Tarmac physics (Jakov's NativeCurrent287 core via tms C API) + Vulkan 1.3 renderer.
//! Modes:
//!   drivechain-native [world.glb]                     windowed, keyboard/gamepad (gilrs), chase cam
//!   drivechain-native --physics-bench 20 [world.glb]  no GPU: bot drives 20 s of sim, prints us/tick
//!   drivechain-native --still out.png [--frames 90] [--size 1600x900] [--gi 0|1|2] [world.glb]   offscreen render to PNG
mod accel; mod dlss; mod render; mod sim; mod tms; mod vk; mod world;
use anyhow::{Context, Result};
use glam::{Mat4, Vec3, Vec4};

fn arg(name: &str) -> Option<String> { let a: Vec<String> = std::env::args().collect(); a.iter().position(|x| x == name).and_then(|i| a.get(i + 1).cloned()) }
fn world_path() -> String {
    let a: Vec<String> = std::env::args().skip(1).collect(); let mut skip = false;
    for x in &a { if skip { skip = false; continue; } if x.starts_with("--") { skip = !matches!(x.as_str(), "--rt-off"); continue; } if x.ends_with(".glb") || x.ends_with(".gltf") { return x.clone(); } }
    std::env::var("DCS_WORLD").unwrap_or_else(|_| "../private-assets/usce/usce.glb".into()) // private, never in git (EXT_meshopt not supported here: use the plain GLB)
}
fn load_textures() -> Vec<(Vec<u8>, Vec<u8>)> {
    let dir = std::env::var("DCS_TEXTURES").unwrap_or_else(|_| "../web/assets/textures".into());
    ["asphalt", "grass", "concrete", "cobble", "facade"].iter().filter_map(|n| {
        let l = |s: &str| image::open(format!("{dir}/{n}_{s}.jpg")).ok().map(|i| i.resize_exact(1024, 1024, image::imageops::FilterType::Triangle).to_rgba8().into_raw());
        Some((l("albedo")?, l("rough")?)) }).collect()
}
fn upload_world(r: &mut render::Renderer, c: &mut vk::Ctx, w: &world::World) -> Result<()> {
    let mut tris = 0;
    for p in &w.parts {
        let surf = if p.collide { p.surf as u32 } else if p.cls >= 0 { p.cls as u32 } else { 10 };
        let v: Vec<render::Vertex> = (0..p.positions.len()).map(|i| render::Vertex { pos: p.positions[i], nrm: p.normals.get(i).copied().unwrap_or([0.0; 3]), uv: p.uvs.get(i).copied().unwrap_or([0.0; 2]) }).collect();
        // drop NaN triangles (seen in converted scenery)
        let idx: Vec<u32> = p.indices.chunks(3).filter(|t| t.len() == 3 && t.iter().all(|&k| p.positions[k as usize].iter().all(|x| x.is_finite()))).flatten().copied().collect();
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
    if let Some(v) = arg("--wet") { s.wet = v.parse().unwrap_or(0.0); } s
}

fn main() -> Result<()> {
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
    let mut r = render::Renderer::new(&mut c, wd, ht, &load_textures(), settings_from_args())?;
    let mut s = sim::Sim::new(w, 3, 0)?; upload_world(&mut r, &mut c, w)?; let ci = upload_car(&mut r, &mut c, &s)?;
    if c.rt { let a = accel::Accel::new(&mut c, &r.meshes)?; r.bind_tlas(&c.device, a.tlas); r.tlas = Some(a); }
    let mut cam = None;
    for f in 0..frames { let i = s.bot(); s.step(i, 8)?; cam = Some(s.chase_camera(cam, 1.0 / 30.0)); let _ = f; }
    place_car(&mut r, &s, &ci); let (cp, ct) = cam.unwrap(); let view = Mat4::look_at_rh(cp, ct, Vec3::Y);
    let (centre, ..) = s.basis();
    for k in 0..3 { r.render(&c, view, cp, 62f32.to_radians(), centre, k as f32)?; r.wait(&c)?; } // 3 frames: warm caches, velocity settles
    r.read_timings(&c);
    let px = r.readback(&mut c)?;
    let f = std::fs::File::create(out)?; let mut e = png::Encoder::new(std::io::BufWriter::new(f), wd, ht); e.set_color(png::ColorType::Rgba); e.set_depth(png::BitDepth::Eight); e.write_header()?.write_image_data(&px)?;
    for (n, t) in render::PASS_NAMES.iter().zip(r.timings.iter()) { eprintln!("  {:18} {:8.2} ms", n, t); }
    eprintln!("wrote {out} ({}x{}), far-cascade renders {}", wd, ht, r.far_renders); Ok(())
}

mod window {
    //! Windowed mode: winit 0.30 + swapchain; the renderer's final rgba8 image is blitted into the swapchain image.
    use super::*;
    use ash::vk as avk;
    use raw_window_handle::{HasDisplayHandle, HasWindowHandle};
    use winit::{application::ApplicationHandler, event::{ElementState, WindowEvent}, event_loop::{ActiveEventLoop, EventLoop}, keyboard::{KeyCode, PhysicalKey}, window::{Window, WindowId}};
    struct App { w: world::World, st: Option<State>, keys: std::collections::HashSet<KeyCode>, gil: Option<gilrs::Gilrs> }
    struct State { win: Window, c: vk::Ctx, r: render::Renderer, s: sim::Sim, ci: CarIdx, surface: avk::SurfaceKHR, sc_d: ash::khr::swapchain::Device, sc: avk::SwapchainKHR, imgs: Vec<avk::Image>, sem_acq: avk::Semaphore, sem_done: avk::Semaphore, last: std::time::Instant, acc: f64, cam: Option<(Vec3, Vec3)>, frames: u32, steer: f32, auto: bool }
    pub fn run(w: world::World) -> Result<()> {
        let el = EventLoop::new()?; let mut app = App { w, st: None, keys: Default::default(), gil: gilrs::Gilrs::new().ok() }; el.run_app(&mut app)?; Ok(())
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
            let sc = unsafe { sc_d.create_swapchain(&avk::SwapchainCreateInfoKHR::default().surface(surface).min_image_count(caps.min_image_count.max(2) + 1).image_format(fmt.format).image_color_space(fmt.color_space)
                .image_extent(avk::Extent2D { width: wd, height: ht }).image_array_layers(1).image_usage(avk::ImageUsageFlags::TRANSFER_DST | avk::ImageUsageFlags::COLOR_ATTACHMENT)
                .pre_transform(caps.current_transform).composite_alpha(avk::CompositeAlphaFlagsKHR::OPAQUE).present_mode(avk::PresentModeKHR::FIFO).clipped(true), None).unwrap() };
            let imgs = unsafe { sc_d.get_swapchain_images(sc).unwrap() };
            eprintln!("Vulkan device: {} | hardware RT (ray query): {} | {}x{} {:?}", c.name, c.rt, wd, ht, fmt.format);
            let mut r = render::Renderer::new(&mut c, wd, ht, &load_textures(), settings_from_args()).unwrap();
            let s = sim::Sim::new(&self.w, 3, 0).unwrap(); upload_world(&mut r, &mut c, &self.w).unwrap(); let ci = upload_car(&mut r, &mut c, &s).unwrap();
            if c.rt { let a = accel::Accel::new(&mut c, &r.meshes).unwrap(); r.bind_tlas(&c.device, a.tlas); r.tlas = Some(a); }
            let sem = || unsafe { c.device.create_semaphore(&avk::SemaphoreCreateInfo::default(), None).unwrap() };
            let (sem_acq, sem_done) = (sem(), sem());
            self.st = Some(State { win, c, r, s, ci, surface, sc_d, sc, imgs, sem_acq, sem_done, last: std::time::Instant::now(), acc: 0.0, cam: None, frames: 0, steer: 0.0, auto: false });
        }
        fn window_event(&mut self, el: &ActiveEventLoop, _: WindowId, ev: WindowEvent) {
            match ev {
                WindowEvent::CloseRequested => el.exit(),
                WindowEvent::KeyboardInput { event, .. } => if let PhysicalKey::Code(k) = event.physical_key {
                    if event.state == ElementState::Pressed { self.keys.insert(k); if let Some(st) = self.st.as_mut() { match k {
                        KeyCode::KeyP => st.auto = !st.auto, KeyCode::KeyR => { let sp = st.s.spawn; st.s.car.reset(&st.s.pw, sp.x, sp.z, sp.yaw); }
                        KeyCode::Digit1 => st.r.settings.gi = 0, KeyCode::Digit2 => st.r.settings.gi = 1, KeyCode::Escape => el.exit(), _ => {} } } } else { self.keys.remove(&k); } },
                WindowEvent::RedrawRequested => { if let Err(e) = self.frame() { eprintln!("frame error: {e:?}"); el.exit(); } }
                _ => {}
            }
        }
        fn about_to_wait(&mut self, _: &ActiveEventLoop) { if let Some(st) = &self.st { st.win.request_redraw(); } }
    }
    impl App {
        fn frame(&mut self) -> Result<()> {
            let k = &self.keys; let Some(st) = self.st.as_mut() else { return Ok(()) };
            let now = std::time::Instant::now(); let dt = (now - st.last).as_secs_f64().min(0.1); st.last = now; st.acc += dt;
            let mut inp = tms::Input::default(); let tgt = (k.contains(&KeyCode::KeyA) || k.contains(&KeyCode::ArrowLeft)) as i32 as f32 - (k.contains(&KeyCode::KeyD) || k.contains(&KeyCode::ArrowRight)) as i32 as f32;
            st.steer += (tgt - st.steer).clamp(-3.0 * dt as f32, 3.0 * dt as f32); inp.steer = st.steer as f64;
            inp.throttle = (k.contains(&KeyCode::KeyW) || k.contains(&KeyCode::ArrowUp)) as i32 as f64; inp.brake = (k.contains(&KeyCode::KeyS) || k.contains(&KeyCode::ArrowDown)) as i32 as f64; inp.handbrake = k.contains(&KeyCode::Space) as i32 as f64;
            if let Some(g) = self.gil.as_mut() { while g.next_event().is_some() {} if let Some((_, pad)) = g.gamepads().next() {
                let sx = pad.value(gilrs::Axis::LeftStickX); if sx.abs() > 0.06 { inp.steer = -sx as f64; }
                let th = pad.button_data(gilrs::Button::RightTrigger2).map(|b| b.value()).unwrap_or(0.0); let br = pad.button_data(gilrs::Button::LeftTrigger2).map(|b| b.value()).unwrap_or(0.0);
                if th > 0.02 { inp.throttle = th as f64; } if br > 0.02 { inp.brake = br as f64; } } }
            if st.auto { inp = st.s.bot(); }
            let ticks = (st.acc * 240.0).floor() as i32; let ticks = ticks.min(24); st.acc -= ticks as f64 / 240.0; if st.acc > 0.1 { st.acc = 0.0; }
            if ticks > 0 { st.s.step(inp, ticks)?; }
            st.cam = Some(st.s.chase_camera(st.cam, dt as f32)); let (cp, ct) = st.cam.unwrap(); place_car(&mut st.r, &st.s, &st.ci); let (centre, ..) = st.s.basis();
            st.r.render(&st.c, Mat4::look_at_rh(cp, ct, Vec3::Y), cp, 62f32.to_radians(), centre, st.frames as f32 / 60.0)?; st.r.wait(&st.c)?;
            let (idx, _) = unsafe { st.sc_d.acquire_next_image(st.sc, u64::MAX, st.sem_acq, avk::Fence::null())? }; let img = st.imgs[idx as usize];
            let (d, out, w, h) = (st.c.device.clone(), st.r.out_image(), st.r.w, st.r.h);
            st.c.one_shot(|cb| unsafe {
                vk::barrier(&d, cb, img, avk::ImageLayout::UNDEFINED, avk::ImageLayout::TRANSFER_DST_OPTIMAL, false, 1);
                let sub = avk::ImageSubresourceLayers { aspect_mask: avk::ImageAspectFlags::COLOR, mip_level: 0, base_array_layer: 0, layer_count: 1 };
                let o = [avk::Offset3D::default(), avk::Offset3D { x: w as i32, y: h as i32, z: 1 }];
                d.cmd_blit_image(cb, out, avk::ImageLayout::GENERAL, img, avk::ImageLayout::TRANSFER_DST_OPTIMAL, &[avk::ImageBlit { src_subresource: sub, src_offsets: o, dst_subresource: sub, dst_offsets: o }], avk::Filter::NEAREST);
                vk::barrier(&d, cb, img, avk::ImageLayout::TRANSFER_DST_OPTIMAL, avk::ImageLayout::PRESENT_SRC_KHR, false, 1);
            })?; // NOTE: simple path - waits on acquire implicitly via queue idle; a production loop would chain semaphores
            let scs = [st.sc]; let ids = [idx]; let waits = [st.sem_acq];
            // acquire semaphore must be consumed: submit an empty batch waiting on it, then present
            let stages = [avk::PipelineStageFlags::ALL_COMMANDS]; let sigs = [st.sem_done];
            unsafe { st.c.device.queue_submit(st.c.queue, &[avk::SubmitInfo::default().wait_semaphores(&waits).wait_dst_stage_mask(&stages).signal_semaphores(&sigs)], avk::Fence::null())?;
                st.sc_d.queue_present(st.c.queue, &avk::PresentInfoKHR::default().wait_semaphores(&sigs).swapchains(&scs).image_indices(&ids))?; }
            st.frames += 1;
            if st.frames % 60 == 0 { let t: Vec<String> = render::PASS_NAMES.iter().zip(st.r.timings.iter()).map(|(n, t)| format!("{n} {t:.2}")).collect();
                st.win.set_title(&format!("DriveChain native | {:.0} km/h | GPU ms: {}", st.s.state.speed() * 3.6, t.join(" | "))); }
            Ok(())
        }
    }
}
