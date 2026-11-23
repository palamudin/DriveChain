//! Vegetation (native): ez-tree trees + shrubs and shader grass. Replaces the stand-in foliage blocks of the Ušće mesh.
//!   * assets: native/assets/vegetation (tools/bake_vegetation.mjs: same ez-tree presets + seeds as the web build) -
//!     per kind branch + leaf meshes, bark/leaf textures, 8x8 hemi-octahedral impostor atlases (albedo+alpha, normal)
//!   * placement: <world dir>/veg_plan.json (tools/usce_veg_prep.py: seeded, at the original foliage positions, road clearance)
//!   * near (< impostor distance): instanced meshes, alpha-tested leaves with coverage-preserving mips, wind in the VS
//!   * far: camera-facing octahedral impostors; in the shadow pass the same impostors face the sun (cheap far tree shadows)
//!   * grass: grass_gen.comp builds the blade buffer per frame (cells around the camera, only inside COL_grass triangles),
//!     drawn with one indirect instanced draw (grass.vert builds a 5-segment blade from gl_VertexIndex)
use crate::vk::{shader, Buffer, Ctx, Image};
use anyhow::{anyhow, Context, Result};
use ash::vk;
use glam::{Mat4, Vec3, Vec4};
use gpu_allocator::vulkan::{AllocationCreateDesc, AllocationScheme};
use gpu_allocator::MemoryLocation;
use std::collections::HashMap;

macro_rules! spv { ($n:literal) => { include_bytes!(concat!(env!("OUT_DIR"), "/", $n, ".spv")) } }

#[derive(Clone, Copy, Debug)]
pub struct VegOpts { pub trees: bool, pub shrubs: bool, pub grass: bool, pub grass_density: f32, pub grass_dist: f32, pub tree_shadows: bool, pub imp_dist: f32, pub max_blades: u32, pub aniso: f32, pub imp_band: f32 }
impl Default for VegOpts { fn default() -> Self { VegOpts { trees: true, shrubs: true, grass: true, grass_density: 24.0, grass_dist: 70.0, tree_shadows: true, imp_dist: 90.0, max_blades: 1_500_000, aniso: 16.0, imp_band: 0.2 } } }
impl VegOpts {
    /// --no-trees --no-shrubs --no-grass --grass-density N --grass-dist M --no-tree-shadows --imp-dist M --imp-band F (crossfade band, fraction of imp-dist) --max-blades N --aniso N
    pub fn from_args() -> VegOpts {
        let a: Vec<String> = std::env::args().collect(); let has = |n: &str| a.iter().any(|x| x == n);
        let num = |n: &str, d: f32| a.iter().position(|x| x == n).and_then(|i| a.get(i + 1)).and_then(|v| v.parse().ok()).unwrap_or(d);
        let d = VegOpts::default();
        VegOpts { trees: !has("--no-trees"), shrubs: !has("--no-shrubs"), grass: !has("--no-grass"), grass_density: num("--grass-density", d.grass_density), grass_dist: num("--grass-dist", d.grass_dist),
            tree_shadows: !has("--no-tree-shadows"), imp_dist: num("--imp-dist", d.imp_dist), max_blades: num("--max-blades", d.max_blades as f32) as u32,
            aniso: num("--aniso", d.aniso), imp_band: num("--imp-band", d.imp_band).clamp(0.0, 0.8) }
    }
}

#[repr(C)] #[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)] pub struct Inst { pos_yaw: [f32; 4], prm: [f32; 4] }
#[repr(C)] #[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)] struct VPC { shadow_vp: Mat4, tint: Vec4, surf: u32, layer: u32, mode: u32, kind_n: u32, fade: Vec4 }
#[repr(C)] #[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)] struct GPC { p: Vec4, max_blades: u32, cell_count: u32, pad: [u32; 2] }

struct Kind { name: String, shrub: bool, br: (Buffer, Buffer, u32), lf: (Buffer, Buffer, u32), bark_tint: Vec4, leaf_tint: Vec4, layer: u32, n: u32,
    list: Vec<Inst>, near: Buffer, far: Buffer, near_n: u32, far_n: u32, tris: u64 }
struct Grass { blades: Buffer, indirect: Buffer, tris: Buffer, cells_buf: Buffer, cells: HashMap<(i32, i32), (u32, u32)>, cell_count: u32, pipe: vk::Pipeline, layout: vk::PipelineLayout, area_m2: f32 }
pub struct Veg { kinds: Vec<Kind>, _tex: Image, _imp: Image, set: vk::DescriptorSet, layout: vk::PipelineLayout,
    p_mesh: vk::Pipeline, p_imp: vk::Pipeline, p_grass: vk::Pipeline, s_mesh: vk::Pipeline, s_imp: vk::Pipeline, grass: Option<Grass>, pub opts: VegOpts,
    pub near_trees: u32, pub far_trees: u32, pub near_tris: u64, pub blades_cap: u32, pub cells_in_range: u32 }

const CELL: f32 = 8.0;

/// RGBA8 mip chain; alpha mips keep the mip-0 alpha-test coverage (Castaño 2010), so leaves don't thin out / shimmer at distance.
pub fn mips(rgba: &[u8], w: u32, h: u32, alpha: bool, cutoff: f32) -> Vec<(u32, u32, Vec<u8>)> {
    let cov = |p: &[u8], s: f32| p.chunks(4).filter(|c| (c[3] as f32 / 255.0 * s) >= cutoff).count() as f32 / (p.len() / 4).max(1) as f32;
    let c0 = cov(rgba, 1.0); let mut out = vec![(w, h, rgba.to_vec())]; let (mut cw, mut ch) = (w, h);
    while cw > 1 || ch > 1 {
        let (nw, nh) = ((cw / 2).max(1), (ch / 2).max(1)); let src = &out.last().unwrap().2; let mut dst = vec![0u8; (nw * nh * 4) as usize];
        for y in 0..nh { for x in 0..nw { let mut acc = [0f32; 4]; let mut wa = 0f32;
            for (dx, dy) in [(0, 0), (1, 0), (0, 1), (1, 1)] { let sx = (x * 2 + dx).min(cw - 1); let sy = (y * 2 + dy).min(ch - 1); let o = ((sy * cw + sx) * 4) as usize;
                let a = if alpha { src[o + 3] as f32 / 255.0 } else { 1.0 }; for k in 0..3 { acc[k] += src[o + k] as f32 * a; } acc[3] += src[o + 3] as f32; wa += a; }
            let o = ((y * nw + x) * 4) as usize; for k in 0..3 { dst[o + k] = if wa > 0.0 { (acc[k] / wa).round() as u8 } else { 0 }; } dst[o + 3] = (acc[3] / 4.0).round() as u8; } }
        if alpha && c0 > 0.0 { // scale alpha so coverage matches mip 0 (binary search on the scale)
            let (mut lo, mut hi) = (0.1f32, 8.0f32); for _ in 0..14 { let m = 0.5 * (lo + hi); if cov(&dst, m) < c0 { lo = m } else { hi = m } }
            let s = 0.5 * (lo + hi); for c in dst.chunks_mut(4) { c[3] = ((c[3] as f32 * s).min(255.0)) as u8; } }
        out.push((nw, nh, dst)); cw = nw; ch = nh;
    }
    out
}

pub fn texture_array(c: &mut Ctx, name: &str, size: u32, layers: &[Vec<(u32, u32, Vec<u8>)>], srgb: &[bool]) -> Result<Image> {
    let nm = layers[0].len() as u32; let n = layers.len() as u32;
    // two formats are not possible in one image: albedo layers sRGB, data layers (normals) handled in the shader via UNORM view
    let fmt = if srgb.iter().all(|&s| s) { vk::Format::R8G8B8A8_SRGB } else { vk::Format::R8G8B8A8_UNORM };
    let ci = vk::ImageCreateInfo::default().image_type(vk::ImageType::TYPE_2D).format(fmt).extent(vk::Extent3D { width: size, height: size, depth: 1 })
        .mip_levels(nm).array_layers(n).samples(vk::SampleCountFlags::TYPE_1).tiling(vk::ImageTiling::OPTIMAL).usage(vk::ImageUsageFlags::SAMPLED | vk::ImageUsageFlags::TRANSFER_DST);
    let image = unsafe { c.device.create_image(&ci, None)? }; let req = unsafe { c.device.get_image_memory_requirements(image) };
    let a = c.alloc.allocate(&AllocationCreateDesc { name, requirements: req, location: MemoryLocation::GpuOnly, linear: false, allocation_scheme: AllocationScheme::GpuAllocatorManaged })?;
    unsafe { c.device.bind_image_memory(image, a.memory(), a.offset())? };
    let mut staging = vec![]; let mut regions = vec![];
    for (l, chain) in layers.iter().enumerate() { for (m, (w, h, px)) in chain.iter().enumerate() {
        regions.push(vk::BufferImageCopy::default().buffer_offset(staging.len() as u64).image_subresource(vk::ImageSubresourceLayers { aspect_mask: vk::ImageAspectFlags::COLOR, mip_level: m as u32, base_array_layer: l as u32, layer_count: 1 })
            .image_extent(vk::Extent3D { width: *w, height: *h, depth: 1 })); staging.extend_from_slice(px); } }
    let sb = c.upload(name, &staging, vk::BufferUsageFlags::TRANSFER_SRC)?; let d = c.device.clone();
    let rng = vk::ImageSubresourceRange { aspect_mask: vk::ImageAspectFlags::COLOR, base_mip_level: 0, level_count: nm, base_array_layer: 0, layer_count: n };
    let bar = |cb, from, to| { let b = [vk::ImageMemoryBarrier2::default().image(image).old_layout(from).new_layout(to).src_stage_mask(vk::PipelineStageFlags2::ALL_COMMANDS).dst_stage_mask(vk::PipelineStageFlags2::ALL_COMMANDS)
        .src_access_mask(vk::AccessFlags2::MEMORY_WRITE).dst_access_mask(vk::AccessFlags2::MEMORY_READ | vk::AccessFlags2::MEMORY_WRITE).subresource_range(rng)]; unsafe { d.cmd_pipeline_barrier2(cb, &vk::DependencyInfo::default().image_memory_barriers(&b)) } };
    c.one_shot(|cb| unsafe { bar(cb, vk::ImageLayout::UNDEFINED, vk::ImageLayout::TRANSFER_DST_OPTIMAL); d.cmd_copy_buffer_to_image(cb, sb.buf, image, vk::ImageLayout::TRANSFER_DST_OPTIMAL, &regions); bar(cb, vk::ImageLayout::TRANSFER_DST_OPTIMAL, vk::ImageLayout::GENERAL); })?;
    c.free_buffer(sb);
    let view = unsafe { c.device.create_image_view(&vk::ImageViewCreateInfo::default().image(image).format(fmt).view_type(vk::ImageViewType::TYPE_2D_ARRAY).subresource_range(rng), None)? };
    Ok(Image { image, view, alloc: Some(a), format: fmt, extent: vk::Extent2D { width: size, height: size }, layers: n })
}

fn load_rgba(path: &std::path::Path, size: u32) -> Result<Vec<u8>> {
    let i = image::open(path).with_context(|| format!("{}", path.display()))?; Ok(i.resize_exact(size, size, image::imageops::FilterType::Triangle).to_rgba8().into_raw())
}

fn pipeline(d: &ash::Device, layout: vk::PipelineLayout, vs: &[u8], fs: &[u8], colors: &[vk::Format], binds: &[vk::VertexInputBindingDescription], attrs: &[vk::VertexInputAttributeDescription], bias: Option<(f32, f32)>) -> Result<vk::Pipeline> {
    let (vm, fm) = (shader(d, vs)?, shader(d, fs)?);
    let stages = [vk::PipelineShaderStageCreateInfo::default().stage(vk::ShaderStageFlags::VERTEX).module(vm).name(c"main"), vk::PipelineShaderStageCreateInfo::default().stage(vk::ShaderStageFlags::FRAGMENT).module(fm).name(c"main")];
    let vi = vk::PipelineVertexInputStateCreateInfo::default().vertex_binding_descriptions(binds).vertex_attribute_descriptions(attrs);
    let ia = vk::PipelineInputAssemblyStateCreateInfo::default().topology(vk::PrimitiveTopology::TRIANGLE_LIST);
    let vp = vk::PipelineViewportStateCreateInfo::default().viewport_count(1).scissor_count(1);
    let rs = vk::PipelineRasterizationStateCreateInfo::default().polygon_mode(vk::PolygonMode::FILL).cull_mode(vk::CullModeFlags::NONE).line_width(1.0)
        .depth_bias_enable(bias.is_some()).depth_bias_constant_factor(bias.map(|b| b.0).unwrap_or(0.0)).depth_bias_slope_factor(bias.map(|b| b.1).unwrap_or(0.0));
    let ms = vk::PipelineMultisampleStateCreateInfo::default().rasterization_samples(vk::SampleCountFlags::TYPE_1);
    let ds = vk::PipelineDepthStencilStateCreateInfo::default().depth_test_enable(true).depth_write_enable(true).depth_compare_op(vk::CompareOp::LESS);
    let cba: Vec<_> = colors.iter().map(|_| vk::PipelineColorBlendAttachmentState::default().color_write_mask(vk::ColorComponentFlags::RGBA)).collect();
    let cb = vk::PipelineColorBlendStateCreateInfo::default().attachments(&cba);
    let dyns = [vk::DynamicState::VIEWPORT, vk::DynamicState::SCISSOR]; let dy = vk::PipelineDynamicStateCreateInfo::default().dynamic_states(&dyns);
    let mut ri = vk::PipelineRenderingCreateInfo::default().color_attachment_formats(colors).depth_attachment_format(vk::Format::D32_SFLOAT);
    let ci = vk::GraphicsPipelineCreateInfo::default().stages(&stages).vertex_input_state(&vi).input_assembly_state(&ia).viewport_state(&vp).rasterization_state(&rs)
        .multisample_state(&ms).depth_stencil_state(&ds).color_blend_state(&cb).dynamic_state(&dy).layout(layout).push_next(&mut ri);
    let p = unsafe { d.create_graphics_pipelines(vk::PipelineCache::null(), &[ci], None).map_err(|e| e.1)?[0] };
    unsafe { d.destroy_shader_module(vm, None); d.destroy_shader_module(fm, None); } Ok(p)
}

impl Veg {
    pub fn new(c: &mut Ctx, ubo: (vk::Buffer, u64), world: &crate::world::World, world_path: &str, opts: VegOpts) -> Result<Veg> {
        let t0 = std::time::Instant::now();
        let adir = std::path::PathBuf::from(std::env::var("DCS_VEG").unwrap_or_else(|_| "../native/assets/vegetation".into()));
        let meta: serde_json::Value = serde_json::from_slice(&std::fs::read(adir.join("vegetation.json")).with_context(|| format!("vegetation assets not found in {} (set DCS_VEG)", adir.display()))?)?;
        let plan_path = std::env::var("DCS_VEGPLAN").map(std::path::PathBuf::from).unwrap_or_else(|_| std::path::Path::new(world_path).with_file_name("veg_plan.json"));
        let plan: serde_json::Value = serde_json::from_slice(&std::fs::read(&plan_path).unwrap_or_else(|_| b"{\"trees\":[],\"shrubs\":[]}".to_vec()))?;
        let kinds_meta = meta["kinds"].as_object().ok_or_else(|| anyhow!("vegetation.json: no kinds"))?;
        let mut tex_layers = vec![]; let mut imp_layers = vec![]; let mut kinds = vec![];
        let hexv = |v: &serde_json::Value| { let h = v.as_u64().unwrap_or(0xffffff) as u32; Vec4::new(((h >> 16) & 255) as f32 / 255.0, ((h >> 8) & 255) as f32 / 255.0, (h & 255) as f32 / 255.0, 1.0) };
        let srgb_lin = |v: Vec4| Vec4::new(v.x.powf(2.2), v.y.powf(2.2), v.z.powf(2.2), 1.0);
        let mut all: Vec<(String, serde_json::Value)> = vec![];
        if opts.trees { for t in plan["trees"].as_array().into_iter().flatten() { all.push((t["kind"].as_str().unwrap_or("").into(), t.clone())); } }
        if opts.shrubs { for t in plan["shrubs"].as_array().into_iter().flatten() { all.push((t["kind"].as_str().unwrap_or("").into(), t.clone())); } }
        for (k, (name, m)) in kinds_meta.iter().enumerate() {
            let f = |key: &str| m["files"][key].as_str().map(|s| adir.join(s));
            let bark = f("bark").map(|p| load_rgba(&p, 512)).transpose()?.unwrap_or_else(|| vec![110; 512 * 512 * 4]);
            let leaf = f("leaf").map(|p| load_rgba(&p, 512)).transpose()?.unwrap_or_else(|| vec![255; 512 * 512 * 4]);
            let at = m["alphaTest"].as_f64().unwrap_or(0.5) as f32;
            tex_layers.push(mips(&bark, 512, 512, false, 0.5)); tex_layers.push(mips(&leaf, 512, 512, true, at.max(0.3)));
            let ia = f("atlasAlbedo").map(|p| load_rgba(&p, 1024)).transpose()?.unwrap_or_else(|| vec![0; 1024 * 1024 * 4]);
            let inn = f("atlasNormal").map(|p| load_rgba(&p, 1024)).transpose()?.unwrap_or_else(|| vec![128; 1024 * 1024 * 4]);
            imp_layers.push(mips(&ia, 1024, 1024, true, 0.5)); imp_layers.push(mips(&inn, 1024, 1024, false, 0.5));
            // geometry
            let bin = std::fs::read(adir.join(format!("{name}.bin")))?; let hdr: [u32; 4] = bytemuck::pod_read_unaligned(&bin[..16]); let mut off = 16usize;
            let mut part = |nv: u32, ni: u32| -> Result<(Buffer, Buffer, u32)> { let vb = &bin[off..off + nv as usize * 32]; off += nv as usize * 32; let ib = &bin[off..off + ni as usize * 4]; off += ni as usize * 4;
                Ok((c.upload("veg-vb", vb, vk::BufferUsageFlags::VERTEX_BUFFER)?, c.upload("veg-ib", ib, vk::BufferUsageFlags::INDEX_BUFFER)?, ni)) };
            let br = part(hdr[0], hdr[1])?; let lf = part(hdr[2], hdr[3])?;
            let (h, base, cy, r) = (m["h"].as_f64().unwrap_or(1.0) as f32, m["base"].as_f64().unwrap_or(0.0) as f32, m["cy"].as_f64().unwrap_or(0.0) as f32, m["R"].as_f64().unwrap_or(1.0) as f32);
            let list: Vec<Inst> = all.iter().filter(|(kn, _)| kn == name).map(|(_, t)| { let g = |k: &str| t[k].as_f64().unwrap_or(0.0) as f32; let s = g("h") / h;
                Inst { pos_yaw: [g("x"), g("y") - base * s, g("z"), g("yaw")], prm: [s, (g("x") * 0.37 + g("z") * 0.23).fract() * 6.28, r * s, cy * s] } }).collect();
            let cap = (list.len().max(1) * 32) as u64;
            let near = c.buffer("veg-near", cap, vk::BufferUsageFlags::VERTEX_BUFFER, MemoryLocation::CpuToGpu)?; let far = c.buffer("veg-far", cap, vk::BufferUsageFlags::VERTEX_BUFFER, MemoryLocation::CpuToGpu)?;
            kinds.push(Kind { name: name.clone(), shrub: m["shrub"].as_bool().unwrap_or(false), tris: (hdr[1] + hdr[3]) as u64 / 3, br, lf, bark_tint: srgb_lin(hexv(&m["barkTint"])), leaf_tint: srgb_lin(hexv(&m["leafTint"])),
                layer: 2 * k as u32, n: m["N"].as_u64().unwrap_or(8) as u32, list, near, far, near_n: 0, far_n: 0 });
        }
        let tex = texture_array(c, "vegTex", 512, &tex_layers, &vec![true; tex_layers.len()])?;
        // impostor atlases hold linear values (baked into a linear 8-bit target without tonemapping): UNORM array, no decode
        let imp = texture_array(c, "impTex", 1024, &imp_layers, &vec![false; imp_layers.len()])?;
        let d = c.device.clone();
        let an = opts.aniso.clamp(1.0, unsafe { c.instance.get_physical_device_properties(c.pdev) }.limits.max_sampler_anisotropy.max(1.0));
        let lin = unsafe { d.create_sampler(&vk::SamplerCreateInfo::default().mag_filter(vk::Filter::LINEAR).min_filter(vk::Filter::LINEAR).mipmap_mode(vk::SamplerMipmapMode::LINEAR)
            .address_mode_u(vk::SamplerAddressMode::REPEAT).address_mode_v(vk::SamplerAddressMode::REPEAT).anisotropy_enable(an > 1.0).max_anisotropy(an).max_lod(16.0), None)? };
        let clampm = unsafe { d.create_sampler(&vk::SamplerCreateInfo::default().mag_filter(vk::Filter::LINEAR).min_filter(vk::Filter::LINEAR).mipmap_mode(vk::SamplerMipmapMode::LINEAR)
            .address_mode_u(vk::SamplerAddressMode::CLAMP_TO_EDGE).address_mode_v(vk::SamplerAddressMode::CLAMP_TO_EDGE).max_lod(16.0), None)? };
        // ---- grass (compute)
        let grass = if opts.grass { Some(Self::make_grass(c, world, opts)?) } else { None };
        // ---- descriptors: 0 ubo, 1 vegTex, 2 impTex, 3 blades, 4 indirect, 5 tris, 6 cells
        let ty = [vk::DescriptorType::UNIFORM_BUFFER, vk::DescriptorType::COMBINED_IMAGE_SAMPLER, vk::DescriptorType::COMBINED_IMAGE_SAMPLER, vk::DescriptorType::STORAGE_BUFFER, vk::DescriptorType::STORAGE_BUFFER, vk::DescriptorType::STORAGE_BUFFER, vk::DescriptorType::STORAGE_BUFFER];
        let binds: Vec<_> = ty.iter().enumerate().map(|(i, t)| vk::DescriptorSetLayoutBinding::default().binding(i as u32).descriptor_type(*t).descriptor_count(1).stage_flags(vk::ShaderStageFlags::ALL)).collect();
        let dsl = unsafe { d.create_descriptor_set_layout(&vk::DescriptorSetLayoutCreateInfo::default().bindings(&binds), None)? };
        let sizes = [vk::DescriptorPoolSize { ty: vk::DescriptorType::UNIFORM_BUFFER, descriptor_count: 1 }, vk::DescriptorPoolSize { ty: vk::DescriptorType::COMBINED_IMAGE_SAMPLER, descriptor_count: 2 }, vk::DescriptorPoolSize { ty: vk::DescriptorType::STORAGE_BUFFER, descriptor_count: 4 }];
        let pool = unsafe { d.create_descriptor_pool(&vk::DescriptorPoolCreateInfo::default().max_sets(1).pool_sizes(&sizes), None)? };
        let set = unsafe { d.allocate_descriptor_sets(&vk::DescriptorSetAllocateInfo::default().descriptor_pool(pool).set_layouts(&[dsl]))?[0] };
        let dummy = c.buffer("veg-dummy", 64, vk::BufferUsageFlags::STORAGE_BUFFER, MemoryLocation::GpuOnly)?;
        let sb = |b: Option<&Buffer>| { let b = b.unwrap_or(&dummy); [vk::DescriptorBufferInfo { buffer: b.buf, offset: 0, range: vk::WHOLE_SIZE }] };
        let (b3, b4, b5, b6) = match &grass { Some(g) => (sb(Some(&g.blades)), sb(Some(&g.indirect)), sb(Some(&g.tris)), sb(Some(&g.cells_buf))), None => (sb(None), sb(None), sb(None), sb(None)) };
        let ub = [vk::DescriptorBufferInfo { buffer: ubo.0, offset: 0, range: ubo.1 }];
        let i1 = [vk::DescriptorImageInfo { sampler: lin, image_view: tex.view, image_layout: vk::ImageLayout::GENERAL }]; let i2 = [vk::DescriptorImageInfo { sampler: clampm, image_view: imp.view, image_layout: vk::ImageLayout::GENERAL }];
        let w = |b: u32| vk::WriteDescriptorSet::default().dst_set(set).dst_binding(b).descriptor_count(1);
        unsafe { d.update_descriptor_sets(&[w(0).descriptor_type(ty[0]).buffer_info(&ub), w(1).descriptor_type(ty[1]).image_info(&i1), w(2).descriptor_type(ty[2]).image_info(&i2),
            w(3).descriptor_type(ty[3]).buffer_info(&b3), w(4).descriptor_type(ty[4]).buffer_info(&b4), w(5).descriptor_type(ty[5]).buffer_info(&b5), w(6).descriptor_type(ty[6]).buffer_info(&b6)], &[]); }
        std::mem::forget(dummy);
        let pcr = [vk::PushConstantRange { stage_flags: vk::ShaderStageFlags::VERTEX | vk::ShaderStageFlags::FRAGMENT, offset: 0, size: std::mem::size_of::<VPC>() as u32 }];
        let layout = unsafe { d.create_pipeline_layout(&vk::PipelineLayoutCreateInfo::default().set_layouts(&[dsl]).push_constant_ranges(&pcr), None)? };
        let gfmt = [vk::Format::R8G8B8A8_UNORM, vk::Format::R16G16B16A16_SFLOAT, vk::Format::R16G16_SFLOAT];
        let vert = |b: u32, loc: u32, off: u32, f: vk::Format| vk::VertexInputAttributeDescription { location: loc, binding: b, format: f, offset: off };
        let mb = [vk::VertexInputBindingDescription { binding: 0, stride: 32, input_rate: vk::VertexInputRate::VERTEX }, vk::VertexInputBindingDescription { binding: 1, stride: 32, input_rate: vk::VertexInputRate::INSTANCE }];
        let ma = [vert(0, 0, 0, vk::Format::R32G32B32_SFLOAT), vert(0, 1, 12, vk::Format::R32G32B32_SFLOAT), vert(0, 2, 24, vk::Format::R32G32_SFLOAT), vert(1, 3, 0, vk::Format::R32G32B32A32_SFLOAT), vert(1, 4, 16, vk::Format::R32G32B32A32_SFLOAT)];
        let ib = [vk::VertexInputBindingDescription { binding: 0, stride: 32, input_rate: vk::VertexInputRate::INSTANCE }];
        let ia = [vert(0, 3, 0, vk::Format::R32G32B32A32_SFLOAT), vert(0, 4, 16, vk::Format::R32G32B32A32_SFLOAT)];
        let p_mesh = pipeline(&d, layout, spv!("veg.vert"), spv!("veg.frag"), &gfmt, &mb, &ma, None)?;
        let p_imp = pipeline(&d, layout, spv!("imp.vert"), spv!("veg.frag"), &gfmt, &ib, &ia, None)?;
        let p_grass = pipeline(&d, layout, spv!("grass.vert"), spv!("veg.frag"), &gfmt, &[], &[], None)?;
        // foliage shadow bias: larger slope term than the world (thin alpha-tested cards self-shadow / acne otherwise)
        let s_mesh = pipeline(&d, layout, spv!("veg.vert"), spv!("veg_shadow.frag"), &[], &mb, &ma, Some((2.0, 3.0)))?;
        let s_imp = pipeline(&d, layout, spv!("imp.vert"), spv!("veg_shadow.frag"), &[], &ib, &ia, Some((4.0, 3.0)))?;
        let grass = match grass { Some(mut g) => {
            let gl = unsafe { d.create_pipeline_layout(&vk::PipelineLayoutCreateInfo::default().set_layouts(&[dsl]).push_constant_ranges(&[vk::PushConstantRange { stage_flags: vk::ShaderStageFlags::COMPUTE, offset: 0, size: std::mem::size_of::<GPC>() as u32 }]), None)? };
            let m = shader(&d, spv!("grass_gen.comp"))?; let st = vk::PipelineShaderStageCreateInfo::default().stage(vk::ShaderStageFlags::COMPUTE).module(m).name(c"main");
            g.pipe = unsafe { d.create_compute_pipelines(vk::PipelineCache::null(), &[vk::ComputePipelineCreateInfo::default().stage(st).layout(gl)], None).map_err(|e| e.1)?[0] }; g.layout = gl; unsafe { d.destroy_shader_module(m, None) }; Some(g) }
            None => None };
        let nt: usize = kinds.iter().map(|k| k.list.len()).sum();
        crate::logln(&format!("vegetation: {} kinds, {} trees+shrubs from {}, grass {} ({:.1} ha, density {}/m2, radius {} m, max {} blades), impostor distance {} m, tree shadows {} | setup {:.0} ms",
            kinds.len(), nt, plan_path.display(), if grass.is_some() { "on" } else { "off" }, grass.as_ref().map(|g| g.area_m2 / 1e4).unwrap_or(0.0), opts.grass_density, opts.grass_dist, opts.max_blades, opts.imp_dist, opts.tree_shadows, t0.elapsed().as_secs_f64() * 1e3));
        Ok(Veg { kinds, _tex: tex, _imp: imp, set, layout, p_mesh, p_imp, p_grass, s_mesh, s_imp, grass, opts, near_trees: 0, far_trees: 0, near_tris: 0, blades_cap: opts.max_blades, cells_in_range: 0 })
    }
    fn make_grass(c: &mut Ctx, world: &crate::world::World, opts: VegOpts) -> Result<Grass> {
        // grass triangles (w=0) + road/kerb/line/pavement triangles (w=1) in the same 8 m cells: blades are rejected where a road
        // surface lies above or near the candidate (the Ušće grass collision mesh continues under some carriageways)
        let mut bins: HashMap<(i32, i32), Vec<([f32; 9], f32)>> = HashMap::new(); let mut area = 0f32;
        for p in world.parts.iter().filter(|p| p.collide && matches!(p.surf, 0 | 1 | 2 | 5 | 6 | 8 | 11)) { let road = p.surf != 2; for t in p.indices.chunks(3) { if t.len() < 3 { continue; }
            let (a, b, cc) = (p.positions[t[0] as usize], p.positions[t[1] as usize], p.positions[t[2] as usize]); let ar = ((b[0] - a[0]) * (cc[2] - a[2]) - (cc[0] - a[0]) * (b[2] - a[2])).abs() * 0.5;
            if !(ar > 1e-4) { continue; } if !road { area += ar; }
            let (x0, x1) = ((a[0].min(b[0]).min(cc[0]) / CELL).floor() as i32, (a[0].max(b[0]).max(cc[0]) / CELL).floor() as i32); let (z0, z1) = ((a[2].min(b[2]).min(cc[2]) / CELL).floor() as i32, (a[2].max(b[2]).max(cc[2]) / CELL).floor() as i32);
            if (x1 - x0 + 1) * (z1 - z0 + 1) > 4096 { continue; }
            for i in x0..=x1 { for k in z0..=z1 { bins.entry((i, k)).or_default().push(([a[0], a[1], a[2], b[0], b[1], b[2], cc[0], cc[1], cc[2]], road as i32 as f32)); } } } }
        bins.retain(|_, l| l.iter().any(|t| t.1 == 0.0)); for l in bins.values_mut() { l.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap()); } // road tris first
        let mut tris: Vec<[f32; 4]> = vec![]; let mut cells = HashMap::new();
        for (key, list) in bins { cells.insert(key, ((tris.len() / 3) as u32, list.len() as u32)); for (t, road) in list { for v in 0..3 { tris.push([t[v * 3], t[v * 3 + 1], t[v * 3 + 2], road]); } } }
        if tris.is_empty() { tris.push([0.0; 4]); }
        let st = vk::BufferUsageFlags::STORAGE_BUFFER;
        let tb = c.upload("grass-tris", &tris, st)?;
        let blades = c.buffer("grass-blades", opts.max_blades.max(1) as u64 * 32, st, MemoryLocation::GpuOnly)?;
        let indirect = c.buffer("grass-indirect", 16, st | vk::BufferUsageFlags::INDIRECT_BUFFER | vk::BufferUsageFlags::TRANSFER_DST, MemoryLocation::GpuOnly)?;
        let cr = (opts.grass_dist / CELL).ceil() as u64 * 2 + 3; let cells_buf = c.buffer("grass-cells", cr * cr * 16, st, MemoryLocation::CpuToGpu)?;
        Ok(Grass { blades, indirect, tris: tb, cells_buf, cells, cell_count: 0, pipe: vk::Pipeline::null(), layout: vk::PipelineLayout::null(), area_m2: area })
    }
    /// CPU per frame (after the previous frame's fence): near/far split + grass cell list.
    pub fn update(&mut self, cam: Vec3) {
        let (mut nn, mut nf, mut tris) = (0, 0, 0u64);
        for k in self.kinds.iter_mut() { let r = if k.shrub { self.opts.imp_dist * 0.6 } else { self.opts.imp_dist }; let b = r * self.opts.imp_band * 0.5; let (n2, f2) = ((r + b) * (r + b), (r - b) * (r - b));
            let mut near = vec![]; let mut far = vec![];
            // inside the crossfade band a tree is in BOTH lists; veg.frag dithers mesh vs impostor by distance (no hard LOD pop)
            for i in &k.list { let dx = i.pos_yaw[0] - cam.x; let dz = i.pos_yaw[2] - cam.z; let d2 = dx * dx + dz * dz; if d2 < n2 { near.push(*i) } if d2 >= f2 { far.push(*i) } }
            if let Some(a) = k.near.alloc.as_mut() { if let Some(m) = a.mapped_slice_mut() { let b: &[u8] = bytemuck::cast_slice(&near); m[..b.len()].copy_from_slice(b); } }
            if let Some(a) = k.far.alloc.as_mut() { if let Some(m) = a.mapped_slice_mut() { let b: &[u8] = bytemuck::cast_slice(&far); m[..b.len()].copy_from_slice(b); } }
            k.near_n = near.len() as u32; k.far_n = far.len() as u32; nn += k.near_n; nf += k.far_n; tris += k.near_n as u64 * k.tris; }
        self.near_trees = nn; self.far_trees = nf; self.near_tris = tris;
        if let Some(g) = self.grass.as_mut() { let rc = (self.opts.grass_dist / CELL).ceil() as i32 + 1; let (ci, ck) = ((cam.x / CELL).floor() as i32, (cam.z / CELL).floor() as i32); let mut list: Vec<(f32, [i32; 4])> = vec![];
            for i in -rc..=rc { for k in -rc..=rc { if let Some(&(f, n)) = g.cells.get(&(ci + i, ck + k)) { let d = (((ci + i) as f32 + 0.5) * CELL - cam.x).hypot(((ck + k) as f32 + 0.5) * CELL - cam.z);
                if d < self.opts.grass_dist + CELL { list.push((d, [ci + i, ck + k, f as i32, n as i32])); } } } }
            list.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap()); let v: Vec<[i32; 4]> = list.iter().map(|x| x.1).collect();
            if let Some(m) = g.cells_buf.alloc.as_mut().and_then(|a| a.mapped_slice_mut()) { let b: &[u8] = bytemuck::cast_slice(&v); let n = b.len().min(m.len()); m[..n].copy_from_slice(&b[..n]); g.cell_count = (n / 16) as u32; }
            self.cells_in_range = g.cell_count; }
    }
    /// Outside any render pass: rebuild the grass blade buffer + indirect args.
    pub fn record_compute(&self, d: &ash::Device, cb: vk::CommandBuffer, mats: &crate::mat::Mats) {
        let Some(g) = &self.grass else { return }; if !mats.visible(crate::mat::G_GRASS) { return; }
        let bar = |src: vk::PipelineStageFlags2, sa: vk::AccessFlags2, dst: vk::PipelineStageFlags2, da: vk::AccessFlags2| { let mb = [vk::MemoryBarrier2::default().src_stage_mask(src).src_access_mask(sa).dst_stage_mask(dst).dst_access_mask(da)];
            unsafe { d.cmd_pipeline_barrier2(cb, &vk::DependencyInfo::default().memory_barriers(&mb)) } };
        unsafe {
            bar(vk::PipelineStageFlags2::ALL_COMMANDS, vk::AccessFlags2::MEMORY_READ | vk::AccessFlags2::MEMORY_WRITE, vk::PipelineStageFlags2::TRANSFER, vk::AccessFlags2::TRANSFER_WRITE);
            d.cmd_fill_buffer(cb, g.indirect.buf, 0, 4, 27); d.cmd_fill_buffer(cb, g.indirect.buf, 4, 12, 0);
            bar(vk::PipelineStageFlags2::TRANSFER, vk::AccessFlags2::TRANSFER_WRITE, vk::PipelineStageFlags2::COMPUTE_SHADER, vk::AccessFlags2::SHADER_STORAGE_READ | vk::AccessFlags2::SHADER_STORAGE_WRITE);
            if g.cell_count > 0 {
                d.cmd_bind_pipeline(cb, vk::PipelineBindPoint::COMPUTE, g.pipe); d.cmd_bind_descriptor_sets(cb, vk::PipelineBindPoint::COMPUTE, g.layout, 0, &[self.set], &[]);
                let pc = GPC { p: Vec4::new(self.opts.grass_density, self.opts.grass_dist, CELL, (self.opts.grass_dist * 0.25).min(18.0)), max_blades: self.opts.max_blades, cell_count: g.cell_count, pad: [0; 2] };
                d.cmd_push_constants(cb, g.layout, vk::ShaderStageFlags::COMPUTE, 0, bytemuck::bytes_of(&pc)); d.cmd_dispatch(cb, g.cell_count, 1, 1);
            }
            bar(vk::PipelineStageFlags2::COMPUTE_SHADER, vk::AccessFlags2::SHADER_STORAGE_WRITE, vk::PipelineStageFlags2::DRAW_INDIRECT | vk::PipelineStageFlags2::VERTEX_SHADER, vk::AccessFlags2::INDIRECT_COMMAND_READ | vk::AccessFlags2::SHADER_STORAGE_READ);
        }
    }
    fn fade(&self, shrub: bool) -> Vec4 { let r = if shrub { self.opts.imp_dist * 0.6 } else { self.opts.imp_dist }; let b = r * self.opts.imp_band * 0.5; Vec4::new(r - b, r + b, r, 0.0) }
    fn push(&self, d: &ash::Device, cb: vk::CommandBuffer, vp: Mat4, tint: Vec4, surf: u32, layer: u32, mode: u32, n: u32, shrub: bool) {
        let pc = VPC { shadow_vp: vp, tint, surf, layer, mode: mode | ((shrub as u32) << 8), kind_n: n, fade: self.fade(shrub) }; unsafe { d.cmd_push_constants(cb, self.layout, vk::ShaderStageFlags::VERTEX | vk::ShaderStageFlags::FRAGMENT, 0, bytemuck::bytes_of(&pc)); }
    }
    /// Inside the G-buffer pass (same attachments as gbuffer.frag).
    pub fn draw_gbuffer(&self, d: &ash::Device, cb: vk::CommandBuffer, mats: &crate::mat::Mats) {
        use crate::mat::*;
        unsafe {
            d.cmd_bind_pipeline(cb, vk::PipelineBindPoint::GRAPHICS, self.p_mesh); d.cmd_bind_descriptor_sets(cb, vk::PipelineBindPoint::GRAPHICS, self.layout, 0, &[self.set], &[]);
            for k in self.kinds.iter().filter(|k| k.near_n > 0) {
                d.cmd_bind_vertex_buffers(cb, 1, &[k.near.buf], &[0]);
                for (part, tint, surf, g) in [(&k.br, k.bark_tint, 200u32, G_BARK), (&k.lf, k.leaf_tint, 201u32, G_LEAVES)] { if part.2 == 0 || !mats.visible(if k.shrub { G_SHRUBS } else { g }) { continue; }
                    self.push(d, cb, Mat4::IDENTITY, tint, surf, k.layer, 0, k.n, k.shrub); d.cmd_bind_vertex_buffers(cb, 0, &[part.0.buf], &[0]); d.cmd_bind_index_buffer(cb, part.1.buf, 0, vk::IndexType::UINT32); d.cmd_draw_indexed(cb, part.2, k.near_n, 0, 0, 0); }
            }
            d.cmd_bind_pipeline(cb, vk::PipelineBindPoint::GRAPHICS, self.p_imp);
            for k in self.kinds.iter().filter(|k| k.far_n > 0 && mats.visible(if k.shrub { G_SHRUBS } else { G_IMP })) { self.push(d, cb, Mat4::IDENTITY, Vec4::ONE, 202, k.layer, 0, k.n, k.shrub); d.cmd_bind_vertex_buffers(cb, 0, &[k.far.buf], &[0]); d.cmd_draw(cb, 6, k.far_n, 0, 0); }
            if let Some(g) = self.grass.as_ref().filter(|_| mats.visible(G_GRASS)) { d.cmd_bind_pipeline(cb, vk::PipelineBindPoint::GRAPHICS, self.p_grass); self.push(d, cb, Mat4::IDENTITY, Vec4::ONE, 203, 0, 0, 1, false); d.cmd_draw_indirect(cb, g.indirect.buf, 0, 1, 16); }
        }
    }
    /// Inside a shadow cascade pass. Near meshes alpha-tested; far trees as sun-facing impostors.
    pub fn draw_shadow(&self, d: &ash::Device, cb: vk::CommandBuffer, vp: Mat4, mats: &crate::mat::Mats) {
        use crate::mat::*;
        if !self.opts.tree_shadows { return; }
        unsafe {
            d.cmd_bind_pipeline(cb, vk::PipelineBindPoint::GRAPHICS, self.s_mesh); d.cmd_bind_descriptor_sets(cb, vk::PipelineBindPoint::GRAPHICS, self.layout, 0, &[self.set], &[]);
            for k in self.kinds.iter().filter(|k| k.near_n > 0) { d.cmd_bind_vertex_buffers(cb, 1, &[k.near.buf], &[0]);
                for (part, surf, g) in [(&k.br, 200u32, G_BARK), (&k.lf, 201u32, G_LEAVES)] { if part.2 == 0 || !mats.casts(if k.shrub { G_SHRUBS } else { g }) { continue; } self.push(d, cb, vp, Vec4::ONE, surf, k.layer, 1, k.n, k.shrub); d.cmd_bind_vertex_buffers(cb, 0, &[part.0.buf], &[0]); d.cmd_bind_index_buffer(cb, part.1.buf, 0, vk::IndexType::UINT32); d.cmd_draw_indexed(cb, part.2, k.near_n, 0, 0, 0); } }
            d.cmd_bind_pipeline(cb, vk::PipelineBindPoint::GRAPHICS, self.s_imp);
            for k in self.kinds.iter().filter(|k| k.far_n > 0 && mats.casts(if k.shrub { G_SHRUBS } else { G_IMP })) { self.push(d, cb, vp, Vec4::ONE, 202, k.layer, 1, k.n, k.shrub); d.cmd_bind_vertex_buffers(cb, 0, &[k.far.buf], &[0]); d.cmd_draw(cb, 6, k.far_n, 0, 0); }
        }
    }
    pub fn stats(&self) -> String { format!("trees {} mesh ({:.1}M tris) + {} impostors | grass cells {} (max {} blades)", self.near_trees, self.near_tris as f64 / 1e6, self.far_trees, self.cells_in_range, self.blades_cap) }
    #[allow(dead_code)] pub fn kind_names(&self) -> Vec<String> { self.kinds.iter().map(|k| k.name.clone()).collect() }
}
