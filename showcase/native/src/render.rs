//! Frame graph (all passes recorded into one command buffer per frame; timestamps around every pass):
//!   0 shadow cascades   raster depth: near +-25 m every frame (world + car), far +-220 m CACHED (world only, re-rendered
//!                       only when the car leaves the inner 45 m or the sun moves)
//!   1 gbuffer           raster: albedo/metal, normal/roughness, velocity, depth
//!   2 shadow mask       compute: RT ray query (1 jittered cone ray/px) or PCSS-lite over the 2 cascades
//!   3 direct lighting   compute: GGX + Lambert * sun * mask
//!   4 screen-probe GI   compute (half res by default): SSRT3-style screen traces, + ray-query continuation on RTX
//!   5 composite         compute: joint-bilateral GI upsample, sky ambient * AO, atmosphere, height fog + volumetric shafts
//!   6 bloom             compute: 5-level 13-tap down / tent up chain
//!   7 post              compute: exposure + ACES -> sRGB  (tonemap BEFORE AA, per the handbook)
//!   8 finish            compute: spatial AA (FXAA-style, no history), CA, vignette, grain
//!   [DLSS hook: between 5 and 6 when built with --features dlss; see dlss.rs]
//! All storage/sampled images live in GENERAL layout; passes are separated by full memory barriers (correctness first,
//! to be tightened after profiling on the 3090/4090).
use crate::vk::{barrier, shader, Buffer, Ctx, Image};
use anyhow::Result;
use ash::vk;
use glam::{Mat4, Vec3, Vec4};
use gpu_allocator::MemoryLocation;

macro_rules! spv { ($n:literal) => { include_bytes!(concat!(env!("OUT_DIR"), "/", $n, ".spv")) } }

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct FrameU { pub view: Mat4, pub proj: Mat4, pub view_proj: Mat4, pub inv_view_proj: Mat4, pub prev_view_proj: Mat4, pub shadow_vp: [Mat4; 2],
    pub cam_pos: Vec4, pub sun_dir: Vec4, pub sun_color: Vec4, pub fog: Vec4, pub post: Vec4, pub res: Vec4, pub frame_info: [u32; 4], pub gi: Vec4 }

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Vertex { pub pos: [f32; 3], pub nrm: [f32; 3], pub uv: [f32; 2] }
#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct GbufPC { model: Mat4, tint: Vec4, surf: u32, pad: [u32; 3] }

pub struct GpuMesh { pub vb: Buffer, pub ib: Buffer, pub count: u32, pub verts: u32, pub surf: u32, pub tint: Vec4, pub model: Mat4, pub dynamic: bool, pub world: bool }

#[derive(Clone, Copy)]
pub struct Settings { pub gi: u32, pub exposure: f32, pub bloom: f32, pub grain: f32, pub vignette: f32, pub fog: f32, pub volumetric: f32, pub gi_intensity: f32, pub wet: f32, pub sun_el: f32, pub sun_az: f32, pub sun_intensity: f32 }
impl Default for Settings { fn default() -> Self { Settings { gi: 1, exposure: 0.85, bloom: 0.08, grain: 0.03, vignette: 0.3, fog: 0.0006, volumetric: 0.6, gi_intensity: 1.2, wet: 0.0, sun_el: 17.0, sun_az: 235.0, sun_intensity: 5.0 } } }

enum Res { Ubo(vk::Buffer, u64), Tex(vk::ImageView, vk::Sampler), Img(vk::ImageView), Tlas(vk::AccelerationStructureKHR) }
struct Pass { pipe: vk::Pipeline, layout: vk::PipelineLayout, dsl: vk::DescriptorSetLayout, set: vk::DescriptorSet }

pub const PASS_NAMES: [&str; 9] = ["shadow cascades", "gbuffer", "shadow mask", "direct light", "screen-probe GI", "composite+fog", "bloom", "tonemap", "AA+lens"];

pub struct Renderer {
    pub w: u32, pub h: u32, ubo: Buffer, pub depth: Image, albedo: Image, normal: Image, vel: Image, mask: Image, direct: Image, gi: Image, hdr: Image,
    bloom: Vec<Image>, ldr: Image, pub out: Image, sm0: Image, sm1: Image, tex_albedo: Image, tex_rough: Image, lin: vk::Sampler, clamp: vk::Sampler, shadow_s: vk::Sampler,
    gbuf_pipe: vk::Pipeline, gbuf_layout: vk::PipelineLayout, gbuf_dsl: vk::DescriptorSetLayout, gbuf_set: vk::DescriptorSet,
    sh_pipe: vk::Pipeline, sh_layout: vk::PipelineLayout,
    p_mask: Pass, p_light: Pass, p_gi: Pass, p_comp: Pass, p_post: Pass, p_fin: Pass, p_down: Vec<Pass>, p_up: Vec<Pass>,
    pool: vk::DescriptorPool, qpool: vk::QueryPool, pub cb: vk::CommandBuffer, pub fence: vk::Fence,
    pub meshes: Vec<GpuMesh>, frame: u32, prev_vp: Mat4, far_centre: Vec3, far_vp: Mat4, pub far_renders: u32, pub settings: Settings,
    pub tlas: Option<crate::accel::Accel>, pub timings: [f32; 9],
    pub veg: Option<crate::veg::Veg>, pub veg_ms: [f32; 2],
}

fn gi_dims(w: u32, h: u32, mode: u32) -> (u32, u32) { if mode == 2 { (w, h) } else { ((w + 1) / 2, (h + 1) / 2) } }

impl Renderer {
    pub fn new(c: &mut Ctx, w: u32, h: u32, textures: &[(Vec<u8>, Vec<u8>)], settings: Settings) -> Result<Renderer> {
        let st = vk::ImageUsageFlags::STORAGE | vk::ImageUsageFlags::SAMPLED;
        let att = vk::ImageUsageFlags::COLOR_ATTACHMENT | vk::ImageUsageFlags::SAMPLED;
        let dep = vk::ImageUsageFlags::DEPTH_STENCIL_ATTACHMENT | vk::ImageUsageFlags::SAMPLED;
        let depth = c.image("depth", w, h, vk::Format::D32_SFLOAT, dep, 1)?;
        let albedo = c.image("albedo", w, h, vk::Format::R8G8B8A8_UNORM, att, 1)?;
        let normal = c.image("normal", w, h, vk::Format::R16G16B16A16_SFLOAT, att, 1)?;
        let vel = c.image("vel", w, h, vk::Format::R16G16_SFLOAT, att, 1)?;
        let mask = c.image("mask", w, h, vk::Format::R8_UNORM, st, 1)?;
        let direct = c.image("direct", w, h, vk::Format::R16G16B16A16_SFLOAT, st, 1)?;
        let (gw, gh) = gi_dims(w, h, settings.gi);
        let gi = c.image("gi", gw, gh, vk::Format::R16G16B16A16_SFLOAT, st, 1)?;
        let hdr = c.image("hdr", w, h, vk::Format::R16G16B16A16_SFLOAT, st, 1)?;
        let mut bloom = vec![]; let (mut bw, mut bh) = (w / 2, h / 2);
        for i in 0..5 { bloom.push(c.image(&format!("bloom{i}"), bw.max(1), bh.max(1), vk::Format::R16G16B16A16_SFLOAT, st, 1)?); bw /= 2; bh /= 2; }
        let ldr = c.image("ldr", w, h, vk::Format::R8G8B8A8_UNORM, st, 1)?;
        let out = c.image("out", w, h, vk::Format::R8G8B8A8_UNORM, st | vk::ImageUsageFlags::TRANSFER_SRC, 1)?;
        let sm0 = c.image("sm0", 2048, 2048, vk::Format::D32_SFLOAT, dep, 1)?;
        let sm1 = c.image("sm1", 4096, 4096, vk::Format::D32_SFLOAT, dep, 1)?;
        let tw = 1024u32; let n = textures.len().max(1) as u32;
        // material arrays with full mip chains (no mips = the main source of asphalt/grass shimmer in motion without TAA)
        let mut la = vec![]; let mut lr = vec![];
        for (a, r) in textures { la.push(crate::veg::mips(a, tw, tw, false, 0.5)); lr.push(crate::veg::mips(r, tw, tw, false, 0.5)); }
        if la.is_empty() { la.push(crate::veg::mips(&vec![128; (tw * tw * 4) as usize], tw, tw, false, 0.5)); lr = la.clone(); }
        let tex_albedo = crate::veg::texture_array(c, "texA", tw, &la, &vec![true; la.len()])?;
        let tex_rough = crate::veg::texture_array(c, "texR", tw, &lr, &vec![false; lr.len()])?;
        let _ = n;
        let d = c.device.clone();
        let all_imgs: Vec<(vk::Image, bool, u32)> = [&depth, &albedo, &normal, &vel, &mask, &direct, &gi, &hdr, &ldr, &out, &sm0, &sm1].iter().map(|i| (i.image, crate::vk::is_depth(i.format), 1)).chain(bloom.iter().map(|i| (i.image, false, 1))).collect();
        c.one_shot(|cb| unsafe {
            for (img, dpt, l) in &all_imgs { barrier(&d, cb, *img, vk::ImageLayout::UNDEFINED, vk::ImageLayout::GENERAL, *dpt, *l); }
        })?;
        let mk_s = |addr: vk::SamplerAddressMode, aniso: bool| unsafe { d.create_sampler(&vk::SamplerCreateInfo::default().mag_filter(vk::Filter::LINEAR).min_filter(vk::Filter::LINEAR).mipmap_mode(vk::SamplerMipmapMode::LINEAR)
            .address_mode_u(addr).address_mode_v(addr).address_mode_w(addr).anisotropy_enable(aniso).max_anisotropy(8.0).max_lod(16.0), None) };
        let lin = mk_s(vk::SamplerAddressMode::REPEAT, true)?; let clamp = mk_s(vk::SamplerAddressMode::CLAMP_TO_EDGE, false)?;
        let shadow_s = unsafe { d.create_sampler(&vk::SamplerCreateInfo::default().mag_filter(vk::Filter::NEAREST).min_filter(vk::Filter::NEAREST).address_mode_u(vk::SamplerAddressMode::CLAMP_TO_EDGE).address_mode_v(vk::SamplerAddressMode::CLAMP_TO_EDGE), None)? };
        let ubo = c.buffer("frameU", std::mem::size_of::<FrameU>() as u64, vk::BufferUsageFlags::UNIFORM_BUFFER, MemoryLocation::CpuToGpu)?;
        let mut sizes = vec![vk::DescriptorPoolSize { ty: vk::DescriptorType::UNIFORM_BUFFER, descriptor_count: 64 }, vk::DescriptorPoolSize { ty: vk::DescriptorType::COMBINED_IMAGE_SAMPLER, descriptor_count: 128 },
            vk::DescriptorPoolSize { ty: vk::DescriptorType::STORAGE_IMAGE, descriptor_count: 64 }];
        if c.rt { sizes.push(vk::DescriptorPoolSize { ty: vk::DescriptorType::ACCELERATION_STRUCTURE_KHR, descriptor_count: 8 }); } // only valid with VK_KHR_acceleration_structure enabled
        let pool = unsafe { d.create_descriptor_pool(&vk::DescriptorPoolCreateInfo::default().max_sets(64).pool_sizes(&sizes), None)? };
        let qpool = unsafe { d.create_query_pool(&vk::QueryPoolCreateInfo::default().query_type(vk::QueryType::TIMESTAMP).query_count(32), None)? };
        let cb = unsafe { d.allocate_command_buffers(&vk::CommandBufferAllocateInfo::default().command_pool(c.pool).command_buffer_count(1))?[0] };
        let fence = unsafe { d.create_fence(&vk::FenceCreateInfo::default().flags(vk::FenceCreateFlags::SIGNALED), None)? };

        // ---- graphics pipelines
        let ubo_r = Res::Ubo(ubo.buf, ubo.size);
        let (gbuf_dsl, gbuf_set) = Self::make_set(&d, pool, &[ubo_r, Res::Tex(tex_albedo.view, lin), Res::Tex(tex_rough.view, lin)])?;
        let pcr = [vk::PushConstantRange { stage_flags: vk::ShaderStageFlags::VERTEX | vk::ShaderStageFlags::FRAGMENT, offset: 0, size: std::mem::size_of::<GbufPC>() as u32 }];
        let gbuf_layout = unsafe { d.create_pipeline_layout(&vk::PipelineLayoutCreateInfo::default().set_layouts(&[gbuf_dsl]).push_constant_ranges(&pcr), None)? };
        let gbuf_pipe = Self::graphics(&d, gbuf_layout, spv!("gbuffer.vert"), spv!("gbuffer.frag"), &[vk::Format::R8G8B8A8_UNORM, vk::Format::R16G16B16A16_SFLOAT, vk::Format::R16G16_SFLOAT], true, false)?;
        let shpc = [vk::PushConstantRange { stage_flags: vk::ShaderStageFlags::VERTEX, offset: 0, size: 64 }];
        let sh_layout = unsafe { d.create_pipeline_layout(&vk::PipelineLayoutCreateInfo::default().push_constant_ranges(&shpc), None)? };
        let sh_pipe = Self::graphics(&d, sh_layout, spv!("shadow.vert"), spv!("shadow.frag"), &[], false, true)?;

        let u = || Res::Ubo(ubo.buf, ubo.size);
        let rq = c.rt;
        let p_mask = Self::compute(&d, pool, if rq { spv!("shadow_rt_rq.comp") } else { spv!("shadow_rt.comp") }, vec![u(), Res::Tex(depth.view, clamp), Res::Tex(normal.view, clamp), Res::Img(mask.view), Res::Tex(sm0.view, shadow_s), Res::Tex(sm1.view, shadow_s)], 0, rq)?;
        let p_light = Self::compute(&d, pool, spv!("lighting.comp"), vec![u(), Res::Tex(depth.view, clamp), Res::Tex(albedo.view, clamp), Res::Tex(normal.view, clamp), Res::Tex(mask.view, clamp), Res::Img(direct.view)], 0, false)?;
        let p_gi = Self::compute(&d, pool, if rq { spv!("gi_probe_rq.comp") } else { spv!("gi_probe.comp") }, vec![u(), Res::Tex(depth.view, clamp), Res::Tex(normal.view, clamp), Res::Tex(direct.view, clamp), Res::Img(gi.view)], 0, rq)?;
        let p_comp = Self::compute(&d, pool, spv!("composite.comp"), vec![u(), Res::Tex(depth.view, clamp), Res::Tex(albedo.view, clamp), Res::Tex(normal.view, clamp), Res::Tex(direct.view, clamp), Res::Tex(gi.view, clamp), Res::Tex(sm1.view, shadow_s), Res::Img(hdr.view)], 0, false)?;
        let mut p_down = vec![]; let mut p_up = vec![];
        for i in 0..5 { let src = if i == 0 { hdr.view } else { bloom[i - 1].view }; p_down.push(Self::compute(&d, pool, spv!("bloom_down.comp"), vec![Res::Tex(src, clamp), Res::Img(bloom[i].view)], 16, false)?); }
        for i in (0..4).rev() { p_up.push(Self::compute(&d, pool, spv!("bloom_up.comp"), vec![Res::Tex(bloom[i + 1].view, clamp), Res::Img(bloom[i].view)], 16, false)?); }
        let p_post = Self::compute(&d, pool, spv!("post.comp"), vec![u(), Res::Tex(hdr.view, clamp), Res::Tex(bloom[0].view, clamp), Res::Img(ldr.view)], 0, false)?;
        let p_fin = Self::compute(&d, pool, spv!("finish.comp"), vec![u(), Res::Tex(ldr.view, clamp), Res::Img(out.view)], 0, false)?;
        Ok(Renderer { w, h, ubo, depth, albedo, normal, vel, mask, direct, gi, hdr, bloom, ldr, out, sm0, sm1, tex_albedo, tex_rough, lin, clamp, shadow_s,
            gbuf_pipe, gbuf_layout, gbuf_dsl, gbuf_set, sh_pipe, sh_layout, p_mask, p_light, p_gi, p_comp, p_post, p_fin, p_down, p_up, pool, qpool, cb, fence,
            meshes: vec![], frame: 0, prev_vp: Mat4::IDENTITY, far_centre: Vec3::splat(1e9), far_vp: Mat4::IDENTITY, far_renders: 0, settings, tlas: None, timings: [0.0; 9], veg: None, veg_ms: [0.0; 2] })
    }
    fn make_set(d: &ash::Device, pool: vk::DescriptorPool, res: &[Res]) -> Result<(vk::DescriptorSetLayout, vk::DescriptorSet)> {
        let ty = |r: &Res| match r { Res::Ubo(..) => vk::DescriptorType::UNIFORM_BUFFER, Res::Tex(..) => vk::DescriptorType::COMBINED_IMAGE_SAMPLER, Res::Img(_) => vk::DescriptorType::STORAGE_IMAGE, Res::Tlas(_) => vk::DescriptorType::ACCELERATION_STRUCTURE_KHR };
        let binds: Vec<_> = res.iter().enumerate().map(|(i, r)| vk::DescriptorSetLayoutBinding::default().binding(i as u32).descriptor_type(ty(r)).descriptor_count(1).stage_flags(vk::ShaderStageFlags::ALL)).collect();
        let dsl = unsafe { d.create_descriptor_set_layout(&vk::DescriptorSetLayoutCreateInfo::default().bindings(&binds), None)? };
        let set = unsafe { d.allocate_descriptor_sets(&vk::DescriptorSetAllocateInfo::default().descriptor_pool(pool).set_layouts(&[dsl]))?[0] };
        Self::write_set(d, set, res); Ok((dsl, set))
    }
    fn write_set(d: &ash::Device, set: vk::DescriptorSet, res: &[Res]) {
        for (i, r) in res.iter().enumerate() {
            let w = vk::WriteDescriptorSet::default().dst_set(set).dst_binding(i as u32).descriptor_count(1);
            unsafe { match r {
                Res::Ubo(b, s) => { let bi = [vk::DescriptorBufferInfo { buffer: *b, offset: 0, range: *s }]; d.update_descriptor_sets(&[w.descriptor_type(vk::DescriptorType::UNIFORM_BUFFER).buffer_info(&bi)], &[]) }
                Res::Tex(v, s) => { let ii = [vk::DescriptorImageInfo { sampler: *s, image_view: *v, image_layout: vk::ImageLayout::GENERAL }]; let ii2 = [vk::DescriptorImageInfo { sampler: *s, image_view: *v, image_layout: vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL }];
                    // material arrays are SHADER_READ_ONLY_OPTIMAL, everything else GENERAL
                    let _ = &ii2; d.update_descriptor_sets(&[w.descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER).image_info(&ii)], &[]) }
                Res::Img(v) => { let ii = [vk::DescriptorImageInfo { sampler: vk::Sampler::null(), image_view: *v, image_layout: vk::ImageLayout::GENERAL }]; d.update_descriptor_sets(&[w.descriptor_type(vk::DescriptorType::STORAGE_IMAGE).image_info(&ii)], &[]) }
                Res::Tlas(a) => { let acc = [*a]; let mut asw = vk::WriteDescriptorSetAccelerationStructureKHR::default().acceleration_structures(&acc);
                    let mut w = w.descriptor_type(vk::DescriptorType::ACCELERATION_STRUCTURE_KHR).push_next(&mut asw); w.descriptor_count = 1; d.update_descriptor_sets(&[w], &[]) }
            } }
        }
    }
    fn compute(d: &ash::Device, pool: vk::DescriptorPool, code: &[u8], mut res: Vec<Res>, pc: u32, tlas_slot: bool) -> Result<Pass> {
        if tlas_slot { res.push(Res::Tlas(vk::AccelerationStructureKHR::null())); }
        // TLAS written later (bind_tlas); write_set skips null handles below by construction of make_set order
        let n = res.len(); let real: Vec<Res> = res.into_iter().enumerate().filter(|(i, _)| !(tlas_slot && *i == n - 1)).map(|(_, r)| r).collect();
        let ty = |r: &Res| match r { Res::Ubo(..) => vk::DescriptorType::UNIFORM_BUFFER, Res::Tex(..) => vk::DescriptorType::COMBINED_IMAGE_SAMPLER, Res::Img(_) => vk::DescriptorType::STORAGE_IMAGE, Res::Tlas(_) => vk::DescriptorType::ACCELERATION_STRUCTURE_KHR };
        let mut binds: Vec<_> = real.iter().enumerate().map(|(i, r)| vk::DescriptorSetLayoutBinding::default().binding(i as u32).descriptor_type(ty(r)).descriptor_count(1).stage_flags(vk::ShaderStageFlags::ALL)).collect();
        if tlas_slot { binds.push(vk::DescriptorSetLayoutBinding::default().binding(real.len() as u32).descriptor_type(vk::DescriptorType::ACCELERATION_STRUCTURE_KHR).descriptor_count(1).stage_flags(vk::ShaderStageFlags::ALL)); }
        let dsl = unsafe { d.create_descriptor_set_layout(&vk::DescriptorSetLayoutCreateInfo::default().bindings(&binds), None)? };
        let set = unsafe { d.allocate_descriptor_sets(&vk::DescriptorSetAllocateInfo::default().descriptor_pool(pool).set_layouts(&[dsl]))?[0] };
        Self::write_set(d, set, &real);
        let pcr = [vk::PushConstantRange { stage_flags: vk::ShaderStageFlags::COMPUTE, offset: 0, size: pc.max(4) }];
        let layout = unsafe { d.create_pipeline_layout(&vk::PipelineLayoutCreateInfo::default().set_layouts(&[dsl]).push_constant_ranges(&pcr), None)? };
        let m = shader(d, code)?;
        let st = vk::PipelineShaderStageCreateInfo::default().stage(vk::ShaderStageFlags::COMPUTE).module(m).name(c"main");
        let pipe = unsafe { d.create_compute_pipelines(vk::PipelineCache::null(), &[vk::ComputePipelineCreateInfo::default().stage(st).layout(layout)], None).map_err(|e| e.1)?[0] };
        unsafe { d.destroy_shader_module(m, None) };
        Ok(Pass { pipe, layout, dsl, set })
    }
    fn graphics(d: &ash::Device, layout: vk::PipelineLayout, vs: &[u8], fs: &[u8], colors: &[vk::Format], full_vertex: bool, bias: bool) -> Result<vk::Pipeline> {
        let (vm, fm) = (shader(d, vs)?, shader(d, fs)?);
        let stages = [vk::PipelineShaderStageCreateInfo::default().stage(vk::ShaderStageFlags::VERTEX).module(vm).name(c"main"), vk::PipelineShaderStageCreateInfo::default().stage(vk::ShaderStageFlags::FRAGMENT).module(fm).name(c"main")];
        let vb = [vk::VertexInputBindingDescription { binding: 0, stride: 32, input_rate: vk::VertexInputRate::VERTEX }];
        let va_full = [vk::VertexInputAttributeDescription { location: 0, binding: 0, format: vk::Format::R32G32B32_SFLOAT, offset: 0 }, vk::VertexInputAttributeDescription { location: 1, binding: 0, format: vk::Format::R32G32B32_SFLOAT, offset: 12 }, vk::VertexInputAttributeDescription { location: 2, binding: 0, format: vk::Format::R32G32_SFLOAT, offset: 24 }];
        let vi = vk::PipelineVertexInputStateCreateInfo::default().vertex_binding_descriptions(&vb).vertex_attribute_descriptions(if full_vertex { &va_full[..] } else { &va_full[..1] });
        let ia = vk::PipelineInputAssemblyStateCreateInfo::default().topology(vk::PrimitiveTopology::TRIANGLE_LIST);
        let vp = vk::PipelineViewportStateCreateInfo::default().viewport_count(1).scissor_count(1);
        let rs = vk::PipelineRasterizationStateCreateInfo::default().polygon_mode(vk::PolygonMode::FILL).cull_mode(vk::CullModeFlags::NONE).line_width(1.0)
            .depth_bias_enable(bias).depth_bias_constant_factor(1.5).depth_bias_slope_factor(2.0);
        let ms = vk::PipelineMultisampleStateCreateInfo::default().rasterization_samples(vk::SampleCountFlags::TYPE_1);
        let ds = vk::PipelineDepthStencilStateCreateInfo::default().depth_test_enable(true).depth_write_enable(true).depth_compare_op(vk::CompareOp::LESS);
        let cba: Vec<_> = colors.iter().map(|_| vk::PipelineColorBlendAttachmentState::default().color_write_mask(vk::ColorComponentFlags::RGBA)).collect();
        let cb = vk::PipelineColorBlendStateCreateInfo::default().attachments(&cba);
        let dyns = [vk::DynamicState::VIEWPORT, vk::DynamicState::SCISSOR]; let dy = vk::PipelineDynamicStateCreateInfo::default().dynamic_states(&dyns);
        let mut ri = vk::PipelineRenderingCreateInfo::default().color_attachment_formats(colors).depth_attachment_format(vk::Format::D32_SFLOAT);
        let ci = vk::GraphicsPipelineCreateInfo::default().stages(&stages).vertex_input_state(&vi).input_assembly_state(&ia).viewport_state(&vp).rasterization_state(&rs)
            .multisample_state(&ms).depth_stencil_state(&ds).color_blend_state(&cb).dynamic_state(&dy).layout(layout).push_next(&mut ri);
        let p = unsafe { d.create_graphics_pipelines(vk::PipelineCache::null(), &[ci], None).map_err(|e| e.1)?[0] };
        unsafe { d.destroy_shader_module(vm, None); d.destroy_shader_module(fm, None); }
        Ok(p)
    }
    pub fn bind_tlas(&self, d: &ash::Device, tlas: vk::AccelerationStructureKHR) {
        Self::write_set_at(d, self.p_mask.set, 6, tlas); Self::write_set_at(d, self.p_gi.set, 5, tlas);
    }
    fn write_set_at(d: &ash::Device, set: vk::DescriptorSet, binding: u32, a: vk::AccelerationStructureKHR) {
        let acc = [a]; let mut asw = vk::WriteDescriptorSetAccelerationStructureKHR::default().acceleration_structures(&acc);
        let mut w = vk::WriteDescriptorSet::default().dst_set(set).dst_binding(binding).descriptor_type(vk::DescriptorType::ACCELERATION_STRUCTURE_KHR).push_next(&mut asw); w.descriptor_count = 1;
        unsafe { d.update_descriptor_sets(&[w], &[]) };
    }
    pub fn add_mesh(&mut self, c: &mut Ctx, v: &[Vertex], i: &[u32], surf: u32, tint: Vec4, dynamic: bool, world: bool) -> Result<usize> {
        let extra = if c.rt { vk::BufferUsageFlags::ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_KHR } else { vk::BufferUsageFlags::empty() };
        let vb = c.upload("vb", v, vk::BufferUsageFlags::VERTEX_BUFFER | extra)?; let ib = c.upload("ib", i, vk::BufferUsageFlags::INDEX_BUFFER | extra)?;
        self.meshes.push(GpuMesh { vb, ib, count: i.len() as u32, verts: v.len() as u32, surf, tint, model: Mat4::IDENTITY, dynamic, world }); Ok(self.meshes.len() - 1)
    }
    fn ortho_sun(sun: Vec3, centre: Vec3, e: f32) -> Mat4 {
        let up = if sun.y.abs() > 0.95 { Vec3::X } else { Vec3::Y };
        Mat4::orthographic_rh(-e, e, -e, e, 1.0, 1200.0) * Mat4::look_at_rh(centre + sun * 500.0, centre, up)
    }
    /// Record + submit one frame. Returns after GPU completion (simple, single frame in flight).
    pub fn render(&mut self, c: &Ctx, view: Mat4, cam_pos: Vec3, fov_y: f32, car_centre: Vec3, time: f32) -> Result<()> {
        let d = &c.device; let s = self.settings;
        let mut proj = Mat4::perspective_rh(fov_y, self.w as f32 / self.h as f32, 0.1, 4000.0); proj.y_axis.y *= -1.0;
        let vp = proj * view;
        let (el, az) = (s.sun_el.to_radians(), s.sun_az.to_radians());
        let sun = Vec3::new(el.cos() * az.sin(), el.sin(), el.cos() * az.cos()).normalize();
        let snap = |v: Vec3, t: f32| Vec3::new((v.x / t).round() * t, v.y, (v.z / t).round() * t);
        let near_vp = Self::ortho_sun(sun, snap(car_centre, 50.0 / 2048.0), 25.0);
        let far_dirty = car_centre.distance(self.far_centre) > 45.0;
        if far_dirty { self.far_centre = car_centre; self.far_vp = Self::ortho_sun(sun, snap(car_centre, 440.0 / 4096.0), 220.0); self.far_renders += 1; }
        let fu = FrameU { view, proj, view_proj: vp, inv_view_proj: vp.inverse(), prev_view_proj: if self.frame == 0 { vp } else { self.prev_vp }, shadow_vp: [near_vp, self.far_vp],
            cam_pos: cam_pos.extend(time), sun_dir: sun.extend(0.0047 * 2.0), sun_color: (Vec3::new(1.0, 0.94, 0.86) * s.sun_intensity).extend(0.0),
            fog: Vec4::new(s.fog, 0.03, s.volumetric, 0.6), post: Vec4::new(s.exposure, s.bloom, s.grain, s.vignette),
            res: Vec4::new(self.w as f32, self.h as f32, 1.0 / self.w as f32, 1.0 / self.h as f32), frame_info: [self.frame, c.rt as u32, s.gi, 0], gi: Vec4::new(s.gi_intensity, 1.0, 6.0, s.wet) };
        self.prev_vp = vp;
        unsafe {
            d.wait_for_fences(&[self.fence], true, u64::MAX)?; d.reset_fences(&[self.fence])?;
            if self.frame > 0 { self.read_timings(c); }
            self.ubo.alloc.as_mut().unwrap().mapped_slice_mut().unwrap()[..std::mem::size_of::<FrameU>()].copy_from_slice(bytemuck::bytes_of(&fu));
            let cb = self.cb; d.begin_command_buffer(cb, &vk::CommandBufferBeginInfo::default().flags(vk::CommandBufferUsageFlags::ONE_TIME_SUBMIT))?;
            d.cmd_reset_query_pool(cb, self.qpool, 0, 32);
            let full = || { let mb = [vk::MemoryBarrier2::default().src_stage_mask(vk::PipelineStageFlags2::ALL_COMMANDS).dst_stage_mask(vk::PipelineStageFlags2::ALL_COMMANDS)
                .src_access_mask(vk::AccessFlags2::MEMORY_WRITE).dst_access_mask(vk::AccessFlags2::MEMORY_READ | vk::AccessFlags2::MEMORY_WRITE)]; d.cmd_pipeline_barrier2(cb, &vk::DependencyInfo::default().memory_barriers(&mb)); };
            let ts = |i: u32| d.cmd_write_timestamp2(cb, vk::PipelineStageFlags2::ALL_COMMANDS, self.qpool, i);
            ts(0);
            if let Some(v) = &self.veg { d.cmd_write_timestamp2(cb, vk::PipelineStageFlags2::ALL_COMMANDS, self.qpool, 20); v.record_compute(d, cb); d.cmd_write_timestamp2(cb, vk::PipelineStageFlags2::ALL_COMMANDS, self.qpool, 21); }
            // 0: shadow cascades
            for (img, vpm, size, only_static) in [(&self.sm0, near_vp, 2048u32, false), (&self.sm1, self.far_vp, 4096u32, true)] {
                if only_static && !far_dirty { continue; }
                let da = vk::RenderingAttachmentInfo::default().image_view(img.view).image_layout(vk::ImageLayout::GENERAL).load_op(vk::AttachmentLoadOp::CLEAR).store_op(vk::AttachmentStoreOp::STORE)
                    .clear_value(vk::ClearValue { depth_stencil: vk::ClearDepthStencilValue { depth: 1.0, stencil: 0 } });
                d.cmd_begin_rendering(cb, &vk::RenderingInfo::default().render_area(vk::Rect2D { offset: vk::Offset2D::default(), extent: vk::Extent2D { width: size, height: size } }).layer_count(1).depth_attachment(&da));
                d.cmd_set_viewport(cb, 0, &[vk::Viewport { x: 0.0, y: 0.0, width: size as f32, height: size as f32, min_depth: 0.0, max_depth: 1.0 }]);
                d.cmd_set_scissor(cb, 0, &[vk::Rect2D { offset: vk::Offset2D::default(), extent: vk::Extent2D { width: size, height: size } }]);
                d.cmd_bind_pipeline(cb, vk::PipelineBindPoint::GRAPHICS, self.sh_pipe);
                for m in self.meshes.iter().filter(|m| !(only_static && m.dynamic)) {
                    let mvp = vpm * m.model; d.cmd_push_constants(cb, self.sh_layout, vk::ShaderStageFlags::VERTEX, 0, bytemuck::bytes_of(&mvp));
                    d.cmd_bind_vertex_buffers(cb, 0, &[m.vb.buf], &[0]); d.cmd_bind_index_buffer(cb, m.ib.buf, 0, vk::IndexType::UINT32); d.cmd_draw_indexed(cb, m.count, 1, 0, 0, 0);
                }
                if let Some(v) = &self.veg { v.draw_shadow(d, cb, vpm); }
                d.cmd_end_rendering(cb);
            }
            full(); ts(1);
            // 1: gbuffer
            let ext = vk::Extent2D { width: self.w, height: self.h };
            let ca = |v: vk::ImageView| vk::RenderingAttachmentInfo::default().image_view(v).image_layout(vk::ImageLayout::GENERAL).load_op(vk::AttachmentLoadOp::CLEAR).store_op(vk::AttachmentStoreOp::STORE).clear_value(vk::ClearValue { color: vk::ClearColorValue { float32: [0.0; 4] } });
            let cas = [ca(self.albedo.view), ca(self.normal.view), ca(self.vel.view)];
            let da = vk::RenderingAttachmentInfo::default().image_view(self.depth.view).image_layout(vk::ImageLayout::GENERAL).load_op(vk::AttachmentLoadOp::CLEAR).store_op(vk::AttachmentStoreOp::STORE).clear_value(vk::ClearValue { depth_stencil: vk::ClearDepthStencilValue { depth: 1.0, stencil: 0 } });
            d.cmd_begin_rendering(cb, &vk::RenderingInfo::default().render_area(vk::Rect2D { offset: vk::Offset2D::default(), extent: ext }).layer_count(1).color_attachments(&cas).depth_attachment(&da));
            d.cmd_set_viewport(cb, 0, &[vk::Viewport { x: 0.0, y: 0.0, width: self.w as f32, height: self.h as f32, min_depth: 0.0, max_depth: 1.0 }]);
            d.cmd_set_scissor(cb, 0, &[vk::Rect2D { offset: vk::Offset2D::default(), extent: ext }]);
            d.cmd_bind_pipeline(cb, vk::PipelineBindPoint::GRAPHICS, self.gbuf_pipe);
            d.cmd_bind_descriptor_sets(cb, vk::PipelineBindPoint::GRAPHICS, self.gbuf_layout, 0, &[self.gbuf_set], &[]);
            for m in &self.meshes {
                let pc = GbufPC { model: m.model, tint: m.tint, surf: m.surf, pad: [0; 3] };
                d.cmd_push_constants(cb, self.gbuf_layout, vk::ShaderStageFlags::VERTEX | vk::ShaderStageFlags::FRAGMENT, 0, bytemuck::bytes_of(&pc));
                d.cmd_bind_vertex_buffers(cb, 0, &[m.vb.buf], &[0]); d.cmd_bind_index_buffer(cb, m.ib.buf, 0, vk::IndexType::UINT32); d.cmd_draw_indexed(cb, m.count, 1, 0, 0, 0);
            }
            if let Some(v) = &self.veg { d.cmd_write_timestamp2(cb, vk::PipelineStageFlags2::ALL_COMMANDS, self.qpool, 22); v.draw_gbuffer(d, cb); d.cmd_write_timestamp2(cb, vk::PipelineStageFlags2::ALL_COMMANDS, self.qpool, 23); }
            d.cmd_end_rendering(cb); full(); ts(2);
            let disp = |p: &Pass, w: u32, h: u32| { d.cmd_bind_pipeline(cb, vk::PipelineBindPoint::COMPUTE, p.pipe); d.cmd_bind_descriptor_sets(cb, vk::PipelineBindPoint::COMPUTE, p.layout, 0, &[p.set], &[]); d.cmd_dispatch(cb, (w + 7) / 8, (h + 7) / 8, 1); };
            disp(&self.p_mask, self.w, self.h); full(); ts(3);
            disp(&self.p_light, self.w, self.h); full(); ts(4);
            if s.gi != 0 { let (gw, gh) = gi_dims(self.w, self.h, s.gi); disp(&self.p_gi, gw, gh); full(); } ts(5);
            disp(&self.p_comp, self.w, self.h); full(); ts(6);
            // DLSS hook (feature "dlss"): crate::dlss::evaluate(cb, hdr, depth, vel) would go here, then post at output res.
            for (i, p) in self.p_down.iter().enumerate() {
                let src_ext = if i == 0 { (self.w, self.h) } else { (self.bloom[i - 1].extent.width, self.bloom[i - 1].extent.height) };
                let pc: [f32; 4] = [1.0 / src_ext.0 as f32, 1.0 / src_ext.1 as f32, 1.0, if i == 0 { 1.0 } else { 0.0 }];
                d.cmd_bind_pipeline(cb, vk::PipelineBindPoint::COMPUTE, p.pipe); d.cmd_bind_descriptor_sets(cb, vk::PipelineBindPoint::COMPUTE, p.layout, 0, &[p.set], &[]);
                d.cmd_push_constants(cb, p.layout, vk::ShaderStageFlags::COMPUTE, 0, bytemuck::cast_slice(&pc));
                d.cmd_dispatch(cb, (self.bloom[i].extent.width + 7) / 8, (self.bloom[i].extent.height + 7) / 8, 1); full();
            }
            for (k, p) in self.p_up.iter().enumerate() { let i = 3 - k; let se = self.bloom[i + 1].extent; let pc: [f32; 4] = [1.0 / se.width as f32, 1.0 / se.height as f32, 1.0, 0.0];
                d.cmd_bind_pipeline(cb, vk::PipelineBindPoint::COMPUTE, p.pipe); d.cmd_bind_descriptor_sets(cb, vk::PipelineBindPoint::COMPUTE, p.layout, 0, &[p.set], &[]);
                d.cmd_push_constants(cb, p.layout, vk::ShaderStageFlags::COMPUTE, 0, bytemuck::cast_slice(&pc));
                d.cmd_dispatch(cb, (self.bloom[i].extent.width + 7) / 8, (self.bloom[i].extent.height + 7) / 8, 1); full(); }
            ts(7);
            disp(&self.p_post, self.w, self.h); full(); ts(8);
            disp(&self.p_fin, self.w, self.h); full(); ts(9);
            d.end_command_buffer(cb)?;
            let cbs = [cb]; d.queue_submit(c.queue, &[vk::SubmitInfo::default().command_buffers(&cbs)], self.fence)?;
        }
        self.frame += 1; Ok(())
    }
    pub fn wait(&self, c: &Ctx) -> Result<()> { unsafe { c.device.wait_for_fences(&[self.fence], true, u64::MAX)?; } Ok(()) }
    pub fn read_timings(&mut self, c: &Ctx) {
        let mut q = [0u64; 10];
        if unsafe { c.device.get_query_pool_results(self.qpool, 0, &mut q, vk::QueryResultFlags::TYPE_64) }.is_ok() {
            for i in 0..9 { self.timings[i] = (q[i + 1].saturating_sub(q[i])) as f32 * c.ts_period * 1e-6; }
        }
        let mut v = [0u64; 4];
        if self.veg.is_some() && unsafe { c.device.get_query_pool_results(self.qpool, 20, &mut v, vk::QueryResultFlags::TYPE_64) }.is_ok() {
            self.veg_ms = [(v[1].saturating_sub(v[0])) as f32 * c.ts_period * 1e-6, (v[3].saturating_sub(v[2])) as f32 * c.ts_period * 1e-6]; }
    }
    /// Copy the final image to host RGBA8 (for stills / CI). Call after render().
    pub fn readback(&self, c: &mut Ctx) -> Result<Vec<u8>> {
        self.wait(c)?;
        let size = (self.w * self.h * 4) as u64; let buf = c.buffer("readback", size, vk::BufferUsageFlags::TRANSFER_DST, MemoryLocation::GpuToCpu)?;
        let (img, w, h, d) = (self.out.image, self.w, self.h, c.device.clone());
        c.one_shot(|cb| unsafe { d.cmd_copy_image_to_buffer(cb, img, vk::ImageLayout::GENERAL, buf.buf, &[vk::BufferImageCopy::default()
            .image_subresource(vk::ImageSubresourceLayers { aspect_mask: vk::ImageAspectFlags::COLOR, mip_level: 0, base_array_layer: 0, layer_count: 1 }).image_extent(vk::Extent3D { width: w, height: h, depth: 1 })]) })?;
        let px = buf.alloc.as_ref().unwrap().mapped_slice().unwrap()[..size as usize].to_vec(); c.free_buffer(buf); Ok(px)
    }
    pub fn out_image(&self) -> vk::Image { self.out.image }
    pub fn ubo_info(&self) -> (vk::Buffer, u64) { (self.ubo.buf, self.ubo.size) }
}
