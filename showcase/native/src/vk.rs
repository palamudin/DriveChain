//! Vulkan 1.3 context + small resource helpers (ash 0.38, gpu-allocator). Device selection prefers a discrete GPU with
//! VK_KHR_ray_query + VK_KHR_acceleration_structure (RTX 3090/4090); falls back to raster/compute-only (e.g. lavapipe CI).
use anyhow::{anyhow, Result};
use ash::vk;
use gpu_allocator::vulkan::{Allocation, AllocationCreateDesc, AllocationScheme, Allocator, AllocatorCreateDesc};
use gpu_allocator::MemoryLocation;
use std::ffi::CStr;

pub struct Ctx {
    pub entry: ash::Entry, pub instance: ash::Instance, pub pdev: vk::PhysicalDevice, pub device: ash::Device,
    pub queue: vk::Queue, pub qfam: u32, pub alloc: std::mem::ManuallyDrop<Allocator>, pub pool: vk::CommandPool,
    pub rt: bool, pub accel: Option<ash::khr::acceleration_structure::Device>, pub ts_period: f32, pub name: String, pub driver: String,
    pub surface_i: Option<ash::khr::surface::Instance>,
}
pub struct Image { pub image: vk::Image, pub view: vk::ImageView, pub alloc: Option<Allocation>, pub format: vk::Format, pub extent: vk::Extent2D, pub layers: u32 }
pub struct Buffer { pub buf: vk::Buffer, pub alloc: Option<Allocation>, pub size: u64, pub addr: u64 }

impl Ctx {
    pub fn new(window_exts: &[*const i8], want_rt: bool) -> Result<Ctx> {
        let entry = unsafe { ash::Entry::load()? };
        let app = vk::ApplicationInfo::default().application_name(c"DriveChain Showcase").api_version(vk::API_VERSION_1_3);
        let mut exts: Vec<*const i8> = window_exts.to_vec();
        let layers_wanted = [c"VK_LAYER_KHRONOS_validation"];
        let have_layers = unsafe { entry.enumerate_instance_layer_properties()? };
        let validation = std::env::var("DCS_VALIDATION").is_ok() && have_layers.iter().any(|l| unsafe { CStr::from_ptr(l.layer_name.as_ptr()) } == layers_wanted[0]);
        let layer_ptrs: Vec<*const i8> = if validation { vec![layers_wanted[0].as_ptr()] } else { vec![] };
        if validation { exts.push(ash::ext::debug_utils::NAME.as_ptr()); }
        let instance = unsafe { entry.create_instance(&vk::InstanceCreateInfo::default().application_info(&app).enabled_extension_names(&exts).enabled_layer_names(&layer_ptrs), None)? };
        let pdevs = unsafe { instance.enumerate_physical_devices()? };
        let score = |p: &vk::PhysicalDevice| { let pr = unsafe { instance.get_physical_device_properties(*p) }; match pr.device_type { vk::PhysicalDeviceType::DISCRETE_GPU => 3, vk::PhysicalDeviceType::INTEGRATED_GPU => 2, _ => 1 } };
        let pdev = *pdevs.iter().max_by_key(|p| score(p)).ok_or_else(|| anyhow!("no Vulkan device"))?;
        let props = unsafe { instance.get_physical_device_properties(pdev) };
        let name = unsafe { CStr::from_ptr(props.device_name.as_ptr()) }.to_string_lossy().into_owned();
        let dv = props.driver_version; let driver = if props.vendor_id == 0x10DE { format!("NVIDIA {}.{:02} (0x{dv:08x}), Vulkan {}.{}.{}", dv >> 22, (dv >> 14) & 0xff, vk::api_version_major(props.api_version), vk::api_version_minor(props.api_version), vk::api_version_patch(props.api_version)) }
            else { format!("vendor 0x{:04x} driver 0x{dv:08x}, Vulkan {}.{}.{}", props.vendor_id, vk::api_version_major(props.api_version), vk::api_version_minor(props.api_version), vk::api_version_patch(props.api_version)) };
        let dev_exts = unsafe { instance.enumerate_device_extension_properties(pdev)? };
        let has = |n: &CStr| dev_exts.iter().any(|e| unsafe { CStr::from_ptr(e.extension_name.as_ptr()) } == n);
        let rt = want_rt && has(ash::khr::ray_query::NAME) && has(ash::khr::acceleration_structure::NAME) && has(ash::khr::deferred_host_operations::NAME);
        let qfam = unsafe { instance.get_physical_device_queue_family_properties(pdev) }.iter().position(|q| q.queue_flags.contains(vk::QueueFlags::GRAPHICS | vk::QueueFlags::COMPUTE)).ok_or_else(|| anyhow!("no gfx queue"))? as u32;
        let mut dexts: Vec<*const i8> = vec![];
        if !window_exts.is_empty() { dexts.push(ash::khr::swapchain::NAME.as_ptr()); }
        if rt { dexts.extend([ash::khr::ray_query::NAME.as_ptr(), ash::khr::acceleration_structure::NAME.as_ptr(), ash::khr::deferred_host_operations::NAME.as_ptr()]); }
        let mut f13 = vk::PhysicalDeviceVulkan13Features::default().dynamic_rendering(true).synchronization2(true);
        let mut f12 = vk::PhysicalDeviceVulkan12Features::default().buffer_device_address(rt).descriptor_indexing(true).scalar_block_layout(true).host_query_reset(true);
        let mut rq = vk::PhysicalDeviceRayQueryFeaturesKHR::default().ray_query(true);
        let mut asf = vk::PhysicalDeviceAccelerationStructureFeaturesKHR::default().acceleration_structure(true);
        let feats = vk::PhysicalDeviceFeatures::default().sampler_anisotropy(true).shader_storage_image_write_without_format(true);
        let prio = [1.0f32];
        let qci = [vk::DeviceQueueCreateInfo::default().queue_family_index(qfam).queue_priorities(&prio)];
        let mut dci = vk::DeviceCreateInfo::default().queue_create_infos(&qci).enabled_extension_names(&dexts).enabled_features(&feats).push_next(&mut f13).push_next(&mut f12);
        if rt { dci = dci.push_next(&mut rq).push_next(&mut asf); }
        let device = unsafe { instance.create_device(pdev, &dci, None)? };
        let queue = unsafe { device.get_device_queue(qfam, 0) };
        let alloc = Allocator::new(&AllocatorCreateDesc { instance: instance.clone(), device: device.clone(), physical_device: pdev, debug_settings: Default::default(), buffer_device_address: rt, allocation_sizes: Default::default() })?;
        let pool = unsafe { device.create_command_pool(&vk::CommandPoolCreateInfo::default().queue_family_index(qfam).flags(vk::CommandPoolCreateFlags::RESET_COMMAND_BUFFER), None)? };
        let accel = rt.then(|| ash::khr::acceleration_structure::Device::new(&instance, &device));
        let surface_i = (!window_exts.is_empty()).then(|| ash::khr::surface::Instance::new(&entry, &instance));
        Ok(Ctx { entry, instance, pdev, device, queue, qfam, alloc: std::mem::ManuallyDrop::new(alloc), pool, rt, accel, ts_period: props.limits.timestamp_period, name, driver, surface_i })
    }
    pub fn image(&mut self, name: &str, w: u32, h: u32, format: vk::Format, usage: vk::ImageUsageFlags, layers: u32) -> Result<Image> {
        let ci = vk::ImageCreateInfo::default().image_type(vk::ImageType::TYPE_2D).format(format).extent(vk::Extent3D { width: w, height: h, depth: 1 })
            .mip_levels(1).array_layers(layers).samples(vk::SampleCountFlags::TYPE_1).tiling(vk::ImageTiling::OPTIMAL).usage(usage);
        let image = unsafe { self.device.create_image(&ci, None)? };
        let req = unsafe { self.device.get_image_memory_requirements(image) };
        let a = self.alloc.allocate(&AllocationCreateDesc { name, requirements: req, location: MemoryLocation::GpuOnly, linear: false, allocation_scheme: AllocationScheme::GpuAllocatorManaged })?;
        unsafe { self.device.bind_image_memory(image, a.memory(), a.offset())? };
        let aspect = if is_depth(format) { vk::ImageAspectFlags::DEPTH } else { vk::ImageAspectFlags::COLOR };
        let view = unsafe { self.device.create_image_view(&vk::ImageViewCreateInfo::default().image(image).format(format)
            .view_type(if layers > 1 { vk::ImageViewType::TYPE_2D_ARRAY } else { vk::ImageViewType::TYPE_2D })
            .subresource_range(vk::ImageSubresourceRange { aspect_mask: aspect, base_mip_level: 0, level_count: 1, base_array_layer: 0, layer_count: layers }), None)? };
        Ok(Image { image, view, alloc: Some(a), format, extent: vk::Extent2D { width: w, height: h }, layers })
    }
    pub fn buffer(&mut self, name: &str, size: u64, usage: vk::BufferUsageFlags, loc: MemoryLocation) -> Result<Buffer> {
        let usage = if self.rt { usage | vk::BufferUsageFlags::SHADER_DEVICE_ADDRESS } else { usage };
        let buf = unsafe { self.device.create_buffer(&vk::BufferCreateInfo::default().size(size.max(16)).usage(usage), None)? };
        let req = unsafe { self.device.get_buffer_memory_requirements(buf) };
        let a = self.alloc.allocate(&AllocationCreateDesc { name, requirements: req, location: loc, linear: true, allocation_scheme: AllocationScheme::GpuAllocatorManaged })?;
        unsafe { self.device.bind_buffer_memory(buf, a.memory(), a.offset())? };
        let addr = if self.rt { unsafe { self.device.get_buffer_device_address(&vk::BufferDeviceAddressInfo::default().buffer(buf)) } } else { 0 };
        Ok(Buffer { buf, alloc: Some(a), size, addr })
    }
    pub fn upload<T: bytemuck::Pod>(&mut self, name: &str, data: &[T], usage: vk::BufferUsageFlags) -> Result<Buffer> {
        let bytes: &[u8] = bytemuck::cast_slice(data);
        let mut b = self.buffer(name, bytes.len() as u64, usage, MemoryLocation::CpuToGpu)?;
        b.alloc.as_mut().unwrap().mapped_slice_mut().ok_or_else(|| anyhow!("unmapped"))?[..bytes.len()].copy_from_slice(bytes);
        Ok(b)
    }
    pub fn one_shot<F: FnOnce(vk::CommandBuffer)>(&self, f: F) -> Result<()> {
        unsafe {
            let cb = self.device.allocate_command_buffers(&vk::CommandBufferAllocateInfo::default().command_pool(self.pool).command_buffer_count(1))?[0];
            self.device.begin_command_buffer(cb, &vk::CommandBufferBeginInfo::default().flags(vk::CommandBufferUsageFlags::ONE_TIME_SUBMIT))?;
            f(cb);
            self.device.end_command_buffer(cb)?;
            let cbs = [cb]; self.device.queue_submit(self.queue, &[vk::SubmitInfo::default().command_buffers(&cbs)], vk::Fence::null())?;
            self.device.queue_wait_idle(self.queue)?; self.device.free_command_buffers(self.pool, &cbs);
        }
        Ok(())
    }
    pub fn free_image(&mut self, mut i: Image) { unsafe { self.device.destroy_image_view(i.view, None); self.device.destroy_image(i.image, None); } if let Some(a) = i.alloc.take() { let _ = self.alloc.free(a); } }
    pub fn free_buffer(&mut self, mut b: Buffer) { unsafe { self.device.destroy_buffer(b.buf, None); } if let Some(a) = b.alloc.take() { let _ = self.alloc.free(a); } }
}
pub fn is_depth(f: vk::Format) -> bool { matches!(f, vk::Format::D32_SFLOAT | vk::Format::D24_UNORM_S8_UINT | vk::Format::D16_UNORM) }

/// Image layout transition with synchronization2 (coarse ALL_COMMANDS scopes: correctness first; tighten after profiling on 4090).
pub fn barrier(d: &ash::Device, cb: vk::CommandBuffer, img: vk::Image, from: vk::ImageLayout, to: vk::ImageLayout, depth: bool, layers: u32) {
    let b = [vk::ImageMemoryBarrier2::default().image(img).old_layout(from).new_layout(to)
        .src_stage_mask(vk::PipelineStageFlags2::ALL_COMMANDS).dst_stage_mask(vk::PipelineStageFlags2::ALL_COMMANDS)
        .src_access_mask(vk::AccessFlags2::MEMORY_WRITE).dst_access_mask(vk::AccessFlags2::MEMORY_READ | vk::AccessFlags2::MEMORY_WRITE)
        .subresource_range(vk::ImageSubresourceRange { aspect_mask: if depth { vk::ImageAspectFlags::DEPTH } else { vk::ImageAspectFlags::COLOR }, base_mip_level: 0, level_count: 1, base_array_layer: 0, layer_count: layers })];
    unsafe { d.cmd_pipeline_barrier2(cb, &vk::DependencyInfo::default().image_memory_barriers(&b)) };
}
pub fn shader(d: &ash::Device, spv: &[u8]) -> Result<vk::ShaderModule> {
    let words = ash::util::read_spv(&mut std::io::Cursor::new(spv))?;
    Ok(unsafe { d.create_shader_module(&vk::ShaderModuleCreateInfo::default().code(&words), None)? })
}
