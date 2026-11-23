//! Hardware ray tracing acceleration structures (VK_KHR_acceleration_structure): one BLAS per static world mesh, one TLAS.
//! Only built when the device exposes ray query (RTX 3090/4090). UNTESTED here: no RT-capable device on the build box.
//! The car is not in the TLAS yet (its contact shadow comes from the near raster cascade); adding it needs a per-frame
//! TLAS refit/rebuild with the car instance transform.
use crate::render::GpuMesh;
use crate::vk::{Buffer, Ctx};
use anyhow::Result;
use ash::vk;
use gpu_allocator::MemoryLocation;

pub struct Accel { pub tlas: vk::AccelerationStructureKHR, blas: Vec<(vk::AccelerationStructureKHR, Buffer)>, tlas_buf: Buffer }

fn build(c: &mut Ctx, geoms: &[vk::AccelerationStructureGeometryKHR], counts: &[u32], ty: vk::AccelerationStructureTypeKHR) -> Result<(vk::AccelerationStructureKHR, Buffer)> {
    let ext = c.accel.clone().unwrap();
    let mut info = vk::AccelerationStructureBuildGeometryInfoKHR::default().ty(ty).flags(vk::BuildAccelerationStructureFlagsKHR::PREFER_FAST_TRACE).mode(vk::BuildAccelerationStructureModeKHR::BUILD).geometries(geoms);
    let mut sizes = vk::AccelerationStructureBuildSizesInfoKHR::default();
    unsafe { ext.get_acceleration_structure_build_sizes(vk::AccelerationStructureBuildTypeKHR::DEVICE, &info, counts, &mut sizes) };
    let buf = c.buffer("as", sizes.acceleration_structure_size, vk::BufferUsageFlags::ACCELERATION_STRUCTURE_STORAGE_KHR, MemoryLocation::GpuOnly)?;
    let a = unsafe { ext.create_acceleration_structure(&vk::AccelerationStructureCreateInfoKHR::default().buffer(buf.buf).size(sizes.acceleration_structure_size).ty(ty), None)? };
    let scratch = c.buffer("scratch", sizes.build_scratch_size + 256, vk::BufferUsageFlags::STORAGE_BUFFER, MemoryLocation::GpuOnly)?;
    info = info.dst_acceleration_structure(a).scratch_data(vk::DeviceOrHostAddressKHR { device_address: (scratch.addr + 255) & !255 });
    let ranges: Vec<vk::AccelerationStructureBuildRangeInfoKHR> = counts.iter().map(|n| vk::AccelerationStructureBuildRangeInfoKHR::default().primitive_count(*n)).collect();
    c.one_shot(|cb| unsafe { ext.cmd_build_acceleration_structures(cb, &[info], &[&ranges]) })?;
    c.free_buffer(scratch);
    Ok((a, buf))
}

impl Accel {
    pub fn new(c: &mut Ctx, meshes: &[GpuMesh]) -> Result<Accel> {
        let ext = c.accel.clone().unwrap();
        let mut blas = vec![]; let mut inst = vec![];
        for m in meshes.iter().filter(|m| m.world && !m.dynamic) {
            let tri = vk::AccelerationStructureGeometryTrianglesDataKHR::default().vertex_format(vk::Format::R32G32B32_SFLOAT)
                .vertex_data(vk::DeviceOrHostAddressConstKHR { device_address: m.vb.addr }).vertex_stride(32).max_vertex(m.verts.saturating_sub(1))
                .index_type(vk::IndexType::UINT32).index_data(vk::DeviceOrHostAddressConstKHR { device_address: m.ib.addr });
            let g = [vk::AccelerationStructureGeometryKHR::default().geometry_type(vk::GeometryTypeKHR::TRIANGLES).flags(vk::GeometryFlagsKHR::OPAQUE).geometry(vk::AccelerationStructureGeometryDataKHR { triangles: tri })];
            let (a, b) = build(c, &g, &[m.count / 3], vk::AccelerationStructureTypeKHR::BOTTOM_LEVEL)?;
            let addr = unsafe { ext.get_acceleration_structure_device_address(&vk::AccelerationStructureDeviceAddressInfoKHR::default().acceleration_structure(a)) };
            let t = m.model.transpose().to_cols_array();
            inst.push(vk::AccelerationStructureInstanceKHR { transform: vk::TransformMatrixKHR { matrix: [t[0], t[1], t[2], t[3], t[4], t[5], t[6], t[7], t[8], t[9], t[10], t[11]] },
                instance_custom_index_and_mask: vk::Packed24_8::new(m.surf, 0xFF), instance_shader_binding_table_record_offset_and_flags: vk::Packed24_8::new(0, vk::GeometryInstanceFlagsKHR::TRIANGLE_FACING_CULL_DISABLE.as_raw() as u8),
                acceleration_structure_reference: vk::AccelerationStructureReferenceKHR { device_handle: addr } });
            blas.push((a, b));
        }
        let bytes: &[u8] = unsafe { std::slice::from_raw_parts(inst.as_ptr() as *const u8, inst.len() * std::mem::size_of::<vk::AccelerationStructureInstanceKHR>()) };
        let ib = c.upload("inst", bytes, vk::BufferUsageFlags::ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_KHR)?;
        let g = [vk::AccelerationStructureGeometryKHR::default().geometry_type(vk::GeometryTypeKHR::INSTANCES)
            .geometry(vk::AccelerationStructureGeometryDataKHR { instances: vk::AccelerationStructureGeometryInstancesDataKHR::default().data(vk::DeviceOrHostAddressConstKHR { device_address: ib.addr }) })];
        let (tlas, tlas_buf) = build(c, &g, &[inst.len() as u32], vk::AccelerationStructureTypeKHR::TOP_LEVEL)?;
        c.free_buffer(ib);
        Ok(Accel { tlas, blas, tlas_buf })
    }
}
