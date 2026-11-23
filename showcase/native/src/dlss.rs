//! DLSS (NVIDIA NGX) integration point. Built only with `--features dlss` and DLSS_SDK set; the C++ shim
//! (native/dlss_shim.cpp, after SmartImposter's MIT shim) wraps NVSDK_NGX_VULKAN_Init / CreateFeature(SuperSampling or
//! RayReconstruction) / EvaluateFeature. Inputs it needs from render.rs: pre-tonemap HDR (hdr), depth, velocity (vel,
//! NDC delta, already written by the gbuffer), jitter offsets (NOT generated yet - DLSS needs a sub-pixel camera jitter
//! sequence, which also means the render must run at the DLSS input resolution). UNTESTED: no SDK and no RTX GPU here.
#[cfg(feature = "dlss")]
extern "C" { pub fn dcs_dlss_available() -> i32; }
pub fn available() -> bool {
    #[cfg(feature = "dlss")] { unsafe { dcs_dlss_available() != 0 } }
    #[cfg(not(feature = "dlss"))] { false }
}
