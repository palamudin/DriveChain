// dlss_shim.cpp - minimal NGX availability probe for the `dlss` feature (own code; full SR/RR evaluate path is TODO and
// should follow NVIDIA's DLSS Programming Guide / SmartImposter's MIT shim). UNTESTED: requires the NVIDIA DLSS SDK.
#include <nvsdk_ngx_vk.h>
extern "C" int dcs_dlss_available() {
  // Real init needs VkInstance/VkPhysicalDevice/VkDevice: NVSDK_NGX_VULKAN_Init(appId, L".", inst, pdev, dev, ...);
  // then NVSDK_NGX_VULKAN_GetCapabilityParameters + NVSDK_CNGXParameter_SuperSampling_Available.
  return 0;
}
