#version 460
#extension GL_GOOGLE_include_directive : enable
#include "common.glsl"
#include "veg_common.glsl"
// Alpha-tested depth for leaves (201) and impostors (202); bark (200) is opaque.
layout(set = 0, binding = 1) uniform sampler2DArray vegTex;
layout(set = 0, binding = 2) uniform sampler2DArray impTex;
layout(location = 2) in vec2 vUv; layout(location = 8) flat in vec2 vCell;
void main() {
  if (pc.surf == 201u) { if (textureLod(vegTex, vec3(vUv, pc.layer + 1u), 1.0).a < 0.5) discard; }
  else if (pc.surf == 202u) { vec2 uv = (vCell + vUv) / float(pc.kindN); uv.y = 1.0 - uv.y; if (textureLod(impTex, vec3(uv, pc.layer), 1.0).a < 0.5) discard; }
}
