#version 460
#extension GL_GOOGLE_include_directive : enable
#include "common.glsl"
#include "veg_common.glsl"
// Alpha-tested depth for leaves (201) and impostors (202); bark (200) is opaque. Mesh/impostor casters switch hard at pc.fade.z
// (no dither in shadow maps: dithered casters would print noise into every receiver).
layout(set = 0, binding = 1) uniform sampler2DArray vegTex;
layout(set = 0, binding = 2) uniform sampler2DArray impTex;
layout(location = 2) in vec2 vUv; layout(location = 8) flat in vec4 vCell; layout(location = 9) flat in float vTreeDist;
void main() {
  if (pc.surf == 202u ? vTreeDist < pc.fade.z : vTreeDist >= pc.fade.z) discard;
  bool opaque = matOpaque(vegGroup());
  if (opaque) return;
  if (pc.surf == 201u) { if (textureLod(vegTex, vec3(vUv, pc.layer + 1u), 1.0).a < 0.5) discard; }
  else if (pc.surf == 202u) { vec2 c = vCell.xy + step(0.5, vCell.zw); vec2 uv = (c + vUv) / float(pc.kindN); uv.y = 1.0 - uv.y; if (textureLod(impTex, vec3(uv, pc.layer), 1.0).a < 0.5) discard; }
}
