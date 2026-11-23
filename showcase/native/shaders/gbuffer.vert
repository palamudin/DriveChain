#version 460
#extension GL_GOOGLE_include_directive : enable
#include "common.glsl"
layout(location = 0) in vec3 inPos; layout(location = 1) in vec3 inNrm; layout(location = 2) in vec2 inUv;
layout(push_constant) uniform PC { mat4 model; vec4 tint; uint surf; uint pad0, pad1, pad2; } pc;
layout(location = 0) out vec3 vPos; layout(location = 1) out vec3 vNrm; layout(location = 2) out vec2 vUv; layout(location = 3) out vec4 vCur; layout(location = 4) out vec4 vPrev;
void main() {
  vec4 wp = pc.model * vec4(inPos, 1.0); vPos = wp.xyz; vNrm = mat3(pc.model) * inNrm; vUv = inUv;
  vCur = F.viewProj * wp; vPrev = F.prevViewProj * wp; gl_Position = vCur;
}
