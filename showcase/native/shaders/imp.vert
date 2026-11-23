#version 460
#extension GL_GOOGLE_include_directive : enable
#include "common.glsl"
#include "veg_common.glsl"
// Far vegetation: octahedral impostor quads (SmartImposter-style, 8x8 hemi-octahedral atlas baked from the same ez-tree mesh).
// Camera pass: quad faces the camera, cell picked from the tree-local view direction. Shadow pass (mode 1): quad faces the sun.
layout(location = 3) in vec4 iPosYaw; layout(location = 4) in vec4 iPrm;
layout(location = 0) out vec3 vPos; layout(location = 1) out vec3 vNrm; layout(location = 2) out vec2 vUv; layout(location = 3) out vec4 vCur; layout(location = 4) out vec4 vPrev;
layout(location = 5) out vec3 vRight; layout(location = 6) out vec3 vUp; layout(location = 7) out vec3 vToCam; layout(location = 8) flat out vec2 vCell;
const vec2 C[6] = vec2[](vec2(-1,-1), vec2(1,-1), vec2(1,1), vec2(-1,-1), vec2(1,1), vec2(-1,1));
void main() {
  vec2 cr = C[gl_VertexIndex % 6]; vec3 ctr = iPosYaw.xyz + vec3(0, iPrm.w, 0); float R = iPrm.z;
  vec3 toCam = pc.mode == 1u ? normalize(F.sunDir.xyz) : normalize(F.camPos.xyz - ctr);
  vec3 right = normalize(cross(vec3(0, 1, 0), toCam)); vec3 up = cross(toCam, right);
  vec3 wp = ctr + (right * cr.x + up * cr.y) * R;
  // tree-local view direction (undo yaw), hemi-octahedral encode -> nearest cell
  float c = cos(iPosYaw.w), s = sin(iPosYaw.w); vec3 lv = normalize(vec3(c * toCam.x - s * toCam.z, max(toCam.y, 0.0), s * toCam.x + c * toCam.z));
  float d = abs(lv.x) + abs(lv.y) + abs(lv.z); vec2 o = lv.xz / d; vec2 e = vec2(o.x + o.y, o.x - o.y) * 0.5 + 0.5;
  vCell = clamp(floor(e * float(pc.kindN)), 0.0, float(pc.kindN - 1u)); vUv = cr * 0.5 + 0.5;
  vRight = right; vUp = up; vToCam = toCam; vPos = wp; vNrm = toCam;
  if (pc.mode == 1u) { gl_Position = pc.shadowVP * vec4(wp, 1); vCur = vPrev = gl_Position; return; }
  vCur = F.viewProj * vec4(wp, 1); vPrev = F.prevViewProj * vec4(wp, 1); gl_Position = vCur;
}
