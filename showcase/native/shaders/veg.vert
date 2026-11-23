#version 460
#extension GL_GOOGLE_include_directive : enable
#include "common.glsl"
#include "veg_common.glsl"
// Near vegetation: ez-tree branch / leaf meshes, instanced (binding 1 = per-instance posYaw + prm).
layout(location = 0) in vec3 inPos; layout(location = 1) in vec3 inNrm; layout(location = 2) in vec2 inUv;
layout(location = 3) in vec4 iPosYaw; layout(location = 4) in vec4 iPrm;
layout(location = 0) out vec3 vPos; layout(location = 1) out vec3 vNrm; layout(location = 2) out vec2 vUv; layout(location = 3) out vec4 vCur; layout(location = 4) out vec4 vPrev;
layout(location = 5) out vec3 vCentre; layout(location = 6) out vec3 vUp; layout(location = 7) out vec3 vToCam; layout(location = 8) flat out vec4 vCell; layout(location = 9) flat out float vTreeDist;
void main() {
  float s = iPrm.x; vec3 root = iPosYaw.xyz; vec3 lp = inPos * s; vec3 wp = root + rotY(lp, iPosYaw.w);
  float t = F.camPos.w; wp += windOffset(wp, root, iPrm.y, t, pc.surf == 201u ? 1.0 : 0.0);
  vUp = vec3(0, 1, 0); vToCam = vec3(0); vCell = vec4(max(iPrm.w * 0.6, 0.4), 0, 0, 0); // x: approx. crown radius (foliage AO) vTreeDist = distance(root.xz, F.camPos.xz);
  vPos = wp; vNrm = rotY(inNrm, iPosYaw.w); vUv = inUv; vCentre = root + vec3(0, iPrm.w, 0);
  if (shadowPass()) { gl_Position = pc.shadowVP * vec4(wp, 1); vCur = vPrev = gl_Position; return; }
  vec3 wpPrev = root + rotY(lp, iPosYaw.w) + windOffset(wp, root, iPrm.y, t - F.opt.w, pc.surf == 201u ? 1.0 : 0.0);
  vCur = F.vpNJ * vec4(wp, 1); vPrev = F.prevVpNJ * vec4(wpPrev, 1); gl_Position = F.viewProj * vec4(wp, 1);
}
