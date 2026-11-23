#version 460
#extension GL_GOOGLE_include_directive : enable
#include "common.glsl"
#include "veg_common.glsl"
// Shader grass: one instance per blade from the compute-generated blade buffer (grass_gen.comp), 5 segments, 9 triangles.
// Technique after Velorexe/unity-geometry-grass-shader (Unlicense): random facing/height/width/bend, segmented curvature,
// wind, base->tip gradient. Blades are widened to >= ~1 px at distance (sub-pixel blades are the main source of shimmer).
struct Blade { vec4 root; vec4 prm; }; // root xyz + hash, prm = (height, width, yaw, bend)
layout(std430, set = 0, binding = 3) readonly buffer Blades { Blade b[]; } B;
layout(location = 0) out vec3 vPos; layout(location = 1) out vec3 vNrm; layout(location = 2) out vec2 vUv; layout(location = 3) out vec4 vCur; layout(location = 4) out vec4 vPrev;
layout(location = 5) out vec3 vX5; layout(location = 6) out vec3 vUp; layout(location = 7) out vec3 vToCam; layout(location = 8) flat out vec2 vCell;
const int SEG = 5;
vec3 bladePos(Blade bl, float t, float side, float time) {
  vec3 root = bl.root.xyz; float h = bl.prm.x, w = bl.prm.y * (1.0 - 0.85 * t);
  float cy = cos(bl.prm.z), sy = sin(bl.prm.z); vec3 fwd = vec3(sy, 0, cy), right = vec3(cy, 0, -sy);
  float gust = sin(time * 1.3 + root.x * 0.21 + root.z * 0.17) * 0.6 + sin(time * 2.9 + root.x * 0.53 - root.z * 0.41) * 0.25 + 0.35;
  float bend = t * t * h * (bl.prm.w + gust * 0.35);
  return root + right * side * w + vec3(0, t * h * (1.0 - t * t * bl.prm.w * 0.35), 0) + fwd * bend + vec3(0.8, 0, 0.6) * t * t * h * gust * 0.25;
}
void main() {
  Blade bl = B.b[gl_InstanceIndex]; int v = gl_VertexIndex; int tri = v / 3, k = v % 3; float t, side;
  if (tri < 2 * (SEG - 1)) { int seg = tri / 2; bool second = (tri & 1) == 1; // quad (seg, seg+1): (0,1,3) (0,3,2) in (left,right) pairs
    int c = second ? int[3](0, 3, 2)[k] : int[3](0, 1, 3)[k]; t = float(seg + (c >> 1)) / float(SEG); side = (c & 1) == 0 ? -0.5 : 0.5; }
  else { t = k == 2 ? 1.0 : float(SEG - 1) / float(SEG); side = k == 0 ? -0.5 : (k == 1 ? 0.5 : 0.0); }
  // keep >= ~1.2 px wide: world width per pixel at this distance
  float dist = distance(bl.root.xyz, F.camPos.xyz); float pxW = 2.0 * dist / (F.proj[1][1] * F.res.y); bl.prm.y = max(bl.prm.y, 1.2 * pxW);
  float time = F.camPos.w; vec3 wp = bladePos(bl, t, side, time);
  float cy = cos(bl.prm.z), sy = sin(bl.prm.z); vec3 fwd = vec3(sy, 0, cy);
  vNrm = normalize(mix(-fwd, vec3(0, 1, 0), clamp(t * 0.6 + 0.35, 0.0, 1.0))); vPos = wp; vUv = vec2(t, bl.root.w); vX5 = vec3(0); vUp = vec3(0, 1, 0); vToCam = vec3(0); vCell = vec2(0);
  vec3 wpPrev = bladePos(bl, t, side, time - 1.0 / 60.0);
  vCur = F.viewProj * vec4(wp, 1); vPrev = F.prevViewProj * vec4(wpPrev, 1); gl_Position = vCur;
}
