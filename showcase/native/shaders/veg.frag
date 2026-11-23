#version 460
#extension GL_GOOGLE_include_directive : enable
#include "common.glsl"
#include "mat_frag.glsl"
#include "veg_common.glsl"
// Vegetation G-buffer: 200 bark, 201 leaves (alpha-tested, coverage-preserving mips), 202 impostor, 203 grass. Same outputs as gbuffer.frag.
layout(set = 0, binding = 1) uniform sampler2DArray vegTex;  // per kind: bark, leaf
layout(set = 0, binding = 2) uniform sampler2DArray impTex;  // per kind: albedo+alpha atlas, normal atlas
layout(location = 0) in vec3 vPos; layout(location = 1) in vec3 vNrm; layout(location = 2) in vec2 vUv; layout(location = 3) in vec4 vCur; layout(location = 4) in vec4 vPrev;
layout(location = 5) in vec3 vX5; layout(location = 6) in vec3 vUp; layout(location = 7) in vec3 vToCam; layout(location = 8) flat in vec4 vCell; layout(location = 9) flat in float vTreeDist;
layout(location = 0) out vec4 oAlbedo; layout(location = 1) out vec4 oNormal; layout(location = 2) out vec2 oVel;
bool gNoMips = false;
vec4 TX(sampler2DArray t, vec3 c) { return gNoMips ? textureLod(t, c, 0.0) : texture(t, c); }
vec4 impCell(vec2 cell, uint layer) { vec2 uv = (cell + vUv) / float(pc.kindN); uv.y = 1.0 - uv.y; return TX(impTex, vec3(uv, layer)); } // atlas rows top-down
vec4 impCellPM(vec2 cell, uint layer, bool pm) { vec4 c = impCell(cell, layer); return pm ? vec4(c.rgb * c.a, c.a) : c; }
vec4 impBlend(uint layer, bool pm) { vec2 b = vCell.xy, w = vCell.zw; // premultiplied blend of the 4 neighbouring views
  return mix(mix(impCellPM(b, layer, pm), impCellPM(b + vec2(1, 0), layer, pm), w.x), mix(impCellPM(b + vec2(0, 1), layer, pm), impCellPM(b + vec2(1, 1), layer, pm), w.x), w.y); }
void main() {
  uint grp = vegGroup(); gNoMips = matNoMips(grp); bool opaque = matOpaque(grp);
  // mesh <-> impostor crossfade band (dithered; with TAA the dither is temporal and resolves to a smooth blend)
  if (pc.surf != 203u && pc.fade.y > pc.fade.x) {
    float f = clamp((vTreeDist - pc.fade.x) / (pc.fade.y - pc.fade.x), 0.0, 1.0);
    float dz = F.taa.z > 0.5 ? fract(ign(gl_FragCoord.xy) + 0.61803399 * float(F.frameInfo.x % 64u)) : ign(gl_FragCoord.xy);
    if (pc.surf == 202u ? dz >= f : dz < f) discard;
  }
  vec3 alb; vec3 n = normalize(vNrm); float rough = 0.85, metal = 0.0, ao = 1.0;
  // distance-scaled alpha threshold: far canopies get fuller (fewer sub-pixel holes that open and close as the camera moves);
  // the coverage-preserving mips keep the mean coverage, this removes the speckle at the cut-off.
  float aRef = mix(0.5, 0.3, smoothstep(12.0, 90.0, distance(vPos, F.camPos.xyz)));
  if (pc.surf == 200u) { alb = TX(vegTex, vec3(vUv, pc.layer)).rgb * pc.tint.rgb; if (!gl_FrontFacing) n = -n; rough = 0.9; }
  else if (pc.surf == 201u) {
    vec4 t = TX(vegTex, vec3(vUv, pc.layer + 1u)); if (!opaque && t.a < aRef) discard;
    alb = t.rgb * pc.tint.rgb;
    // Two-sided leaf cards: orient the card normal to the OUTWARD canopy hemisphere instead of flipping it with gl_FrontFacing.
    // The facing flip is view-dependent: as the camera moves, cards cross edge-on and their lighting jumps between front and back
    // (a main source of the leaf "boiling"). Then lean strongly on the canopy-sphere normal (stable, soft canopy shading).
    vec3 outw = normalize(vPos - vX5 + vec3(0, 1e-3, 0)); if (dot(n, outw) < 0.0) n = -n;
    n = normalize(mix(n, outw, 0.75)); rough = 0.8;
    // canopy AO (world-locked: depth inside the crown + up/down), replaces the per-pixel screen-space GI on leaves (see composite)
    float rr = clamp(length(vPos - vX5) / vCell.x, 0.0, 1.0); ao = mix(0.35, 1.0, rr * rr) * (0.7 + 0.3 * (0.5 + 0.5 * outw.y));
  } else if (pc.surf == 202u) {
    vec4 a = impBlend(pc.layer, true); if (!opaque && a.a < aRef) discard;
    vec3 nb = impBlend(pc.layer + 1u, false).xyz * 2.0 - 1.0;
    alb = a.rgb / max(a.a, 1e-3); n = normalize(vX5 * nb.x + vUp * nb.y + vToCam * nb.z); rough = 0.85; ao = 0.55 + 0.35 * (0.5 + 0.5 * n.y);
  } else { // 203 grass: base -> tip gradient, blended to the ground colour at the root
    float t = vUv.x, hv = vUv.y; vec3 base = vec3(0.05, 0.09, 0.025), tip = mix(vec3(0.24, 0.42, 0.08), vec3(0.38, 0.46, 0.12), hv), ground = vec3(0.07, 0.09, 0.035);
    alb = mix(mix(ground, base, smoothstep(0.0, 0.15, t)), tip, pow(t, 1.4)); rough = 0.6; ao = mix(0.6, 1.0, t);
    // distance LOD: blades lose contrast (colour -> lawn mean, normal -> up) so sub-pixel blades can't pop. Without TAA this starts
    // much closer (non-temporal anti-shimmer path); with TAA the history resolves the sub-pixel blades, so detail is kept further out.
    vec2 fr = F.taa.z > 0.5 ? vec2(14.0, 50.0) : vec2(5.0, 26.0);
    float fd = smoothstep(fr.x, fr.y, distance(vPos, F.camPos.xyz)); alb = mix(alb, vec3(0.10, 0.16, 0.045), fd * 0.8); n = normalize(mix(n, vec3(0, 1, 0), fd)); ao = mix(ao, 0.8, fd);
  }
  matFinish(grp, alb, n, rough, metal);
  vec2 cur = vCur.xy / vCur.w, prev = vPrev.xy / vPrev.w;
  // foliage (leaves, impostors, grass): roughness + 4 flags it for lighting/composite, albedo.a carries the world-locked foliage AO
  if (pc.surf != 200u && rough >= 0.0) { rough += 4.0; metal = ao; }
  oAlbedo = vec4(alb, metal); oNormal = vec4(n, rough); oVel = (cur - prev) * 0.5;
}
