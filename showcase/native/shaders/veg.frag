#version 460
#extension GL_GOOGLE_include_directive : enable
#include "common.glsl"
#include "veg_common.glsl"
// Vegetation G-buffer: 200 bark, 201 leaves (alpha-tested, coverage-preserving mips), 202 impostor, 203 grass. Same outputs as gbuffer.frag.
layout(set = 0, binding = 1) uniform sampler2DArray vegTex;  // per kind: bark, leaf
layout(set = 0, binding = 2) uniform sampler2DArray impTex;  // per kind: albedo+alpha atlas, normal atlas
layout(location = 0) in vec3 vPos; layout(location = 1) in vec3 vNrm; layout(location = 2) in vec2 vUv; layout(location = 3) in vec4 vCur; layout(location = 4) in vec4 vPrev;
layout(location = 5) in vec3 vX5; layout(location = 6) in vec3 vUp; layout(location = 7) in vec3 vToCam; layout(location = 8) flat in vec2 vCell;
layout(location = 0) out vec4 oAlbedo; layout(location = 1) out vec4 oNormal; layout(location = 2) out vec2 oVel;
void main() {
  vec3 alb; vec3 n = normalize(vNrm); float rough = 0.85;
  if (pc.surf == 200u) { alb = texture(vegTex, vec3(vUv, pc.layer)).rgb * pc.tint.rgb; if (!gl_FrontFacing) n = -n; rough = 0.9; }
  else if (pc.surf == 201u) {
    vec4 t = texture(vegTex, vec3(vUv, pc.layer + 1u)); if (t.a < 0.5) discard;
    alb = t.rgb * pc.tint.rgb; if (!gl_FrontFacing) n = -n;
    // blend the card normal toward the canopy sphere normal: soft, stable canopy lighting instead of per-card noise
    n = normalize(mix(n, normalize(vPos - vX5), 0.55)); rough = 0.7;
  } else if (pc.surf == 202u) {
    vec2 uv = (vCell + vUv) / float(pc.kindN); uv.y = 1.0 - uv.y; // atlas PNG rows are top-down; bake cell (i,j) has j up
    vec4 a = texture(impTex, vec3(uv, pc.layer)); if (a.a < 0.5) discard;
    vec3 nb = texture(impTex, vec3(uv, pc.layer + 1u)).xyz * 2.0 - 1.0;
    alb = a.rgb; n = normalize(vX5 * nb.x + vUp * nb.y + vToCam * nb.z); rough = 0.8;
  } else { // 203 grass: base -> tip gradient, blended to the ground colour at the root
    float t = vUv.x, hv = vUv.y; vec3 base = vec3(0.035, 0.07, 0.018), tip = mix(vec3(0.24, 0.42, 0.08), vec3(0.38, 0.46, 0.12), hv), ground = vec3(0.07, 0.09, 0.035);
    alb = mix(mix(ground, base, smoothstep(0.0, 0.15, t)), tip, pow(t, 1.4)); rough = 0.6;
    // anti-shimmer without TAA: past ~12 m blades lose contrast (colour -> lawn mean, normal -> up), so sub-pixel motion can't pop
    float fd = smoothstep(12.0, 45.0, distance(vPos, F.camPos.xyz)); alb = mix(alb, vec3(0.10, 0.16, 0.045), fd * 0.75); n = normalize(mix(n, vec3(0, 1, 0), fd));
  }
  vec2 cur = vCur.xy / vCur.w, prev = vPrev.xy / vPrev.w;
  oAlbedo = vec4(alb, 0.0); oNormal = vec4(n, rough); oVel = (cur - prev) * 0.5;
}
