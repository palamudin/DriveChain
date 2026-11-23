#version 460
#extension GL_GOOGLE_include_directive : enable
#include "common.glsl"
#include "mat_frag.glsl"
// Material layers in the albedo/rough texture arrays: 0 asphalt 1 grass 2 concrete 3 cobble 4 facade (generated, publishable).
layout(set = 0, binding = 1) uniform sampler2DArray albedoTex;
layout(set = 0, binding = 2) uniform sampler2DArray roughTex;
layout(push_constant) uniform PC { mat4 model; vec4 tint; uint surf; uint pad0, pad1, pad2; } pc;
layout(location = 0) in vec3 vPos; layout(location = 1) in vec3 vNrm; layout(location = 2) in vec2 vUv; layout(location = 3) in vec4 vCur; layout(location = 4) in vec4 vPrev;
layout(location = 0) out vec4 oAlbedo; layout(location = 1) out vec4 oNormal; layout(location = 2) out vec2 oVel;
bool gNoMips = false;
vec4 TX(sampler2DArray t, vec3 c) { return gNoMips ? textureLod(t, c, 0.0) : texture(t, c); }
vec4 tri(float layer, float s, vec3 n, sampler2DArray t) { vec3 w = pow(abs(n), vec3(4)); w /= (w.x + w.y + w.z);
  return TX(t, vec3(vPos.zy * s, layer)) * w.x + TX(t, vec3(vPos.xz * s, layer)) * w.y + TX(t, vec3(vPos.xy * s, layer)) * w.z; }
uint groupOf(uint s) { return (s == 0u || s == 8u) ? G_ASPHALT : (s == 5u || s == 12u) ? G_MARKINGS : s == 1u ? G_KERBS : (s == 2u || s == 3u || s == 6u || s == 7u || s == 11u) ? G_TERRAIN : s >= 100u ? G_CAR : s == 32u ? G_LEAVES : G_PROPS; }
void main() {
  vec3 n = normalize(vNrm); if (dot(n, n) < 0.5 || any(isnan(n))) n = normalize(cross(dFdx(vPos), dFdy(vPos)));
  if (!gl_FrontFacing) n = -n;
  vec3 alb = vec3(0.5); float rough = 0.8, metal = 0.0; uint s = pc.surf; uint grp = groupOf(s); gNoMips = matNoMips(grp);
  if (s == 0u || s == 5u || s == 8u) { // asphalt + rubbered line (uv.y = lateral m from racing line), wet darkening
    vec2 uv = vPos.xz / 5.0; vec3 a = TX(albedoTex, vec3(uv, 0)).rgb, b = TX(albedoTex, vec3(mat2(0.8, -0.6, 0.6, 0.8) * uv * 0.137, 0)).rgb;
    alb = a * clamp(dot(b, vec3(0.33)) * 1.6 + 0.2, 0.6, 1.25) * 1.7; rough = TX(roughTex, vec3(uv, 0)).r;
    float rub = smoothstep(2.4, 0.4, abs(vUv.y)) * 0.45; alb *= 1.0 - rub; rough *= 1.0 - rub * 0.5;
    alb *= mix(1.0, 0.55, F.gi.w); rough = clamp(rough * mix(1.0, 0.18, F.gi.w), 0.04, 1.0);
  } else if (s == 1u) { float st = step(0.5, fract(vUv.x / 3.0)); alb = mix(vec3(0.46, 0.008, 0.005), vec3(0.8, 0.78, 0.74), st); rough = 0.55; }
  else if (s == 2u) { alb = TX(albedoTex, vec3(vPos.xz / 3.0, 1)).rgb; rough = 0.9; }
  else if (s == 6u) { alb = TX(albedoTex, vec3(vPos.xz / 2.5, 3)).rgb; rough = TX(roughTex, vec3(vPos.xz / 2.5, 3)).r; }
  else if (s == 9u) { alb = tri(2.0, 1.0 / 3.0, n, albedoTex).rgb * 0.42; rough = 0.85; }
  else if (s == 11u) { alb = TX(albedoTex, vec3(vPos.xz / 4.0, 2)).rgb * 0.6; rough = 0.8; }
  else if (s == 10u) { alb = tri(4.0, 1.0 / 9.0, n, albedoTex).rgb * 0.62; rough = 0.75; }           // buildings (cls 10)
  else if (s == 32u) { alb = TX(albedoTex, vec3(vPos.xz * 0.11 + vPos.y * 0.07, 1)).rgb * vec3(0.55, 0.7, 0.45); rough = 0.8; } // foliage
  else if (s == 33u) { alb = vec3(0.012); rough = 0.92; }                                                // tyre walls
  else if (s == 34u) { alb = vec3(0.3, 0.32, 0.34); rough = 0.45; metal = 0.7; }                         // fences
  else if (s == 12u) { alb = vec3(0.62, 0.62, 0.58); rough = 0.6; }                                     // paint decals (markings)
  else if (s >= 100u) { alb = pc.tint.rgb; rough = pc.tint.a; metal = s == 103u ? 1.0 : 0.0; }        // car parts
  vec2 cur = vCur.xy / vCur.w, prev = vPrev.xy / vPrev.w;
  matFinish(grp, alb, n, rough, metal);
  oAlbedo = vec4(alb, metal); oNormal = vec4(n, rough); oVel = (cur - prev) * 0.5;
}
