// common.glsl - shared frame constants (std140, mirrored by render.rs::FrameU)
layout(set = 0, binding = 0, std140) uniform FrameU {
  mat4 view; mat4 proj; mat4 viewProj; mat4 invViewProj; mat4 prevViewProj;
  mat4 shadowVP[2];          // [0] near dynamic cascade (every frame, incl. car), [1] far static cascade (cached)
  vec4 camPos;               // xyz, w = time
  vec4 sunDir;               // xyz towards sun, w = sun angular radius (rad)
  vec4 sunColor;             // rgb * intensity
  vec4 fog;                  // density, height falloff, volumetric strength, anisotropy g
  vec4 post;                 // exposure, bloom, grain, vignette
  vec4 res;                  // width, height, 1/w, 1/h
  uvec4 frameInfo;           // frame index, rt available, gi mode (0 off 1 half 2 full), debug view
  vec4 gi;                   // gi intensity, ao intensity, radius (m), wet
} F;
const float PI = 3.14159265;
float ign(vec2 p) { return fract(52.9829189 * fract(dot(p, vec2(0.06711056, 0.00583715)))); } // interleaved gradient noise
vec3 worldFromDepth(vec2 uv, float d) { vec4 c = F.invViewProj * vec4(uv * 2.0 - 1.0, d, 1.0); return c.xyz / c.w; }
// Compact single-scattering atmosphere (Rayleigh + Mie, planet radius 6360 km), 16 view steps x 4 light steps.
vec2 raySphere(vec3 ro, vec3 rd, float r) { float b = dot(ro, rd), c = dot(ro, ro) - r * r, h = b * b - c; if (h < 0.0) return vec2(-1); h = sqrt(h); return vec2(-b - h, -b + h); }
vec3 atmosphere(vec3 rd, vec3 sun) {
  const float Rg = 6360e3, Ra = 6460e3; const vec3 bR = vec3(5.8e-6, 13.5e-6, 33.1e-6); const float bM = 21e-6, Hr = 8e3, Hm = 1.2e3;
  vec3 ro = vec3(0, Rg + 2.0, 0); vec2 t = raySphere(ro, rd, Ra); float tmax = t.y; vec2 tg = raySphere(ro, rd, Rg); if (tg.x > 0.0) tmax = tg.x;
  float seg = tmax / 16.0, odR = 0.0, odM = 0.0; vec3 sR = vec3(0), sM = vec3(0); float mu = dot(rd, sun);
  float pR = 3.0 / (16.0 * PI) * (1.0 + mu * mu); float g = 0.76; float pM = 3.0 / (8.0 * PI) * ((1.0 - g * g) * (1.0 + mu * mu)) / ((2.0 + g * g) * pow(1.0 + g * g - 2.0 * g * mu, 1.5));
  for (int i = 0; i < 16; i++) {
    vec3 p = ro + rd * (seg * (float(i) + 0.5)); float h = length(p) - Rg; float hr = exp(-h / Hr) * seg, hm = exp(-h / Hm) * seg; odR += hr; odM += hm;
    vec2 tl = raySphere(p, sun, Ra); float sl = tl.y / 4.0, lR = 0.0, lM = 0.0; bool ok = true;
    for (int j = 0; j < 4; j++) { vec3 q = p + sun * (sl * (float(j) + 0.5)); float hh = length(q) - Rg; if (hh < 0.0) { ok = false; break; } lR += exp(-hh / Hr) * sl; lM += exp(-hh / Hm) * sl; }
    if (ok) { vec3 tau = bR * (odR + lR) + bM * 1.1 * (odM + lM); vec3 att = exp(-tau); sR += att * hr; sM += att * hm; }
  }
  return 20.0 * (sR * bR * pR + sM * bM * pM);
}
vec3 skyAmbient(vec3 n) { // cheap irradiance proxy: sample atmosphere in 3 directions (evaluated per pixel; cache in an SH probe later)
  vec3 up = atmosphere(normalize(vec3(0.0, 1.0, 0.0)), F.sunDir.xyz), hz = atmosphere(normalize(vec3(F.sunDir.x, 0.05, F.sunDir.z) * -1.0 + vec3(0, 0.1, 0)), F.sunDir.xyz);
  return mix(hz * 0.6, up, clamp(n.y * 0.5 + 0.5, 0.0, 1.0)) * PI * 0.35;
}
