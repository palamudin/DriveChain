// mat_frag.glsl - fragment-stage helpers for the material groups (needs derivatives: fragment shaders only)
// Final G-buffer write rules for a group: flat = unlit textured, debug = unlit solid colour (normal.w < 0 marks unlit),
// nospec = roughness + 2 (lighting drops the specular lobe). Geometric specular AA (Tokuyoshi & Kaplanyan 2019) on the normal's
// screen-space variance: widens the GGX lobe where the normal changes faster than a pixel (leaf cards, grass, kerbs at grazing angles).
float specAA(vec3 n, float rough) {
  if (F.opt.x < 0.5) return rough; vec3 dx = dFdx(n), dy = dFdy(n); float v = 0.25 * (dot(dx, dx) + dot(dy, dy)); float k = min(2.0 * v, 0.18);
  float a = rough * rough; return sqrt(sqrt(clamp(a * a + k, 0.0, 1.0)));
}
void matFinish(uint g, inout vec3 alb, inout vec3 n, inout float rough, inout float metal) {
  uint w = matW(g); uint m = w & 3u;
  rough = specAA(n, rough);
  if (m == 1u) { rough = -1.0; metal = 0.0; } else if (m == 2u) { alb = groupColor(g); rough = -1.0; metal = 0.0; }
  else if ((w & 128u) != 0u) { rough += 2.0; metal = 0.0; }
}
