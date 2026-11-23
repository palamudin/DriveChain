// veg_common.glsl - vegetation instance layout + wind, shared by veg.vert / imp.vert / veg_shadow.vert
// Instance: posYaw = (x, y, z, yaw); prm = (scale, wind phase, impostor half-size (world m), impostor centre height above root (world m))
layout(push_constant) uniform VPC { mat4 shadowVP; vec4 tint; uint surf; uint layer; uint mode; uint kindN; } pc; // mode: 0 camera, 1 shadow pass
vec3 rotY(vec3 p, float a) { float c = cos(a), s = sin(a); return vec3(c * p.x + s * p.z, p.y, -s * p.x + c * p.z); }
// gentle trunk sway + leaf flutter, amplitude grows with height^2 (world metres above the root); stable (time-only)
vec3 windOffset(vec3 wp, vec3 root, float phase, float t, float flutter) {
  float h = max(wp.y - root.y, 0.0); float k = h * h * 0.0009;
  vec2 dir = vec2(0.8, 0.6); float g = sin(t * 0.9 + phase) * 0.6 + sin(t * 2.1 + phase * 1.7) * 0.25 + 0.35;
  vec3 o = vec3(dir.x, 0.0, dir.y) * g * k;
  o += flutter * vec3(sin(t * 6.3 + dot(wp, vec3(1.7, 2.3, 1.1))), 0.0, cos(t * 5.1 + dot(wp, vec3(2.1, 1.3, 1.9)))) * 0.03 * min(h * 0.1, 1.0);
  return o;
}
