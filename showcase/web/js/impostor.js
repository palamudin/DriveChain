// impostor.js - far-field impostor cache (cached static work, per the handbook; idea after SmartImposter's
// "render once, reuse many frames" impostors, but applied to the whole far city ring instead of per-object octahedra).
//
// Every world chunk farther than `radius` from the car is moved to layer 3 and dropped from the main view, shadow and GI
// passes. Those chunks are rendered into a cube map from a capture point near the car, ONE FACE PER FRAME into a back buffer
// (6-frame amortised refresh, double-buffered so there is never a half-updated cube on screen). The cube is drawn as a
// distant shell between the sky and the near world. Refresh is triggered when the car moves > `recaptureDist` from the last
// capture point, so on a straight it costs 1/6 of a far-ring render per frame and nothing when parked.
// Known limits: parallax error grows with distance from the capture point (bounded by recaptureDist / radius ~ 9 deg),
// and far geometry does not cast into the near shadow cascade (it is outside the 220 m shadow box anyway).
import * as THREE from 'three/webgpu';
import { cubeTexture, positionWorld, cameraPosition, normalize, vec4, float, Fn, mix } from 'three/tsl';
export class FarImpostors {
  constructor(renderer, scene, chunks, { radius = 260, recaptureDist = 40, size = 768 } = {}) {
    this.r = renderer; this.scene = scene; this.chunks = chunks; this.radius = radius; this.recapture = recaptureDist;
    this.rts = [0, 1].map(() => new THREE.CubeRenderTarget(size, { type: THREE.HalfFloatType, generateMipmaps: false }));
    this.cams = [0, 1].map((i) => { const c = new THREE.CubeCamera(1, 6000, this.rts[i]); c.children.forEach(k => k.layers.set(3)); c.layers.set(3); scene.add(c); return c; });
    this.front = 0; this.face = -1; this.capturePos = new THREE.Vector3(1e9, 0, 0); this.enabled = false; this.captures = 0;
    this.frontTex = cubeTexture(this.rts[0].texture);
    const mat = new THREE.MeshBasicNodeMaterial({ side: THREE.BackSide, transparent: true, depthWrite: false, fog: false });
    mat.colorNode = Fn(() => { const d = normalize(positionWorld.sub(cameraPosition)); return this.frontTex.sample(d.mul(vec4(-1, 1, 1, 0).xyz)); })();
    mat.fog = false; this.shell = new THREE.Mesh(new THREE.SphereGeometry(3000, 48, 24), mat); this.shell.renderOrder = -1; this.shell.frustumCulled = false; this.shell.name = 'far-impostor-shell';
    this.mat = mat;
  }
  setEnabled(on) {
    this.enabled = on; if (on && !this.shell.parent) this.scene.add(this.shell); if (!on) { this.shell.removeFromParent(); for (const c of this.chunks) c.layers.set(0); this.capturePos.set(1e9, 0, 0); this.face = -1; this.captures = 0; this.farTris = 0; }
  }
  // layer 0 = drawn in the main view (near set of the CURRENT front cube); layer 3 = drawn into the cube being captured.
  classify(p) { let far = 0; for (const c of this.chunks) { const d = Math.hypot(c.userData.center.x - p.x, c.userData.center.z - p.z) - (c.geometry.boundingSphere.radius || 0);
      c.userData.nextFar = d > this.radius; if (c.userData.nextFar) c.layers.enable(3); else c.layers.disable(3); far += c.userData.nextFar; } return far; }
  commit() { let tris = 0; for (const c of this.chunks) { if (c.userData.nextFar) { c.layers.disable(0); tris += c.geometry.index.count / 3; } else c.layers.enable(0); } this.farTris = tris; }
  update(carPos, camPos) {
    if (!this.enabled || !carPos) return;
    this.shell.position.copy(camPos);
    if (this.face < 0 && carPos.distanceTo(this.capturePos) > this.recapture) {
      // start a new amortised capture into the back buffer. First ever capture is done synchronously so frame 1 is complete.
      this.pending = carPos.clone().setY(carPos.y + 2); this.farCount = this.classify(this.pending); this.face = 0;
      const back = this.cams[1 - this.front]; back.position.copy(this.pending); back.updateMatrixWorld(true);
      if (this.captures === 0) { while (this.face >= 0) this._face(); return; }
    }
    if (this.face >= 0) this._face();
  }
  _face() {
    const backIdx = 1 - this.front, cam = this.cams[backIdx], rt = this.rts[backIdx], r = this.r;
    const prevRT = r.getRenderTarget(), prevClear = r.getClearColor(new THREE.Color()), prevAlpha = r.getClearAlpha();
    r.setClearColor(0x000000, 0); r.setRenderTarget(rt, this.face); r.clear(); r.render(this.scene, cam.children[this.face]);
    r.setRenderTarget(prevRT); r.setClearColor(prevClear, prevAlpha);
    if (++this.face === 6) { this.face = -1; this.front = backIdx; this.frontTex.value = rt.texture; this.capturePos.copy(this.pending); this.captures++; this.commit(); }
  }
}
