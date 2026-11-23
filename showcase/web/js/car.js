// car.js - car visual driven entirely by physics state (body pose + per-wheel centre/orientation from tms state).
// Visual mesh is swappable: a local private OBJ set (miata stand-in, never published) or a procedural coupe (own code).
import * as THREE from 'three/webgpu';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import { MTLLoader } from 'three/addons/loaders/MTLLoader.js';

// DriveChain car frame (x fwd, y left, z up) -> basis built at runtime from physics fwd/up.
const WHEELS = ['fl', 'fr', 'rl', 'rr'];
export class CarVisual {
  constructor(scene) {
    this.scene = scene; this.root = new THREE.Group(); this.body = new THREE.Group(); this.root.add(this.body);
    this.wheels = WHEELS.map(() => { const g = new THREE.Group(); scene.add(g); return g; });
    scene.add(this.root); this.meshWheel = null; this.stretch = 1; this.fitted = false; this.kind = 'none';
    this._m = new THREE.Matrix4(); this._q = new THREE.Quaternion(); this.layer = 2;
  }
  setLayer(l) { this.layer = l; const f = (o) => { o.layers.enable(l); }; this.root.traverse(f); this.wheels.forEach(w => w.traverse(f)); }
  async loadPrivateObj(dir) {
    const cfgTxt = await (await fetch(dir + 'mesh.cfg')).text(); if (!/wheel_fl/.test(cfgTxt)) throw new Error('no mesh.cfg');
    const cfg = {}; cfgTxt.split('\n').forEach(l => { const m = l.match(/^(\w+)\s*=\s*(.*)$/); if (m) cfg[m[1]] = m[2].trim().split(/\s+/).map(Number); });
    const load = async (name) => {
      const mtl = await new MTLLoader().setPath(dir).loadAsync(name + '.mtl'); mtl.preload();
      const o = await new OBJLoader().setMaterials(mtl).setPath(dir).loadAsync(name + '.obj');
      o.traverse(c => { if (c.isMesh) { c.material = (Array.isArray(c.material) ? c.material : [c.material]).map(convertMat); if (c.material.length === 1) c.material = c.material[0]; c.castShadow = true; c.receiveShadow = true; } });
      return o;
    };
    const body = await load('body'); this.body.add(body);
    try { const st = await load('steer'); st.position.fromArray(cfg.steer_center); this.body.add(st); } catch (e) {}
    for (let i = 0; i < 4; i++) {
      const w = await load('wheel_' + WHEELS[i]); this.wheels[i].add(w);
      try { const cal = await load('caliper_' + WHEELS[i]); this.wheels[i].userData.caliper = cal; cal.userData.noSpin = true; } catch (e) {}
    }
    this.meshWheelPos = WHEELS.map(k => new THREE.Vector3().fromArray(cfg['wheel_' + k]));
    this.wheelRadius = cfg.wheel_radius_mesh?.[0] || 0.3; this.kind = 'private-obj'; this._spinGroups(); return this;
  }
  buildProcedural() {
    // Own procedural low-poly coupe (publishable). Car frame: x fwd, y left, z up. Wheelbase 2.56, track 1.40 (own proportions).
    const paint = new THREE.MeshPhysicalNodeMaterial({ color: 0xb01a14, roughness: 0.32, metalness: 0.1, clearcoat: 1, clearcoatRoughness: 0.04 });
    const glass = new THREE.MeshPhysicalNodeMaterial({ color: 0x0b0d10, roughness: 0.05, metalness: 0, clearcoat: 1 });
    const trim = new THREE.MeshStandardNodeMaterial({ color: 0x111111, roughness: 0.6 });
    const chrome = new THREE.MeshStandardNodeMaterial({ color: 0xdddddd, roughness: 0.15, metalness: 1 });
    const side = new THREE.Shape();
    const P = [[-2.15, -0.25], [2.05, -0.25], [2.18, 0.0], [2.12, 0.18], [1.0, 0.33], [0.42, 0.40], [-0.05, 0.86], [-1.15, 0.86], [-1.65, 0.45], [-2.18, 0.40], [-2.2, 0.0]];
    side.moveTo(...P[0]); P.slice(1).forEach(p => side.lineTo(...p)); side.closePath();
    const g = new THREE.ExtrudeGeometry(side, { depth: 1.52, bevelEnabled: true, bevelThickness: 0.08, bevelSize: 0.06, bevelSegments: 3, steps: 1 });
    g.translate(0, 0, -0.76); g.rotateX(Math.PI / 2); // shape XY -> car XZ, extrude along -Y.. centre
    const b = new THREE.Mesh(g, paint); b.castShadow = b.receiveShadow = true; b.position.z = 0.0; this.body.add(b);
    const gl = new THREE.ExtrudeGeometry((() => { const s = new THREE.Shape(); s.moveTo(0.38, 0.42); s.lineTo(-0.06, 0.82); s.lineTo(-1.12, 0.82); s.lineTo(-1.58, 0.47); s.closePath(); return s; })(), { depth: 1.42, bevelEnabled: false });
    gl.translate(0, 0, -0.71); gl.rotateX(Math.PI / 2); const gm = new THREE.Mesh(gl, glass); gm.scale.set(1.0, 1.0, 1.0); gm.position.z = 0.005; this.body.add(gm);
    const bump = (x) => { const m = new THREE.Mesh(new THREE.BoxGeometry(0.12, 1.62, 0.16), chrome); m.position.set(x, 0, -0.12); this.body.add(m); };
    bump(2.18); bump(-2.22);
    const lamp = new THREE.MeshStandardNodeMaterial({ color: 0xffffff, emissive: 0xfff2d8, emissiveIntensity: 2, roughness: 0.2 });
    const tail = new THREE.MeshStandardNodeMaterial({ color: 0x400000, emissive: 0xff1a0a, emissiveIntensity: 1.5 });
    for (const y of [0.5, -0.5]) { const h = new THREE.Mesh(new THREE.CylinderGeometry(0.09, 0.09, 0.05, 20), lamp); h.rotation.z = Math.PI / 2; h.position.set(2.2, y, 0.05); this.body.add(h);
      const t = new THREE.Mesh(new THREE.BoxGeometry(0.04, 0.35, 0.1), tail); t.position.set(-2.27, y * 1.1, 0.22); this.body.add(t); }
    const under = new THREE.Mesh(new THREE.BoxGeometry(4.2, 1.5, 0.06), trim); under.position.z = -0.28; this.body.add(under);
    // wheels: tyre torus-ish cylinder + 5-spoke rim
    const tyreG = new THREE.CylinderGeometry(0.31, 0.31, 0.2, 40, 1);
    const tyreM = new THREE.MeshStandardNodeMaterial({ color: 0x151515, roughness: 0.9 });
    const rimM = new THREE.MeshStandardNodeMaterial({ color: 0xb8bcc0, roughness: 0.25, metalness: 1 });
    for (let i = 0; i < 4; i++) {
      const w = new THREE.Group(); const t = new THREE.Mesh(tyreG, tyreM); t.castShadow = true; w.add(t);
      const rim = new THREE.Mesh(new THREE.CylinderGeometry(0.2, 0.2, 0.205, 5, 1), rimM); w.add(rim);
      for (let k = 0; k < 5; k++) { const s = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.03, 0.36), rimM); s.rotation.y = k * Math.PI / 5; s.position.y = (i % 2 ? -1 : 1) * 0.105; w.add(s); }
      this.wheels[i].add(w);
    }
    this.meshWheelPos = [[1.28, 0.70, -0.12], [1.28, -0.70, -0.12], [-1.28, 0.70, -0.12], [-1.28, -0.70, -0.12]].map(a => new THREE.Vector3(...a));
    this.wheelRadius = 0.31; this.kind = 'procedural'; this._spinGroups(); return this;
  }
  _spinGroups() {
    // per wheel: outer group (positioned+steered by physics quat), inner spin group
    this.wheels.forEach((w) => { const spin = new THREE.Group(); [...w.children].forEach(c => spin.add(c)); w.add(spin); w.userData.spin = spin; if (w.userData.caliper) w.add(w.userData.caliper); });
  }
  // state: Float32Array(80) from tms_car_state. Physics wheel quats are in the physics car convention; we only use them for the
  // steer angle; orientation is reconstructed from wheel centres so mesh handedness never matters.
  update(s, alpha = 1) {
    const W = (i) => new THREE.Vector3(s[16 + i * 16], s[17 + i * 16], s[18 + i * 16]);
    const w = [0, 1, 2, 3].map(W);
    const front = w[0].clone().add(w[1]).multiplyScalar(0.5), rear = w[2].clone().add(w[3]).multiplyScalar(0.5);
    const F = front.clone().sub(rear); const wb = F.length(); F.normalize();
    // physics wheel 0 may be left or right depending on convention -> pick lateral from body quat up instead
    const q = new THREE.Quaternion(s[3], s[4], s[5], s[6]); // tms packs x,y,z,w
    let U = new THREE.Vector3(0, 1, 0).applyQuaternion(q);
    if (Math.abs(U.y) < 0.3) U = new THREE.Vector3(0, 1, 0);
    const L = new THREE.Vector3().crossVectors(U, F).normalize(); U.crossVectors(F, L).normalize();
    this._m.makeBasis(F, L, U);
    const centre = front.clone().add(rear).multiplyScalar(0.5);
    if (!this.fitted) { // fit mesh to physics wheel layout once
      const mf = this.meshWheelPos[0].x, mr = this.meshWheelPos[2].x; this.stretch = wb / (mf - mr);
      this.meshCentre = this.meshWheelPos.reduce((a, b) => a.add(b), new THREE.Vector3()).multiplyScalar(0.25);
      // physics wheel 0 = which side? sign of L·(w0-w1)
      this.leftIndexSwap = L.dot(w[0].clone().sub(w[1])) < 0; this.fitted = true;
    }
    this.root.matrixAutoUpdate = false;
    const off = this.meshCentre.clone(); off.x *= this.stretch;
    const T = new THREE.Matrix4().makeTranslation(-off.x, -off.y, -off.z);
    const S = new THREE.Matrix4().makeScale(this.stretch, 1, 1);
    this.root.matrix.copy(this._m).setPosition(centre).multiply(T).multiply(S); this.root.matrixWorldNeedsUpdate = true;
    this.chassisF = F; this.chassisU = U; this.chassisL = L; this.centre = centre;
    for (let i = 0; i < 4; i++) {
      const pi = this.leftIndexSwap ? [1, 0, 3, 2][i] : i; // visual wheel i (fl,fr,rl,rr) <- physics wheel pi
      const g = this.wheels[i]; g.position.copy(w[pi]);
      // steer: angle between physics wheel forward and chassis forward
      const wq = new THREE.Quaternion(s[16 + pi * 16 + 3], s[16 + pi * 16 + 4], s[16 + pi * 16 + 5], s[16 + pi * 16 + 6]);
      const fwdW = new THREE.Vector3(0, 0, 1).applyQuaternion(wq); const fwdC = new THREE.Vector3(0, 0, 1).applyQuaternion(q);
      let steer = Math.atan2(new THREE.Vector3().crossVectors(fwdC, fwdW).dot(U), fwdC.dot(fwdW));
      if (!isFinite(steer) || Math.abs(steer) > 0.8) steer = 0;
      const R = new THREE.Matrix4().makeBasis(F, L, U); g.quaternion.setFromRotationMatrix(R).multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), steer * (this.steerSign || 1)));
      const sp = g.userData.spin; sp.rotation.y = -(s[16 + pi * 16 + 7] || 0) * (this.spinSign || 1);
    }
  }
}
function convertMat(m) {
  const n = new THREE.MeshStandardNodeMaterial({ color: m.color, map: m.map || null, roughness: 0.6, metalness: 0 });
  if (m.map) m.map.colorSpace = THREE.SRGBColorSpace;
  if (/chassis|paint|body/i.test(m.name)) { const p = new THREE.MeshPhysicalNodeMaterial({ color: m.color, roughness: 0.3, metalness: 0.15, clearcoat: 1, clearcoatRoughness: 0.03 }); p.name = m.name; return p; }
  if (/chrome|rim/i.test(m.name)) { n.metalness = 1; n.roughness = 0.2; }
  if (/glass|window/i.test(m.name)) { n.transparent = true; n.opacity = 0.4; n.roughness = 0.05; }
  if (/tyre|tire|rubber/i.test(m.name)) { n.roughness = 0.9; }
  if (m.transparent || m.opacity < 1) { n.transparent = true; n.opacity = m.opacity; }
  n.name = m.name; n.alphaTest = m.map ? 0.3 : 0; return n;
}
