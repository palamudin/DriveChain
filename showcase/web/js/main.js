// main.js - DriveChain Showcase (web): Tarmac physics (wasm, worker) + three.js r187 WebGPU/WebGL2 renderer.
import * as THREE from 'three/webgpu';
import { uniform, vec2, vec3, vec4, float, positionWorld, cameraPosition, normalize, dot, max, pow, mix, exp, fog, color, Fn, smoothstep, texture, step, clamp, abs, uv } from 'three/tsl';
import { SkyMesh } from 'three/addons/objects/SkyMesh.js';
import { loadWorld, buildCollision, makeShowcaseMaterials, makeRawMaterials, applyMaterials, look } from './world.js';
import { makeProceduralWorld } from './proc-world.js';
import { CarVisual } from './car.js';
import { Input } from './input.js';
import { Bot } from './bot.js';
import { chunkMesh } from './chunks.js';
import { buildPipeline, TIERS, postParams } from './post.js';
import { FarImpostors } from './impostor.js';

const Q = new URLSearchParams(location.search);
const CFG = {
  world: Q.get('world') ?? 'private/usce/usce.meshopt.glb',      // private, gitignored; falls back to the procedural track
  car: Q.get('carmesh') ?? 'private/miata_mesh/',                  // private OBJ set; falls back to procedural coupe
  carId: +(Q.get('car') ?? 3), tune: +(Q.get('tune') ?? 0), tier: Q.get('tier') ?? 'high',
  webgl: Q.has('webgl'), auto: Q.has('auto'), still: Q.get('still'), spawn: +(Q.get('spawn') ?? 0), sunEl: +(Q.get('sunel') ?? 24), sunAz: +(Q.get('sunaz') ?? 215),
  warm: +(Q.get('warm') ?? 0), bench: Q.has('bench'), dpr: +(Q.get('dpr') ?? Math.min(devicePixelRatio, 1.5)), wet: +(Q.get('wet') ?? 0),
};
const $ = (id) => document.getElementById(id);
const log = (...a) => { console.log(...a); const el = $('log'); if (el) el.textContent = a.join(' '); };
window.__dcs = { ready: false, frames: 0, errors: [] };
addEventListener('error', (e) => window.__dcs.errors.push(String(e.message)));

// ---------------------------------------------------------------- renderer
const renderer = new THREE.WebGPURenderer({ antialias: false, forceWebGL: CFG.webgl, trackTimestamp: true, powerPreference: 'high-performance' });
await renderer.init();
// WebGPU device loss (driver reset, or headless SwiftShader, where Dawn drops the instance) -> fall back to WebGL2 once.
renderer.onDeviceLost = (info) => { console.error('WebGPU device lost', info && info.message); if (!CFG.webgl && !sessionStorage.getItem('dcs.fellBack')) { sessionStorage.setItem('dcs.fellBack', '1'); const u = new URL(location.href); u.searchParams.set('webgl', '1'); location.replace(u); } };
renderer.setPixelRatio(CFG.dpr); renderer.setSize(innerWidth, innerHeight);
renderer.toneMapping = THREE.AgXToneMapping; renderer.toneMappingExposure = 1.0;
renderer.shadowMap.enabled = true; renderer.shadowMap.type = Q.get('shadowtype') === 'pcf' ? THREE.PCFSoftShadowMap : THREE.VSMShadowMap;
document.body.appendChild(renderer.domElement);
const backend = renderer.backend.isWebGPUBackend ? 'WebGPU' : 'WebGL2';
log('renderer', backend);

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(62, innerWidth / innerHeight, 0.1, 4000);
addEventListener('resize', () => { camera.aspect = innerWidth / innerHeight; camera.updateProjectionMatrix(); renderer.setSize(innerWidth, innerHeight); });

// ---------------------------------------------------------------- sun / sky / fog
let envIntensity = +(Q.get('env') ?? 0.07), sunIntensity = +(Q.get('sun') ?? 5), exposure = +(Q.get('exp') ?? 0.7);
const sun = new THREE.DirectionalLight(0xfff1dc, 3.4); sun.castShadow = true; scene.add(sun, sun.target);
const hemi = new THREE.HemisphereLight(0xbcd3ff, 0x4a4436, 0.0); scene.add(hemi); // only used in raw tier
const sunDir = new THREE.Vector3();
function setSun(elDeg, azDeg) {
  const el = THREE.MathUtils.degToRad(elDeg), az = THREE.MathUtils.degToRad(azDeg);
  sunDir.set(Math.cos(el) * Math.sin(az), Math.sin(el), Math.cos(el) * Math.cos(az)).normalize(); look.sunDir.value.copy(sunDir);
}
setSun(CFG.sunEl, CFG.sunAz);
const sky = new SkyMesh(); sky.scale.setScalar(3500); sky.turbidity.value = 2.4; sky.rayleigh.value = 1.3; sky.mieCoefficient.value = 0.004; sky.mieDirectionalG.value = 0.85;
sky.cloudCoverage && (sky.cloudCoverage.value = 0.35);
// height fog with sun in-scatter (analytic exponential height fog, colour from a sky-matched horizon + forward-scatter lobe)
const fogU = { density: uniform(+(Q.get('fog') ?? 0.0006)), falloff: uniform(0.03), horizon: uniform(new THREE.Color(0.62, 0.70, 0.82)), sunCol: uniform(new THREE.Color(1.0, 0.82, 0.6)) };
const fogNode = Fn(() => {
  const v = positionWorld.sub(cameraPosition), d = v.length(), dir = v.div(d);
  const dy = v.y.mul(fogU.falloff); const hf = exp(cameraPosition.y.mul(fogU.falloff).negate());
  const integ = mix(float(1), float(1).sub(exp(dy.negate())).div(dy), step(1e-4, abs(dy)));
  const f = float(1).sub(exp(fogU.density.mul(d).mul(hf).mul(integ).negate())).clamp(0, 0.92);
  const s = pow(max(dot(dir, look.sunDir), 0), 6).mul(0.9);
  return fog(mix(fogU.horizon, fogU.sunCol, s), f);
})();

// sky -> environment cube (static => cached; recaptured only when sun changes)
const skyScene = new THREE.Scene(); const skyForEnv = new SkyMesh(); skyForEnv.scale.setScalar(100);
['turbidity', 'rayleigh', 'mieCoefficient', 'mieDirectionalG'].forEach(k => skyForEnv[k].value = sky[k].value);
skyForEnv.cloudCoverage && (skyForEnv.cloudCoverage.value = 0); skyScene.add(skyForEnv);
const envRT = new THREE.CubeRenderTarget(256, { type: THREE.HalfFloatType, generateMipmaps: true, minFilter: THREE.LinearMipmapLinearFilter });
const envCam = new THREE.CubeCamera(0.1, 1000, envRT); skyScene.add(envCam);
function captureEnv() { sky.sunPosition.value.copy(sunDir).multiplyScalar(1000); skyForEnv.sunPosition.value.copy(sunDir).multiplyScalar(1000); envCam.update(renderer, skyScene); }

// ---------------------------------------------------------------- world
let world, worldKind;
try { world = await loadWorld(CFG.world, (f) => log('loading world', (f * 100).toFixed(0) + '%')); worldKind = 'gltf:' + CFG.world; }
catch (e) { log('world load failed, using procedural track:', e.message); const w = makeProceduralWorld(); world = { root: w.root, meta: w.meta, colliders: [], visuals: [], spawns: w.meta.spawns };
  w.root.traverse(o => { if (!o.isMesh) return; const t = o.userData.tarmac || {}; (t.collide ? world.colliders : world.visuals).push({ mesh: o, surf: t.surf ?? 0, cls: t.cls ?? null, name: o.name }); }); worldKind = 'procedural'; }
const col = buildCollision(world);
log('world', world.meta.name || worldKind, 'collision tris', col.indices.length / 3);
// chunk big meshes for frustum culling; keep entries for material swapping
const worldGroup = new THREE.Group(); scene.add(worldGroup);
const entries = [];
for (const e of [...world.colliders, ...world.visuals]) {
  e.mesh.updateMatrixWorld(true); const parts = chunkMesh(e.mesh, 96);
  for (const m of parts) { m.castShadow = true; m.receiveShadow = true; if (m === e.mesh) { m.removeFromParent(); m.matrixAutoUpdate = false; m.matrix.copy(e.mesh.matrixWorld); }
    worldGroup.add(m); entries.push({ mesh: m, surf: e.surf, cls: e.cls, name: e.name, collide: world.colliders.includes(e) }); }
}
for (const e of entries) e.mesh.material.name = e.mesh.material.name || '';
const worldSet = { colliders: entries.filter(e => e.collide), visuals: entries.filter(e => !e.collide) };
const MAT = { showcase: makeShowcaseMaterials(), raw: makeRawMaterials() };
const worldBounds = new THREE.Box3().setFromObject(worldGroup);
const impostors = new FarImpostors(renderer, scene, entries.map(e => e.mesh).filter(m => m.userData.center), { radius: +(Q.get('farr') ?? 260) });
let impostorsOn = Q.get('imp') !== '0';

// ---------------------------------------------------------------- car + physics
const car = new CarVisual(scene);
try { await car.loadPrivateObj(CFG.car); } catch (e) { log('private car mesh unavailable -> procedural coupe'); car.buildProcedural(); }
car.setLayer(2);
const spawn = world.spawns[Math.min(CFG.spawn, world.spawns.length - 1)];
const worker = new Worker(new URL('./physics-worker.js', import.meta.url), { type: 'module' });
let state = null, physInfo = { usPerTick: 0, ticks: 0 }, physReady = false, physVersion = '';
const pending = [];
worker.onmessage = (e) => { const m = e.data;
  if (m.type === 'ready') { physReady = true; state = m.state; physVersion = m.version; log('physics ready', m.version, 'world build', m.worldMs.toFixed(0) + ' ms'); }
  else if (m.type === 'state') { state = m.state; physInfo = m; inflight = false; }
  else if (m.type === 'error') { log('PHYSICS ERROR', m.error); window.__dcs.errors.push(m.error); inflight = false; } };
worker.postMessage({ type: 'init', positions: col.positions, indices: col.indices, surf: col.surf, spawn, carId: CFG.carId, tune: CFG.tune }, [col.positions.buffer, col.indices.buffer, col.surf.buffer]);
let inflight = false;

// ---------------------------------------------------------------- car contact shadow (dynamic, every frame) on top of cached sun map
// Cached static cascade: the sun shadow map renders only world geometry and is re-rendered when the car leaves the inner
// region or the sun moves. The car gets its own small sun-aligned depth map every frame, sampled in receivedShadowNode.
const carShadow = { size: 1024, extent: 4.5, rt: null, cam: new THREE.OrthographicCamera(-4.5, 4.5, 4.5, -4.5, 0.1, 80), matrix: uniform(new THREE.Matrix4()), on: uniform(1), flipY: uniform(renderer.backend.isWebGPUBackend ? 1 : 0), zRemap: uniform(renderer.backend.isWebGPUBackend ? 0 : 1) };
carShadow.rt = new THREE.RenderTarget(carShadow.size, carShadow.size, { depthBuffer: true }); carShadow.rt.depthTexture = new THREE.DepthTexture(carShadow.size, carShadow.size);
carShadow.cam.layers.set(2);
const carShadowFactor = Fn(() => {
  const p = carShadow.matrix.mul(vec4(positionWorld, 1)); const n = p.xyz.div(p.w);
  const u = n.x.mul(0.5).add(0.5), vv = mix(n.y.mul(0.5).add(0.5), n.y.mul(-0.5).add(0.5), carShadow.flipY);
  const zRef = mix(n.z, n.z.mul(0.5).add(0.5), carShadow.zRemap).sub(0.0015);
  const inside = step(0.0, u).mul(step(u, 1.0)).mul(step(0.0, vv)).mul(step(vv, 1.0));
  const t = float(1.5 / carShadow.size); let lit = float(0);
  for (let i = -1; i <= 1; i++) for (let j = -1; j <= 1; j++) lit = lit.add(step(zRef, texture(carShadow.rt.depthTexture, vec2(u.add(t.mul(i)), vv.add(t.mul(j)))).x));
  return mix(float(1), lit.div(9), inside.mul(carShadow.on));
})();
if (Q.get('carshadow') !== '0') for (const m of Object.values(MAT.showcase)) { m.receivedShadowNode = Fn(([s]) => s.mul(carShadowFactor)); }
function renderCarShadow() {
  if (!car.centre) return;
  const c = carShadow.cam; c.position.copy(car.centre).addScaledVector(sunDir, 40); c.lookAt(car.centre); c.updateMatrixWorld(); c.updateProjectionMatrix();
  const prevBg = scene.background, prevOv = scene.overrideMaterial; scene.overrideMaterial = shadowDepthMat;
  renderer.setRenderTarget(carShadow.rt); renderer.render(scene, c); renderer.setRenderTarget(null); scene.overrideMaterial = prevOv;
  carShadow.matrix.value.multiplyMatrices(c.projectionMatrix, c.matrixWorldInverse);
}
const shadowDepthMat = new THREE.MeshBasicNodeMaterial({ colorWrite: false });

// ---------------------------------------------------------------- tiers
let tier = TIERS[CFG.tier] ? CFG.tier : 'high', pipe = null, shadowCache = Q.get('shadowcache') !== '0', lastShadowCentre = new THREE.Vector3(1e9, 0, 0);
function applyTier(name) {
  tier = name; const T = TIERS[name];
  applyMaterials(worldSet, T.materials === 'raw' ? MAT.raw : MAT.showcase);
  scene.fogNode = T.sky ? fogNode : null; scene.background = T.sky ? null : new THREE.Color(0x87a8c8);
  if (T.sky && !sky.parent) scene.add(sky); if (!T.sky && sky.parent) sky.removeFromParent();
  scene.environment = T.sky ? envRT.texture : null; scene.environmentIntensity = envIntensity; hemi.intensity = T.sky ? 0 : 1.1;
  sun.intensity = T.sky ? sunIntensity : 1.6; sun.castShadow = !!T.shadows; renderer.toneMapping = T.post ? THREE.ACESFilmicToneMapping : THREE.NoToneMapping; renderer.toneMappingExposure = T.post ? exposure : 1;
  if (T.shadows) { sun.shadow.mapSize.set(T.shadowSize, T.shadowSize); if (sun.shadow.map) { sun.shadow.map.dispose(); sun.shadow.map = null; }
    const s = sun.shadow.camera; s.left = s.bottom = -110; s.right = s.top = 110; s.near = 1; s.far = 900; s.updateProjectionMatrix();
    sun.shadow.radius = 3; sun.shadow.blurSamples = 10; sun.shadow.bias = -0.0002; sun.shadow.normalBias = 0.02; sun.shadow.autoUpdate = !shadowCache; sun.shadow.needsUpdate = true; }
  car.root.traverse(o => { if (o.isMesh) o.castShadow = !shadowCache || !T.shadows; }); car.wheels.forEach(w => w.traverse(o => { if (o.isMesh) o.castShadow = !shadowCache; }));
  carShadow.on.value = T.shadows && shadowCache ? 1 : 0;
  if (pipe) pipe.pp.dispose?.(); pipe = buildPipeline(renderer, scene, camera, name, sun);
  impostors.setEnabled(impostorsOn && !!T.impostors);
  $('tier').textContent = T.label + (impostors.enabled ? ' + far impostors' : ''); lastShadowCentre.set(1e9, 0, 0);
}
captureEnv(); { const target = tier; applyTier(CFG.warm ? 'raw' : target); tier = target; }

// ---------------------------------------------------------------- camera
const camModes = ['chase', 'bumper', 'tv', 'orbit']; let camMode = Q.get('cam') ?? 'chase';
const camPos = new THREE.Vector3(spawn.x, 5, spawn.z), camLook = new THREE.Vector3(); let orbit = { yaw: 2.4, pitch: 0.18, dist: 7.5, drag: null };
renderer.domElement.addEventListener('pointerdown', (e) => { orbit.drag = [e.clientX, e.clientY, orbit.yaw, orbit.pitch]; if (camMode !== 'orbit') camMode = 'orbit'; });
addEventListener('pointerup', () => orbit.drag = null);
addEventListener('pointermove', (e) => { if (!orbit.drag) return; orbit.yaw = orbit.drag[2] - (e.clientX - orbit.drag[0]) * 0.006; orbit.pitch = Math.max(-0.05, Math.min(1.3, orbit.drag[3] + (e.clientY - orbit.drag[1]) * 0.005)); });
addEventListener('wheel', (e) => { orbit.dist = Math.max(3, Math.min(40, orbit.dist * (1 + Math.sign(e.deltaY) * 0.1))); });
function updateCamera(dt) {
  if (!car.centre) return;
  const F = car.chassisF, U = new THREE.Vector3(0, 1, 0), L = car.chassisL, C = car.centre;
  let want = new THREE.Vector3(), target = new THREE.Vector3(), k = 1 - Math.exp(-dt * 6);
  if (camMode === 'chase') { const flat = new THREE.Vector3(F.x, 0, F.z).normalize(); want.copy(C).addScaledVector(flat, -6.2).addScaledVector(U, 2.0); target.copy(C).addScaledVector(flat, 2.5).addScaledVector(U, 0.7); }
  else if (camMode === 'bumper') { want.copy(C).addScaledVector(F, 1.4).addScaledVector(car.chassisU, 0.55); target.copy(want).addScaledVector(F, 10); k = 1; }
  else if (camMode === 'tv') { if (!this || true) { if (!updateCamera.tv || updateCamera.tv.distanceTo(C) > 70) updateCamera.tv = C.clone().addScaledVector(F, 40).addScaledVector(L, 12).setY(C.y + 3.5); } want.copy(updateCamera.tv); target.copy(C); k = 1; }
  else { const d = orbit.dist; want.set(C.x + Math.sin(orbit.yaw) * Math.cos(orbit.pitch) * d, C.y + 0.6 + Math.sin(orbit.pitch) * d, C.z + Math.cos(orbit.yaw) * Math.cos(orbit.pitch) * d); target.copy(C).setY(C.y + 0.4); k = 1 - Math.exp(-dt * 20); }
  camPos.lerp(want, k); camLook.lerp(target, k === 1 ? 1 : Math.min(1, k * 1.6)); camera.position.copy(camPos); camera.lookAt(camLook);
  if (camMode === 'tv') camera.fov = THREE.MathUtils.clamp(2 * Math.atan(3.2 / camPos.distanceTo(C)) * 57.3, 8, 62); else camera.fov = camMode === 'bumper' ? 72 : 62;
  camera.updateProjectionMatrix();
}

// ---------------------------------------------------------------- input / HUD
const input = new Input(); const bot = world.meta.line ? new Bot(world.meta.line) : null; let auto = CFG.auto && !!bot;
let gear = 1, autoBox = true, shiftCooldown = 0, timings = { gpu: 0, cpu: 0 };
addEventListener('keydown', (e) => {
  if (e.code === 'Digit1') applyTier('raw'); if (e.code === 'Digit2') applyTier('base'); if (e.code === 'Digit3') applyTier('high'); if (e.code === 'Digit4') applyTier('showcase');
  if (e.code === 'KeyR') { worker.postMessage({ type: 'reset', spawn }); gear = 1; }
  if (e.code === 'KeyV') camMode = camModes[(camModes.indexOf(camMode) + 1) % camModes.length];
  if (e.code === 'KeyP') auto = !auto && !!bot; if (e.code === 'KeyM') autoBox = !autoBox;
  if (e.code === 'KeyH') $('help').classList.toggle('hide');
  if (e.code === 'KeyK') { shadowCache = !shadowCache; applyTier(tier); }
  if (e.code === 'KeyC') { const m = input.startCalib(); log('wheel calibration:', m); }
  if (e.code === 'KeyT') { CFG.sunEl = (CFG.sunEl + 8) % 70 + 3; setSun(CFG.sunEl, CFG.sunAz); captureEnv(); lastShadowCentre.set(1e9, 0, 0); }
  if (e.code === 'KeyI') { impostorsOn = !impostorsOn; applyTier(tier); }
  if (e.code === 'KeyG') { look.wet.value = look.wet.value > 0.5 ? 0 : 1; }
});
look.wet.value = CFG.wet;

// ---------------------------------------------------------------- loop
const clock = new THREE.Timer(); clock.connect?.(document); let fpsAcc = 0, fpsN = 0, fps = 0, gpuMs = 0, frame = 0, simTime = 0;
async function frameLoop() {
  clock.update(); const dt = Math.min(clock.getDelta(), 0.1); frame++;
  const fixedDt = CFG.still ? 1 / 30 : dt; // stills: deterministic sim time per rendered frame
  if (physReady && !inflight && !window.__dcs.paused) {
    let inp = input.read(fixedDt);
    if (input.calib) $('log').textContent = 'wheel calibration: ' + input.calibMsg;
    if (auto && state && car.chassisF) { const b = bot.drive({ x: car.centre.x, z: car.centre.z, fx: car.chassisF.x, fz: car.chassisF.z, lx: car.chassisL.x, lz: car.chassisL.z, speed: state[10] }); inp = { ...inp, ...b }; }
    // gearbox: auto by rpm, or manual (Q/E, paddles)
    const rpm = state ? state[11] : 0, spd = state ? state[10] : 0; shiftCooldown -= fixedDt;
    if (autoBox && shiftCooldown <= 0) { if (rpm > 6200 && gear < 5) { gear++; shiftCooldown = 0.6; } else if (rpm < 2400 && gear > 1) { gear--; shiftCooldown = 0.6; } }
    if (inp.gearUp) gear = Math.min(5, gear + 1); if (inp.gearDown) gear = Math.max(-1, gear - 1);
    if (!autoBox && gear === 0) gear = 1;
    if (autoBox && spd < 1.0 && inp.brake > 0.5 && inp.throttle < 0.05) { /* hold */ }
    const running = state ? state[13] : 1;
    // physics steer: + = right in the solver's (left-handed, Unity) frame -> negate our +left convention
    worker.postMessage({ type: 'step', dt: fixedDt, input: { throttle: inp.throttle, brake: inp.brake, steer: -inp.steer, handbrake: inp.handbrake, clutch: inp.clutch, gear, starter: running ? 0 : 1 } });
    inflight = true; simTime += fixedDt; window.__dcs.lastInput = inp;
  }
  if (CFG.warm && frame === CFG.warm) { if (CFG.still) window.__dcs.paused = true; applyTier(tier); } // stills: drive on the cheap tier, then freeze and switch
  if (state) car.update(state);
  updateCamera(dt || 1 / 60);
  // sun shadow: centred on car, texel-snapped, cached (re-rendered only when the car leaves the inner 45 m)
  if (car.centre && sun.castShadow) {
    if (!shadowCache || car.centre.distanceTo(lastShadowCentre) > 45) {
      lastShadowCentre.copy(car.centre); const texel = 220 / sun.shadow.mapSize.x;
      const c = car.centre.clone(); c.x = Math.round(c.x / texel) * texel; c.z = Math.round(c.z / texel) * texel;
      sun.target.position.copy(c); sun.position.copy(c).addScaledVector(sunDir, 400); sun.target.updateMatrixWorld(); sun.updateMatrixWorld();
      sun.shadow.needsUpdate = true; window.__dcs.shadowRenders = (window.__dcs.shadowRenders || 0) + 1;
    }
    if (shadowCache && TIERS[tier].shadows) renderCarShadow();
  }
  sky.position.copy(camera.position);
  impostors.update(car.centre, camera.position);
  const t0 = performance.now();
  pipe.pp.render();
  timings.cpu = performance.now() - t0;
  if (frame % 30 === 0 && renderer.backend.trackTimestamp) { try { await renderer.resolveTimestampsAsync(THREE.TimestampQuery.RENDER); gpuMs = renderer.info.render.timestamp || 0; } catch (e) {} }
  fpsAcc += dt; fpsN++; if (fpsAcc > 0.5) { fps = fpsN / fpsAcc; fpsAcc = 0; fpsN = 0; }
  if (state) {
    $('spd').textContent = (state[10] * 3.6).toFixed(0); $('gear').textContent = gear < 0 ? 'R' : gear === 0 ? 'N' : String(state[12] | 0); $('rpm').style.width = Math.min(100, state[11] / 72) + '%';
    $('stats').textContent = `${backend} | ${fps.toFixed(0)} fps | GPU ${gpuMs ? gpuMs.toFixed(2) + ' ms' : 'n/a'} | CPU submit ${timings.cpu.toFixed(1)} ms | physics ${physInfo.usPerTick ? physInfo.usPerTick.toFixed(0) : '-'} us/tick x ${physInfo.ticks} | shadow ${shadowCache ? 'cached' : 'per-frame'} (${window.__dcs.shadowRenders || 0} renders) | tris ${(renderer.info.render.triangles / 1e3).toFixed(0)}k${impostors.enabled ? ` (far ring ${(impostors.farTris / 1e3 || 0).toFixed(0)}k tris cached, ${impostors.captures} captures)` : ''} | ${input.mode}${auto ? ' | AUTOPILOT' : ''}`;
  }
  window.__dcs.frames = frame; window.__dcs.simTime = simTime; window.__dcs.state = state && Array.from(state.slice(0, 16)); window.__dcs.gpuMs = gpuMs; window.__dcs.backend = backend; window.__dcs.ready = !!state && physReady;
  requestAnimationFrame(frameLoop);
}
$('hud').classList.remove('hide'); requestAnimationFrame(frameLoop);
window.__dcs.applyTier = applyTier; window.__dcs.setCam = (m, o) => { camMode = m; if (o) Object.assign(orbit, o); }; window.__dcs.setSun = (e, a) => { setSun(e, a); captureEnv(); lastShadowCentre.set(1e9, 0, 0); };
window.__dcs.setAuto = (v) => auto = v && !!bot; window.__dcs.cfg = CFG; window.__dcs.worldKind = worldKind; window.__dcs.carKind = () => car.kind; window.__dcs.physVersion = () => physVersion;
