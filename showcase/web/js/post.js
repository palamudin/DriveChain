// post.js - tiered render pipeline built from three r187 TSL nodes.
// Order follows the Threat Interactive handbook: linear HDR composite -> tonemap (renderOutput) -> spatial AA (SMAA,
// no temporal accumulation) -> display-space lens effects (CA, grain, vignette) so grain/CA never get smeared by AA.
import * as THREE from 'three/webgpu';
import { saturation, pass, mrt, output, normalView, diffuseColor, packNormalToRGB, unpackRGBToNormal, sample, vec4, vec3, vec2, float, uniform, renderOutput, uv, mix, smoothstep, length, add, Fn, texture } from 'three/tsl';
import { ssgi } from 'three/addons/tsl/display/SSGINode.js';
import { bloom } from 'three/addons/tsl/display/BloomNode.js';
import { smaa } from 'three/addons/tsl/display/SMAANode.js';
import { fxaa } from 'three/addons/tsl/display/FXAANode.js';
import { film } from 'three/addons/tsl/display/FilmNode.js';
import { chromaticAberration } from 'three/addons/tsl/display/ChromaticAberrationNode.js';
import { ao as gtao } from 'three/addons/tsl/display/GTAONode.js';
import { godrays } from 'three/addons/tsl/display/GodraysNode.js';
import { lensDirt } from 'three/addons/tsl/display/lensDirt.js';

export const TIERS = {
  raw:      { label: 'RAW (old DriveChain demo look)', materials: 'raw', shadows: false, sky: false, post: false },
  base:     { label: 'BASE', materials: 'showcase', shadows: true, shadowSize: 2048, sky: true, post: true, gi: 'off', aa: 'smaa', bloom: true, lens: false },
  high:     { label: 'HIGH (half-res SSGI)', materials: 'showcase', shadows: true, shadowSize: 4096, sky: true, post: true, gi: 'half', aa: 'smaa', bloom: true, lens: true, impostors: true },
  showcase: { label: 'SHOWCASE (full-res SSGI + godrays)', materials: 'showcase', shadows: true, shadowSize: 4096, sky: true, post: true, gi: 'full', aa: 'smaa', bloom: true, lens: true, godrays: true, impostors: true },
};

export const postParams = {
  exposure: uniform(1.0), grain: uniform(0.035), ca: uniform(0.12), vignette: uniform(0.32), bloomStrength: uniform(0.14), dirt: uniform(0.35), sat: uniform(1.12),
  giIntensity: +(new URLSearchParams(location.search).get('gi') ?? 1.2), aoIntensity: 1.0,
};

function makeDirtTexture() { // own procedural lens-dirt (publishable)
  const c = document.createElement('canvas'); c.width = c.height = 512; const g = c.getContext('2d');
  g.fillStyle = '#000'; g.fillRect(0, 0, 512, 512); let seed = 7; const r = () => (seed = (seed * 16807) % 2147483647) / 2147483647;
  for (let i = 0; i < 260; i++) { const x = r() * 512, y = r() * 512, rad = 3 + r() * r() * 38, a = 0.03 + r() * 0.12;
    const gr = g.createRadialGradient(x, y, 0, x, y, rad); gr.addColorStop(0, `rgba(255,250,240,${a})`); gr.addColorStop(1, 'rgba(255,250,240,0)'); g.fillStyle = gr; g.beginPath(); g.arc(x, y, rad, 0, 7); g.fill(); }
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; return t;
}

export function buildPipeline(renderer, scene, camera, tier, sun) {
  const T = TIERS[tier];
  const pp = new THREE.RenderPipeline(renderer);
  pp.outputColorTransform = false; // we place renderOutput() ourselves (tonemap before AA)
  const nodes = {};
  if (!T.post) { pp.outputColorTransform = true; pp.outputNode = pass(scene, camera); return { pp, nodes }; }
  const scenePass = pass(scene, camera, { samples: 0 });
  const wantGI = T.gi && T.gi !== 'off';
  if (wantGI) scenePass.setMRT(mrt({ output, diffuseColor, normal: packNormalToRGB(normalView) }));
  const color = scenePass.getTextureNode('output');
  const depth = scenePass.getTextureNode('depth');
  let hdr = color;
  if (wantGI) {
    const diffuse = scenePass.getTextureNode('diffuseColor');
    const normalTex = scenePass.getTextureNode('normal');
    const sceneNormal = sample((u) => unpackRGBToNormal(normalTex.sample(u)));
    const gi = ssgi(color, depth, sceneNormal, camera);
    gi.resolutionScale = T.gi === 'half' ? 0.5 : 1.0;  // half-res + depth-aware centre sampling is built into SSGINode
    gi.useTemporalFiltering = false;                     // handbook: no TAA dependence -> spatial-only, more slices instead
    gi.sliceCount.value = T.gi === 'half' ? 2 : 3; gi.stepCount.value = T.gi === 'half' ? 8 : 12;
    gi.radius.value = 6; gi.giIntensity.value = postParams.giIntensity; gi.aoIntensity.value = postParams.aoIntensity; gi.thickness.value = 0.6;
    nodes.gi = gi;
    const giRGB = gi.rgb, aoA = gi.a;
    hdr = vec4(add(color.rgb.mul(aoA), diffuse.rgb.mul(giRGB)), color.a);
  }
  if (T.godrays && sun && new URLSearchParams(location.search).get('godrays') !== '0') {
    try { const gr = godrays(depth, camera, sun); nodes.godrays = gr; hdr = vec4(hdr.rgb.add(gr.rgb.mul(vec3(1.0, 0.93, 0.82)).mul(0.35)), hdr.a); } catch (e) { console.warn('godrays unavailable', e); }
  }
  if (T.bloom) { const b = bloom(hdr, postParams.bloomStrength.value, 0.4, 0.95); nodes.bloom = b; hdr = vec4(hdr.rgb.add(b.rgb), hdr.a);
    if (T.lens) { const dirtTex = makeDirtTexture(); hdr = vec4(hdr.rgb.add(lensDirt(b, texture(dirtTex), postParams.dirt).rgb), hdr.a); } }
  let ldr = renderOutput(hdr);                       // tonemap + sRGB here
  ldr = vec4(saturation(ldr.rgb, postParams.sat), 1);  // light grade in display space
  if (T.aa === 'smaa') ldr = smaa(ldr); else if (T.aa === 'fxaa') ldr = fxaa(ldr);
  if (T.lens) ldr = chromaticAberration(ldr, postParams.ca, vec2(0.5, 0.5), 1.0);
  const vig = Fn(() => { const d = length(uv().sub(0.5)); return float(1).sub(smoothstep(0.35, 0.95, d).mul(postParams.vignette)); })();
  ldr = vec4(ldr.rgb.mul(vig), 1);
  ldr = film(ldr, postParams.grain);
  pp.outputNode = ldr; nodes.scenePass = scenePass;
  return { pp, nodes };
}
