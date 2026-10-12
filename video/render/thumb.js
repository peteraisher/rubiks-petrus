// Thumbnail for the video post: the headline time, petrus' cube with its
// first block solved and lit, and the four finishing times.
import * as THREE from 'three';
import { buildCubies, mat4 } from './cubemesh.js';
import { buildTimeline } from './timeline.js';

const W = 1080;
const H = 560;
const SUPERSAMPLE = 2;
/// Petrus state shown: after this many steps (1 = 2x2x2 block).
const STEPS_DONE = 1;
const DIM = 0.12;
/// Camera azimuth/elevation (degrees) that faces the block at DBL.
const VIEW = [-138, -30];
const CAM_DIST = 9.6;
const CAM_FOV = 30;
const DEG = Math.PI / 180;
const LABELS = { petrus: 'Petrus', min2phase: 'Kociemba two-phase', cfop: 'CFOP', roux: 'Roux' };

function human(ns) {
  const sig3 = (x) => x.toLocaleString('en-US', { maximumSignificantDigits: 3 });
  if (ns < 1e3) return [`${Math.round(ns)}`, 'ns'];
  if (ns < 1e6) return [sig3(ns / 1e3), 'µs'];
  return [sig3(ns / 1e6), 'ms'];
}

async function main() {
  const [data, nodes] = await Promise.all(['data/solve.json', 'data/nodes.json'].map((f) => fetch(f).then((r) => r.json())));
  const tl = buildTimeline(data, nodes);
  const petrus = tl.solvers.find((s) => s.id === 'petrus');
  const [num, unit] = human(petrus.total);
  document.getElementById('big').innerHTML = `${num}<span> ${unit}</span>`;
  document.getElementById('ladder').innerHTML = [...tl.solvers]
    .sort((a, b) => a.total - b.total)
    .map((s) => `<div class="row"><b>${LABELS[s.id]}</b><span>${human(s.total).join(' ')}</span></div>`)
    .join('');

  const renderer = new THREE.WebGLRenderer({ canvas: document.getElementById('c'), antialias: true, alpha: true, preserveDrawingBuffer: true });
  renderer.setPixelRatio(SUPERSAMPLE);
  renderer.setSize(W, H, false);
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(CAM_FOV, W / H, 0.1, 100);
  scene.add(new THREE.HemisphereLight(0xffffff, 0x404650, 1.6));
  const key = new THREE.DirectionalLight(0xffffff, 1.8);
  key.position.set(2, 3, 4);
  camera.add(key);
  scene.add(camera);

  const groups = buildCubies(scene, petrus.cubies);
  const k = petrus.steps[STEPS_DONE - 1].first + petrus.steps[STEPS_DONE - 1].count;
  const lit = petrus.solvedAtStepEnd[STEPS_DONE];
  petrus.cubies.forEach((c, i) => {
    groups[i].matrix.copy(mat4(petrus.snapshots[k][i]));
    groups[i].matrixWorldNeedsUpdate = true;
    for (const m of groups[i].mats) m.color.copy(m.base).multiplyScalar(lit.has(c.id) ? 1 : DIM);
  });
  const [az, el] = VIEW;
  camera.position.set(Math.sin(az * DEG) * Math.cos(el * DEG), Math.sin(el * DEG), Math.cos(az * DEG) * Math.cos(el * DEG)).multiplyScalar(CAM_DIST);
  camera.lookAt(0, 0, 0);
  renderer.render(scene, camera);
  window.ready = true;
}

main().catch((e) => {
  window.error = String(e.stack || e);
  throw e;
});
