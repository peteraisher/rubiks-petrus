// Thumbnail for the video post: the first version's headline time struck
// through, the new one (thistle's), thistle's cube in the squares group
// (after three of its four phases), and the four finishing times.
import * as THREE from 'three';
import { buildCubies, mat4 } from './cubemesh.js';
import { buildTimeline } from './timeline.js';

const W = 1080;
const H = 470;
const SUPERSAMPLE = 2;
/// The solver shown, and its state: after this many steps (3 = in the
/// squares group, only half turns left).
const HERO = 'thistle';
const STEPS_DONE = 3;
/// What the first published version of the video claimed.
const PUBLISHED_NS = 2209;
/// Camera azimuth/elevation (degrees).
const VIEW = [35, 28];
const CAM_DIST = 9.6;
const CAM_FOV = 30;
const DEG = Math.PI / 180;
const LABELS = { thistle: 'Thistlethwaite', petrus: 'Petrus', min2phase: 'Kociemba two-phase', cfop: 'CFOP' };

function human(ns) {
  const sig3 = (x) => x.toLocaleString('en-US', { maximumSignificantDigits: 3 });
  if (ns < 1e3) return [`${Math.round(ns)}`, 'ns'];
  if (ns < 1e6) return [sig3(ns / 1e3), 'µs'];
  return [sig3(ns / 1e6), 'ms'];
}

async function main() {
  const [data, nodes] = await Promise.all(['data/solve.json', 'data/nodes.json'].map((f) => fetch(f).then((r) => r.json())));
  const tl = buildTimeline(data, nodes);
  const hero = tl.solvers.find((s) => s.id === HERO);
  const [num, unit] = human(hero.total);
  document.getElementById('was').innerHTML = `${human(PUBLISHED_NS).join(' ')}<i></i>`;
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

  const groups = buildCubies(scene, hero.cubies);
  const k = hero.steps[STEPS_DONE - 1].first + hero.steps[STEPS_DONE - 1].count;
  hero.cubies.forEach((c, i) => {
    groups[i].matrix.copy(mat4(hero.snapshots[k][i]));
    groups[i].matrixWorldNeedsUpdate = true;
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
