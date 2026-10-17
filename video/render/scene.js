// Draws one frame of the video: four cubes, clocks and counters, all as a
// pure function of the frame number (window.renderFrame) so the capture
// script can step through frames at its own pace.
import * as THREE from 'three';
import { buildCubies, mat4 } from './cubemesh.js';
import { quarterMatrix, mulMat } from './cube.js';
import { buildTimeline, clock, solverState, videoAt, FPS, INTRO, HOLD, SUMMARY_FADE } from './timeline.js';

const CANVAS_W = 464;
const CANVAS_H = 318;
const SUPERSAMPLE = 2;
/// Brightness of pieces a solver has not solved yet.
const DIM = 0.22;
const DIM_IN = 0.6;
/// Newly solved pieces brighten over this long and glow for FLASH.
const BRIGHTEN = 0.25;
const FLASH = 0.7;
const FLASH_GLOW = 0.55;
const CAM_EASE = 0.9;
const CAM_DIST = 10.8;
const CAM_FOV = 25;
const SWAY_DEG = 6;
const SWAY_PERIOD = 8;
const HOME_VIEW = [35, 28];
/// Camera (azimuth, elevation) in degrees for each step, chosen so the
/// pieces being solved face the viewer: blocks from below, last layer from
/// above.
const VIEWS = {
  petrus: [[-140, -26], [-75, -26], [-25, 24], [40, -22], [28, 38]],
  min2phase: [[-20, 26]],
  cfop: [[25, -38], [40, -18], [25, 40], [30, 32]],
  roux: [[-60, -10], [60, -10], [25, 40], [0, 30]],
};
const LABELS = {
  petrus: ['Petrus', 'this project · Rust'],
  min2phase: ['min2phase*', 'Kociemba two-phase'],
  cfop: ['CFOP', 'rcuber'],
  roux: ['Roux', 'rcuber'],
};
const DEG = Math.PI / 180;
const NS_PER_S = 1e9;

const smoothstep = (x) => (x <= 0 ? 0 : x >= 1 ? 1 : x * x * (3 - 2 * x));
const lerp = (a, b, w) => a + (b - a) * w;
const comma = (n) => Math.round(n).toLocaleString('en-US');
// Plain numbers only: at most three significant digits, never exponents.
const sig3 = (x) => x.toLocaleString('en-US', { maximumSignificantDigits: 3 });
function human(ns) {
  if (ns < 1e3) return `${Math.round(ns)} ns`;
  if (ns < 1e6) return `${sig3(ns / 1e3)} µs`;
  if (ns < 1e9) return `${sig3(ns / 1e6)} ms`;
  return `${sig3(ns / 1e9)} s`;
}
const times = (x) => `${comma(x)}×`;
/// The HUD appears at the start, at each speed change and at the finish,
/// stays for HUD_HOLD seconds and fades.
const HUD_FIRST = 0.4;
const HUD_HOLD = 2.2;
const HUD_FADE_IN = 0.2;
const HUD_FADE_OUT = 0.4;
const HUD_POP = 0.08;


class Panel {
  constructor(tl, s, grid) {
    this.tl = tl;
    this.s = s;
    const el = document.createElement('div');
    el.className = 'panel' + (s.id === 'petrus' ? ' hero' : '');
    const [name, sub] = LABELS[s.id];
    el.innerHTML = `<div class="name"><b>${name}</b><span>${sub}</span></div><div class="badge"></div>
      <canvas></canvas><div class="step"></div><div class="timer"></div>
      <div class="stats"><span>moves <b class="moves"></b></span><span>nodes <b class="nodes"></b></span></div>
      <div class="bar">${s.steps.map(() => '<div><i></i></div>').join('')}</div>`;
    grid.appendChild(el);
    this.el = el;
    this.q = (sel) => el.querySelector(sel);
    // Bar segments in proportion to each step's measured time, but wide
    // enough to see.
    const total = s.steps.reduce((a, st) => a + (st.t1 - st.t0), 0);
    this.bars = [...el.querySelectorAll('.bar div')];
    this.bars.forEach((b, i) => (b.style.flexGrow = Math.max(0.012, (s.steps[i].t1 - s.steps[i].t0) / total)));

    const canvas = this.q('canvas');
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: true });
    this.renderer.setPixelRatio(SUPERSAMPLE);
    this.renderer.setSize(CANVAS_W, CANVAS_H, false);
    this.renderer.setClearColor(0x121820);
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(CAM_FOV, CANVAS_W / CANVAS_H, 0.1, 100);
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x404650, 1.6));
    const key = new THREE.DirectionalLight(0xffffff, 1.6);
    key.position.set(2, 3, 4);
    this.camera.add(key);
    this.scene.add(this.camera);

    this.groups = buildCubies(this.scene, s.cubies);
    // Video times at which each step starts and ends.
    this.stepStart = s.steps.map((st) => videoAt(tl, st.t0));
    this.stepEnd = s.steps.map((st) => videoAt(tl, st.t1));
  }

  cameraAt(v, st) {
    let [az, el] = HOME_VIEW;
    const keys = VIEWS[this.s.id].map((view, i) => [this.stepStart[i] - (i === 0 ? CAM_EASE : CAM_EASE / 2), view]);
    keys.push([this.stepEnd[this.stepEnd.length - 1], HOME_VIEW]);
    for (const [kv, [kaz, kel]] of keys) {
      const w = smoothstep((v - kv) / CAM_EASE);
      az = lerp(az, kaz, w);
      el = lerp(el, kel, w);
    }
    az += SWAY_DEG * Math.sin((2 * Math.PI * v) / SWAY_PERIOD);
    const c = this.camera;
    c.position.set(Math.sin(az * DEG) * Math.cos(el * DEG), Math.sin(el * DEG), Math.cos(az * DEG) * Math.cos(el * DEG)).multiplyScalar(CAM_DIST);
    c.lookAt(0, 0, 0);
  }

  draw(v, t, order) {
    const s = this.s;
    const st = solverState(s, t);
    // Cubies: state after st.k moves, plus the move in progress.
    const rots = s.snapshots[st.k];
    let partial = null;
    if (st.cur && st.frac > 0) {
      const m = st.cur;
      const angle = m.quarters * (Math.PI / 2) * smoothstep(st.frac);
      const axis = new THREE.Vector3(m.axis === 0 ? 1 : 0, m.axis === 1 ? 1 : 0, m.axis === 2 ? 1 : 0);
      partial = { m, rot: new THREE.Matrix4().makeRotationAxis(axis, angle) };
    }
    // Which pieces are lit: the ones solved when the last step finished.
    const racing = v >= INTRO;
    const dimIn = smoothstep((v - INTRO) / DIM_IN);
    const lit = s.solvedAtStepEnd[st.step];
    const prev = st.step > 0 ? s.solvedAtStepEnd[st.step - 1] : null;
    const since = st.step > 0 ? v - this.stepEnd[st.step - 1] : Infinity;
    s.cubies.forEach((c, i) => {
      const rot = rots[i];
      const M = mat4(rot);
      if (partial) {
        const p = [0, 1, 2].map((r) => rot[r * 3] * c.home[0] + rot[r * 3 + 1] * c.home[1] + rot[r * 3 + 2] * c.home[2]);
        if (partial.m.layers.includes(p[partial.m.axis])) M.premultiply(partial.rot);
      }
      const g = this.groups[i];
      g.matrix.copy(M);
      g.matrixWorldNeedsUpdate = true;
      let b = 1, glow = 0;
      if (racing && !st.done) {
        if (lit.has(c.id)) {
          const fresh = prev && !prev.has(c.id);
          if (fresh) {
            b = lerp(DIM, 1, smoothstep(since / BRIGHTEN));
            glow = FLASH_GLOW * (1 - smoothstep(since / FLASH));
          }
        } else b = lerp(1, DIM, dimIn);
      } else if (st.done) {
        const fresh = !s.solvedAtStepEnd[s.steps.length - 1].has(c.id);
        if (fresh) {
          b = lerp(DIM, 1, smoothstep(since / BRIGHTEN));
          glow = FLASH_GLOW * (1 - smoothstep(since / FLASH));
        }
      }
      for (const m of g.mats) {
        m.color.copy(m.base).multiplyScalar(b);
        m.emissive.copy(m.base).multiplyScalar(glow);
      }
    });
    this.cameraAt(v, st);
    this.renderer.render(this.scene, this.camera);

    // Text.
    const n = s.steps.length;
    const stepEl = this.q('.step');
    if (!racing) stepEl.innerHTML = '<span class="n">Ready</span>';
    else if (st.done) stepEl.innerHTML = 'Solved';
    else if (n === 1) stepEl.innerHTML = s.steps[0].name;
    else stepEl.innerHTML = `<span class="n">Step ${st.step + 1}/${n} ·</span> ${s.steps[st.step].name}`;
    this.q('.timer').innerHTML = `${comma(st.t)}<small>ns</small>`;
    this.q('.moves').textContent = st.htm;
    this.q('.nodes').textContent = comma(st.nodes);
    this.el.classList.toggle('finished', st.done);
    const rank = order.indexOf(s.id) + 1;
    this.q('.badge').textContent = `#${rank} ✓`;
    this.bars.forEach((b, i) => {
      const step = s.steps[i];
      const f = Math.max(0, Math.min(1, (t - step.t0) / (step.t1 - step.t0)));
      b.querySelector('i').style.width = `${f * 100}%`;
      b.classList.toggle('cur', i === st.step && racing && !st.done);
    });
  }
}

async function main() {
  const [data, nodes] = await Promise.all(['data/solve.json', 'data/nodes.json'].map((f) => fetch(f).then((r) => r.json())));
  const tl = buildTimeline(data, nodes);
  const by = Object.fromEntries(tl.solvers.map((s) => [s.id, s]));
  const order = [...tl.solvers].sort((a, b) => a.total - b.total).map((s) => s.id);
  const petrus = by.petrus;
  const grid = document.getElementById('grid');
  const panels = ['petrus', 'min2phase', 'cfop', 'roux'].map((id) => new Panel(tl, by[id], grid));

  document.getElementById('title').innerHTML = `Solving a Rubik's Cube in <em>${human(petrus.total)}</em>`;
  document.getElementById('sub').textContent = `Same scramble, four solvers, real measured times · ${data.cpu}, one thread`;
  document.getElementById('footer').innerHTML =
    `<b>*</b> min2phase (Kociemba's two-phase algorithm) finds much shorter solutions but is a computer method, not one a person can follow. ` +
    `CFOP and Roux: rcuber 0.7.20; min2phase 0.2.4 (Rust crates). Times: median of single solves of this scramble, each after 2 ms of solving other cubes. ` +
    `Nodes: search states generated; for petrus, table lookups.`;

  const rows = order.map((id) => {
    const s = by[id];
    const [name, sub] = LABELS[id];
    return `<tr class="${id === 'petrus' ? 'hero' : ''}"><td>${name}<span>${s.method === name ? sub : s.method}</span></td>` +
      `<td>${human(s.total)}</td><td>${s.htm}</td><td>${comma(s.nodes)}</td></tr>`;
  });
  const rival = by[order[1]];
  document.getElementById('summary').innerHTML =
    `<h2>Petrus: <em>${comma(rival.total / petrus.total)}× faster</em><br>than the next-fastest solver tested</h2>` +
    `<table><tr><th>SOLVER</th><th>TIME</th><th>MOVES</th><th>NODES</th></tr>${rows.join('')}</table>` +
    `<div class="note"><b>petrus</b> follows Lars Petrus' method: 2x2x2 block, 2x2x3 block, edge orientation, ` +
    `first two layers with R and U only, then the whole last layer in one look. Each step's optimal solution ` +
    `is one lookup in tables built at compile time, with cube moves as ARM NEON table lookups.</div>` +
    `<div class="note">* min2phase's ${by.min2phase.htm} moves are far fewer than any human method, at ${comma(by.min2phase.total / petrus.total)}× the time.</div>`;

  const hud = document.getElementById('hud');
  const el = (id) => document.getElementById(id);
  el('hudDots').innerHTML = tl.segments.map(() => '<i></i>').join('');
  const dots = [...el('hudDots').children];
  const realTotal = tl.segments[tl.segments.length - 1].t1;

  window.frameCount = Math.ceil(tl.duration * FPS);
  window.renderFrame = (i) => {
    const v = i / FPS;
    const { t, rate, stage } = clock(tl, v);
    for (const p of panels) p.draw(v, t, order);
    dots.forEach((d, k) => d.classList.toggle('on', k <= stage && v >= INTRO));
    // Visible around each event: title (until the start), start, every
    // speed change, finish.
    const events = [[HUD_FIRST, INTRO - HUD_FIRST + HUD_HOLD], ...tl.segments.slice(1).map((sg) => [sg.v0, HUD_HOLD]), [tl.raceEnd, HUD_HOLD]];
    let show = 0, pop = 0;
    for (const [ev, hold] of events) {
      if (v < ev) continue;
      const a = smoothstep((v - ev) / HUD_FADE_IN) * (1 - smoothstep((v - ev - hold) / HUD_FADE_OUT));
      if (a > show) [show, pop] = [a, 1 - smoothstep((v - ev) / (HUD_FADE_IN * 2))];
    }
    if (v < INTRO) {
      el('hudIcon').textContent = '❚❚';
      el('hudLabel').textContent = 'READY';
      el('hudBig').textContent = times(NS_PER_S / rate);
      el('hudSmall').textContent = 'slower than real time';
    } else if (v < tl.raceEnd) {
      const sped = stage > 0;
      el('hudIcon').textContent = sped ? '▶▶' : '▶';
      el('hudLabel').textContent = sped ? 'SPEEDING UP' : 'SLOW MOTION';
      el('hudBig').textContent = times(NS_PER_S / rate);
      el('hudSmall').textContent = `slower · 1 s = ${human(rate)}`;
    } else {
      el('hudIcon').textContent = '■';
      el('hudLabel').textContent = 'FINISHED';
      el('hudBig').textContent = human(realTotal);
      el('hudSmall').textContent = 'of real time for all four';
    }
    hud.style.opacity = show;
    hud.style.transform = `translate(-50%, -50%) scale(${1 - HUD_POP * pop})`;
    summary.style.opacity = smoothstep((v - tl.raceEnd - HOLD) / SUMMARY_FADE);
  };
  window.timeline = { duration: tl.duration, raceEnd: tl.raceEnd };
  window.renderFrame(0);
  window.ready = true;
}

main().catch((e) => {
  window.error = String(e.stack || e);
  throw e;
});
