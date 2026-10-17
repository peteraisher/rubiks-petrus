// Maps video time to solver time and works out what each solver is doing
// at any moment. Everything here is a pure function of the exported data,
// so any frame can be rendered on its own.
import { CubeModel, parseSeq } from './cube.js';

export const FPS = 60;
/// Seconds of title before the clocks start.
export const INTRO = 2.5;
/// Least video seconds per move for petrus' first four steps, and for its
/// last-layer lookup, its quickest step (a quick burst: the speed never
/// drops).
const SEC_PER_MOVE_PETRUS = 0.17;
const SEC_PER_MOVE_LOOKUP = 0.04;
/// Petrus' stage lasts at most about this long (so the video stays short).
const PETRUS_STAGE_SEC = 11;
/// After petrus, each speed lasts about this long; a finish less than
/// MERGE_RATIO times later than the stage's first finish shares its speed.
const STAGE_SEC = 4;
const MERGE_RATIO = 5;
/// Speeds are round numbers: 1, 2 or 5 times a power of ten.
const NICE = [1, 2, 5];
/// After the last solver finishes: pause, then the summary card.
export const HOLD = 0.8;
export const SUMMARY_FADE = 0.6;
export const SUMMARY = 5.5;
/// Relative tolerance when comparing summed step times.
const EPS = 1e-9;

/// Half-turn metric weight of a move token: slices 2, rotations 0.
export const htmWeight = (name) => (/^[MES]/.test(name) ? 2 : /^[xyz]/.test(name) ? 0 : 1);

export function buildTimeline(data, nodes) {
  const solvers = data.solvers.map((s, si) => {
    // Step times are measured one step at a time; scale them so they add
    // up to the measured time of the whole solve.
    const sum = s.steps.reduce((a, st) => a + st.ns, 0);
    const scale = s.total_ns / sum;
    let t = 0;
    const model = new CubeModel(data.facelets);
    const snapshots = [model.cubies.map((c) => c.rot)];
    const moves = [];
    let solvedAtStepEnd = [model.solvedIds()];
    const steps = s.steps.map((st, i) => {
      const ms = parseSeq(st.moves);
      const t0 = t;
      t += st.ns * scale;
      const first = moves.length;
      ms.forEach((m, j) => {
        moves.push({ ...m, step: i, t0: t0 + ((t - t0) * j) / ms.length, t1: t0 + ((t - t0) * (j + 1)) / ms.length });
        model.apply(m);
        snapshots.push(model.cubies.map((c) => c.rot));
      });
      solvedAtStepEnd.push(model.solvedIds());
      return {
        name: st.name,
        t0,
        t1: t,
        first,
        count: ms.length,
        htm: ms.reduce((a, m) => a + htmWeight(m.name), 0),
        nodes: nodes?.solvers[si]?.steps[i]?.nodes ?? st.nodes ?? 0,
      };
    });
    if (!model.isSolved()) throw new Error(`${s.id} does not solve the cube`);
    return {
      id: s.id,
      name: s.name,
      method: s.method,
      total: s.total_ns,
      steps,
      moves,
      snapshots,
      solvedAtStepEnd,
      cubies: model.cubies,
      htm: steps.reduce((a, st) => a + st.htm, 0),
      nodes: steps.reduce((a, st) => a + st.nodes, 0),
    };
  });

  // Speed segments: a few stages, each at a round rate, never slowing.
  const segments = [];
  let v = INTRO;
  const push = (t0, t1, rate) => {
    segments.push({ v0: v, v1: v + (t1 - t0) / rate, t0, t1, rate });
    v += (t1 - t0) / rate;
  };
  const petrus = solvers.find((s) => s.id === 'petrus');
  // Petrus at one slow-motion rate, slow enough for its busiest step but
  // no longer than PETRUS_STAGE_SEC in all.
  const last = petrus.steps.length - 1;
  let rate = nice(
    Math.max(
      petrus.total / PETRUS_STAGE_SEC,
      Math.min(...petrus.steps.map((st, i) => (st.t1 - st.t0) / (st.count * (i === last ? SEC_PER_MOVE_LOOKUP : SEC_PER_MOVE_PETRUS)))),
    ),
  );
  push(0, petrus.total, rate);
  // Then one stage per group of finishes.
  const finishes = solvers.map((s) => s.total).filter((t) => t > petrus.total * (1 + EPS)).sort((a, b) => a - b);
  let t = petrus.total;
  for (let i = 0; i < finishes.length; ) {
    let e = finishes[i++];
    while (i < finishes.length && finishes[i] < e * MERGE_RATIO) e = finishes[i++];
    rate = Math.max(rate, nice((e - t) / STAGE_SEC));
    push(t, e, rate);
    t = e;
  }
  const raceEnd = v;
  return { solvers, segments, raceEnd, duration: raceEnd + HOLD + SUMMARY_FADE + SUMMARY };
}

/// The round number (see NICE) nearest to x on a log scale.
function nice(x) {
  const e = Math.floor(Math.log10(x));
  const cands = [e - 1, e, e + 1].flatMap((k) => NICE.map((n) => n * 10 ** k));
  return cands.reduce((a, b) => (Math.abs(Math.log(b / x)) < Math.abs(Math.log(a / x)) ? b : a));
}

/// Solver time (ns) and current rate (ns per video second) at video time v.
export function clock(tl, v) {
  const segs = tl.segments;
  if (v <= segs[0].v0) return { t: 0, rate: segs[0].rate, stage: 0, since: v - segs[0].v0 };
  for (const [i, s] of segs.entries()) {
    if (v <= s.v1) return { t: s.t0 + (v - s.v0) * s.rate, rate: s.rate, stage: i, since: v - s.v0 };
  }
  const last = segs[segs.length - 1];
  return { t: last.t1, rate: 0, stage: segs.length, since: v - last.v1 };
}

/// Video time at which solver time t is reached.
export function videoAt(tl, t) {
  for (const s of tl.segments) if (t <= s.t1) return s.v0 + ((t - s.t0) / (s.t1 - s.t0)) * (s.v1 - s.v0);
  return tl.raceEnd;
}

/// What one solver is doing at solver time t.
export function solverState(s, t) {
  const done = t >= s.total;
  let k = 0;
  while (k < s.moves.length && s.moves[k].t1 <= t) k++;
  const cur = done ? null : s.moves[k];
  const frac = cur && t > cur.t0 ? (t - cur.t0) / (cur.t1 - cur.t0) : 0;
  let step = s.steps.findIndex((st) => t < st.t1);
  if (step < 0) step = s.steps.length;
  const stepsDone = s.steps.slice(0, step);
  const curStep = s.steps[step];
  const stepFrac = curStep ? Math.max(0, (t - curStep.t0) / (curStep.t1 - curStep.t0)) : 0;
  const htm = s.moves.slice(0, k).reduce((a, m) => a + htmWeight(m.name), 0);
  const nodes = stepsDone.reduce((a, st) => a + st.nodes, 0) + (curStep ? Math.floor(curStep.nodes * stepFrac) : 0);
  return { done, k, cur, frac, step, htm, nodes, t: Math.min(t, s.total) };
}
