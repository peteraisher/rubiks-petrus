// Maps video time to solver time and works out what each solver is doing
// at any moment. Everything here is a pure function of the exported data,
// so any frame can be rendered on its own.
import { CubeModel, parseSeq } from './cube.js';

export const FPS = 60;
/// Seconds of title before the clocks start.
export const INTRO = 2.5;
/// Video seconds per move: petrus (slow motion), then the others.
const SEC_PER_MOVE_PETRUS = 0.17;
const SEC_PER_MOVE_RIVAL = 0.08;
/// Shortest stretch of video at one speed.
const MIN_SEGMENT = 0.4;
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

  // Speed segments: (video start, solver start, solver end, video seconds).
  const segments = [];
  let v = INTRO;
  const push = (t0, t1, dur) => {
    segments.push({ v0: v, v1: v + dur, t0, t1 });
    v += dur;
  };
  const petrus = solvers.find((s) => s.id === 'petrus');
  // Petrus' searches at one constant slow-motion rate, chosen so that the
  // busiest step still gets SEC_PER_MOVE_PETRUS per move; the last-layer
  // table lookup is too short for that and gets its own slower rate.
  const search = petrus.steps.slice(0, -1);
  const rate = Math.min(...search.map((st) => (st.t1 - st.t0) / (st.count * SEC_PER_MOVE_PETRUS)));
  const searchEnd = search[search.length - 1].t1;
  push(0, searchEnd, searchEnd / rate);
  const ll = petrus.steps[petrus.steps.length - 1];
  push(ll.t0, ll.t1, ll.count * SEC_PER_MOVE_PETRUS);
  // The rest: one segment between consecutive step ends, long enough for
  // the moves any solver makes in it.
  const ends = [...new Set(solvers.flatMap((s) => s.steps.map((st) => st.t1)))].filter((t) => t > petrus.total * (1 + EPS)).sort((a, b) => a - b);
  let t = petrus.total;
  for (const e of ends) {
    const busiest = Math.max(
      ...solvers.map((s) => s.moves.reduce((a, m) => a + Math.max(0, Math.min(e, m.t1) - Math.max(t, m.t0)) / (m.t1 - m.t0), 0)),
    );
    push(t, e, Math.max(MIN_SEGMENT, busiest * SEC_PER_MOVE_RIVAL));
    t = e;
  }
  const raceEnd = v;
  return { solvers, segments, raceEnd, duration: raceEnd + HOLD + SUMMARY_FADE + SUMMARY };
}

/// Solver time (ns) and current rate (ns per video second) at video time v.
export function clock(tl, v) {
  const segs = tl.segments;
  if (v <= segs[0].v0) return { t: 0, rate: (segs[0].t1 - segs[0].t0) / (segs[0].v1 - segs[0].v0) };
  for (const s of segs) {
    if (v <= s.v1) {
      const rate = (s.t1 - s.t0) / (s.v1 - s.v0);
      return { t: s.t0 + (v - s.v0) * rate, rate };
    }
  }
  const last = segs[segs.length - 1];
  return { t: last.t1, rate: 0 };
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
