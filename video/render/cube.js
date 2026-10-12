// Cubie-level 3x3x3 model for the renderer, independent of three.js so it
// can be tested in Node. Axes: x = R, y = U, z = F. Each cubie keeps its
// starting position and an integer rotation matrix; stickers are
// (start position, start normal, colour letter).

export const FACES = 'URFDLB';
export const STICKERS_PER_FACE = 9;
const N = 3;

// Position and outward normal of facelet r*3+c on each face, in Kociemba
// URFDLB order (U1 at the back left seen from above, etc.).
const FACE_GEOMETRY = {
  U: { n: [0, 1, 0], at: (r, c) => [c - 1, 1, r - 1] },
  R: { n: [1, 0, 0], at: (r, c) => [1, 1 - r, 1 - c] },
  F: { n: [0, 0, 1], at: (r, c) => [c - 1, 1 - r, 1] },
  D: { n: [0, -1, 0], at: (r, c) => [c - 1, -1, 1 - r] },
  L: { n: [-1, 0, 0], at: (r, c) => [-1, 1 - r, c - 1] },
  B: { n: [0, 0, -1], at: (r, c) => [1 - c, 1 - r, -1] },
};

export function faceletGeometry() {
  const out = [];
  for (const f of FACES) {
    const g = FACE_GEOMETRY[f];
    for (let i = 0; i < STICKERS_PER_FACE; i++) out.push({ pos: g.at(Math.floor(i / N), i % N), n: g.n });
  }
  return out;
}

// A turn: rotate the cubies with coordinate `axis` in `layers` by
// `quarters` quarter turns, positive = counter-clockwise looking from +axis.
// Face turns are clockwise looking at the face, so R = -1 quarter about +x.
const AXIS = { x: 0, y: 1, z: 2 };
const TURNS = {
  R: ['x', [1], -1], L: ['x', [-1], 1], M: ['x', [0], 1],
  U: ['y', [1], -1], D: ['y', [-1], 1], E: ['y', [0], 1],
  F: ['z', [1], -1], B: ['z', [-1], 1], S: ['z', [0], -1],
  Rw: ['x', [0, 1], -1], Lw: ['x', [-1, 0], 1],
  Uw: ['y', [0, 1], -1], Dw: ['y', [-1, 0], 1],
  Fw: ['z', [0, 1], -1], Bw: ['z', [-1, 0], 1],
  x: ['x', [-1, 0, 1], -1], y: ['y', [-1, 0, 1], -1], z: ['z', [-1, 0, 1], -1],
};

export function parseMove(tok) {
  const m = /^([A-Z]w?|[MESxyz])(2|'|3|2')?$/.exec(tok);
  if (!m || !TURNS[m[1]]) throw new Error(`bad move ${tok}`);
  const [axis, layers, dir] = TURNS[m[1]];
  const times = m[2] === '2' || m[2] === "2'" ? 2 : m[2] === "'" || m[2] === '3' ? -1 : 1;
  return { name: tok, axis: AXIS[axis], layers, quarters: dir * times, rotation: /^[xyz]$/.test(m[1]) };
}

export const parseSeq = (s) => s.split(/\s+/).filter(Boolean).map(parseMove);

// Integer rotation matrix (row major, 3x3) for q quarter turns about axis a.
export function quarterMatrix(a, q) {
  const k = ((q % 4) + 4) % 4;
  const c = [1, 0, -1, 0][k], s = [0, 1, 0, -1][k];
  if (a === 0) return [1, 0, 0, 0, c, -s, 0, s, c];
  if (a === 1) return [c, 0, s, 0, 1, 0, -s, 0, c];
  return [c, -s, 0, s, c, 0, 0, 0, 1];
}

export const mulMat = (A, B) => {
  const C = new Array(9).fill(0);
  for (let i = 0; i < N; i++) for (let j = 0; j < N; j++) for (let k = 0; k < N; k++) C[i * N + j] += A[i * N + k] * B[k * N + j];
  return C;
};
export const mulVec = (A, v) => [0, 1, 2].map((i) => A[i * N] * v[0] + A[i * N + 1] * v[1] + A[i * N + 2] * v[2]);
const IDENTITY = [1, 0, 0, 0, 1, 0, 0, 0, 1];
// Where each colour's face points in the solved cube.
const HOME_DIR = { U: [0, 1, 0], R: [1, 0, 0], F: [0, 0, 1], D: [0, -1, 0], L: [-1, 0, 0], B: [0, 0, -1] };
const key = (v) => v.join(',');

export class CubeModel {
  // facelets: 54 letters URFDLB (Kociemba order).
  constructor(facelets) {
    this.cubies = [];
    const byPos = new Map();
    for (let x = -1; x <= 1; x++)
      for (let y = -1; y <= 1; y++)
        for (let z = -1; z <= 1; z++) {
          if (!x && !y && !z) continue;
          const c = { id: this.cubies.length, home: [x, y, z], rot: IDENTITY, stickers: [] };
          this.cubies.push(c);
          byPos.set(key(c.home), c);
        }
    faceletGeometry().forEach((g, i) => byPos.get(key(g.pos)).stickers.push({ n: g.n, color: facelets[i] }));
    // Whole-cube rotation so far (x, y, z): the frame "solved" refers to.
    this.frame = IDENTITY;
  }

  clone() {
    const m = Object.create(CubeModel.prototype);
    m.cubies = this.cubies.map((c) => ({ ...c }));
    m.frame = this.frame;
    return m;
  }

  pos(c) {
    return mulVec(c.rot, c.home);
  }

  inLayer(c, mv) {
    return mv.layers.includes(this.pos(c)[mv.axis]);
  }

  apply(mv) {
    const R = quarterMatrix(mv.axis, mv.quarters);
    for (const c of this.cubies) if (this.inLayer(c, mv)) c.rot = mulMat(R, c.rot);
    if (mv.rotation) this.frame = mulMat(R, this.frame);
    return this;
  }

  applySeq(moves) {
    for (const m of moves) this.apply(m);
    return this;
  }

  // Colour of the sticker at a position facing a direction, or null.
  colorAt(pos, n) {
    for (const c of this.cubies) {
      if (key(this.pos(c)) !== key(pos)) continue;
      for (const s of c.stickers) if (key(mulVec(c.rot, s.n)) === key(n)) return s.color;
    }
    return null;
  }

  facelets() {
    return faceletGeometry().map((g) => this.colorAt(g.pos, g.n)).join('');
  }

  // A cubie (centres included) is solved when each sticker faces where its
  // colour belongs, allowing for whole-cube rotations. Measured against
  // the cube's frame rather than the centres, since Roux moves the centres
  // with M slices while its blocks are already solved.
  isCubieSolved(c) {
    return c.stickers.every((s) => key(mulVec(c.rot, s.n)) === key(mulVec(this.frame, HOME_DIR[s.color])));
  }

  solvedIds() {
    return new Set(this.cubies.filter((c) => this.isCubieSolved(c)).map((c) => c.id));
  }

  isSolved() {
    return this.cubies.every((c) => this.isCubieSolved(c));
  }
}
