// Checks the renderer's cube model against the Rust solver: petrus' cube
// after every step must match the exported facelets, and every solver's
// moves must solve the scramble.  node test.mjs
import { readFileSync } from 'node:fs';
import { CubeModel, parseSeq } from './cube.js';

const data = JSON.parse(readFileSync(new URL('./data/solve.json', import.meta.url)));
let failed = 0;
const check = (ok, what) => {
  console.log(`${ok ? 'ok  ' : 'FAIL'} ${what}`);
  if (!ok) failed++;
};

const SOLVED = 'UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB';
check(new CubeModel(SOLVED).facelets() === SOLVED, 'facelet round trip');
check(new CubeModel(data.facelets).facelets() === data.facelets, 'scramble round trip');
// R U R' U' has order 6; superflip-free sanity on slices and rotations
const order = (seq) => {
  const m = new CubeModel(SOLVED);
  for (let i = 1; i <= 1260; i++) if (m.applySeq(parseSeq(seq)).facelets() === SOLVED) return i;
  return -1;
};
check(order("R U R' U'") === 6, 'sexy move has order 6');
check(order('M2 E2 S2') === 2, 'M2 E2 S2 has order 2');
check(order("Rw L'") > 0 && new CubeModel(SOLVED).applySeq(parseSeq("Rw L' x'")).facelets() === SOLVED, "Rw L' x' is the identity");
check(new CubeModel(SOLVED).applySeq(parseSeq("x y z")).isSolved(), 'rotations keep the cube solved');

for (const s of data.solvers) {
  const m = new CubeModel(data.facelets);
  for (const st of s.steps) {
    m.applySeq(parseSeq(st.moves));
    if (st.facelets) check(m.facelets() === st.facelets, `${s.id}: ${st.name} matches Rust`);
  }
  check(m.isSolved(), `${s.id} (${s.method}) solves the scramble`);
}
process.exit(failed ? 1 : 0);
