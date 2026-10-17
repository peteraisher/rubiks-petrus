//! The single chain (`fast`, `fast5`, `fast6`): Thistlethwaite with each
//! phase's first stored algorithm, one table lookup per phase (or per half
//! of a split phase), written so that no branch depends on the cube.

use crate::algs::{alg_moves, apply_padded, pad_identity, pairs};
use crate::coord::*;
use crate::merge::{append_lanes, Out};
use crate::solver::Solution;
use crate::tables::Tables;
use petrus::cube::*;

/// Phase `P`: look up the case's first algorithm, take it and append it
/// to the solution. Returns its length.
#[inline(always)]
fn phase<const P: usize>(t: &Tables, x: &mut Cube, out: &mut Out) -> u8 {
    join::<P>(phase_step::<P>(t, x), out)
}

/// Half `H` of split phase `P` (see [`N_HALVES`]), as `phase`.
#[inline(always)]
fn half<const H: usize, const P: usize>(t: &Tables, x: &mut Cube, out: &mut Out) -> u8 {
    join::<P>(half_step::<H, P>(t, x), out)
}

/// Append phase `P`'s step to the solution, merging turns across the
/// boundary (the first phase starts it).
#[inline(always)]
fn join<const P: usize>(s: Step, out: &mut Out) -> u8 {
    if P == 0 {
        out.start(s.moves, s.n);
    } else {
        out.append(s.moves, s.n);
    }
    s.n as u8
}

/// Phase `P` of the chain on its own: its moves and the cube after them.
fn lone_phase<const P: usize, const PAIRS: usize>(t: &Tables, c: &Cube) -> (Moves<LANES>, Cube) {
    let (r, sym) = t.row_of::<P>(t.index::<P>(c));
    let a = t.first::<P>(r);
    let n = alg_len(a);
    let moves = pad_identity(alg_moves::<P>(a, sym), n);
    (Moves::from_array(moves, n), apply_padded::<PAIRS>(c, &moves))
}

/// Phase `p` (0-based) of `fast`'s chain on its own, for tools that time
/// phases separately: the case's first stored algorithm and the cube after
/// it. `c` must be in the phase's starting group (through phases `..p`).
pub fn chain_phase(t: &Tables, p: usize, c: &Cube) -> (Moves<LANES>, Cube) {
    match p {
        0 => lone_phase::<0, { pairs(PHASE_MAX_LEN[0]) }>(t, c),
        1 => lone_phase::<1, { pairs(PHASE_MAX_LEN[1]) }>(t, c),
        2 => lone_phase::<2, { pairs(PHASE_MAX_LEN[2]) }>(t, c),
        _ => lone_phase::<LAST, { pairs(PHASE_MAX_LEN[LAST]) }>(t, c),
    }
}

/// The chain with phase 2 and/or 4 as two halves (`S2`, `S4`).
#[inline(never)]
pub(crate) fn solve_chain<const S2: bool, const S4: bool>(t: &Tables, cube: &Cube) -> Solution {
    let mut x = *cube;
    let mut out = Out::new();
    let p1 = phase::<0>(t, &mut x, &mut out);
    let p2 = if S2 {
        half::<HALF_SLICE, PHASE_2>(t, &mut x, &mut out) + half::<HALF_TWIST, PHASE_2>(t, &mut x, &mut out)
    } else {
        phase::<PHASE_2>(t, &mut x, &mut out)
    };
    let p3 = phase::<2>(t, &mut x, &mut out);
    let p4 = if S4 {
        half::<HALF_CORNERS, LAST>(t, &mut x, &mut out) + half::<HALF_EDGES, LAST>(t, &mut x, &mut out)
    } else {
        phase::<LAST>(t, &mut x, &mut out)
    };
    let evals = N_PHASES as u32 + S2 as u32 + S4 as u32;
    Solution { moves: out.moves(), phase_lens: [p1, p2, p3, p4], start: 0, evals }
}

/// Phase `P`'s algorithm `a` (symmetry `sym`) as moves padded with the
/// identity, and its length; applied as `PAIRS` move pairs, or not at all
/// for `PAIRS` = 0 (the last phase: nothing reads the solved cube).
#[inline(always)]
fn take<const P: usize, const PAIRS: usize>(x: &mut Cube, a: u64, sym: usize) -> Step {
    let n = alg_len(a);
    debug_assert!(PAIRS == 0 || n <= 2 * PAIRS);
    let moves = pad_identity(alg_moves::<P>(a, sym), n);
    if PAIRS > 0 {
        *x = apply_padded::<PAIRS>(x, &moves);
    }
    Step { moves, n }
}

/// One cube's algorithm for a phase (or half).
#[derive(Clone, Copy)]
struct Step {
    moves: [u8; LANES],
    n: usize,
}

/// Phase `P`'s step: look up the case's first algorithm and take it.
#[inline(always)]
fn phase_step<const P: usize>(t: &Tables, x: &mut Cube) -> Step {
    let (r, sym) = t.row_of::<P>(t.index::<P>(x));
    let a = t.first::<P>(r);
    match P {
        0 => take::<P, { pairs(PHASE_MAX_LEN[0]) }>(x, a, sym),
        1 => take::<P, { pairs(PHASE_MAX_LEN[1]) }>(x, a, sym),
        2 => take::<P, { pairs(PHASE_MAX_LEN[2]) }>(x, a, sym),
        _ => take::<P, 0>(x, a, sym),
    }
}

/// Half `H` of split phase `P`'s step.
#[inline(always)]
fn half_step<const H: usize, const P: usize>(t: &Tables, x: &mut Cube) -> Step {
    debug_assert!(HALF_PHASE[H] == P);
    let a = t.half::<H>(t.half_case::<H>(x));
    match H {
        HALF_SLICE => take::<P, { pairs(HALF_MAX_LEN[HALF_SLICE]) }>(x, a, 0),
        HALF_TWIST => take::<P, { pairs(HALF_MAX_LEN[HALF_TWIST]) }>(x, a, 0),
        HALF_CORNERS => take::<P, { pairs(HALF_MAX_LEN[HALF_CORNERS]) }>(x, a, 0),
        _ => take::<P, 0>(x, a, 0),
    }
}

// ------------------------------------------------------------- batches

/// `chains`, not inlined into the caller.
#[inline(never)]
pub(crate) fn solve_chains<const K: usize, const S2: bool, const S4: bool, const LANE_MERGE: bool>(
    t: &Tables,
    cubes: &[Cube; K],
) -> [Solution; K] {
    chains::<K, S2, S4, LANE_MERGE>(t, cubes)
}

/// `K` chains at once, interleaved phase by phase: the cubes' lookups are
/// independent, so each phase's `K` table loads are in flight together.
/// With `LANE_MERGE` the solutions are merged in NEON lanes (`K` at most
/// 16), otherwise one by one.
#[inline(always)]
fn chains<const K: usize, const S2: bool, const S4: bool, const LANE_MERGE: bool>(
    t: &Tables,
    cubes: &[Cube; K],
) -> [Solution; K] {
    let mut c =
        Chains::<K, LANE_MERGE> { x: *cubes, out: std::array::from_fn(|_| Out::new()), lens: [[0; N_PHASES]; K] };
    c.next(0, |x| phase_step::<0>(t, x));
    if S2 {
        c.next(PHASE_2, |x| half_step::<HALF_SLICE, PHASE_2>(t, x));
        c.next(PHASE_2, |x| half_step::<HALF_TWIST, PHASE_2>(t, x));
    } else {
        c.next(PHASE_2, |x| phase_step::<PHASE_2>(t, x));
    }
    c.next(2, |x| phase_step::<2>(t, x));
    if S4 {
        c.next(LAST, |x| half_step::<HALF_CORNERS, LAST>(t, x));
        c.next(LAST, |x| half_step::<HALF_EDGES, LAST>(t, x));
    } else {
        c.next(LAST, |x| phase_step::<LAST>(t, x));
    }
    let evals = N_PHASES as u32 + S2 as u32 + S4 as u32;
    std::array::from_fn(|i| Solution { moves: c.out[i].moves(), phase_lens: c.lens[i], start: 0, evals })
}

/// `K` chains in progress.
struct Chains<const K: usize, const LANE_MERGE: bool> {
    x: [Cube; K],
    out: [Out; K],
    lens: [[u8; N_PHASES]; K],
}

impl<const K: usize, const LANE_MERGE: bool> Chains<K, LANE_MERGE> {
    /// Take `step` (of phase `p`) on every cube, then join each solution
    /// (the first phase starts it).
    #[inline(always)]
    fn next(&mut self, p: usize, mut step: impl FnMut(&mut Cube) -> Step) {
        let mut moves = [[0; LANES]; K];
        let mut n = [0; K];
        for i in 0..K {
            let s = step(&mut self.x[i]);
            (moves[i], n[i]) = (s.moves, s.n);
            self.lens[i][p] += s.n as u8;
        }
        if p == 0 {
            for i in 0..K {
                self.out[i].start(moves[i], n[i]);
            }
        } else if LANE_MERGE {
            append_lanes(&mut self.out, &moves, &n);
        } else {
            for i in 0..K {
                self.out[i].append(moves[i], n[i]);
            }
        }
    }
}
