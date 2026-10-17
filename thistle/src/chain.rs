//! The single chain (`fast`, `fast5`, `fast6`): Thistlethwaite with each
//! phase's first stored algorithm, one table lookup per phase (or per half
//! of a split phase), written so that no branch depends on the cube.

use crate::algs::{alg_moves, apply_padded, pad_identity, pairs};
use crate::coord::*;
use crate::merge::Out;
use crate::solver::Solution;
use crate::tables::Tables;
use petrus::cube::*;

/// Apply phase `P`'s algorithm `a` (symmetry `sym`) as `PAIRS` move pairs,
/// or not at all for `PAIRS` = 0 (the last phase: nothing reads the solved
/// cube), and append it to the solution. Returns its length.
#[inline(always)]
fn take<const P: usize, const PAIRS: usize>(x: &mut Cube, a: u64, sym: usize, out: &mut Out) -> u8 {
    let n = alg_len(a);
    debug_assert!(PAIRS == 0 || n <= 2 * PAIRS);
    let moves = pad_identity(alg_moves::<P>(a, sym), n);
    if PAIRS > 0 {
        *x = apply_padded::<PAIRS>(x, &moves);
    }
    if P == 0 {
        out.start(moves, n);
    } else {
        out.append(moves, n);
    }
    n as u8
}

/// Phase `P`: look up the case's first algorithm and take it.
#[inline(always)]
fn phase<const P: usize>(t: &Tables, x: &mut Cube, out: &mut Out) -> u8 {
    let (r, sym) = t.row_of::<P>(t.index::<P>(x));
    let a = t.first::<P>(r);
    match P {
        0 => take::<P, { pairs(PHASE_MAX_LEN[0]) }>(x, a, sym, out),
        1 => take::<P, { pairs(PHASE_MAX_LEN[1]) }>(x, a, sym, out),
        2 => take::<P, { pairs(PHASE_MAX_LEN[2]) }>(x, a, sym, out),
        _ => take::<P, 0>(x, a, sym, out),
    }
}

/// Half `H` of split phase `P` (see [`N_HALVES`]).
#[inline(always)]
fn half<const H: usize, const P: usize>(t: &Tables, x: &mut Cube, out: &mut Out) -> u8 {
    debug_assert!(HALF_PHASE[H] == P);
    let a = t.half::<H>(t.half_case::<H>(x));
    match H {
        HALF_SLICE => take::<P, { pairs(HALF_MAX_LEN[HALF_SLICE]) }>(x, a, 0, out),
        HALF_TWIST => take::<P, { pairs(HALF_MAX_LEN[HALF_TWIST]) }>(x, a, 0, out),
        HALF_CORNERS => take::<P, { pairs(HALF_MAX_LEN[HALF_CORNERS]) }>(x, a, 0, out),
        _ => take::<P, 0>(x, a, 0, out),
    }
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
