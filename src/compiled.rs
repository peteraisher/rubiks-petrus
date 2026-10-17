//! Compiled steps: the optimal algorithm the step search returns, stored
//! per case, so a greedy solve is one lookup per step instead of a search.
//!
//! Once the earlier steps are done, a step's starting state is small: the
//! 2x2x2 depends on 4 pieces (253 440 states), the expansion to 2x2x3 on
//! DLF, DF and FL (6048 cases), EO on the flips of the seven other edges
//! (64) and F2L on 56 700 <R,U> states.
//!
//! The step search (IDA* over canonical sequences, moves in the step's
//! order) returns the lexicographically first optimal sequence whatever
//! its heuristic: pruning with an admissible bound only drops subtrees
//! without a solution, and none can reach the goal early (that would be a
//! shorter solution). So these tables are built with any admissible bound,
//! and solves that use them return exactly what the search returns. Shared
//! with `build.rs`, which builds them.

use crate::cube::*;
use crate::defs::*;

/// An algorithm packed in a `u64`: its length in the low bits, then each
/// move as its slot in the step's move list.
pub const LEN_BITS: u32 = 5;
/// Marks an index no reachable case maps to.
pub const NONE: u64 = u64::MAX;

/// Bits per move slot for a step with `n` moves.
pub const fn slot_bits(n: usize) -> u32 {
    usize::BITS - (n - 1).leading_zeros()
}

pub fn pack(moves: &[u8], step_moves: &[u8]) -> u64 {
    let bits = slot_bits(step_moves.len());
    assert!(moves.len() < 1 << LEN_BITS && LEN_BITS + bits * moves.len() as u32 <= u64::BITS);
    moves.iter().enumerate().fold(moves.len() as u64, |a, (i, m)| {
        let slot = step_moves.iter().position(|x| x == m).expect("move outside the step's set");
        a | (slot as u64) << (LEN_BITS + bits * i as u32)
    })
}

#[inline(always)]
pub fn alg_len(a: u64) -> usize {
    (a & ((1 << LEN_BITS) - 1)) as usize
}

#[inline(always)]
pub fn alg_move(a: u64, step_moves: &[u8], i: usize) -> u8 {
    let bits = slot_bits(step_moves.len());
    step_moves[((a >> (LEN_BITS + bits * i as u32)) & ((1 << bits) - 1)) as usize]
}

/// Index of the expansion step's case: the lanes of DLF, DF and FL.
pub const EXPAND_CASES: usize = LANE_VALUES * LANE_VALUES * LANE_VALUES;
#[inline(always)]
pub fn expand_index(c: &Cube) -> usize {
    (c.c[DLF] as usize * LANE_VALUES + c.e[DF] as usize) * LANE_VALUES + c.e[FL] as usize
}

#[inline(always)]
fn canonical(last: u8, f: u8) -> bool {
    !(f == last || opposite(f) == last && f < last)
}

/// One cube per index for every state reachable from solved with `moves`.
fn reachable(moves: &[u8], size: usize, key: impl Fn(&Cube) -> usize) -> Vec<Option<Cube>> {
    let mut seen = vec![None; size];
    seen[key(&Cube::SOLVED)] = Some(Cube::SOLVED);
    let mut frontier = vec![Cube::SOLVED];
    while !frontier.is_empty() {
        let mut next = Vec::new();
        for c in &frontier {
            for &m in moves {
                let n = c.apply_move(m as usize);
                let slot = &mut seen[key(&n)];
                if slot.is_none() {
                    *slot = Some(n);
                    next.push(n);
                }
            }
        }
        frontier = next;
    }
    seen
}

/// The lexicographically first optimal canonical sequence reaching `goal`,
/// by iterative deepening with the admissible bound `h`.
fn first_optimal(c: &Cube, moves: &[u8], h: &dyn Fn(&Cube) -> u8, goal: &dyn Fn(&Cube) -> bool) -> Vec<u8> {
    fn dfs(
        c: &Cube,
        moves: &[u8],
        h: &dyn Fn(&Cube) -> u8,
        goal: &dyn Fn(&Cube) -> bool,
        togo: u8,
        last: u8,
        path: &mut Vec<u8>,
    ) -> bool {
        if togo == 0 {
            return goal(c);
        }
        for &m in moves {
            let f = face(m);
            if !canonical(last, f) {
                continue;
            }
            let n = c.apply_move(m as usize);
            if h(&n) >= togo {
                continue;
            }
            path.push(m);
            if dfs(&n, moves, h, goal, togo - 1, f, path) {
                return true;
            }
            path.pop();
        }
        false
    }
    let mut path = Vec::new();
    let mut depth = h(c);
    while !dfs(c, moves, h, goal, depth, NO_FACE, &mut path) {
        depth += 1;
    }
    path
}

/// Whether the given corners and edges are home and oriented.
fn solved(c: &Cube, corners: &[usize], edges: &[usize]) -> bool {
    corners.iter().all(|&i| c.c[i] as usize == i) && edges.iter().all(|&i| c.e[i] as usize == i)
}

/// The 2x2x3's edges (its corners are `BLOCK_223_D.0`).
const BLOCK_223_EDGES: [usize; 5] = [DF, DL, DB, FL, BL];

/// The 2x2x3 is bounded by four "2 corners + 3 edges" halves.
pub const N_223_HALVES: usize = 4;

/// The pruning tables the generator searches with.
pub struct Bounds<'a> {
    pub t222: &'a Table222,
    pub t222_front: &'a Table222,
    pub t223: [&'a Table223; N_223_HALVES],
    pub teo: &'a TableEo,
    pub tf2l: &'a Table223,
}

/// The compiled steps, in solve order.
pub const N_COMPILED: usize = 4;
pub const NAMES: [&str; N_COMPILED] = ["c222", "cexpand", "ceo", "cf2l"];
pub const C222: usize = 0;
pub const CEXPAND: usize = 1;
pub const CEO: usize = 2;
pub const CF2L: usize = 3;

/// The compiled steps (2x2x2, expansion, EO, F2L), packed algorithms by
/// case index, `NONE` where no case maps.
pub fn build(b: &Bounds) -> [Vec<u64>; N_COMPILED] {
    let block222 = |c: &Cube| solved(c, &BLOCK_222.0, &BLOCK_222.1);
    let block223 = |c: &Cube| solved(c, &BLOCK_223_D.0, &BLOCK_223_EDGES);
    let h223 = |c: &Cube| b.t223.iter().map(|t| t.get(c)).fold(b.t222.get(c).max(b.t222_front.get(c)), u8::max);

    // 2x2x2 and F2L: their tables are exact, a bound like any other
    let mut c222 = vec![NONE; b.t222.size];
    for c in reachable(&ALL_MOVES, b.t222.size, |c| b.t222.index(c)).iter().flatten() {
        c222[b.t222.index(c)] = pack(&first_optimal(c, &ALL_MOVES, &|x| b.t222.get(x), &block222), &ALL_MOVES);
    }
    let f2l_goal = |c: &Cube| solved(c, &F2L_REST.0, &F2L_REST.1);
    let mut cf2l = vec![NONE; b.tf2l.size];
    for c in reachable(&RU_MOVES, b.tf2l.size, |c| b.tf2l.index(c)).iter().flatten() {
        cf2l[b.tf2l.index(c)] = pack(&first_optimal(c, &RU_MOVES, &|x| b.tf2l.get(x), &f2l_goal), &RU_MOVES);
    }

    // expansion: DLF, DF and FL anywhere off the solved 2x2x2
    let mut cexp = vec![NONE; EXPAND_CASES];
    let off = |v: usize, pos: &[usize], n: usize| !pos.contains(&(v % n));
    for vc in (0..LANE_VALUES).filter(|&v| off(v, &BLOCK_222.0, N_CORNERS)) {
        for vf in (0..LANE_VALUES).filter(|&v| off(v, &BLOCK_222.1, N_EDGES)) {
            for vl in (0..LANE_VALUES).filter(|&v| off(v, &BLOCK_222.1, N_EDGES) && v % N_EDGES != vf % N_EDGES) {
                let mut c = Cube::SOLVED;
                (c.c[DLF], c.e[DF], c.e[FL]) = (vc as u8, vf as u8, vl as u8);
                cexp[expand_index(&c)] = pack(&first_optimal(&c, &ALL_MOVES, &h223, &block223), &ALL_MOVES);
            }
        }
    }

    // EO: any even set of flips of the edges off the solved 2x2x3
    let free: Vec<usize> = (0..N_EDGES).filter(|e| !BLOCK_223_EDGES.contains(e)).collect();
    let heo = |c: &Cube| b.teo.get(c).max(b.t223[0].get(c)).max(b.t223[1].get(c));
    let eo_goal = |c: &Cube| block223(c) && c.eo12() == 0;
    let mut ceo = vec![NONE; EO_MASKS];
    for mask in (0..1usize << free.len()).filter(|m| m.count_ones() % EDGE_FLIPS as u32 == 0) {
        let mut c = Cube::SOLVED;
        for (k, &e) in free.iter().enumerate() {
            c.e[e] += (N_EDGES * ((mask >> k) & 1)) as u8;
        }
        ceo[c.eo12() as usize] = pack(&first_optimal(&c, &RUFB_MOVES, &heo, &eo_goal), &RUFB_MOVES);
    }
    [c222, cexp, ceo, cf2l]
}
