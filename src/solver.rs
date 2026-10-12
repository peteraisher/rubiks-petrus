//! Petrus method solver.
//!
//! Canonical frame (every orientation is mapped onto this one by symmetry
//! conjugation, so one set of tables serves all 48):
//!
//! 1. 2x2x2 block at DBL (corner DBL, edges DL DB BL)
//! 2. expand to the 2x2x3 block on D-L (+ corner DLF, edges DF FL)
//! 3. orient all edges for the F/B axis using <R,U,F,B>, keeping the block
//! 4. finish the first two layers using <R,U> only
//! 5. to 7. last layer by optimal lookup: Lars' steps (corner permutation, corner
//!    twist, edge permutation), COLL + EPLL, or one-look ZBLL
//!
//! Each step is solved optimally (fewest moves for that step) by IDA* or by
//! descending an exact distance table. A beam over orientations and over
//! alternative step solutions trades time for shorter totals.

use crate::cube::*;
use crate::facelet::{NSYM, SYM_MOVE};
use crate::ll::{LastLayer, LlMode};
use crate::tables::PieceTable;

pub const BLOCK_222: (&[usize], &[usize]) = (&[DBL], &[DL, DB, BL]);
pub const BLOCK_222_FRONT: (&[usize], &[usize]) = (&[DLF], &[DL, DF, FL]);
/// The 2x2x3 split into two overlapping halves that fit in a table each.
pub const BLOCK_223_D: (&[usize], &[usize]) = (&[DLF, DBL], &[DF, DL, DB]);
pub const BLOCK_223_L: (&[usize], &[usize]) = (&[DLF, DBL], &[DL, FL, BL]);
pub const F2L_REST: (&[usize], &[usize]) = (&[DFR, DRB], &[DR, FR, BR]);

/// Longest single-step solution searched for (far above any real step).
pub const MAX_STEP_DEPTH: usize = 24;

pub const ALL_MOVES: [u8; NMOVES] = {
    let mut a = [0u8; NMOVES];
    let mut i = 0;
    while i < NMOVES {
        a[i] = i as u8;
        i += 1;
    }
    a
};
/// All turns of the given faces; `M` must be `N * POWERS`.
const fn face_moves<const N: usize, const M: usize>(faces: [u8; N]) -> [u8; M] {
    assert!(M == N * POWERS);
    let mut a = [0u8; M];
    let mut i = 0;
    while i < M {
        a[i] = mv(faces[i / POWERS], (i % POWERS) as u8);
        i += 1;
    }
    a
}
pub const RUFB_MOVES: [u8; 4 * POWERS] = face_moves([FACE_U, FACE_R, FACE_F, FACE_B]);
pub const RU_MOVES: [u8; 2 * POWERS] = face_moves([FACE_U, FACE_R]);

/// Byte masks selecting the lanes of a piece set, for whole-vector compares.
const fn lane_mask(lanes: &[usize]) -> u128 {
    let mut m = 0u128;
    let mut i = 0;
    while i < lanes.len() {
        m |= (u8::MAX as u128) << (lanes[i] * u8::BITS as usize);
        i += 1;
    }
    m
}

#[derive(Clone, Copy)]
struct Goal {
    e: u128,
    c: u128,
    eo: bool,
}

impl Goal {
    const fn of(sets: &[(&[usize], &[usize])], eo: bool) -> Goal {
        let mut g = Goal { e: 0, c: 0, eo };
        let mut i = 0;
        while i < sets.len() {
            g.c |= lane_mask(sets[i].0);
            g.e |= lane_mask(sets[i].1);
            i += 1;
        }
        g
    }

    #[inline(always)]
    fn met(&self, c: &Cube) -> bool {
        let se = u128::from_ne_bytes(Cube::SOLVED.e);
        let sc = u128::from_ne_bytes(Cube::SOLVED.c);
        (u128::from_ne_bytes(c.e) ^ se) & self.e == 0
            && (u128::from_ne_bytes(c.c) ^ sc) & self.c == 0
            && (!self.eo || c.eo12() == 0)
    }
}

const GOAL_222: Goal = Goal::of(&[BLOCK_222], false);
const GOAL_223: Goal = Goal::of(&[BLOCK_223_D, BLOCK_223_L], false);
const GOAL_EO: Goal = Goal::of(&[BLOCK_223_D, BLOCK_223_L], true);
const GOAL_F2L: Goal = Goal::of(&[BLOCK_223_D, BLOCK_223_L, F2L_REST], true);

/// All pruning data, built once and shared read-only between threads.
pub struct Tables {
    pub t222: PieceTable,
    pub t222_front: PieceTable,
    pub t223_d: PieceTable,
    pub t223_l: PieceTable,
    /// Edge orientation plus the two block corners, under <R,U,F,B>.
    pub teo: PieceTable,
    /// Remaining F2L pieces under <R,U>: exact.
    pub tf2l: PieceTable,
    pub ll: LastLayer,
}

impl Tables {
    pub fn new() -> Tables {
        std::thread::scope(|s| {
            let a = s.spawn(|| PieceTable::build(BLOCK_222.0, BLOCK_222.1, false, &ALL_MOVES));
            let b = s.spawn(|| PieceTable::build(BLOCK_222_FRONT.0, BLOCK_222_FRONT.1, false, &ALL_MOVES));
            let c = s.spawn(|| PieceTable::build(BLOCK_223_D.0, BLOCK_223_D.1, false, &ALL_MOVES));
            let d = s.spawn(|| PieceTable::build(BLOCK_223_L.0, BLOCK_223_L.1, false, &ALL_MOVES));
            let e = s.spawn(|| PieceTable::build(BLOCK_223_D.0, &[], true, &RUFB_MOVES));
            let f = s.spawn(|| PieceTable::build(F2L_REST.0, F2L_REST.1, false, &RU_MOVES));
            Tables {
                t222: a.join().unwrap(),
                t222_front: b.join().unwrap(),
                t223_d: c.join().unwrap(),
                t223_l: d.join().unwrap(),
                teo: e.join().unwrap(),
                tf2l: f.join().unwrap(),
                ll: LastLayer::load_embedded().expect("embedded last-layer table"),
            }
        })
    }
}

impl Default for Tables {
    fn default() -> Self {
        Tables::new()
    }
}

/// One searchable step: move set, goal and admissible heuristic.
pub trait Step {
    const NAME: &'static str;
    const MOVES: &'static [u8];
    fn goal(c: &Cube) -> bool;
    fn h(t: &Tables, c: &Cube) -> u8;
}

pub struct Block222;
pub struct Expand223;
pub struct Block223;
pub struct EdgeOrient;
pub struct FirstTwoLayers;

impl Step for Block222 {
    const NAME: &'static str = "2x2x2";
    const MOVES: &'static [u8] = &ALL_MOVES;
    fn goal(c: &Cube) -> bool {
        GOAL_222.met(c)
    }
    #[inline(always)]
    fn h(t: &Tables, c: &Cube) -> u8 {
        t.t222.get(c)
    }
}

#[inline(always)]
fn h_223(t: &Tables, c: &Cube) -> u8 {
    t.t223_d.get(c).max(t.t223_l.get(c)).max(t.t222.get(c)).max(t.t222_front.get(c))
}

impl Step for Expand223 {
    const NAME: &'static str = "2x2x3";
    const MOVES: &'static [u8] = &ALL_MOVES;
    fn goal(c: &Cube) -> bool {
        GOAL_223.met(c)
    }
    #[inline(always)]
    fn h(t: &Tables, c: &Cube) -> u8 {
        h_223(t, c)
    }
}

impl Step for Block223 {
    const NAME: &'static str = "2x2x3";
    const MOVES: &'static [u8] = &ALL_MOVES;
    fn goal(c: &Cube) -> bool {
        GOAL_223.met(c)
    }
    #[inline(always)]
    fn h(t: &Tables, c: &Cube) -> u8 {
        h_223(t, c)
    }
}

impl Step for EdgeOrient {
    const NAME: &'static str = "EO";
    const MOVES: &'static [u8] = &RUFB_MOVES;
    fn goal(c: &Cube) -> bool {
        GOAL_EO.met(c)
    }
    #[inline(always)]
    fn h(t: &Tables, c: &Cube) -> u8 {
        t.teo.get(c).max(t.t223_d.get(c)).max(t.t223_l.get(c))
    }
}

impl Step for FirstTwoLayers {
    const NAME: &'static str = "F2L";
    const MOVES: &'static [u8] = &RU_MOVES;
    fn goal(c: &Cube) -> bool {
        GOAL_F2L.met(c)
    }
    #[inline(always)]
    fn h(t: &Tables, c: &Cube) -> u8 {
        t.tf2l.get(c)
    }
}

struct Dfs<'a> {
    t: &'a Tables,
    path: [u8; MAX_STEP_DEPTH],
    found: Vec<Vec<u8>>,
    want: usize,
    nodes: u64,
}

impl Dfs<'_> {
    /// Extend the path at `depth` by exactly `togo` more moves. Returns true
    /// once enough solutions have been collected.
    fn run<S: Step>(&mut self, c: &Cube, depth: usize, togo: u8, last: u8) -> bool {
        for &m in S::MOVES {
            let f = face(m);
            // canonical order: no repeated face; opposite faces in one order only
            if f == last || opposite(f) == last && f < last {
                continue;
            }
            let n = c.apply_move(m as usize);
            self.nodes += 1;
            let h = S::h(self.t, &n);
            if h >= togo {
                continue;
            }
            self.path[depth] = m;
            if togo == 1 {
                if S::goal(&n) {
                    self.found.push(self.path[..=depth].to_vec());
                    if self.found.len() >= self.want {
                        return true;
                    }
                }
                continue;
            }
            // reaching the goal early means a shorter solution already exists
            if h == 0 && S::goal(&n) {
                continue;
            }
            if self.run::<S>(&n, depth + 1, togo - 1, f) {
                return true;
            }
        }
        false
    }
}

/// Up to `want` solutions for step `S`, shortest first: all optimal ones,
/// then (if `slack` > 0) ones up to `slack` moves longer.
pub fn solve_step<S: Step>(t: &Tables, c: &Cube, want: usize, slack: u8, nodes: &mut u64) -> Vec<Vec<u8>> {
    if S::goal(c) {
        return vec![Vec::new()];
    }
    let mut d = Dfs { t, path: [0; MAX_STEP_DEPTH], found: Vec::new(), want, nodes: 0 };
    let mut depth = S::h(t, c).max(1);
    let mut optimal = None;
    while (depth as usize) < MAX_STEP_DEPTH {
        d.run::<S>(c, 0, depth, NO_FACE);
        if !d.found.is_empty() && optimal.is_none() {
            optimal = Some(depth);
        }
        if d.found.len() >= want || optimal.is_some_and(|o| depth >= o + slack) {
            break;
        }
        depth += 1;
    }
    *nodes += d.nodes;
    assert!(!d.found.is_empty(), "{} step unsolvable", S::NAME);
    d.found
}

// ------------------------------------------------------------------ driver

#[derive(Clone, Debug)]
pub struct Options {
    /// Orientations tried: 1 (fixed), 24 (all rotations), 48 (plus mirrors).
    pub orientations: usize,
    /// Partial solutions kept after each step (1 = greedy).
    pub beam: usize,
    /// Alternative solutions generated per step and partial solution.
    pub alts: usize,
    /// Alternatives may be this many moves longer than the step optimum.
    pub slack: u8,
    /// Solve the 2x2x3 directly (steps 1+2 jointly optimal) instead of
    /// an optimal 2x2x2 followed by an optimal expansion.
    pub joint_blocks: bool,
    pub ll: LlMode,
}

impl Options {
    /// Lars Petrus' seven steps, fixed orientation, each step optimal.
    pub const CLASSIC: Options =
        Options { orientations: 1, beam: 1, alts: 1, slack: 0, joint_blocks: false, ll: LlMode::Lars };
    /// Like classic but with a one-look last layer.
    pub const FAST: Options = Options { ll: LlMode::Zbll, ..Options::CLASSIC };
    /// Greedy over all 48 orientations.
    pub const NEUTRAL: Options = Options { orientations: NSYM, ..Options::FAST };
    /// Beam search over orientations and step alternatives.
    pub const SHORT: Options =
        Options { orientations: NSYM, beam: 8, alts: 4, slack: 1, joint_blocks: false, ll: LlMode::Zbll };

    pub fn preset(name: &str) -> Option<Options> {
        Some(match name {
            "classic" => Options::CLASSIC,
            "fast" => Options::FAST,
            "neutral" => Options::NEUTRAL,
            "short" => Options::SHORT,
            _ => return None,
        })
    }
}

impl Default for Options {
    fn default() -> Self {
        Options::CLASSIC
    }
}

#[derive(Clone, Debug)]
pub struct StepMoves {
    pub name: &'static str,
    pub moves: Vec<u8>,
}

#[derive(Clone, Debug)]
pub struct Solution {
    /// Symmetry the solve was performed under (0 = as given).
    pub sym: usize,
    /// Per-step moves in the original frame, before cross-step cancellation.
    pub steps: Vec<StepMoves>,
    /// Whole solution with cancellations applied.
    pub moves: Vec<u8>,
    /// Search nodes expanded (for profiling).
    pub nodes: u64,
}

#[derive(Clone)]
struct Partial {
    sym: usize,
    state: Cube,
    steps: Vec<StepMoves>,
    merged: Vec<u8>,
}

impl Partial {
    fn extend(&self, name: &'static str, moves: &[u8]) -> Partial {
        let mut p = self.clone();
        p.state = p.state.apply_seq(moves);
        seq_append(&mut p.merged, moves);
        p.steps.push(StepMoves { name, moves: moves.to_vec() });
        p
    }
}

fn expand<S: Step>(t: &Tables, opts: &Options, beam: Vec<Partial>, nodes: &mut u64) -> Vec<Partial> {
    let mut next = Vec::new();
    for p in &beam {
        for sol in solve_step::<S>(t, &p.state, opts.alts, opts.slack, nodes) {
            next.push(p.extend(S::NAME, &sol));
        }
    }
    prune(next, opts.beam)
}

fn prune(mut v: Vec<Partial>, width: usize) -> Vec<Partial> {
    v.sort_by_key(|p| p.merged.len());
    v.truncate(width.max(1));
    v
}

pub fn solve(t: &Tables, cube: &Cube, opts: &Options) -> Solution {
    let n_orient = opts.orientations.clamp(1, NSYM);
    let mut nodes = 0;
    let mut beam: Vec<Partial> = (0..n_orient)
        .map(|sym| Partial { sym, state: cube.conj(sym), steps: Vec::new(), merged: Vec::new() })
        .collect();
    if opts.joint_blocks {
        beam = expand::<Block223>(t, opts, beam, &mut nodes);
    } else {
        beam = expand::<Block222>(t, opts, beam, &mut nodes);
        beam = expand::<Expand223>(t, opts, beam, &mut nodes);
    }
    beam = expand::<EdgeOrient>(t, opts, beam, &mut nodes);
    beam = expand::<FirstTwoLayers>(t, opts, beam, &mut nodes);
    let finished: Vec<Partial> = beam
        .iter()
        .map(|p| {
            let mut p = p.clone();
            for (name, alg) in t.ll.solve(&p.state, opts.ll) {
                p = p.extend(name, alg);
            }
            debug_assert!(p.state.is_solved());
            p
        })
        .collect();
    let best = prune(finished, 1).remove(0);
    let to_orig = |q: &[u8]| q.iter().map(|&m| SYM_MOVE[best.sym][m as usize]).collect::<Vec<u8>>();
    Solution {
        sym: best.sym,
        steps: best.steps.iter().map(|s| StepMoves { name: s.name, moves: to_orig(&s.moves) }).collect(),
        moves: to_orig(&best.merged),
        nodes,
    }
}

