//! Petrus method solver.
//!
//! Canonical frame (every orientation is mapped onto this one by symmetry
//! conjugation, so one set of tables serves all 48):
//!
//! 1. 2x2x2 block at DBL (corner DBL, edges DL DB BL)
//! 2. expand to the 2x2x3 block on D-L (+ corner DLF, edges DF FL)
//! 3. orient all edges for the F/B axis using <R,U,F,B>, keeping the block
//! 4. finish the first two layers using <R,U> only
//! 5. to 7. last layer by optimal lookup: Lars' steps (corner permutation,
//!    corner twist, edge permutation), COLL + EPLL, or one-look ZBLL
//!
//! Each step is solved optimally (fewest moves for that step) by IDA* or by
//! descending an exact distance table. A beam over orientations and over
//! alternative step solutions trades time for shorter totals.

use crate::cube::*;
use crate::facelet::{NSYM, SYM_MOVE};
pub use crate::defs::*;
use crate::ll::{LastLayer, LlMode};
/// Longest single-step solution searched for (far above any real step).
pub const MAX_STEP_DEPTH: usize = 24;
/// Capacity of a whole solution (seven steps, generously bounded).
pub const MAX_SOLUTION: usize = 128;
pub const MAX_STEPS: usize = 8;

pub type StepSeq = Moves<MAX_STEP_DEPTH>;
pub type FullSeq = Moves<MAX_SOLUTION>;

/// Byte mask selecting the given lanes, for whole-vector compares.
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
    const fn new<const NC: usize, const NE: usize>(p: &PieceSet<NC, NE>, eo: bool) -> Goal {
        Goal { c: lane_mask(&p.0), e: lane_mask(&p.1), eo }
    }

    const fn and<const NC: usize, const NE: usize>(self, p: &PieceSet<NC, NE>) -> Goal {
        Goal { c: self.c | lane_mask(&p.0), e: self.e | lane_mask(&p.1), eo: self.eo }
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

const GOAL_222: Goal = Goal::new(&BLOCK_222, false);
const GOAL_223: Goal = Goal::new(&BLOCK_223_D, false).and(&BLOCK_223_L);
const GOAL_EO: Goal = Goal::new(&BLOCK_223_D, true).and(&BLOCK_223_L);
const GOAL_F2L: Goal = GOAL_EO.and(&F2L_REST);

/// All pruning data, built once and shared read-only between threads.
pub struct Tables {
    pub t222: Table222,
    pub t222_front: Table222,
    pub t223_d: Table223,
    pub t223_l: Table223,
    /// Edge orientation plus the two block corners, under <R,U,F,B>.
    pub teo: TableEo,
    /// Remaining F2L pieces under <R,U>: exact.
    pub tf2l: Table223,
    pub ll: LastLayer,
}

macro_rules! embedded {
    ($name:literal) => {
        include_bytes!(concat!(env!("OUT_DIR"), "/", $name, ".bin"))
    };
}

impl Tables {
    /// Tables embedded in the binary at build time: no startup cost.
    pub fn new() -> Tables {
        Tables {
            t222: T222.with_data(embedded!("t222")),
            t222_front: T222_FRONT.with_data(embedded!("t222_front")),
            t223_d: T223_D.with_data(embedded!("t223_d")),
            t223_l: T223_L.with_data(embedded!("t223_l")),
            teo: TEO.with_data(embedded!("teo")),
            tf2l: TF2L.with_data(embedded!("tf2l")),
            ll: LastLayer::load_embedded().expect("embedded last-layer table"),
        }
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
    path: StepSeq,
    found: &'a mut Vec<StepSeq>,
    want: usize,
    nodes: u64,
}

impl Dfs<'_> {
    /// Extend the path by exactly `togo` more moves. Returns true once
    /// enough solutions have been collected.
    fn run<S: Step>(&mut self, c: &Cube, togo: u8, last: u8) -> bool {
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
            if togo == 1 {
                if S::goal(&n) {
                    let mut sol = self.path;
                    sol.push(m);
                    self.found.push(sol);
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
            self.path.push(m);
            let done = self.run::<S>(&n, togo - 1, f);
            self.path.pop();
            if done {
                return true;
            }
        }
        false
    }
}

/// Appends up to `want` solutions for step `S` to `out`, shortest first: all
/// optimal ones, then (if `slack` > 0) ones up to `slack` moves longer.
/// Returns the number of search nodes expanded.
pub fn solve_step_into<S: Step>(t: &Tables, c: &Cube, want: usize, slack: u8, out: &mut Vec<StepSeq>) -> u64 {
    let start = out.len();
    if S::goal(c) {
        out.push(StepSeq::default());
        return 0;
    }
    let mut d = Dfs { t, path: StepSeq::default(), found: out, want: start + want, nodes: 0 };
    let mut depth = S::h(t, c).max(1);
    let mut optimal = None;
    while (depth as usize) < MAX_STEP_DEPTH {
        d.run::<S>(c, depth, NO_FACE);
        if d.found.len() > start && optimal.is_none() {
            optimal = Some(depth);
        }
        if d.found.len() >= d.want || optimal.is_some_and(|o| depth >= o + slack) {
            break;
        }
        depth += 1;
    }
    assert!(d.found.len() > start, "{} step unsolvable", S::NAME);
    d.nodes
}

/// Convenience wrapper returning owned solutions.
pub fn solve_step<S: Step>(t: &Tables, c: &Cube, want: usize, slack: u8, nodes: &mut u64) -> Vec<Vec<u8>> {
    let mut out = Vec::new();
    *nodes += solve_step_into::<S>(t, c, want, slack, &mut out);
    out.iter().map(|s| s.as_slice().to_vec()).collect()
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

/// A partial solution in the canonical frame of its orientation. Plain old
/// data so the beam can copy it around without touching the allocator.
#[derive(Clone, Copy)]
struct Partial {
    sym: u8,
    nsteps: u8,
    state: Cube,
    /// Step solutions back to back; step i ends at `ends[i]`.
    raw: FullSeq,
    ends: [u8; MAX_STEPS],
    merged: FullSeq,
}

impl Partial {
    fn extend(&self, moves: &[u8]) -> Partial {
        let mut p = *self;
        p.state = p.state.apply_seq(moves);
        p.raw.extend(moves);
        p.merged.extend_merged(moves);
        p.ends[p.nsteps as usize] = p.raw.len() as u8;
        p.nsteps += 1;
        p
    }
}

struct Beam<'a> {
    t: &'a Tables,
    opts: &'a Options,
    names: Vec<&'static str>,
    cur: Vec<Partial>,
    next: Vec<Partial>,
    sols: Vec<StepSeq>,
    nodes: u64,
}

impl Beam<'_> {
    fn expand<S: Step>(&mut self) {
        self.next.clear();
        for p in &self.cur {
            self.sols.clear();
            self.nodes += solve_step_into::<S>(self.t, &p.state, self.opts.alts, self.opts.slack, &mut self.sols);
            for s in &self.sols {
                self.next.push(p.extend(s.as_slice()));
            }
        }
        std::mem::swap(&mut self.cur, &mut self.next);
        keep_best(&mut self.cur, self.opts.beam);
        self.names.push(S::NAME);
    }
}

/// Keep the `width` shortest partial solutions (stable for ties).
fn keep_best(v: &mut Vec<Partial>, width: usize) {
    let width = width.max(1);
    if width == 1 {
        if let Some(best) = v.iter().enumerate().min_by_key(|(_, p)| p.merged.len()).map(|(i, _)| i) {
            v.swap(0, best);
        }
    } else {
        v.sort_by_key(|p| p.merged.len());
    }
    v.truncate(width);
}

pub fn solve(t: &Tables, cube: &Cube, opts: &Options) -> Solution {
    let n_orient = opts.orientations.clamp(1, NSYM);
    let mut b = Beam {
        t,
        opts,
        names: Vec::with_capacity(MAX_STEPS),
        cur: (0..n_orient)
            .map(|sym| Partial {
                sym: sym as u8,
                nsteps: 0,
                state: cube.conj(sym),
                raw: FullSeq::default(),
                ends: [0; MAX_STEPS],
                merged: FullSeq::default(),
            })
            .collect(),
        next: Vec::new(),
        sols: Vec::new(),
        nodes: 0,
    };
    if opts.joint_blocks {
        b.expand::<Block223>();
    } else {
        b.expand::<Block222>();
        b.expand::<Expand223>();
    }
    b.expand::<EdgeOrient>();
    b.expand::<FirstTwoLayers>();
    let mut ll_names = Vec::new();
    for p in b.cur.iter_mut() {
        ll_names.clear();
        for (name, alg) in t.ll.solve(&p.state, opts.ll) {
            *p = p.extend(alg);
            ll_names.push(name);
        }
        debug_assert!(p.state.is_solved());
    }
    keep_best(&mut b.cur, 1);
    let best = &b.cur[0];
    let sym = best.sym as usize;
    let to_orig = |q: &[u8]| q.iter().map(|&m| SYM_MOVE[sym][m as usize]).collect::<Vec<u8>>();
    let mut steps = Vec::with_capacity(best.nsteps as usize);
    let mut start = 0;
    for (i, name) in b.names.iter().chain(ll_names.iter()).enumerate() {
        let end = best.ends[i] as usize;
        steps.push(StepMoves { name, moves: to_orig(&best.raw.as_slice()[start..end]) });
        start = end;
    }
    Solution { sym, steps, moves: to_orig(best.merged.as_slice()), nodes: b.nodes }
}
