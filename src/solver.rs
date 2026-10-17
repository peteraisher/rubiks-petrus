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

use crate::compiled::{self, alg_len, alg_move, expand_index, N_COMPILED};
use crate::cube::*;
pub use crate::defs::*;
use crate::facelet::{NSYM, SYM_MOVE};
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
    pub t223_fd: Table223,
    pub t223_fb: Table223,
    /// Edge orientation plus the two block corners, under <R,U,F,B>.
    pub teo: TableEo,
    /// Remaining F2L pieces under <R,U>: exact.
    pub tf2l: Table223,
    pub ll: LastLayer,
    /// Each compiled step's algorithms (little-endian `u64` by case index,
    /// see [`compiled`]).
    compiled: [&'static [u8]; N_COMPILED],
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
            t223_fd: T223_FD.with_data(embedded!("t223_fd")),
            t223_fb: T223_FB.with_data(embedded!("t223_fb")),
            teo: TEO.with_data(embedded!("teo")),
            tf2l: TF2L.with_data(embedded!("tf2l")),
            ll: LastLayer::load_embedded().expect("embedded last-layer table"),
            compiled: [embedded!("c222"), embedded!("cexpand"), embedded!("ceo"), embedded!("cf2l")],
        }
    }

    /// Compiled step `k`'s embedded table (for tests).
    pub fn compiled_table(&self, k: usize) -> &[u8] {
        self.compiled[k]
    }

    /// Compiled step `k`'s algorithm for case `i`, if a case maps there.
    #[inline(always)]
    fn compiled_alg(&self, k: usize, i: usize) -> Option<u64> {
        const WORD: usize = std::mem::size_of::<u64>();
        let b = self.compiled[k].get(i * WORD..(i + 1) * WORD)?;
        Some(u64::from_le_bytes(b.try_into().unwrap())).filter(|&a| a != compiled::NONE)
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
    /// `h(c) >= bound`, but free to stop at the first table that proves it:
    /// most search nodes are pruned, so this saves most lookups.
    #[inline(always)]
    fn exceeds(t: &Tables, c: &Cube, bound: u8) -> bool {
        Self::h(t, c) >= bound
    }
    /// The first optimal solution the search would return, from the step's
    /// compiled table, when the step has one and `c` is a case it covers.
    #[inline(always)]
    fn compiled(_t: &Tables, _c: &Cube) -> Option<u64> {
        None
    }
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
    #[inline(always)]
    fn compiled(t: &Tables, c: &Cube) -> Option<u64> {
        t.compiled_alg(compiled::C222, t.t222.index(c))
    }
}

#[inline(always)]
fn h_223(t: &Tables, c: &Cube) -> u8 {
    [&t.t223_d, &t.t223_l, &t.t223_fd, &t.t223_fb]
        .iter()
        .map(|x| x.get(c))
        .fold(t.t222.get(c).max(t.t222_front.get(c)), u8::max)
}

#[inline(always)]
fn exceeds_223(t: &Tables, c: &Cube, bound: u8) -> bool {
    // small (cache-resident) 2x2x2 tables first, large 2x2x3 halves after
    t.t222.get(c) >= bound
        || t.t222_front.get(c) >= bound
        || t.t223_d.get(c) >= bound
        || t.t223_l.get(c) >= bound
        || t.t223_fd.get(c) >= bound
        || t.t223_fb.get(c) >= bound
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
    #[inline(always)]
    fn exceeds(t: &Tables, c: &Cube, bound: u8) -> bool {
        exceeds_223(t, c, bound)
    }
    #[inline(always)]
    fn compiled(t: &Tables, c: &Cube) -> Option<u64> {
        // expansion cases are DLF, DF and FL around a solved 2x2x2
        if !GOAL_222.met(c) {
            return None;
        }
        t.compiled_alg(compiled::CEXPAND, expand_index(c))
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
    #[inline(always)]
    fn exceeds(t: &Tables, c: &Cube, bound: u8) -> bool {
        exceeds_223(t, c, bound)
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
    #[inline(always)]
    fn exceeds(t: &Tables, c: &Cube, bound: u8) -> bool {
        t.teo.get(c) >= bound || t.t223_d.get(c) >= bound || t.t223_l.get(c) >= bound
    }
    #[inline(always)]
    fn compiled(t: &Tables, c: &Cube) -> Option<u64> {
        // EO cases are the flips of a cube with the 2x2x3 solved
        if !GOAL_223.met(c) {
            return None;
        }
        t.compiled_alg(compiled::CEO, c.eo12() as usize)
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
    #[inline(always)]
    fn compiled(t: &Tables, c: &Cube) -> Option<u64> {
        // F2L cases are <R,U> states of a cube with the 2x2x3 and EO solved
        if !GOAL_EO.met(c) {
            return None;
        }
        t.compiled_alg(compiled::CF2L, t.tf2l.index(c))
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
            if S::exceeds(self.t, &n, togo) {
                continue;
            }
            if S::goal(&n) {
                // reaching the goal early means a shorter solution exists
                if togo == 1 {
                    let mut sol = self.path;
                    sol.push(m);
                    self.found.push(sol);
                    if self.found.len() >= self.want {
                        return true;
                    }
                }
                continue;
            }
            if togo == 1 {
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
/// optimal ones, then (if `slack` > 0) ones up to `slack` moves longer, and
/// none longer than `max_len` (the caller's "could still matter" bound; may
/// leave `out` unchanged). Returns the number of search nodes expanded.
///
/// One optimal solution of a compiled step is a table lookup (the same
/// solution the search finds, see [`compiled`]).
pub fn solve_step_into<S: Step>(
    t: &Tables,
    c: &Cube,
    want: usize,
    slack: u8,
    max_len: u8,
    out: &mut Vec<StepSeq>,
) -> u64 {
    if want == 1 && slack == 0 {
        if let Some(a) = S::compiled(t, c) {
            let n = alg_len(a);
            if n == 0 || n <= max_len.min(MAX_STEP_DEPTH as u8 - 1) as usize {
                let mut sol = StepSeq::default();
                for i in 0..n {
                    sol.push(alg_move(a, S::MOVES, i));
                }
                out.push(sol);
            }
            return 0;
        }
    }
    search_step_into::<S>(t, c, want, slack, max_len, out)
}

/// [`solve_step_into`] by search only, compiled table or not.
pub fn search_step_into<S: Step>(
    t: &Tables,
    c: &Cube,
    want: usize,
    slack: u8,
    max_len: u8,
    out: &mut Vec<StepSeq>,
) -> u64 {
    let start = out.len();
    if S::goal(c) {
        out.push(StepSeq::default());
        return 0;
    }
    let mut d = Dfs { t, path: StepSeq::default(), found: out, want: start + want, nodes: 0 };
    let mut depth = S::h(t, c).max(1);
    let mut optimal = None;
    let max_len = max_len.min(MAX_STEP_DEPTH as u8 - 1);
    while depth <= max_len {
        d.run::<S>(c, depth, NO_FACE);
        if d.found.len() > start && optimal.is_none() {
            optimal = Some(depth);
        }
        if d.found.len() >= d.want || optimal.is_some_and(|o| depth >= o + slack) {
            break;
        }
        depth += 1;
    }
    d.nodes
}

/// [`solve_step`] by search only (for tests and tools).
pub fn search_step<S: Step>(t: &Tables, c: &Cube, want: usize, slack: u8, nodes: &mut u64) -> Vec<Vec<u8>> {
    let mut out = Vec::new();
    *nodes += search_step_into::<S>(t, c, want, slack, u8::MAX, &mut out);
    assert!(!out.is_empty(), "{} step unsolvable", S::NAME);
    out.iter().map(|s| s.as_slice().to_vec()).collect()
}

/// Convenience wrapper returning owned solutions (always at least one).
pub fn solve_step<S: Step>(t: &Tables, c: &Cube, want: usize, slack: u8, nodes: &mut u64) -> Vec<Vec<u8>> {
    let mut out = Vec::new();
    *nodes += solve_step_into::<S>(t, c, want, slack, u8::MAX, &mut out);
    assert!(!out.is_empty(), "{} step unsolvable", S::NAME);
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
    /// Rank beam candidates by moves so far plus a lower bound on the next
    /// step (exact for F2L and the last layer) instead of moves so far.
    pub lookahead: bool,
}

impl Options {
    /// Lars Petrus' seven steps, fixed orientation, each step optimal.
    pub const CLASSIC: Options = Options {
        orientations: 1,
        beam: 1,
        alts: 1,
        slack: 0,
        joint_blocks: false,
        ll: LlMode::Lars,
        lookahead: false,
    };
    /// Like classic but with a one-look last layer.
    pub const FAST: Options = Options { ll: LlMode::Zbll, ..Options::CLASSIC };
    /// Greedy over all 48 orientations.
    pub const NEUTRAL: Options = Options { orientations: NSYM, ..Options::FAST };
    /// Beam search over orientations and step alternatives, ranked with
    /// lookahead (~30 moves in a few hundred microseconds).
    pub const SHORT: Options =
        Options { orientations: NSYM, beam: 16, alts: 4, slack: 1, lookahead: true, ..Options::FAST };
    /// A wider beam (~28 moves in about a millisecond).
    pub const SHORTEST: Options = Options { beam: 64, alts: 8, ..Options::SHORT };

    pub fn preset(name: &str) -> Option<Options> {
        Some(match name {
            "classic" => Options::CLASSIC,
            "fast" => Options::FAST,
            "neutral" => Options::NEUTRAL,
            "short" => Options::SHORT,
            "shortest" => Options::SHORTEST,
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
    /// Ranking key: moves so far plus (optionally) a lower bound on the next
    /// step, so the beam prefers partials that leave an easy continuation.
    key: usize,
}

impl Partial {
    fn extend(&self, moves: &[u8]) -> Partial {
        let mut p = *self;
        p.state = p.state.apply_seq(moves);
        p.raw.extend(moves);
        p.merged.extend_merged(moves);
        p.ends[p.nsteps as usize] = p.raw.len() as u8;
        p.nsteps += 1;
        p.key = p.merged.len();
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

/// Turns merged across a step boundary can shorten the running total; when
/// bounding a partial's length after the next step, allow for this many.
/// (Longer cascades of cancellation are possible but vanishingly rare, and
/// this only steers which candidates the beam explores.)
const BOUNDARY_SAVING: usize = 2;

impl Beam<'_> {
    /// Expand the beam by one step. Partials are searched in order of a
    /// lower bound on their length after the step (current length plus the
    /// step heuristic); once the beam is full and no remaining partial can
    /// beat its worst member, the rest are skipped. With exact heuristics
    /// (2x2x2, F2L) choosing among 48 orientations costs 48 table lookups
    /// plus one search, not 48 searches.
    ///
    /// `next_h` bounds the step after this one; with `Options::lookahead`
    /// it is added to each child's ranking key.
    fn expand<S: Step>(&mut self, next_h: impl Fn(&Cube) -> usize) {
        let width = self.opts.beam.max(1);
        let mut order: Vec<(usize, usize)> = self
            .cur
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let saving = if p.merged.is_empty() { 0 } else { BOUNDARY_SAVING };
                ((p.merged.len() + S::h(self.t, &p.state) as usize).saturating_sub(saving), i)
            })
            .collect();
        order.sort_unstable();
        self.next.clear();
        for &(bound, i) in &order {
            let p = &self.cur[i];
            // once the beam is full, a child only matters if it beats the
            // worst member, which caps both the order scan and the search
            let max_len = if self.next.len() >= width {
                let worst = self.next[width - 1].key;
                if worst <= bound {
                    break;
                }
                let saving = if p.merged.is_empty() { 0 } else { BOUNDARY_SAVING };
                (worst + saving - 1 - p.merged.len()).min(u8::MAX as usize) as u8
            } else {
                u8::MAX
            };
            self.sols.clear();
            self.nodes +=
                solve_step_into::<S>(self.t, &p.state, self.opts.alts, self.opts.slack, max_len, &mut self.sols);
            for s in &self.sols {
                let mut child = p.extend(s.as_slice());
                if self.opts.lookahead {
                    child.key += next_h(&child.state);
                }
                self.next.push(child);
            }
            keep_best(&mut self.next, width);
        }
        assert!(!self.next.is_empty(), "{} step unsolvable", S::NAME);
        std::mem::swap(&mut self.cur, &mut self.next);
        self.names.push(S::NAME);
    }
}

/// Keep the `width` best-ranked partial solutions (stable for ties).
fn keep_best(v: &mut Vec<Partial>, width: usize) {
    let width = width.max(1);
    if width == 1 {
        if let Some(best) = v.iter().enumerate().min_by_key(|(_, p)| p.key).map(|(i, _)| i) {
            v.swap(0, best);
        }
    } else {
        v.sort_by_key(|p| p.key);
    }
    v.truncate(width);
}

/// One orientation, one partial: each step's first optimal solution in turn
/// (a lookup for the compiled steps), without the beam's bookkeeping.
fn solve_greedy(t: &Tables, cube: &Cube, opts: &Options) -> Solution {
    struct Run<'a> {
        t: &'a Tables,
        state: Cube,
        merged: FullSeq,
        steps: Vec<StepMoves>,
        sols: Vec<StepSeq>,
        nodes: u64,
    }
    impl Run<'_> {
        fn push(&mut self, name: &'static str, moves: &[u8]) {
            self.state = self.state.apply_seq(moves);
            self.merged.extend_merged(moves);
            self.steps.push(StepMoves { name, moves: moves.to_vec() });
        }

        fn step<S: Step>(&mut self) {
            self.sols.clear();
            self.nodes += solve_step_into::<S>(self.t, &self.state, 1, 0, u8::MAX, &mut self.sols);
            let sol = self.sols[0];
            self.push(S::NAME, sol.as_slice());
        }
    }
    let mut r = Run {
        t,
        state: *cube,
        merged: FullSeq::default(),
        steps: Vec::with_capacity(MAX_STEPS),
        sols: Vec::new(),
        nodes: 0,
    };
    if opts.joint_blocks {
        r.step::<Block223>();
    } else {
        r.step::<Block222>();
        r.step::<Expand223>();
    }
    r.step::<EdgeOrient>();
    r.step::<FirstTwoLayers>();
    let state = r.state;
    t.ll.each_step(&state, opts.ll, |name, alg| r.push(name, alg));
    debug_assert!(r.state.is_solved());
    Solution { sym: 0, steps: r.steps, moves: r.merged.as_slice().to_vec(), nodes: r.nodes }
}

pub fn solve(t: &Tables, cube: &Cube, opts: &Options) -> Solution {
    let n_orient = opts.orientations.clamp(1, NSYM);
    if n_orient == 1 && opts.beam <= 1 && opts.alts <= 1 {
        return solve_greedy(t, cube, opts);
    }
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
                key: 0,
            })
            .collect(),
        next: Vec::new(),
        sols: Vec::new(),
        nodes: 0,
    };
    let h = |x: u8| x as usize;
    if opts.joint_blocks {
        b.expand::<Block223>(|c| h(EdgeOrient::h(t, c)));
    } else {
        b.expand::<Block222>(|c| h(Expand223::h(t, c)));
        b.expand::<Expand223>(|c| h(EdgeOrient::h(t, c)));
    }
    b.expand::<EdgeOrient>(|c| h(FirstTwoLayers::h(t, c)));
    b.expand::<FirstTwoLayers>(|c| t.ll.cost(c, opts.ll));
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
