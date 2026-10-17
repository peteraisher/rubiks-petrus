//! The beam (`short`, `shorter`, `shortest`): over starting orientations and
//! stored alternative algorithms, keeping the best partial solutions after
//! each phase, ranked by moves so far plus the next phase's exact distance.

use crate::algs::{alg_move, apply};
use crate::coord::*;
use crate::merge::{append_merged, merge, NO_MOVE};
use crate::solver::{Options, Seq, Solution, MAX_LEN, MAX_STARTS, MAX_WIDTH, STARTS};
use crate::tables::Tables;
use petrus::cube::*;
use petrus::facelet::SYM_MOVE;
use std::mem::MaybeUninit;

/// A partial solution in its start's frame.
#[derive(Clone, Copy)]
struct Node {
    cube: Cube,
    /// The next phase's case index, and the ranking key: moves so far plus
    /// that phase's exact distance.
    next: u32,
    key: u8,
    len: u8,
    /// Last move of the sequence so far, for merging (`NO_MOVE` if none).
    last: u8,
    parent: u8,
    /// The algorithm that led here, and its phase symmetry.
    alg: u64,
    sym: u8,
    /// The next phase's row of alternatives and its symmetry, filled in
    /// (and prefetched) once the node is kept.
    row: u32,
    row_sym: u8,
}

/// A level of the beam: the partial solutions kept after a phase, in the
/// order they were generated. Stored uninitialised so that a solve does not
/// clear kilobytes it never reads.
struct Level {
    v: [MaybeUninit<Node>; MAX_WIDTH],
    n: usize,
}

impl Level {
    #[inline(always)]
    fn nodes(&self) -> &[Node] {
        // SAFETY: the first `n` entries are always initialised.
        unsafe { std::slice::from_raw_parts(self.v.as_ptr().cast::<Node>(), self.n) }
    }

    #[inline(always)]
    fn push(&mut self, x: Node) {
        self.v[self.n] = MaybeUninit::new(x);
        self.n += 1;
    }
}

/// Most alternatives tried per partial in any phase.
const MAX_TRIED: usize = 16;
const MAX_KIDS: usize = MAX_WIDTH * MAX_TRIED;
const _: () = assert!(MAX_STARTS <= MAX_WIDTH && MAX_ALTS[0] <= MAX_TRIED && MAX_ALTS[2] <= MAX_TRIED);

/// All children of one phase, before selection. Each is stored without a
/// test, so generating them takes no data-dependent branch.
struct Kids {
    v: [MaybeUninit<Node>; MAX_KIDS],
    key: [u8; MAX_KIDS],
    n: usize,
}

impl Kids {
    #[inline(always)]
    fn push(&mut self, x: Node) {
        self.v[self.n] = MaybeUninit::new(x);
        self.key[self.n] = x.key;
        self.n += 1;
    }

    /// Move the best `width` children into `out`, earlier ones first among
    /// equal keys; missing alternatives (key `u8::MAX`) are never chosen.
    /// `keep` is applied to each one kept.
    #[inline(always)]
    fn select(&self, width: usize, out: &mut Level, mut keep: impl FnMut(&mut Node)) {
        let keys = &self.key[..self.n];
        let at_most = |t: u8| keys.iter().filter(|&&k| k <= t).count();
        let want = width.min(at_most(u8::MAX - 1));
        // the cutoff: the smallest key with `want` children at or below it
        let mut cut = keys.iter().copied().min().unwrap_or(0);
        while at_most(cut) < want {
            cut += 1;
        }
        let mut ties = want - keys.iter().filter(|&&k| k < cut).count();
        // compact the chosen indices with conditional increments, no branches
        let mut pick = [0u16; MAX_WIDTH + 1];
        let mut m = 0;
        for (i, &k) in keys.iter().enumerate() {
            let tie = k == cut && ties > 0;
            ties -= tie as usize;
            pick[m] = i as u16;
            m += (k < cut || tie) as usize;
        }
        for (j, &i) in pick[..m].iter().enumerate() {
            // SAFETY: every index below `n` was pushed
            let mut x = unsafe { self.v[i as usize].assume_init() };
            keep(&mut x);
            out.v[j] = MaybeUninit::new(x);
        }
        out.n = m;
    }
}

struct Search<'a> {
    t: &'a Tables,
    levels: [Level; N_PHASES],
    kids: Kids,
    /// Alternatives each phase will try, for prefetching.
    next_alts: [usize; N_PHASES],
    evals: u32,
}

impl<'a> Search<'a> {
    fn new(t: &'a Tables, o: &Options) -> Search<'a> {
        // only the counts need initialising: nodes are written before they are read
        let mut levels = MaybeUninit::<[Level; N_PHASES]>::uninit();
        let mut kids = MaybeUninit::<Kids>::uninit();
        // SAFETY: writes through raw pointers to fields of the uninitialised values
        unsafe {
            for i in 0..N_PHASES {
                std::ptr::addr_of_mut!((*levels.as_mut_ptr())[i].n).write(0);
            }
            std::ptr::addr_of_mut!((*kids.as_mut_ptr()).n).write(0);
        }
        // SAFETY: the counts are initialised and every other field is MaybeUninit
        let (levels, kids) = unsafe { (levels.assume_init(), kids.assume_init()) };
        let next_alts = [o.alts[0], o.alts[1], o.alts[2], 1];
        Search { t, levels, kids, next_alts, evals: 0 }
    }

    /// Level 0: the cube in each of the first `starts` starting frames.
    fn starts(&mut self, cube: &Cube, starts: usize) {
        let t = self.t;
        let inverse = if starts > 1 { cube.inverse() } else { *cube };
        for (i, &(sym, inv)) in STARTS[..starts].iter().enumerate() {
            let c = if inv { inverse } else { *cube };
            let c = if sym == 0 { c } else { c.conj(sym as usize) };
            let next = t.index::<0>(&c);
            t.prefetch_row::<0>(next, self.next_alts[0]);
            self.levels[0].push(Node {
                cube: c,
                next: next as u32,
                key: t.dist::<0>(next),
                len: 0,
                last: NO_MOVE,
                parent: i as u8,
                alg: 0,
                sym: 0,
                row: next as u32,
                row_sym: 0,
            });
        }
    }

    /// Extend every node of level `P` by up to `alts` stored algorithms of
    /// phase `P`, keeping the best `width` children in level `Q` = `P + 1`,
    /// ranked by length plus phase `Q`'s distance.
    #[inline(always)]
    fn expand<const P: usize, const Q: usize>(&mut self, alts: usize, width: usize) {
        let kids = &mut self.kids;
        kids.n = 0;
        for (pi, p) in self.levels[P].nodes().iter().enumerate() {
            let (row, sym) = (self.t.row::<P>(p.row as usize), p.row_sym as usize);
            for &raw in &row[..alts] {
                // a missing alternative is evaluated as the first one and ranked last
                let valid = raw != NO_ALG;
                let a = if valid { raw } else { row[0] };
                let n = alg_len(a);
                let (save, last) = match n {
                    0 => (0, p.last),
                    1 => merge(p.last, alg_move::<P>(a, sym, 0)),
                    _ => (merge(p.last, alg_move::<P>(a, sym, 0)).0, alg_move::<P>(a, sym, n - 1)),
                };
                let len = p.len + n as u8 - save;
                let cube = apply::<P>(&p.cube, a, sym);
                let next = self.t.index::<Q>(&cube);
                let key = if valid { len + self.t.dist::<Q>(next) } else { u8::MAX };
                kids.push(Node {
                    cube,
                    next: next as u32,
                    key,
                    len,
                    last,
                    parent: pi as u8,
                    alg: a,
                    sym: sym as u8,
                    row: 0,
                    row_sym: 0,
                });
            }
        }
        self.evals += kids.n as u32;
        let (t, next_alts) = (self.t, self.next_alts[Q]);
        kids.select(width, &mut self.levels[Q], |x| {
            let (r, sym) = t.row_of::<Q>(x.next as usize);
            (x.row, x.row_sym) = (r as u32, sym as u8);
            t.prefetch_row::<Q>(r, next_alts);
        });
    }

    /// Phase 4 has one algorithm per case. The ranking merged turns only
    /// approximately, so the partials tied for the best key are finished and
    /// the shortest kept.
    fn best(&self) -> Solution {
        let last = self.levels[LAST].nodes();
        let min_key = last.iter().map(|n| n.key).min().unwrap();
        let mut best: Option<Solution> = None;
        for node in last.iter().filter(|n| n.key == min_key) {
            let sol = self.assemble(node);
            if best.as_ref().is_none_or(|b| sol.moves.len() < b.moves.len()) {
                best = Some(sol);
            }
        }
        best.unwrap()
    }

    /// The whole solution through `node` (a partial after phase 3), in the
    /// original frame, with turns merged across phase boundaries.
    fn assemble(&self, node: &Node) -> Solution {
        let last = self.t.row::<LAST>(node.row as usize)[0];
        let mut path = [*node; LAST];
        for p in (0..LAST - 1).rev() {
            path[p] = self.levels[p + 1].nodes()[path[p + 1].parent as usize];
        }
        let start = self.levels[0].nodes()[path[0].parent as usize].parent;
        let (sym, inv) = STARTS[start as usize];
        let frame = &SYM_MOVE[sym as usize];
        let algs = [path[0].alg, path[1].alg, path[2].alg, last];
        let mut buf = [0u8; MAX_LEN];
        let mut len = 0;
        append::<0>(&mut buf, &mut len, algs[0], path[0].sym, frame);
        append::<1>(&mut buf, &mut len, algs[1], path[1].sym, frame);
        append::<2>(&mut buf, &mut len, algs[2], path[2].sym, frame);
        append::<LAST>(&mut buf, &mut len, last, node.row_sym, frame);
        if inv {
            buf[..len].reverse();
            for m in &mut buf[..len] {
                *m = inverse_move(*m);
            }
        }
        let phase_lens = algs.map(|a| alg_len(a) as u8);
        Solution { moves: Seq::from_slice(&buf[..len]), phase_lens, start, evals: self.evals }
    }
}

/// Append phase `P`'s algorithm `a`, its moves mapped through phase
/// symmetry `sym` and then the start's frame.
fn append<const P: usize>(buf: &mut [u8; MAX_LEN], len: &mut usize, a: u64, sym: u8, frame: &[u8; NMOVES]) {
    append_merged(buf, len, (0..alg_len(a)).map(|i| frame[alg_move::<P>(a, sym as usize, i) as usize]));
}

/// The beam over `o.starts` starts, `o.width` partials and `o.alts`
/// alternatives (`o` clamped).
pub(crate) fn solve_beam(t: &Tables, cube: &Cube, o: &Options) -> Solution {
    let mut s = Search::new(t, o);
    s.starts(cube, o.starts);
    s.expand::<0, 1>(o.alts[0], o.width);
    s.expand::<1, 2>(o.alts[1], o.width);
    s.expand::<2, LAST>(o.alts[2], o.width);
    s.best()
}
