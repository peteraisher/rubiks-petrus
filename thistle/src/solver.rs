//! The solver's interface: options and presets, the solution, and
//! `solve`. With one start, a beam of one and one alternative per phase it
//! is plain Thistlethwaite with optimal phases, one table lookup per phase
//! (`chain`); otherwise a beam over starting orientations and stored
//! alternatives (`beam`).

use crate::beam::solve_beam;
use crate::chain::solve_chain;
use crate::coord::*;
use petrus::cube::*;
use petrus::facelet::{NSYM, SYM_MOVE};

pub use crate::chain::chain_phase;
pub use crate::tables::Tables;

// ------------------------------------------------------------- options

/// Starting points: the six symmetries that order the three axes
/// differently, each applied to the cube and to its inverse.
pub const MAX_STARTS: usize = 2 * N_AXIS_ORDERS;
const N_AXES: usize = 3;
const N_AXIS_ORDERS: usize = 6;

/// (symmetry, solve the inverse), interleaved so that a prefix of length 2
/// is the cube and its inverse.
pub const STARTS: [(u8, bool); MAX_STARTS] = {
    let mut syms = [0u8; N_AXIS_ORDERS];
    let mut seen = [[false; N_AXES]; N_AXES];
    let mut n = 0;
    let mut s = 0;
    while s < NSYM {
        let u = face(SYM_MOVE[s][mv(FACE_U, 0) as usize]) as usize % N_AXES;
        let f = face(SYM_MOVE[s][mv(FACE_F, 0) as usize]) as usize % N_AXES;
        if !seen[u][f] {
            seen[u][f] = true;
            syms[n] = s as u8;
            n += 1;
        }
        s += 1;
    }
    assert!(n == N_AXIS_ORDERS);
    let mut out = [(0u8, false); MAX_STARTS];
    let mut i = 0;
    while i < MAX_STARTS {
        out[i] = (syms[i / 2], i % 2 == 1);
        i += 1;
    }
    out
};

/// Partial solutions kept between phases, at most.
pub const MAX_WIDTH: usize = 64;

/// Phases the single chain splits in two (see [`N_HALVES`]): smaller tables,
/// more moves.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Split {
    pub phase2: bool,
    pub phase4: bool,
}

#[derive(Clone, Debug)]
pub struct Options {
    /// Starting orientations tried (1..=12, see [`STARTS`]).
    pub starts: usize,
    /// Partial solutions kept after each phase.
    pub width: usize,
    /// Stored alternative algorithms tried in phases 1, 2 and 3.
    pub alts: [usize; N_PHASES - 1],
    /// Phases split in two; only the single chain (one start, width and
    /// alternative) splits, the beam uses whole phases.
    pub split: Split,
}

impl Options {
    /// Thistlethwaite with each phase optimal: four lookups.
    pub const FAST: Options =
        Options { starts: 1, width: 1, alts: [1, 1, 1], split: Split { phase2: false, phase4: false } };
    /// Five phases: phase 2 split, its 1.4 MB table replaced by 20 KB.
    pub const FAST5: Options = Options { split: Split { phase2: true, phase4: false }, ..Options::FAST };
    /// Six phases: phases 2 and 4 split, every table in L1 or L2.
    pub const FAST6: Options = Options { split: Split { phase2: true, phase4: true }, ..Options::FAST };
    /// About a microsecond.
    pub const SHORT: Options = Options { starts: 4, width: 4, alts: [4, 2, 8], ..Options::FAST };
    /// About the time of petrus' `neutral` (and of `fast` before its steps
    /// were compiled).
    pub const SHORTER: Options = Options { starts: MAX_STARTS, width: 8, alts: [4, 4, 8], ..Options::FAST };
    pub const SHORTEST: Options = Options { starts: MAX_STARTS, width: 32, alts: [16, 4, 16], ..Options::FAST };

    pub fn preset(name: &str) -> Option<Options> {
        Some(match name {
            "fast" => Options::FAST,
            "fast5" => Options::FAST5,
            "fast6" => Options::FAST6,
            "short" => Options::SHORT,
            "shorter" => Options::SHORTER,
            "shortest" => Options::SHORTEST,
            _ => return None,
        })
    }

    pub(crate) fn clamped(&self) -> Options {
        let mut o = self.clone();
        o.starts = o.starts.clamp(1, MAX_STARTS);
        o.width = o.width.clamp(1, MAX_WIDTH);
        for p in 0..o.alts.len() {
            o.alts[p] = o.alts[p].clamp(1, MAX_ALTS[p]);
        }
        o
    }
}

// ------------------------------------------------------------- solution

/// Longest solution before cancellation: 7 + 10 + 13 + 15 moves for whole
/// phases, 7 + (6 + 10) + 13 + (4 + 14) with both split.
pub const MAX_LEN: usize = 54;
pub type Seq = Moves<MAX_LEN>;

#[derive(Clone, Copy)]
pub struct Solution {
    /// The solution, with turns merged across phase boundaries.
    pub moves: Seq,
    /// Moves of each phase before merging.
    pub phase_lens: [u8; N_PHASES],
    /// The start it came from (see [`STARTS`]).
    pub start: u8,
    /// Candidate algorithms evaluated.
    pub evals: u32,
}

pub fn solve(t: &Tables, cube: &Cube, opts: &Options) -> Solution {
    let o = opts.clamped();
    if o.starts == 1 && o.width == 1 && o.alts.iter().all(|&a| a == 1) {
        return match (o.split.phase2, o.split.phase4) {
            (false, false) => solve_chain::<false, false>(t, cube),
            (true, false) => solve_chain::<true, false>(t, cube),
            (false, true) => solve_chain::<false, true>(t, cube),
            (true, true) => solve_chain::<true, true>(t, cube),
        };
    }
    solve_beam(t, cube, &o)
}
