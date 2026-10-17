//! Merging turns across phase boundaries: the beam's estimate while
//! ranking, the general merge (as petrus' `Moves::push_merged`) for its
//! final solution, and the single chain's table-driven version.
//!
//! Each phase's algorithm is canonical, so once two of its moves went in
//! unmerged, the rest cannot merge: only the first few moves of each
//! algorithm are merged one at a time.

use crate::algs::{shift_down, IDENTITY, IOTA};
use crate::solver::{Seq, MAX_LEN};
use petrus::cube::*;

/// No move: the face of none.
pub(crate) const NO_MOVE: u8 = NMOVES as u8;
const QUARTERS: u8 = 4;
/// Most moves a phase boundary can save in the ranking (a full cancel).
const MAX_SAVE: u8 = 2;

/// Moves saved by appending `first` after `last`, and the merged move
/// (`NO_MOVE` if they cancel). The ranking only merges adjacent turns of one
/// face; the final solution merges exactly.
#[inline(always)]
pub(crate) fn merge(last: u8, first: u8) -> (u8, u8) {
    if last == NO_MOVE || face(last) != face(first) {
        return (0, first);
    }
    let turns = (power(last) + power(first) + 2) % QUARTERS;
    if turns == 0 {
        (MAX_SAVE, NO_MOVE)
    } else {
        (1, mv(face(first), turns - 1))
    }
}

/// Append `m` to `q[..*n]`, merging with an earlier turn of the same face,
/// also one hidden behind a turn of the opposite face (as petrus'
/// `Moves::push_merged`).
#[inline(always)]
fn push_merged<const N: usize>(q: &mut [u8; N], n: &mut usize, m: u8) {
    let f = face(m);
    let i = if *n >= 1 && face(q[*n - 1]) == f {
        *n - 1
    } else if *n >= 2 && face(q[*n - 1]) == opposite(f) && face(q[*n - 2]) == f {
        *n - 2
    } else {
        q[*n] = m;
        *n += 1;
        return;
    };
    let turns = (power(q[i]) + power(m) + 2) % QUARTERS;
    if turns != 0 {
        q[i] = mv(f, turns - 1);
    } else {
        q.copy_within(i + 1..*n, i);
        *n -= 1;
    }
}

/// Append `moves` to `q[..*n]`, merging turns across the boundary.
#[inline(always)]
pub(crate) fn append_merged(q: &mut [u8; MAX_LEN], n: &mut usize, moves: impl Iterator<Item = u8>) {
    let mut plain = 0;
    for m in moves {
        if plain >= 2 {
            q[*n] = m;
            *n += 1;
        } else {
            let before = *n;
            push_merged(q, n, m);
            plain = if *n > before { plain + 1 } else { 0 };
        }
    }
}

// ------------------------------------------------------------- the chain's merge

/// How a move joins a solution, by the solution's last two moves (see
/// `push_merged`); `NO_MOVE` (also the padding of an algorithm's moves)
/// stands for no move.
const APPEND: u8 = 0;
const MERGE_LAST: u8 = 1;
const MERGE_HIDDEN: u8 = 2;
const CANCEL_LAST: u8 = 3;
const CANCEL_HIDDEN: u8 = 4;
const NOTHING: u8 = 5;
const MOVE_FIELD: u32 = 5;
/// How far before the end a merge into the hidden second-last turn stores.
const SHRINK: usize = 2;
const MOVE_VALUES: usize = NMOVES + 1;

/// By (second-last, last, new move): the outcome << `MOVE_FIELD` | the
/// merged move.
static MERGE: [u8; MOVE_VALUES * MOVE_VALUES * MOVE_VALUES] = {
    let mut t = [0u8; MOVE_VALUES * MOVE_VALUES * MOVE_VALUES];
    let mut i = 0;
    while i < t.len() {
        let (a2, a1, m) =
            ((i / MOVE_VALUES / MOVE_VALUES) as u8, (i / MOVE_VALUES % MOVE_VALUES) as u8, (i % MOVE_VALUES) as u8);
        // NO_MOVE's face is none of the six
        let f = face(m);
        let same = face(a1) == f;
        let hidden = face(a1) == opposite(f) && face(a2) == f;
        t[i] = if m == NO_MOVE {
            NOTHING << MOVE_FIELD
        } else if same || hidden {
            let target = if same { a1 } else { a2 };
            let turns = (power(target) + power(m) + 2) % QUARTERS;
            match (same, turns) {
                (true, 0) => CANCEL_LAST << MOVE_FIELD,
                (false, 0) => CANCEL_HIDDEN << MOVE_FIELD,
                (true, _) => MERGE_LAST << MOVE_FIELD | mv(f, turns - 1),
                (false, _) => MERGE_HIDDEN << MOVE_FIELD | mv(f, turns - 1),
            }
        } else {
            APPEND << MOVE_FIELD
        };
        i += 1;
    }
    t
};

const BUF_LEN: usize = MAX_LEN + LANES;
/// Moves of history kept in the window.
const WIN: usize = 8;
const LANE_BITS: u32 = u8::BITS;
const TOP: u32 = (WIN as u32 - 1) * LANE_BITS;
const SECOND: u32 = TOP - LANE_BITS;
const BYTE: u64 = u8::MAX as u64;
const NO_MOVES: u64 = u64::from_le_bytes([NO_MOVE; WIN]);

/// Moves merged at each phase boundary before the rest of the algorithm is
/// stored at once; a cascade longer than this (cancellations exposing
/// further merges) continues in a loop.
const MERGE_STEPS: usize = 3;
const _: () = assert!(MERGE_STEPS + 2 <= WIN);
// padding is `NO_MOVE` to the merge table and the identity to the pair tables
const _: () = assert!(IDENTITY == NO_MOVE as usize);

/// The single chain's solution as it is built, merged with no branch on
/// the cube except the merge outcome's (mostly "append", predicted well):
/// a branch the CPU cannot predict would also discard the next solve, which
/// it has already started.
///
/// The moves go to `buf`, which is only written; merging reads `win`, the
/// last 8 moves held in a register (byte 7 the last, `NO_MOVE` before the
/// start), so each outcome is a shift or a byte insert. Each boundary uses
/// at most `MERGE_STEPS` of them; the slow path reloads them.
pub(crate) struct Out {
    buf: [u8; BUF_LEN],
    n: usize,
    win: u64,
}

#[inline(always)]
fn set_lane(w: u64, at: u32, v: u8) -> u64 {
    (w & !(BYTE << at)) | (v as u64) << at
}

impl Out {
    #[inline(always)]
    pub(crate) fn new() -> Out {
        Out { buf: [0; BUF_LEN], n: 0, win: NO_MOVES }
    }

    /// Append the first `n` of `moves` (the first algorithm: nothing to
    /// merge with).
    #[inline(always)]
    pub(crate) fn start(&mut self, moves: [u8; LANES], n: usize) {
        self.extend(moves, 0, n);
    }

    /// Append the first `n` of `moves` (padded with `NO_MOVE`), merging
    /// turns across the boundary.
    #[inline(always)]
    pub(crate) fn append(&mut self, moves: [u8; LANES], n: usize) {
        let mut plain = 0;
        for &m in moves.iter().take(MERGE_STEPS) {
            plain = if self.push(m) { 0 } else { plain + 1 };
        }
        let mut k = MERGE_STEPS.min(n);
        if k < n && plain < 2 {
            k = self.keep_merging(&moves, k, n);
        }
        self.extend(moves, k, n);
    }

    /// The solution so far.
    #[inline(always)]
    pub(crate) fn moves(&self) -> Seq {
        Seq::from_array(self.buf[..MAX_LEN].try_into().unwrap(), self.n)
    }

    /// `push_merged` for move `m` (`NO_MOVE`: nothing); returns whether it
    /// merged.
    #[inline(always)]
    fn push(&mut self, m: u8) -> bool {
        let w = self.win;
        let (a1, a2) = ((w >> TOP) as u8, (w >> SECOND) as u8);
        let e = MERGE[(a2 as usize * MOVE_VALUES + a1 as usize) * MOVE_VALUES + m as usize];
        let (code, c) = (e >> MOVE_FIELD, e & ((1 << MOVE_FIELD) - 1));
        let hidden_kept = set_lane(w, SECOND, a1);
        // a `match`: its branches are predicted (mostly APPEND), so pushes do
        // not wait on each other; forced selects measured slower
        let (win, pos, val, grow) = match code {
            APPEND => ((w >> LANE_BITS) | (m as u64) << TOP, self.n, m, 1),
            MERGE_LAST => (set_lane(w, TOP, c), self.n - 1, c, 0),
            MERGE_HIDDEN => (set_lane(w, SECOND, c), self.n - SHRINK, c, 0),
            CANCEL_LAST => (w << LANE_BITS | NO_MOVE as u64, self.n - 1, a1, -1),
            // a cancelled hidden turn: the last moves down over it
            CANCEL_HIDDEN => (hidden_kept << LANE_BITS | NO_MOVE as u64, self.n - SHRINK, a1, -1),
            _ => (w, self.n, m, 0),
        };
        self.win = win;
        self.buf[pos] = val;
        self.n = self.n.wrapping_add_signed(grow);
        (MERGE_LAST..=CANCEL_HIDDEN).contains(&code)
    }

    /// Rare: still merging after `MERGE_STEPS` moves. Merge from `moves[k]`
    /// until two moves went in unmerged, refreshing the window from the
    /// buffer each time; returns the moves consumed.
    #[cold]
    #[inline(never)]
    fn keep_merging(&mut self, moves: &[u8; LANES], mut k: usize, n: usize) -> usize {
        let mut plain = 0;
        while k < n && plain < 2 {
            self.reload();
            plain = if self.push(moves[k]) { 0 } else { plain + 1 };
            k += 1;
        }
        self.reload();
        k
    }

    /// Reload the window from the buffer (after cancellations used it up).
    fn reload(&mut self) {
        let mut w = [NO_MOVE; WIN];
        let k = self.n.min(WIN);
        w[WIN - k..].copy_from_slice(&self.buf[self.n - k..self.n]);
        self.win = u64::from_le_bytes(w);
    }

    /// Append `moves[k..n]`, which cannot merge, with one store.
    #[inline(always)]
    fn extend(&mut self, moves: [u8; LANES], k: usize, n: usize) {
        let rest = shift_down(moves, k);
        self.buf[self.n..self.n + LANES].copy_from_slice(&rest);
        let r = n - k;
        self.n += r;
        self.win = last_lanes(self.win, rest, r);
    }
}

/// The last 8 of window `w`'s moves followed by `rest[..r]`.
#[inline(always)]
fn last_lanes(w: u64, rest: [u8; LANES], r: usize) -> u64 {
    #[cfg(target_arch = "aarch64")]
    // SAFETY: NEON is mandatory on aarch64.
    unsafe {
        use std::arch::aarch64::*;
        let v = std::mem::transmute::<[u8; LANES], uint8x16_t>(rest);
        // the moves in order: w's 8, then rest's 16
        let t = uint8x16x2_t(vcombine_u8(vcreate_u8(w), vget_low_u8(v)), vcombine_u8(vget_high_u8(v), vdup_n_u8(0)));
        let idx = vadd_u8(vld1_u8(IOTA.as_ptr()), vdup_n_u8(r as u8));
        vget_lane_u64::<0>(vreinterpret_u64_u8(vqtbl2_u8(t, idx)))
    }
    #[cfg(not(target_arch = "aarch64"))]
    {
        let mut all = [0u8; WIN + LANES];
        all[..WIN].copy_from_slice(&w.to_le_bytes());
        all[WIN..].copy_from_slice(&rest);
        u64::from_le_bytes(all[r..r + WIN].try_into().unwrap())
    }
}
