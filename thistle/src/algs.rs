//! Stored algorithms: unpacking their moves (mapped back through the
//! symmetry their case was reduced by) and applying them to a cube, two
//! moves per lane-table lookup.

use crate::coord::*;
use petrus::cube::*;
use petrus::facelet::SYM_MOVE;

/// Each phase's move slots mapped through each of its symmetries.
static SLOT_MOVE: [[[u8; NMOVES]; MAX_SYMS]; N_PHASES] = {
    let mut t = [[[0u8; NMOVES]; MAX_SYMS]; N_PHASES];
    let mut p = 0;
    while p < N_PHASES {
        let mut k = 0;
        while k < PHASE_SYMS[p].len() {
            let mut i = 0;
            while i < PHASE_MOVES[p].len() {
                t[p][k][i] = SYM_MOVE[PHASE_SYMS[p][k] as usize][PHASE_MOVES[p][i] as usize];
                i += 1;
            }
            k += 1;
        }
        p += 1;
    }
    t
};

/// Move `i` of phase `P`'s algorithm `a`, mapped through symmetry `sym`.
#[inline(always)]
pub(crate) fn alg_move<const P: usize>(a: u64, sym: usize, i: usize) -> u8 {
    SLOT_MOVE[P][sym][alg_slot(a, MOVE_BITS[P], i)]
}

/// Phase `p`'s algorithm `a` as a move sequence.
pub(crate) fn decode(p: usize, a: u64, sym: usize) -> Vec<u8> {
    (0..alg_len(a)).map(|i| SLOT_MOVE[p][sym][alg_slot(a, MOVE_BITS[p], i)]).collect()
}

// ------------------------------------------------------------- unpacking

/// Phases 2-4's move slots (at most 16) mapped through each symmetry, as
/// one shuffle table each.
static SLOT_MOVE16: [[[u8; LANES]; MAX_SYMS]; N_PHASES] = {
    let mut t = [[[0u8; LANES]; MAX_SYMS]; N_PHASES];
    let mut p = 1;
    while p < N_PHASES {
        assert!(PHASE_MOVES[p].len() <= LANES);
        let mut k = 0;
        while k < MAX_SYMS {
            let mut i = 0;
            while i < PHASE_MOVES[p].len() {
                t[p][k][i] = SLOT_MOVE[p][k][i];
                i += 1;
            }
            k += 1;
        }
        p += 1;
    }
    t
};

const NIBBLE: usize = 4;
const _: () = assert!(MOVE_BITS[0] == u8::BITS as usize && MOVE_BITS[1] == NIBBLE && MOVE_BITS[2] == NIBBLE);
const _: () = assert!(MOVE_BITS[LAST] == NIBBLE && PHASE_SYMS[0].len() == 1);

/// 0, 1, ..., 15: lane indices.
pub(crate) static IOTA: [u8; LANES] = {
    let mut t = [0u8; LANES];
    let mut i = 0;
    while i < LANES {
        t[i] = i as u8;
        i += 1;
    }
    t
};

/// Algorithm `a`'s moves one per byte, mapped through symmetry `sym`
/// (bytes past its length are unspecified).
#[inline(always)]
pub(crate) fn alg_moves<const P: usize>(a: u64, sym: usize) -> [u8; LANES] {
    #[cfg(target_arch = "aarch64")]
    // SAFETY: NEON is mandatory on aarch64; the shuffle table holds 16 bytes.
    unsafe {
        use std::arch::aarch64::*;
        let v = vcreate_u8(a >> LEN_BITS);
        let out = if P == 0 {
            // byte slots, and phase 1 is not reduced: slot = move
            vcombine_u8(v, vdup_n_u8(0))
        } else {
            let lo = vand_u8(v, vdup_n_u8((1 << NIBBLE) - 1));
            let hi = vshr_n_u8::<{ NIBBLE as i32 }>(v);
            let slots = vcombine_u8(vzip1_u8(lo, hi), vzip2_u8(lo, hi));
            vqtbl1q_u8(vld1q_u8(SLOT_MOVE16[P][sym].as_ptr()), slots)
        };
        std::mem::transmute::<uint8x16_t, [u8; LANES]>(out)
    }
    #[cfg(not(target_arch = "aarch64"))]
    {
        let mut out = [0u8; LANES];
        for (i, m) in out.iter_mut().enumerate().take(alg_len(a)) {
            *m = alg_move::<P>(a, sym, i);
        }
        out
    }
}

/// `moves` shifted down by `k` lanes (zeros shifted in).
#[inline(always)]
pub(crate) fn shift_down(moves: [u8; LANES], k: usize) -> [u8; LANES] {
    #[cfg(target_arch = "aarch64")]
    // SAFETY: NEON is mandatory on aarch64.
    unsafe {
        use std::arch::aarch64::*;
        let idx = vaddq_u8(vld1q_u8(IOTA.as_ptr()), vdupq_n_u8(k as u8));
        let v = std::mem::transmute::<[u8; LANES], uint8x16_t>(moves);
        std::mem::transmute::<uint8x16_t, [u8; LANES]>(vqtbl1q_u8(v, idx))
    }
    #[cfg(not(target_arch = "aarch64"))]
    {
        let mut out = [0u8; LANES];
        out[..LANES - k].copy_from_slice(&moves[k..]);
        out
    }
}

/// The first `n` of `moves` with every later lane set to `IDENTITY`.
#[inline(always)]
pub(crate) fn pad_identity(moves: [u8; LANES], n: usize) -> [u8; LANES] {
    #[cfg(target_arch = "aarch64")]
    // SAFETY: NEON is mandatory on aarch64.
    unsafe {
        use std::arch::aarch64::*;
        let keep = vcltq_u8(vld1q_u8(IOTA.as_ptr()), vdupq_n_u8(n as u8));
        let v = std::mem::transmute::<[u8; LANES], uint8x16_t>(moves);
        std::mem::transmute::<uint8x16_t, [u8; LANES]>(vbslq_u8(keep, v, vdupq_n_u8(IDENTITY as u8)))
    }
    #[cfg(not(target_arch = "aarch64"))]
    {
        let mut out = moves;
        out[n..].fill(IDENTITY as u8);
        out
    }
}

// ------------------------------------------------------------- applying

/// Every pair of moves composed into one lane-table pair, so an algorithm
/// is applied two moves per lookup (half the dependent table lookups). Index
/// `IDENTITY` is no move, so the pairs also hold single moves and padding.
#[repr(C, align(64))]
struct PairTables([[LaneTables; NMOVES + 1]; NMOVES + 1]);
pub(crate) const IDENTITY: usize = NMOVES;

static PAIR: PairTables = PairTables({
    let mut t = [[[[0u8; TABLE_LEN]; 2]; NMOVES + 1]; NMOVES + 1];
    let mut a = 0;
    while a <= NMOVES {
        let mut b = 0;
        while b <= NMOVES {
            let ca = if a == IDENTITY { Cubie::SOLVED } else { MOVE_CUBIE[a] };
            let cb = if b == IDENTITY { Cubie::SOLVED } else { MOVE_CUBIE[b] };
            t[a][b] = lane_tables(&ca.mul(&cb));
            b += 1;
        }
        a += 1;
    }
    t
});

/// Single moves and, last, the identity.
#[repr(C, align(64))]
struct SingleTables([LaneTables; NMOVES + 1]);

static SINGLE: SingleTables = SingleTables({
    let mut t = [lane_tables(&Cubie::SOLVED); NMOVES + 1];
    let mut m = 0;
    while m < NMOVES {
        t[m] = lane_tables(&MOVE_CUBIE[m]);
        m += 1;
    }
    t
});

/// Move pairs covering an algorithm of up to `n` moves.
pub(crate) const fn pairs(n: usize) -> usize {
    n.div_ceil(2)
}

/// Apply `moves` padded by `pad_identity`: a fixed number of pairs, so the
/// loop's trip count does not depend on the cube (no mispredicted exit).
#[inline(always)]
pub(crate) fn apply_padded<const PAIRS: usize>(c: &Cube, moves: &[u8; LANES]) -> Cube {
    let mut x = *c;
    for i in 0..PAIRS {
        x = x.apply_tables(&PAIR.0[moves[2 * i] as usize][moves[2 * i + 1] as usize]);
    }
    x
}

/// Apply phase `P`'s algorithm `a`, mapped through symmetry `sym`, as many
/// pairs as it needs.
#[inline(always)]
pub(crate) fn apply<const P: usize>(c: &Cube, a: u64, sym: usize) -> Cube {
    let n = alg_len(a);
    let mut x = *c;
    let mut i = 0;
    while i + 1 < n {
        x = x.apply_tables(&PAIR.0[alg_move::<P>(a, sym, i) as usize][alg_move::<P>(a, sym, i + 1) as usize]);
        i += 2;
    }
    // an odd last move, or the identity: the length's parity costs no branch
    let tail = if n % 2 == 1 { alg_move::<P>(a, sym, n - 1) as usize } else { IDENTITY };
    x.apply_tables(&SINGLE.0[tail])
}
