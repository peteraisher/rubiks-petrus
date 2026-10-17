//! Phase coordinates: where a cube stands within each phase's coset space.
//!
//! Thistlethwaite's chain, in Kociemba's orientation conventions (F/B quarter
//! turns flip edges, U/D quarter turns keep corner twist):
//!
//! | phase | from | to | coordinate | cases |
//! |---|---|---|---|---|
//! | 1 | any cube | G1 = <U,D,R,L,F2,B2> | edge orientation | 2048 |
//! | 2 | G1 | G2 = <U,D,R2,L2,F2,B2> | corner twist x E-slice edges | 1 082 565 |
//! | 3 | G2 | G3 = <U2,D2,R2,L2,F2,B2> | corner coset x M-slice edges | 29 400 |
//! | 4 | G3 | solved | the element itself | 663 552 |
//!
//! Each coordinate is read straight off petrus' lane vectors (piece ->
//! position + orientation) as a sum of per-lane lookups plus at most one
//! rank table, and is only meaningful for cubes already in the phase's
//! starting group. Shared with `build.rs`.

use petrus::cube::*;
use petrus::facelet::{NSYM, SYM_MOVE};

pub const N_PHASES: usize = 4;
/// Index of the last phase, which solves the cube.
pub const LAST: usize = N_PHASES - 1;

/// Moves of each phase's group, in petrus' move order.
pub const MOVES1: [u8; NMOVES] = {
    let mut a = [0u8; NMOVES];
    let mut i = 0;
    while i < NMOVES {
        a[i] = i as u8;
        i += 1;
    }
    a
};
const HALF: u8 = 1;
pub const MOVES2: [u8; 14] = [
    mv(FACE_U, 0),
    mv(FACE_U, 1),
    mv(FACE_U, 2),
    mv(FACE_R, 0),
    mv(FACE_R, 1),
    mv(FACE_R, 2),
    mv(FACE_F, HALF),
    mv(FACE_D, 0),
    mv(FACE_D, 1),
    mv(FACE_D, 2),
    mv(FACE_L, 0),
    mv(FACE_L, 1),
    mv(FACE_L, 2),
    mv(FACE_B, HALF),
];
pub const MOVES3: [u8; 10] = [
    mv(FACE_U, 0),
    mv(FACE_U, 1),
    mv(FACE_U, 2),
    mv(FACE_R, HALF),
    mv(FACE_F, HALF),
    mv(FACE_D, 0),
    mv(FACE_D, 1),
    mv(FACE_D, 2),
    mv(FACE_L, HALF),
    mv(FACE_B, HALF),
];
pub const MOVES4: [u8; 6] =
    [mv(FACE_U, HALF), mv(FACE_R, HALF), mv(FACE_F, HALF), mv(FACE_D, HALF), mv(FACE_L, HALF), mv(FACE_B, HALF)];

/// Each phase's moves, by slot.
pub const PHASE_MOVES: [&[u8]; N_PHASES] = [&MOVES1, &MOVES2, &MOVES3, &MOVES4];

// ------------------------------------------------------------- sizes

/// Edge-orientation masks with even parity: the last bit is implied.
pub const N1: usize = 1 << (N_EDGES - 1);
/// Corner twists by position; the last corner's twist is implied.
pub const TWISTS: usize = CORNER_TWISTS.pow(N_CORNERS as u32 - 1);
const SLICE_EDGES: usize = 4;
/// Placements of the four E-slice edges among 12 positions.
pub const SLICE_COMBS: usize = binom(N_EDGES, SLICE_EDGES);
pub const N2: usize = TWISTS * SLICE_COMBS;
pub const CORNER_PERMS: usize = factorial(N_CORNERS);
/// Corner permutations modulo the squares group's 96 corner permutations.
pub const CORNER_COSETS: usize = 420;
pub const G3_CORNER_PERMS: usize = CORNER_PERMS / CORNER_COSETS;
/// U/D-layer edge positions, where the M-slice edges sit in G2.
const UD_EDGES: usize = N_EDGES - SLICE_EDGES;
pub const M_COMBS: usize = binom(UD_EDGES, SLICE_EDGES);
pub const N3: usize = CORNER_COSETS * M_COMBS;
/// Permutations of one slice's four edges.
pub const SLICE_PERMS: usize = factorial(SLICE_EDGES);
/// Three slices' edge permutations with even total parity.
pub const G3_EDGE_PERMS: usize = SLICE_PERMS * SLICE_PERMS * SLICE_PERMS / 2;
pub const N4: usize = G3_CORNER_PERMS * G3_EDGE_PERMS;

pub const PHASE_SIZES: [usize; N_PHASES] = [N1, N2, N3, N4];
/// Each phase's longest optimal algorithm (its diameter, asserted by the
/// generator).
pub const PHASE_MAX_LEN: [usize; N_PHASES] = [7, 10, 13, 15];

const fn binom(n: usize, k: usize) -> usize {
    if k > n {
        return 0;
    }
    let mut r = 1;
    let mut i = 0;
    while i < k {
        r = r * (n - i) / (i + 1);
        i += 1;
    }
    r
}

const fn factorial(n: usize) -> usize {
    let mut r = 1;
    let mut i = 2;
    while i <= n {
        r *= i;
        i += 1;
    }
    r
}

/// Rank of a 4-element subset given as a bit mask (colex order), or
/// `u16::MAX` for masks of another size.
const fn subset_ranks<const N: usize>() -> [u16; N] {
    let mut t = [u16::MAX; N];
    let mut mask = 0;
    while mask < N {
        if (mask as u32).count_ones() as usize == SLICE_EDGES {
            let mut r = 0;
            let mut k = 0;
            let mut p = 0;
            while p < usize::BITS as usize && (1 << p) <= mask {
                if mask & (1 << p) != 0 {
                    k += 1;
                    r += binom(p, k);
                }
                p += 1;
            }
            t[mask] = r as u16;
        }
        mask += 1;
    }
    t
}

const EO_MASK_BITS: usize = N1 - 1;
/// E-slice placement rank by 12-bit position mask.
static SLICE_RANK: [u16; 1 << N_EDGES] = subset_ranks::<{ 1 << N_EDGES }>();
/// M-slice placement rank by 8-bit position mask (U/D-layer positions).
static M_RANK: [u16; 1 << UD_EDGES] = subset_ranks::<{ 1 << UD_EDGES }>();

/// Corner lane value (pos + 8 * twist) -> twist * 3^pos, the last position
/// left out (its twist is implied).
static TWIST_WEIGHT: [u16; TABLE_LEN] = {
    let mut t = [0u16; TABLE_LEN];
    let mut v = 0;
    while v < LANE_VALUES {
        let (pos, twist) = (v % N_CORNERS, v / N_CORNERS);
        if pos < N_CORNERS - 1 {
            t[v] = (twist * CORNER_TWISTS.pow(pos as u32)) as u16;
        }
        v += 1;
    }
    t
};

// ------------------------------------------------------------- piece groups

/// Groups of four pieces that each stay among their own positions in the
/// squares group G3: two corner tetrads and the three edge slices.
pub type Four = [usize; SLICE_EDGES];
pub const TETRAD_A: Four = [URF, ULB, DLF, DRB];
pub const TETRAD_B: Four = [UFL, UBR, DFR, DBL];
pub const M_SLICE: Four = [UF, UB, DF, DB];
pub const S_SLICE: Four = [UR, UL, DR, DL];
pub const E_SLICE: Four = [FR, FL, BL, BR];

/// The six pairs (i < j) of a group of four, in comparison-bit order.
const PAIRS: [(usize, usize); 6] = [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)];
/// An order pattern: bit k set when pair k is inverted (its first piece sits
/// at the higher position). It fixes the pieces' relative order, so for
/// four pieces on four known positions it is their permutation.
pub const PATTERNS: usize = 1 << PAIRS.len();
const PATTERN_BITS: usize = PAIRS.len();

/// Lexicographic rank (0..24) of the permutation with a given order
/// pattern: its Lehmer digits count the inverted pairs starting at each
/// piece. Ranks 2k and 2k+1 differ in the last pair only, so they have
/// opposite parity.
static PATTERN_RANK: [u8; PATTERNS] = {
    let mut t = [0u8; PATTERNS];
    let mut p = 0;
    while p < PATTERNS {
        let mut r = 0;
        let mut k = 0;
        while k < PAIRS.len() {
            if p & (1 << k) != 0 {
                // digit of the pair's first piece, weighted by (3 - i)!
                r += factorial(SLICE_EDGES - 1 - PAIRS[k].0);
            }
            k += 1;
        }
        t[p] = r as u8;
        p += 1;
    }
    t
};

/// Corner tetrads in the squares group: 6 classes of tetrad orders per
/// placement of tetrad A (see `gen::corner_tables`).
pub const TETRAD_CLASSES: usize = CORNER_COSETS / M_COMBS;

/// Indexed by corner order patterns (tetrad A | tetrad B << 6): the class
/// of the pair of orders under the squares group's corner permutations
/// (0..6), and for members of that group, their index (0..96, else
/// `u8::MAX`).
pub struct CornerTables {
    pub class: &'static [u8],
    pub g3: &'static [u8],
}

// ------------------------------------------------------------- lane kernels

/// Shuffle indices putting each pair's first (`SECOND` = false) or second
/// piece of groups `lo` and `hi` in lanes 0..6 and 8..14; other lanes get
/// an out-of-range index, which a table lookup turns into 0.
const fn pair_lanes(lo: Four, hi: Four, second: bool) -> [u8; LANES] {
    let mut t = [u8::MAX; LANES];
    let half = LANES / 2;
    let mut k = 0;
    while k < PAIRS.len() {
        let (i, j) = PAIRS[k];
        let q = if second { j } else { i };
        t[k] = lo[q] as u8;
        t[half + k] = hi[q] as u8;
        k += 1;
    }
    t
}

/// Byte i: 1 << (i % 8) for the pair lanes, 0 elsewhere.
const PAIR_WEIGHTS: [u8; LANES] = {
    let mut t = [0u8; LANES];
    let mut k = 0;
    while k < PAIRS.len() {
        t[k] = 1 << k;
        t[LANES / 2 + k] = 1 << k;
        k += 1;
    }
    t
};

/// 0xFF in the lanes of the given pieces.
const fn lanes_of(g: Four) -> [u8; LANES] {
    let mut t = [0u8; LANES];
    let mut k = 0;
    while k < g.len() {
        t[g[k]] = u8::MAX;
        k += 1;
    }
    t
}

/// Lane value -> bit of its position, low byte (positions 0..8) and high
/// byte (8..16); values past the table give 0.
const fn position_bits(high: bool) -> [u8; LANES] {
    let mut t = [0u8; LANES];
    let mut v = 0;
    while v < LANES {
        if (v >= u8::BITS as usize) == high {
            t[v] = 1 << (v % u8::BITS as usize);
        }
        v += 1;
    }
    t
}

const fn split_bytes(w: &[u16; TABLE_LEN], high: bool) -> [u8; TABLE_LEN] {
    let mut t = [0u8; TABLE_LEN];
    let mut i = 0;
    while i < TABLE_LEN {
        t[i] = if high { (w[i] >> u8::BITS) as u8 } else { w[i] as u8 };
        i += 1;
    }
    t
}

static CORNER_FIRST: [u8; LANES] = pair_lanes(TETRAD_A, TETRAD_B, false);
static CORNER_SECOND: [u8; LANES] = pair_lanes(TETRAD_A, TETRAD_B, true);
static MS_FIRST: [u8; LANES] = pair_lanes(M_SLICE, S_SLICE, false);
static MS_SECOND: [u8; LANES] = pair_lanes(M_SLICE, S_SLICE, true);
static E_FIRST: [u8; LANES] = pair_lanes(E_SLICE, E_SLICE, false);
static E_SECOND: [u8; LANES] = pair_lanes(E_SLICE, E_SLICE, true);
static WEIGHTS: [u8; LANES] = PAIR_WEIGHTS;
static TETRAD_A_LANES: [u8; LANES] = lanes_of(TETRAD_A);
static M_LANES: [u8; LANES] = lanes_of(M_SLICE);
static E_LANES: [u8; LANES] = lanes_of(E_SLICE);
static BITS_LO: [u8; LANES] = position_bits(false);
static BITS_HI: [u8; LANES] = position_bits(true);
static TWIST_LO: [u8; TABLE_LEN] = split_bytes(&TWIST_WEIGHT, false);
static TWIST_HI: [u8; TABLE_LEN] = split_bytes(&TWIST_WEIGHT, true);

#[cfg(target_arch = "aarch64")]
mod simd {
    use super::*;
    use std::arch::aarch64::*;

    #[inline(always)]
    fn v(x: &[u8; LANES]) -> uint8x16_t {
        // SAFETY: NEON is mandatory on aarch64; the array holds 16 bytes.
        unsafe { vld1q_u8(x.as_ptr()) }
    }

    /// Order patterns of the two groups `first`/`second` were built for.
    #[inline(always)]
    pub fn patterns(x: &[u8; LANES], first: &[u8; LANES], second: &[u8; LANES]) -> (usize, usize) {
        // SAFETY: NEON is mandatory on aarch64.
        unsafe {
            let x = v(x);
            let gt = vcgtq_u8(vqtbl1q_u8(x, v(first)), vqtbl1q_u8(x, v(second)));
            let w = vandq_u8(gt, v(&WEIGHTS));
            (vaddv_u8(vget_low_u8(w)) as usize, vaddv_u8(vget_high_u8(w)) as usize)
        }
    }

    /// Bit mask of the positions (lane values 0..16) held by `lanes`.
    #[inline(always)]
    pub fn positions(x: &[u8; LANES], lanes: &[u8; LANES]) -> usize {
        // SAFETY: NEON is mandatory on aarch64.
        unsafe {
            let x = v(x);
            let sel = v(lanes);
            let lo = vandq_u8(vqtbl1q_u8(v(&BITS_LO), x), sel);
            let hi = vandq_u8(vqtbl1q_u8(v(&BITS_HI), x), sel);
            vaddvq_u8(lo) as usize | (vaddvq_u8(hi) as usize) << u8::BITS
        }
    }

    #[inline(always)]
    pub fn twist(c: &[u8; LANES]) -> usize {
        // SAFETY: NEON is mandatory on aarch64; the tables hold 32 bytes.
        unsafe {
            let x = v(c);
            let lo = vqtbl2q_u8(vld1q_u8_x2(TWIST_LO.as_ptr()), x);
            let hi = vqtbl2q_u8(vld1q_u8_x2(TWIST_HI.as_ptr()), x);
            vaddlvq_u8(lo) as usize + ((vaddlvq_u8(hi) as usize) << u8::BITS)
        }
    }
}

#[cfg(not(target_arch = "aarch64"))]
mod simd {
    use super::*;

    #[inline(always)]
    pub fn patterns(x: &[u8; LANES], first: &[u8; LANES], second: &[u8; LANES]) -> (usize, usize) {
        let pick = |i: usize| x.get(first[i] as usize).copied().unwrap_or(0);
        let mut w = [0u8; LANES];
        for i in 0..LANES {
            let b = x.get(second[i] as usize).copied().unwrap_or(0);
            w[i] = if pick(i) > b { WEIGHTS[i] } else { 0 };
        }
        let half = LANES / 2;
        (w[..half].iter().map(|&b| b as usize).sum(), w[half..].iter().map(|&b| b as usize).sum())
    }

    #[inline(always)]
    pub fn positions(x: &[u8; LANES], lanes: &[u8; LANES]) -> usize {
        (0..LANES).filter(|&i| lanes[i] != 0 && (x[i] as usize) < LANES).map(|i| 1 << x[i]).sum()
    }

    #[inline(always)]
    pub fn twist(c: &[u8; LANES]) -> usize {
        c.iter().map(|&x| TWIST_WEIGHT[x as usize % TABLE_LEN] as usize).sum()
    }
}

// ------------------------------------------------------------- coordinates

#[inline(always)]
pub fn idx1(c: &Cube) -> usize {
    c.eo12() as usize & EO_MASK_BITS
}

/// Corner twist by position, 0..3^7.
#[inline(always)]
pub fn twist(c: &Cube) -> usize {
    simd::twist(&c.c)
}

/// Placement of the E-slice edges, 0..495. Requires a cube in G1 (edges
/// oriented, so edge lane value = position).
#[inline(always)]
pub fn slice_rank(c: &Cube) -> usize {
    SLICE_RANK[simd::positions(&c.e, &E_LANES)] as usize
}

/// Requires a cube in G1.
#[inline(always)]
pub fn idx2(c: &Cube) -> usize {
    twist(c) * SLICE_COMBS + slice_rank(c)
}

/// Positions of tetrad A (8-bit mask) and the order patterns of both
/// tetrads (A | B << 6), for a cube with corners twisted correctly.
#[inline(always)]
pub fn corner_patterns(c: &Cube) -> (usize, usize) {
    let (a, b) = simd::patterns(&c.c, &CORNER_FIRST, &CORNER_SECOND);
    (simd::positions(&c.c, &TETRAD_A_LANES), a | b << PATTERN_BITS)
}

/// Requires a cube in G2.
#[inline(always)]
pub fn idx3(ct: &CornerTables, c: &Cube) -> usize {
    let (placement, orders) = corner_patterns(c);
    let coset = M_RANK[placement] as usize * TETRAD_CLASSES + ct.class[orders] as usize;
    coset * M_COMBS + M_RANK[simd::positions(&c.e, &M_LANES)] as usize
}

/// The squares group's corner permutation, 0..96. Requires a cube in G3.
#[inline(always)]
pub fn g3_corners(ct: &CornerTables, c: &Cube) -> usize {
    ct.g3[corner_patterns(c).1] as usize
}

/// The squares group's edge arrangement (the E-slice's parity is implied
/// by the corners'), 0..6912. Requires a cube in G3.
#[inline(always)]
pub fn g3_edges(c: &Cube) -> usize {
    let (m, s) = simd::patterns(&c.e, &MS_FIRST, &MS_SECOND);
    let (e, _) = simd::patterns(&c.e, &E_FIRST, &E_SECOND);
    let rank = |p: usize| PATTERN_RANK[p] as usize;
    (rank(m) * SLICE_PERMS + rank(s)) * (SLICE_PERMS / 2) + rank(e) / 2
}

/// Requires a cube in G3.
#[inline(always)]
pub fn idx4(ct: &CornerTables, c: &Cube) -> usize {
    g3_corners(ct, c) * G3_EDGE_PERMS + g3_edges(c)
}

// ------------------------------------------------------------- split phases

/// Phases 2 and 4 can each be split in two smaller tables, at the cost of
/// moves: the first half solves one coordinate optimally, the second solves
/// the rest optimally with the phase's moves, restoring the first's on the
/// way. The halves, in order: E-slice placement (G1 moves); corner twist
/// with the E-slice placed; the corner permutation (half turns); the edges
/// with the corners solved.
pub const N_HALVES: usize = 4;
pub const HALF_SLICE: usize = 0;
pub const HALF_TWIST: usize = 1;
pub const HALF_CORNERS: usize = 2;
pub const HALF_EDGES: usize = 3;
/// Phase 2, the first phase that can be split.
pub const PHASE_2: usize = 1;
pub const HALF_SIZES: [usize; N_HALVES] = [SLICE_COMBS, TWISTS, G3_CORNER_PERMS, G3_EDGE_PERMS];
/// Each half's longest algorithm (asserted by the generator).
pub const HALF_MAX_LEN: [usize; N_HALVES] = [5, 10, 4, 14];
/// The phase each half belongs to (its moves and move slots).
pub const HALF_PHASE: [usize; N_HALVES] = [PHASE_2, PHASE_2, LAST, LAST];
// ------------------------------------------------------------- joining phases

/// The join table (`gen::join_table`): by the solution's last two moves and
/// the next algorithm's first two (each a move or `NO_MOVE`), what merging
/// those two does. An entry's bytes 0-2 are the moves that replace the
/// solution's last `drop`, byte 3 packs `drop`, their count and the
/// algorithm moves consumed (2 bits each) and a flag for the rare cases
/// that need the exact general merge.
pub const JOIN_VALUES: usize = NMOVES + 1;
pub const JOIN_SIZE: usize = JOIN_VALUES * JOIN_VALUES * JOIN_VALUES * JOIN_VALUES;
pub const JOIN_MAX_REPLACE: usize = 3;
pub const JOIN_FIELD_BITS: u32 = 2;
pub const JOIN_DROP: u32 = (JOIN_MAX_REPLACE as u32) * u8::BITS;
pub const JOIN_REPLACE: u32 = JOIN_DROP + JOIN_FIELD_BITS;
pub const JOIN_TAKEN: u32 = JOIN_REPLACE + JOIN_FIELD_BITS;
pub const JOIN_COLD: u32 = 1 << (JOIN_TAKEN + JOIN_FIELD_BITS);
pub const JOIN_FIELD: u32 = (1 << JOIN_FIELD_BITS) - 1;

/// Index of the join entry for moves `m0`, `m1` after `a2`, `a1`.
#[inline(always)]
pub fn join_index(a2: u8, a1: u8, m0: u8, m1: u8) -> usize {
    // two independent halves: the algorithm's is known before the solution's
    let pair = |x: u8, y: u8| x as usize * JOIN_VALUES + y as usize;
    pair(a2, a1) * (JOIN_VALUES * JOIN_VALUES) + pair(m0, m1)
}

/// Halves stored composed as well (see [`COMPOSED`]): all but the last.
pub const HALF_COMPOSED: [bool; N_HALVES] = [true, true, true, false];

// ------------------------------------------------------------- symmetry

/// Symmetries (identity first) that keep the U/D axis in place, and also
/// the R/L axis when `keep_r`. Those keeping every axis map every group of
/// the chain to itself; those keeping the U/D axis map G2 and G3 to
/// themselves. A case and its conjugates then have the same distance and
/// conjugate algorithms.
const fn axis_syms<const N: usize>(keep_r: bool) -> [u8; N] {
    let mut out = [0u8; N];
    let mut n = 0;
    let mut s = 0;
    while s < NSYM {
        let n_axes = (N_FACES / 2) as u8;
        let u_axis = face(SYM_MOVE[s][mv(FACE_U, 0) as usize]) % n_axes;
        let r_axis = face(SYM_MOVE[s][mv(FACE_R, 0) as usize]) % n_axes;
        if u_axis == FACE_U && (r_axis == FACE_R || !keep_r) {
            out[n] = s as u8;
            n += 1;
        }
        s += 1;
    }
    assert!(n == N && out[0] == 0);
    out
}

pub const N_CHAIN_SYMS: usize = 8;
pub const CHAIN_SYMS: [u8; N_CHAIN_SYMS] = axis_syms(true);
pub const N_UD_SYMS: usize = 16;
pub const UD_SYMS: [u8; N_UD_SYMS] = axis_syms(false);
pub const MAX_SYMS: usize = N_UD_SYMS;
pub const SYM_BITS: usize = MAX_SYMS.trailing_zeros() as usize;
/// The symmetries each phase's tables are reduced by (phase 1 is stored
/// whole).
pub const PHASE_SYMS: [&[u8]; N_PHASES] = [&[0], &CHAIN_SYMS, &UD_SYMS, &UD_SYMS];

/// Phases 2-4 index a case as `major * MINOR + minor`. Algorithm tables
/// are stored only for symmetry classes of the minor part (E-slice and
/// M-slice placements, edge arrangement in G3) under the phase's symmetries
/// (`PHASE_SYMS`): a cube is conjugated so that its minor part is its class
/// representative, its major part (twist, corner coset, corner permutation)
/// through a per-symmetry table.
pub const MINOR: [usize; N_PHASES] = [N1, SLICE_COMBS, M_COMBS, G3_EDGE_PERMS];
pub const MAJOR: [usize; N_PHASES] = [1, TWISTS, CORNER_COSETS, G3_CORNER_PERMS];

// ------------------------------------------------------------- algorithms

/// An algorithm packed in a `u64`: the length in the low bits, then each
/// move as its slot in the phase's move list.
pub const LEN_BITS: usize = 4;
/// Phase 1's 18 moves take a byte each (7 at most), the others' 14 or fewer
/// a nibble (15 at most): every algorithm unpacks with vector shuffles.
pub const MOVE_BITS: [usize; N_PHASES] = [8, 4, 4, 4];
/// Marks an unused alternative slot.
pub const NO_ALG: u64 = u64::MAX;

#[inline(always)]
pub fn alg_len(a: u64) -> usize {
    (a & ((1 << LEN_BITS) - 1)) as usize
}

#[inline(always)]
pub fn alg_slot(a: u64, bits: usize, i: usize) -> usize {
    ((a >> (LEN_BITS + bits * i)) & ((1 << bits) - 1)) as usize
}

pub fn encode(slots: &[usize], bits: usize) -> u64 {
    assert!(slots.len() < 1 << LEN_BITS && LEN_BITS + bits * slots.len() <= u64::BITS as usize);
    let mut a = slots.len() as u64;
    for (i, &s) in slots.iter().enumerate() {
        assert!(s < 1 << bits);
        a |= (s as u64) << (LEN_BITS + bits * i);
    }
    a
}

/// Alternative algorithms stored per case, per phase (powers of two).
/// Phase 2 and 4 store one: the search gains most from choosing among
/// phase 1 and phase 3 solutions, and phase 4 is final.
pub const MAX_ALTS: [usize; N_PHASES] = [16, 4, 16, 1];

/// Phases whose cases also store their first algorithm composed into one
/// permutation (as lane tables), so that the chain applies it with one
/// lookup: phase 1 (128 KB) and phase 3 (1.9 MB). Phase 2's would be 69 MB,
/// and nothing reads the cube after phase 4.
pub const COMPOSED: [bool; N_PHASES] = [true, false, true, false];
