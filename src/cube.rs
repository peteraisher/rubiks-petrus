//! Cube model.
//!
//! Two representations live here:
//!
//! * [`Cubie`]: Kociemba's "what is at each position" arrays. Scalar, simple,
//!   used as the reference model and for conversions.
//! * [`Cube`]: the hot-path "where is each piece" form. One byte per piece
//!   holding `pos + N_EDGES*ori` (edges) or `pos + N_CORNERS*ori` (corners),
//!   packed into two 16-byte vectors. Applying any fixed
//!   permutation-with-twists (a face turn, a whole algorithm) maps every lane
//!   through the same 32-entry table, so a move is exactly one `tbl`
//!   instruction per vector on aarch64 (`vqtbl2q_u8`). Unused lanes hold
//!   [`EMPTY_LANE`] and every table maps the values above the real ones to
//!   themselves, so whole-vector comparisons just work.
//!
//! Move and symmetry tables are derived at compile time from 3D sticker
//! geometry (see [`crate::facelet`]), not typed in by hand.

use crate::facelet;

pub const N_FACES: usize = 6;
/// Quarter, half and inverse quarter turn of each face.
pub const POWERS: usize = 3;
pub const NMOVES: usize = N_FACES * POWERS;
pub const N_CORNERS: usize = 8;
pub const N_EDGES: usize = 12;
pub const CORNER_TWISTS: usize = 3;
pub const EDGE_FLIPS: usize = 2;
/// Bytes per state vector.
pub const LANES: usize = 16;
/// Distinct (position, orientation) values of one lane; the same for both
/// piece kinds: 12 edge positions x 2 flips = 8 corner positions x 3 twists.
pub const LANE_VALUES: usize = N_EDGES * EDGE_FLIPS;
const _: () = assert!(LANE_VALUES == N_CORNERS * CORNER_TWISTS);
pub const EMPTY_LANE: u8 = LANE_VALUES as u8;
/// Entries in a lane lookup table (what one two-register `tbl` can index).
pub const TABLE_LEN: usize = 2 * LANES;
/// Number of edge-orientation masks (one bit per edge position).
pub const EO_MASKS: usize = 1 << N_EDGES;
/// Index of the edge / corner table in a pair of lane tables.
pub const EDGE_TABLE: usize = 0;
pub const CORNER_TABLE: usize = 1;

pub const MOVE_NAMES: [&str; NMOVES] = [
    "U", "U2", "U'", "R", "R2", "R'", "F", "F2", "F'", "D", "D2", "D'", "L", "L2", "L'", "B", "B2",
    "B'",
];
pub const FACE_CHARS: [u8; N_FACES] = *b"URFDLB";

// Faces, in move order.
pub const FACE_U: u8 = 0;
pub const FACE_R: u8 = 1;
pub const FACE_F: u8 = 2;
pub const FACE_D: u8 = 3;
pub const FACE_L: u8 = 4;
pub const FACE_B: u8 = 5;
/// Sentinel for "no previous face" in canonical move ordering.
pub const NO_FACE: u8 = N_FACES as u8;

// Corner / edge numbering (Kociemba).
pub const URF: usize = 0;
pub const UFL: usize = 1;
pub const ULB: usize = 2;
pub const UBR: usize = 3;
pub const DFR: usize = 4;
pub const DLF: usize = 5;
pub const DBL: usize = 6;
pub const DRB: usize = 7;
pub const UR: usize = 0;
pub const UF: usize = 1;
pub const UL: usize = 2;
pub const UB: usize = 3;
pub const DR: usize = 4;
pub const DF: usize = 5;
pub const DL: usize = 6;
pub const DB: usize = 7;
pub const FR: usize = 8;
pub const FL: usize = 9;
pub const BL: usize = 10;
pub const BR: usize = 11;

#[inline(always)]
pub const fn mv(face: u8, power: u8) -> u8 {
    face * POWERS as u8 + power
}

#[inline(always)]
pub const fn face(m: u8) -> u8 {
    m / POWERS as u8
}

/// 0 = quarter turn, 1 = half turn, 2 = inverse quarter turn.
#[inline(always)]
pub const fn power(m: u8) -> u8 {
    m % POWERS as u8
}

#[inline(always)]
pub const fn opposite(f: u8) -> u8 {
    (f + (N_FACES / 2) as u8) % N_FACES as u8
}

pub const fn inverse_move(m: u8) -> u8 {
    mv(face(m), POWERS as u8 - 1 - power(m))
}

#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub struct Cubie {
    pub cp: [u8; N_CORNERS],
    pub co: [u8; N_CORNERS],
    pub ep: [u8; N_EDGES],
    pub eo: [u8; N_EDGES],
}

const fn identity<const N: usize>() -> [u8; N] {
    let mut a = [0u8; N];
    let mut i = 0;
    while i < N {
        a[i] = i as u8;
        i += 1;
    }
    a
}

impl Cubie {
    pub const SOLVED: Cubie = Cubie {
        cp: identity::<N_CORNERS>(),
        co: [0; N_CORNERS],
        ep: identity::<N_EDGES>(),
        eo: [0; N_EDGES],
    };

    /// `self` followed by `b`.
    pub const fn mul(&self, b: &Cubie) -> Cubie {
        let mut r = Cubie::SOLVED;
        let mut i = 0;
        while i < N_CORNERS {
            r.cp[i] = self.cp[b.cp[i] as usize];
            r.co[i] = (self.co[b.cp[i] as usize] + b.co[i]) % CORNER_TWISTS as u8;
            i += 1;
        }
        let mut i = 0;
        while i < N_EDGES {
            r.ep[i] = self.ep[b.ep[i] as usize];
            r.eo[i] = (self.eo[b.ep[i] as usize] + b.eo[i]) % EDGE_FLIPS as u8;
            i += 1;
        }
        r
    }

    pub fn inverse(&self) -> Cubie {
        let mut r = Cubie::SOLVED;
        for i in 0..N_CORNERS {
            let p = self.cp[i] as usize;
            r.cp[p] = i as u8;
            r.co[p] = (CORNER_TWISTS as u8 - self.co[i]) % CORNER_TWISTS as u8;
        }
        for i in 0..N_EDGES {
            let p = self.ep[i] as usize;
            r.ep[p] = i as u8;
            r.eo[p] = self.eo[i];
        }
        r
    }

    pub fn is_valid(&self) -> bool {
        is_permutation(&self.cp)
            && is_permutation(&self.ep)
            && self.co.iter().all(|&x| (x as usize) < CORNER_TWISTS)
            && self.eo.iter().all(|&x| (x as usize) < EDGE_FLIPS)
            && self.co.iter().map(|&x| x as usize).sum::<usize>() % CORNER_TWISTS == 0
            && self.eo.iter().map(|&x| x as usize).sum::<usize>() % EDGE_FLIPS == 0
            && parity(&self.cp) == parity(&self.ep)
    }
}

fn is_permutation(p: &[u8]) -> bool {
    let mut seen = [false; LANES];
    p.iter().all(|&x| (x as usize) < p.len() && !std::mem::replace(&mut seen[x as usize], true))
}

/// True for odd permutations.
pub fn parity(p: &[u8]) -> bool {
    let mut odd = false;
    for i in 0..p.len() {
        for j in i + 1..p.len() {
            if p[i] > p[j] {
                odd = !odd;
            }
        }
    }
    odd
}

/// Cubie form of the 18 face turns, derived from sticker geometry.
pub const MOVE_CUBIE: [Cubie; NMOVES] = {
    let mut out = [Cubie::SOLVED; NMOVES];
    let mut f = 0;
    while f < N_FACES {
        let quarter = facelet::facelets_to_cubie(&facelet::apply_perm(
            &facelet::SOLVED_FACELETS,
            &facelet::face_turn_perm(f),
        ));
        let mut acc = quarter;
        let mut p = 0;
        while p < POWERS {
            out[f * POWERS + p] = acc;
            acc = acc.mul(&quarter);
            p += 1;
        }
        f += 1;
    }
    out
};

/// A pair of lane tables `[edges, corners]`: index = (pos, ori) of a piece
/// before, value = (pos, ori) after applying the element.
pub type LaneTables = [[u8; TABLE_LEN]; 2];

pub const fn lane_tables(k: &Cubie) -> LaneTables {
    let mut te = identity::<TABLE_LEN>();
    let mut tc = identity::<TABLE_LEN>();
    // a piece at position q moves to the position p with k.ep[p] == q
    let mut p = 0;
    while p < N_EDGES {
        let q = k.ep[p] as usize;
        let mut r = 0;
        while r < EDGE_FLIPS {
            te[q + N_EDGES * r] = (p + N_EDGES * ((r + k.eo[p] as usize) % EDGE_FLIPS)) as u8;
            r += 1;
        }
        p += 1;
    }
    let mut p = 0;
    while p < N_CORNERS {
        let q = k.cp[p] as usize;
        let mut r = 0;
        while r < CORNER_TWISTS {
            tc[q + N_CORNERS * r] = (p + N_CORNERS * ((r + k.co[p] as usize) % CORNER_TWISTS)) as u8;
            r += 1;
        }
        p += 1;
    }
    let mut t = [[0u8; TABLE_LEN]; 2];
    t[EDGE_TABLE] = te;
    t[CORNER_TABLE] = tc;
    t
}

#[repr(C, align(32))]
pub struct MoveTables(pub [LaneTables; NMOVES]);

pub static MT: MoveTables = MoveTables({
    let mut t = [[[0u8; TABLE_LEN]; 2]; NMOVES];
    let mut m = 0;
    while m < NMOVES {
        t[m] = lane_tables(&MOVE_CUBIE[m]);
        m += 1;
    }
    t
});

/// Where-is cube state. See module docs.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
#[repr(C, align(16))]
pub struct Cube {
    pub e: [u8; LANES],
    pub c: [u8; LANES],
}

impl std::fmt::Debug for Cube {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Cube(e={:?}, c={:?})", &self.e[..N_EDGES], &self.c[..N_CORNERS])
    }
}

/// Lookup tables mapping an edge position to its bit in the low / high byte
/// of the 12-bit orientation mask (positions beyond the edges map to 0).
const fn eo_bit_table(byte: usize) -> [u8; LANES] {
    let mut t = [0u8; LANES];
    let mut p = 0;
    while p < N_EDGES {
        if p / u8::BITS as usize == byte {
            t[p] = 1 << (p % u8::BITS as usize);
        }
        p += 1;
    }
    t
}
const EO_BITS_LO: [u8; LANES] = eo_bit_table(0);
const EO_BITS_HI: [u8; LANES] = eo_bit_table(1);

#[cfg(target_arch = "aarch64")]
mod simd {
    use super::*;
    use std::arch::aarch64::*;

    #[inline(always)]
    pub fn tbl32(t: &[u8; TABLE_LEN], v: [u8; LANES]) -> [u8; LANES] {
        // SAFETY: NEON is mandatory on aarch64; `t` holds exactly 32 bytes.
        unsafe {
            let tt = vld1q_u8_x2(t.as_ptr());
            let r = vqtbl2q_u8(tt, std::mem::transmute::<[u8; LANES], uint8x16_t>(v));
            std::mem::transmute::<uint8x16_t, [u8; LANES]>(r)
        }
    }

    #[inline(always)]
    pub fn eo12(e: [u8; LANES]) -> u32 {
        // SAFETY: NEON is mandatory on aarch64; all loads are of 16-byte arrays.
        unsafe {
            let v = std::mem::transmute::<[u8; LANES], uint8x16_t>(e);
            let n_edges = vdupq_n_u8(N_EDGES as u8);
            // flipped lanes hold N_EDGES + pos; empty lanes hold EMPTY_LANE
            let flipped = vandq_u8(vcgeq_u8(v, n_edges), vcltq_u8(v, vdupq_n_u8(EMPTY_LANE)));
            let pos = vsubq_u8(v, vandq_u8(flipped, n_edges));
            let lo = vandq_u8(vqtbl1q_u8(vld1q_u8(EO_BITS_LO.as_ptr()), pos), flipped);
            let hi = vandq_u8(vqtbl1q_u8(vld1q_u8(EO_BITS_HI.as_ptr()), pos), flipped);
            vaddvq_u8(lo) as u32 | (vaddvq_u8(hi) as u32) << u8::BITS
        }
    }
}

#[cfg(not(target_arch = "aarch64"))]
mod simd {
    use super::*;

    #[inline(always)]
    pub fn tbl32(t: &[u8; TABLE_LEN], v: [u8; LANES]) -> [u8; LANES] {
        v.map(|x| t[x as usize % TABLE_LEN])
    }

    #[inline(always)]
    pub fn eo12(e: [u8; LANES]) -> u32 {
        let mut m = 0;
        for &x in &e[..N_EDGES] {
            if x as usize >= N_EDGES {
                m |= 1 << (x as usize - N_EDGES);
            }
        }
        m
    }
}

impl Cube {
    pub const SOLVED: Cube = {
        let mut e = [EMPTY_LANE; LANES];
        let mut c = [EMPTY_LANE; LANES];
        let mut i = 0;
        while i < N_EDGES {
            e[i] = i as u8;
            i += 1;
        }
        let mut i = 0;
        while i < N_CORNERS {
            c[i] = i as u8;
            i += 1;
        }
        Cube { e, c }
    };

    #[inline(always)]
    pub fn apply_move(&self, m: usize) -> Cube {
        self.apply_tables(&MT.0[m])
    }

    /// Apply an arbitrary element given in lane-table form.
    #[inline(always)]
    pub fn apply_tables(&self, t: &LaneTables) -> Cube {
        Cube { e: simd::tbl32(&t[EDGE_TABLE], self.e), c: simd::tbl32(&t[CORNER_TABLE], self.c) }
    }

    pub fn apply_seq(&self, s: &[u8]) -> Cube {
        s.iter().fold(*self, |c, &m| c.apply_move(m as usize))
    }

    pub fn from_seq(s: &[u8]) -> Cube {
        Cube::SOLVED.apply_seq(s)
    }

    /// `self` followed by `b`.
    pub fn compose(&self, b: &Cube) -> Cube {
        self.apply_tables(&lane_tables(&b.to_cubie()))
    }

    pub fn inverse(&self) -> Cube {
        Cube::from_cubie(&self.to_cubie().inverse())
    }

    /// Bit mask of flipped edge positions (bit p = orientation of the edge at p).
    #[inline(always)]
    pub fn eo12(&self) -> u32 {
        simd::eo12(self.e)
    }

    pub fn is_solved(&self) -> bool {
        *self == Cube::SOLVED
    }

    pub fn from_cubie(k: &Cubie) -> Cube {
        let mut c = Cube::SOLVED;
        for p in 0..N_CORNERS {
            c.c[k.cp[p] as usize] = (p + N_CORNERS * k.co[p] as usize) as u8;
        }
        for p in 0..N_EDGES {
            c.e[k.ep[p] as usize] = (p + N_EDGES * k.eo[p] as usize) as u8;
        }
        c
    }

    pub fn to_cubie(&self) -> Cubie {
        let mut k = Cubie::SOLVED;
        for i in 0..N_CORNERS {
            let v = self.c[i] as usize;
            k.cp[v % N_CORNERS] = i as u8;
            k.co[v % N_CORNERS] = (v / N_CORNERS) as u8;
        }
        for i in 0..N_EDGES {
            let v = self.e[i] as usize;
            k.ep[v % N_EDGES] = i as u8;
            k.eo[v % N_EDGES] = (v / N_EDGES) as u8;
        }
        k
    }

    pub fn is_valid(&self) -> bool {
        let lanes_ok = |v: &[u8; LANES], n: usize| {
            v[n..].iter().all(|&x| x == EMPTY_LANE) && v[..n].iter().all(|&x| x < EMPTY_LANE)
        };
        lanes_ok(&self.e, N_EDGES) && lanes_ok(&self.c, N_CORNERS) && self.to_cubie().is_valid()
    }

    /// Uniformly random solvable state.
    pub fn random(rng: &mut Rng) -> Cube {
        let mut k = Cubie::SOLVED;
        shuffle(&mut k.cp, rng);
        shuffle(&mut k.ep, rng);
        if parity(&k.cp) != parity(&k.ep) {
            k.ep.swap(0, 1);
        }
        randomise_orientation(&mut k.co, CORNER_TWISTS, rng);
        randomise_orientation(&mut k.eo, EDGE_FLIPS, rng);
        Cube::from_cubie(&k)
    }

    pub fn from_facelets(s: &str) -> Result<Cube, String> {
        let k = facelet::parse_facelets(s)?;
        if !k.is_valid() {
            return Err("facelets describe an unsolvable cube".into());
        }
        Ok(Cube::from_cubie(&k))
    }

    pub fn to_facelets(&self) -> String {
        let f = facelet::cubie_to_facelets(&self.to_cubie());
        f.iter().map(|&c| FACE_CHARS[c as usize] as char).collect()
    }

    /// `S * X * S^-1`: the same physical cube viewed after whole-cube symmetry
    /// `sym`. Solving the result with moves `q` solves `self` with
    /// `SYM_MOVE[sym][q]`.
    #[inline]
    pub fn conj(&self, sym: usize) -> Cube {
        let t = &facelet::CONJ;
        let mut y = Cube::SOLVED;
        for i in 0..N_CORNERS {
            y.c[t.corner_piece[sym][i] as usize] = t.corner_value[sym][i][self.c[i] as usize];
        }
        for i in 0..N_EDGES {
            y.e[t.edge_piece[sym][i] as usize] = t.edge_value[sym][i][self.e[i] as usize];
        }
        y
    }

    /// Reference implementation of [`Cube::conj`] through facelets.
    pub fn conj_reference(&self, sym: usize) -> Cube {
        let f = facelet::cubie_to_facelets(&self.to_cubie());
        let g = facelet::conj_facelets(&f, sym);
        Cube::from_cubie(&facelet::facelets_to_cubie(&g))
    }
}

fn shuffle(a: &mut [u8], rng: &mut Rng) {
    for i in (1..a.len()).rev() {
        let j = rng.below(i + 1);
        a.swap(i, j);
    }
}

/// Random orientations whose sum is 0 mod `n`.
fn randomise_orientation(o: &mut [u8], n: usize, rng: &mut Rng) {
    let last = o.len() - 1;
    let mut sum = 0;
    for x in &mut o[..last] {
        *x = rng.below(n) as u8;
        sum += *x as usize;
    }
    o[last] = ((n - sum % n) % n) as u8;
}

/// xoshiro256** seeded through splitmix64 (constants from the reference
/// implementations by Blackman and Vigna).
#[derive(Clone)]
pub struct Rng([u64; XOSHIRO_WORDS]);

const XOSHIRO_WORDS: usize = 4;

const SPLITMIX_GAMMA: u64 = 0x9E37_79B9_7F4A_7C15;
const SPLITMIX_MUL1: u64 = 0xBF58_476D_1CE4_E5B9;
const SPLITMIX_MUL2: u64 = 0x94D0_49BB_1331_11EB;
const SPLITMIX_SHIFTS: [u32; 3] = [30, 27, 31];
const XOSHIRO_MUL1: u64 = 5;
const XOSHIRO_ROT1: u32 = 7;
const XOSHIRO_MUL2: u64 = 9;
const XOSHIRO_SHIFT: u32 = 17;
const XOSHIRO_ROT2: u32 = 45;

impl Rng {
    pub fn new(seed: u64) -> Rng {
        let mut z = seed;
        let [s1, s2, s3] = SPLITMIX_SHIFTS;
        Rng([(); XOSHIRO_WORDS].map(|_| {
            z = z.wrapping_add(SPLITMIX_GAMMA);
            let mut r = z;
            r = (r ^ (r >> s1)).wrapping_mul(SPLITMIX_MUL1);
            r = (r ^ (r >> s2)).wrapping_mul(SPLITMIX_MUL2);
            r ^ (r >> s3)
        }))
    }

    pub fn next_u64(&mut self) -> u64 {
        let [a, b, c, d] = &mut self.0;
        let r = b.wrapping_mul(XOSHIRO_MUL1).rotate_left(XOSHIRO_ROT1).wrapping_mul(XOSHIRO_MUL2);
        let t = *b << XOSHIRO_SHIFT;
        *c ^= *a;
        *d ^= *b;
        *b ^= *c;
        *a ^= *d;
        *c ^= t;
        *d = d.rotate_left(XOSHIRO_ROT2);
        r
    }

    /// Uniform-enough value in `0..n` for small `n`.
    pub fn below(&mut self, n: usize) -> usize {
        (self.next_u64() % n as u64) as usize
    }
}

// ---------------------------------------------------------------- sequences

const QUARTERS: u8 = 4;

/// Where appending `m` to `q` merges into an earlier turn of the same face
/// (possibly hidden behind an opposite-face turn, as in `U D U'`), and the
/// merged move, or None if it cancels completely.
#[inline(always)]
fn merge_target(q: &[u8], m: u8) -> Option<(usize, Option<u8>)> {
    let f = face(m);
    let n = q.len();
    let i = if n >= 1 && face(q[n - 1]) == f {
        n - 1
    } else if n >= 2 && face(q[n - 1]) == opposite(f) && face(q[n - 2]) == f {
        n - 2
    } else {
        return None;
    };
    let turns = (power(q[i]) + 1 + power(m) + 1) % QUARTERS;
    Some((i, (turns != 0).then(|| mv(f, turns - 1))))
}

/// Append `m`, merging with a previous turn of the same face, including one
/// hidden behind a turn of the opposite face (e.g. `U D U'` -> `D`).
pub fn seq_push(q: &mut Vec<u8>, m: u8) {
    match merge_target(q, m) {
        None => q.push(m),
        Some((i, Some(merged))) => q[i] = merged,
        Some((i, None)) => {
            q.remove(i);
        }
    }
}

/// Fixed-capacity move sequence: no heap traffic in the solver's hot path.
#[derive(Clone, Copy)]
pub struct Moves<const N: usize> {
    len: u8,
    m: [u8; N],
}

impl<const N: usize> Default for Moves<N> {
    fn default() -> Self {
        Moves { len: 0, m: [0; N] }
    }
}

impl<const N: usize> Moves<N> {
    pub fn from_slice(s: &[u8]) -> Self {
        let mut r = Self::default();
        r.m[..s.len()].copy_from_slice(s);
        r.len = s.len() as u8;
        r
    }

    #[inline(always)]
    pub fn as_slice(&self) -> &[u8] {
        &self.m[..self.len as usize]
    }

    #[inline(always)]
    pub fn len(&self) -> usize {
        self.len as usize
    }

    #[inline(always)]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    #[inline(always)]
    pub fn push(&mut self, m: u8) {
        self.m[self.len as usize] = m;
        self.len += 1;
    }

    #[inline(always)]
    pub fn pop(&mut self) {
        self.len -= 1;
    }

    pub fn extend(&mut self, s: &[u8]) {
        for &m in s {
            self.push(m);
        }
    }

    /// Append with cancellation, as [`seq_push`].
    pub fn push_merged(&mut self, m: u8) {
        match merge_target(self.as_slice(), m) {
            None => self.push(m),
            Some((i, Some(merged))) => self.m[i] = merged,
            Some((i, None)) => {
                let n = self.len as usize;
                self.m.copy_within(i + 1..n, i);
                self.len -= 1;
            }
        }
    }

    pub fn extend_merged(&mut self, s: &[u8]) {
        for &m in s {
            self.push_merged(m);
        }
    }
}

pub fn seq_append(q: &mut Vec<u8>, r: &[u8]) {
    for &m in r {
        seq_push(q, m);
    }
}

pub fn seq_inverse(q: &[u8]) -> Vec<u8> {
    q.iter().rev().map(|&m| inverse_move(m)).collect()
}

pub fn seq_to_string(q: &[u8]) -> String {
    q.iter().map(|&m| MOVE_NAMES[m as usize]).collect::<Vec<_>>().join(" ")
}

pub fn parse_seq(s: &str) -> Result<Vec<u8>, String> {
    let mut out = Vec::new();
    for tok in s.split_whitespace() {
        let b = tok.as_bytes();
        let f = FACE_CHARS
            .iter()
            .position(|&c| c == b[0])
            .ok_or_else(|| format!("bad move '{tok}'"))? as u8;
        let p = match &tok[1..] {
            "" | "1" => 0,
            "2" | "2'" => 1,
            "'" | "3" => 2,
            _ => return Err(format!("bad move '{tok}'")),
        };
        out.push(mv(f, p));
    }
    Ok(out)
}
