//! Cube model.
//!
//! Two representations live here:
//!
//! * [`Cubie`]: Kociemba's "what is at each position" arrays. Scalar, simple,
//!   used as the reference model and for conversions.
//! * [`Cube`]: the hot-path "where is each piece" form. One byte per piece
//!   holding `pos + 12*ori` (edges) or `pos + 8*ori` (corners), packed into two
//!   16-byte vectors. Applying any fixed permutation-with-twists (a face turn,
//!   a whole algorithm) maps every lane through the same 32-entry table, so a
//!   move is exactly one `tbl` instruction per vector on aarch64
//!   (`vqtbl2q_u8`). Unused lanes hold 24 and every table maps 24..31 to
//!   itself, so whole-vector comparisons just work.
//!
//! Move and symmetry tables are derived at compile time from 3D sticker
//! geometry (see [`crate::facelet`]), not typed in by hand.

use crate::facelet;

pub const NMOVES: usize = 18;
pub const MOVE_NAMES: [&str; NMOVES] = [
    "U", "U2", "U'", "R", "R2", "R'", "F", "F2", "F'", "D", "D2", "D'", "L", "L2", "L'", "B", "B2",
    "B'",
];
pub const FACE_CHARS: [u8; 6] = *b"URFDLB";

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

#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub struct Cubie {
    pub cp: [u8; 8],
    pub co: [u8; 8],
    pub ep: [u8; 12],
    pub eo: [u8; 12],
}

impl Cubie {
    pub const SOLVED: Cubie = Cubie {
        cp: [0, 1, 2, 3, 4, 5, 6, 7],
        co: [0; 8],
        ep: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11],
        eo: [0; 12],
    };

    /// `self` followed by `b`.
    pub const fn mul(&self, b: &Cubie) -> Cubie {
        let mut r = Cubie::SOLVED;
        let mut i = 0;
        while i < 8 {
            r.cp[i] = self.cp[b.cp[i] as usize];
            r.co[i] = (self.co[b.cp[i] as usize] + b.co[i]) % 3;
            i += 1;
        }
        let mut i = 0;
        while i < 12 {
            r.ep[i] = self.ep[b.ep[i] as usize];
            r.eo[i] = (self.eo[b.ep[i] as usize] + b.eo[i]) % 2;
            i += 1;
        }
        r
    }

    pub fn inverse(&self) -> Cubie {
        let mut r = Cubie::SOLVED;
        for i in 0..8 {
            let p = self.cp[i] as usize;
            r.cp[p] = i as u8;
            r.co[p] = (3 - self.co[i]) % 3;
        }
        for i in 0..12 {
            let p = self.ep[i] as usize;
            r.ep[p] = i as u8;
            r.eo[p] = self.eo[i];
        }
        r
    }

    pub fn is_valid(&self) -> bool {
        let mut seen_c = [false; 8];
        let mut seen_e = [false; 12];
        for &p in &self.cp {
            if p >= 8 || seen_c[p as usize] {
                return false;
            }
            seen_c[p as usize] = true;
        }
        for &p in &self.ep {
            if p >= 12 || seen_e[p as usize] {
                return false;
            }
            seen_e[p as usize] = true;
        }
        let co: u32 = self.co.iter().map(|&x| x as u32).sum();
        let eo: u32 = self.eo.iter().map(|&x| x as u32).sum();
        co % 3 == 0
            && eo % 2 == 0
            && self.co.iter().all(|&x| x < 3)
            && self.eo.iter().all(|&x| x < 2)
            && parity(&self.cp) == parity(&self.ep)
    }
}

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
    while f < 6 {
        let q = facelet::facelets_to_cubie(&facelet::apply_perm(
            &facelet::SOLVED_FACELETS,
            &facelet::face_turn_perm(f),
        ));
        out[f * 3] = q;
        out[f * 3 + 1] = q.mul(&q);
        out[f * 3 + 2] = q.mul(&q).mul(&q);
        f += 1;
    }
    out
};

/// 32-entry lane tables for a cubie element: index = (pos, ori) of a piece
/// before, value = (pos, ori) after.
pub const fn lane_tables(k: &Cubie) -> [[u8; 32]; 2] {
    let mut te = [0u8; 32];
    let mut tc = [0u8; 32];
    let mut i = 24;
    while i < 32 {
        te[i] = i as u8;
        tc[i] = i as u8;
        i += 1;
    }
    // a piece at position q moves to the position p with k.ep[p] == q
    let mut p = 0;
    while p < 12 {
        let q = k.ep[p] as usize;
        let mut r = 0;
        while r < 2 {
            te[q + 12 * r] = (p + 12 * ((r + k.eo[p] as usize) % 2)) as u8;
            r += 1;
        }
        p += 1;
    }
    let mut p = 0;
    while p < 8 {
        let q = k.cp[p] as usize;
        let mut r = 0;
        while r < 3 {
            tc[q + 8 * r] = (p + 8 * ((r + k.co[p] as usize) % 3)) as u8;
            r += 1;
        }
        p += 1;
    }
    [te, tc]
}

#[repr(C, align(32))]
pub struct MoveTables(pub [[[u8; 32]; 2]; NMOVES]);

/// MT.0[m][0] = edge table, MT.0[m][1] = corner table.
pub static MT: MoveTables = MoveTables({
    let mut t = [[[0u8; 32]; 2]; NMOVES];
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
    pub e: [u8; 16],
    pub c: [u8; 16],
}

impl std::fmt::Debug for Cube {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Cube(e={:?}, c={:?})", &self.e[..12], &self.c[..8])
    }
}

#[cfg(target_arch = "aarch64")]
mod simd {
    use std::arch::aarch64::*;
    #[inline(always)]
    pub fn tbl32(t: &[u8; 32], v: [u8; 16]) -> [u8; 16] {
        // SAFETY: NEON is mandatory on aarch64; pointers are to 32 valid bytes.
        unsafe {
            let tt = vld1q_u8_x2(t.as_ptr());
            let r = vqtbl2q_u8(tt, std::mem::transmute::<[u8; 16], uint8x16_t>(v));
            std::mem::transmute::<uint8x16_t, [u8; 16]>(r)
        }
    }
    /// Bit mask of flipped edge positions.
    #[inline(always)]
    pub fn eo12(e: [u8; 16]) -> u32 {
        const LO: [u8; 16] = [1, 2, 4, 8, 16, 32, 64, 128, 0, 0, 0, 0, 0, 0, 0, 0];
        const HI: [u8; 16] = [0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 4, 8, 0, 0, 0, 0];
        unsafe {
            let v = std::mem::transmute::<[u8; 16], uint8x16_t>(e);
            // lanes >= 12 and < 24 are flipped; positions are v - 12
            let flipped = vandq_u8(vcgeq_u8(v, vdupq_n_u8(12)), vcltq_u8(v, vdupq_n_u8(24)));
            let pos = vsubq_u8(v, vandq_u8(flipped, vdupq_n_u8(12)));
            let lo = vandq_u8(vqtbl1q_u8(vld1q_u8(LO.as_ptr()), pos), flipped);
            let hi = vandq_u8(vqtbl1q_u8(vld1q_u8(HI.as_ptr()), pos), flipped);
            vaddvq_u8(lo) as u32 | (vaddvq_u8(hi) as u32) << 8
        }
    }
}

#[cfg(not(target_arch = "aarch64"))]
mod simd {
    #[inline(always)]
    pub fn tbl32(t: &[u8; 32], v: [u8; 16]) -> [u8; 16] {
        let mut r = [0u8; 16];
        for i in 0..16 {
            r[i] = t[(v[i] & 31) as usize];
        }
        r
    }
    #[inline(always)]
    pub fn eo12(e: [u8; 16]) -> u32 {
        let mut m = 0;
        for i in 0..12 {
            if e[i] >= 12 {
                m |= 1 << (e[i] - 12);
            }
        }
        m
    }
}

impl Cube {
    pub const SOLVED: Cube = Cube {
        e: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 24, 24, 24, 24],
        c: [0, 1, 2, 3, 4, 5, 6, 7, 24, 24, 24, 24, 24, 24, 24, 24],
    };

    #[inline(always)]
    pub fn apply_move(&self, m: usize) -> Cube {
        let t = &MT.0[m];
        Cube { e: simd::tbl32(&t[0], self.e), c: simd::tbl32(&t[1], self.c) }
    }

    /// Apply an arbitrary element given in lane-table form.
    #[inline(always)]
    pub fn apply_tables(&self, t: &[[u8; 32]; 2]) -> Cube {
        Cube { e: simd::tbl32(&t[0], self.e), c: simd::tbl32(&t[1], self.c) }
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

    #[inline(always)]
    pub fn eo12(&self) -> u32 {
        simd::eo12(self.e)
    }

    pub fn is_solved(&self) -> bool {
        *self == Cube::SOLVED
    }

    pub fn from_cubie(k: &Cubie) -> Cube {
        let mut c = Cube::SOLVED;
        for p in 0..8 {
            c.c[k.cp[p] as usize] = (p + 8 * k.co[p] as usize) as u8;
        }
        for p in 0..12 {
            c.e[k.ep[p] as usize] = (p + 12 * k.eo[p] as usize) as u8;
        }
        c
    }

    pub fn to_cubie(&self) -> Cubie {
        let mut k = Cubie::SOLVED;
        for i in 0..8 {
            let v = self.c[i] as usize;
            k.cp[v % 8] = i as u8;
            k.co[v % 8] = (v / 8) as u8;
        }
        for i in 0..12 {
            let v = self.e[i] as usize;
            k.ep[v % 12] = i as u8;
            k.eo[v % 12] = (v / 12) as u8;
        }
        k
    }

    pub fn is_valid(&self) -> bool {
        if self.e[12..].iter().any(|&x| x != 24) || self.c[8..].iter().any(|&x| x != 24) {
            return false;
        }
        if self.e[..12].iter().any(|&x| x >= 24) || self.c[..8].iter().any(|&x| x >= 24) {
            return false;
        }
        self.to_cubie().is_valid()
    }

    pub fn random(rng: &mut Rng) -> Cube {
        loop {
            let mut k = Cubie::SOLVED;
            shuffle(&mut k.cp, rng);
            shuffle(&mut k.ep, rng);
            if parity(&k.cp) != parity(&k.ep) {
                k.ep.swap(0, 1);
            }
            let mut s = 0;
            for i in 0..7 {
                k.co[i] = (rng.next() % 3) as u8;
                s += k.co[i];
            }
            k.co[7] = (3 - s % 3) % 3;
            let mut s = 0;
            for i in 0..11 {
                k.eo[i] = (rng.next() % 2) as u8;
                s += k.eo[i];
            }
            k.eo[11] = s % 2;
            if k.is_valid() {
                return Cube::from_cubie(&k);
            }
        }
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
    pub fn conj(&self, sym: usize) -> Cube {
        let f = facelet::cubie_to_facelets(&self.to_cubie());
        let g = facelet::conj_facelets(&f, sym);
        Cube::from_cubie(&facelet::facelets_to_cubie(&g))
    }
}

fn shuffle(a: &mut [u8], rng: &mut Rng) {
    for i in (1..a.len()).rev() {
        let j = (rng.next() % (i as u64 + 1)) as usize;
        a.swap(i, j);
    }
}

/// xoshiro256**
#[derive(Clone)]
pub struct Rng([u64; 4]);

impl Rng {
    pub fn new(seed: u64) -> Rng {
        let mut z = seed;
        let mut s = [0u64; 4];
        for x in &mut s {
            // splitmix64
            z = z.wrapping_add(0x9E3779B97F4A7C15);
            let mut r = z;
            r = (r ^ (r >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
            r = (r ^ (r >> 27)).wrapping_mul(0x94D049BB133111EB);
            *x = r ^ (r >> 31);
        }
        Rng(s)
    }
    pub fn next(&mut self) -> u64 {
        let s = &mut self.0;
        let r = s[1].wrapping_mul(5).rotate_left(7).wrapping_mul(9);
        let t = s[1] << 17;
        s[2] ^= s[0];
        s[3] ^= s[1];
        s[1] ^= s[2];
        s[0] ^= s[3];
        s[2] ^= t;
        s[3] = s[3].rotate_left(45);
        r
    }
}

// ---------------------------------------------------------------- sequences

#[inline(always)]
pub const fn face(m: u8) -> u8 {
    m / 3
}

/// Append `m`, merging with a previous turn of the same face, including one
/// hidden behind a turn of the opposite face (e.g. `U D U'` -> `D`).
pub fn seq_push(q: &mut Vec<u8>, m: u8) {
    let f = face(m);
    let n = q.len();
    let target = if n >= 1 && face(q[n - 1]) == f {
        Some(n - 1)
    } else if n >= 2 && face(q[n - 1]) == (f + 3) % 6 && face(q[n - 2]) == f {
        Some(n - 2)
    } else {
        None
    };
    match target {
        None => q.push(m),
        Some(i) => {
            let turns = (q[i] % 3 + 1 + m % 3 + 1) % 4;
            if turns == 0 {
                q.remove(i);
            } else {
                q[i] = f * 3 + turns - 1;
            }
        }
    }
}

pub fn seq_append(q: &mut Vec<u8>, r: &[u8]) {
    for &m in r {
        seq_push(q, m);
    }
}

pub fn seq_inverse(q: &[u8]) -> Vec<u8> {
    q.iter().rev().map(|&m| m / 3 * 3 + (2 - m % 3)).collect()
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
        out.push(f * 3 + p);
    }
    Ok(out)
}
