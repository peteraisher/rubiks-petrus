//! The tables `build.rs` generates, embedded in the binary and used in
//! place: per phase, distances and stored algorithms by case, with the
//! symmetry tables that map a case to its stored row.

use crate::algs::decode;
use crate::coord::*;
use petrus::cube::*;

#[repr(C, align(64))]
struct Aligned<B: ?Sized>(B);

macro_rules! embedded {
    ($name:literal) => {
        &Aligned(*include_bytes!(concat!(env!("OUT_DIR"), "/", $name, ".bin"))).0
    };
}

static CLASS: &[u8] = embedded!("class");
static G3: &[u8] = embedded!("g3");
static DIST: [&[u8]; N_PHASES] = [embedded!("dist1"), embedded!("dist2"), embedded!("dist3"), embedded!("dist4")];
static ALGS: [&[u8]; N_PHASES] = [embedded!("alg1"), embedded!("alg2"), embedded!("alg3"), embedded!("alg4")];
/// Column 0 of each phase's algorithms, for the single chain (phase 4
/// stores one alternative, so its table is the same).
static FIRST: [&[u8]; N_PHASES] = [embedded!("first1"), embedded!("first2"), embedded!("first3"), embedded!("alg4")];
/// The halves of the split phases (see [`N_HALVES`]), by case.
static HALVES: [&[u8]; N_HALVES] = [embedded!("half1"), embedded!("half2"), embedded!("half3"), embedded!("half4")];
static MINOR_SYM: [&[u8]; N_PHASES] = [&[], embedded!("minor2"), embedded!("minor3"), embedded!("minor4")];
static MAJOR_CONJ: [&[u8]; N_PHASES] = [&[], embedded!("major2"), embedded!("major3"), embedded!("major4")];

/// Reinterpret embedded little-endian bytes as words (the data is aligned).
fn words<T: Copy>(b: &'static [u8]) -> &'static [T] {
    const { assert!(cfg!(target_endian = "little")) };
    // SAFETY: T is a plain integer type, every bit pattern is valid, and the
    // embedded arrays are 64-byte aligned (checked).
    let (pre, w, post) = unsafe { b.align_to::<T>() };
    assert!(pre.is_empty() && post.is_empty(), "misaligned table");
    w
}

const CACHE_LINE: usize = 64;

/// Hint that `p` will be read soon.
#[inline(always)]
fn prefetch<T>(p: *const T) {
    #[cfg(target_arch = "aarch64")]
    // SAFETY: a prefetch has no architectural effect and cannot fault.
    unsafe {
        std::arch::asm!("prfm pldl1keep, [{0}]", in(reg) p, options(nostack, preserves_flags, readonly));
    }
    #[cfg(not(target_arch = "aarch64"))]
    let _ = p;
}

/// All tables, embedded in the binary at build time.
pub struct Tables {
    ct: CornerTables,
    dist: [&'static [u8]; N_PHASES],
    /// Phase 1 by case; phases 2-4 by (minor class, major), see [`MINOR`].
    algs: [&'static [u64]; N_PHASES],
    /// Each row's first alternative alone.
    first: [&'static [u64]; N_PHASES],
    halves: [&'static [u64]; N_HALVES],
    minor_sym: [&'static [u16]; N_PHASES],
    major_conj: [&'static [u16]; N_PHASES],
}

impl Tables {
    pub fn new() -> Tables {
        let t = Tables {
            ct: CornerTables { class: CLASS, g3: G3 },
            dist: DIST,
            algs: ALGS.map(words),
            first: FIRST.map(words),
            halves: HALVES.map(words),
            minor_sym: MINOR_SYM.map(words),
            major_conj: MAJOR_CONJ.map(words),
        };
        for p in 0..N_PHASES {
            assert_eq!(t.dist[p].len(), PHASE_SIZES[p]);
            if p > 0 {
                assert_eq!(t.minor_sym[p].len(), MINOR[p]);
                assert_eq!(t.major_conj[p].len(), MAJOR[p] * PHASE_SYMS[p].len());
                let classes = t.minor_sym[p].iter().map(|&x| x as usize >> SYM_BITS).max().unwrap() + 1;
                assert_eq!(t.algs[p].len(), classes * MAJOR[p] * MAX_ALTS[p]);
            }
            assert_eq!(t.first[p].len() * MAX_ALTS[p], t.algs[p].len());
        }
        for h in 0..N_HALVES {
            assert_eq!(t.halves[h].len(), HALF_SIZES[h]);
        }
        t
    }

    /// Phase `P`'s case of `c` (a cube in the phase's starting group).
    #[inline(always)]
    pub(crate) fn index<const P: usize>(&self, c: &Cube) -> usize {
        match P {
            0 => idx1(c),
            1 => idx2(c),
            2 => idx3(&self.ct, c),
            _ => idx4(&self.ct, c),
        }
    }

    /// Distance of case `i` of phase `P` from that phase's goal.
    #[inline(always)]
    pub(crate) fn dist<const P: usize>(&self, i: usize) -> u8 {
        // SAFETY: indices come from the coordinate functions, which stay
        // below PHASE_SIZES[P] for cubes in the phase's group (asserted in
        // debug builds and the tests).
        debug_assert!(i < PHASE_SIZES[P]);
        unsafe { *self.dist[P].get_unchecked(i) }
    }

    /// The row of stored alternatives for case `i` of phase `P`, and the
    /// phase symmetry their moves are to be mapped through.
    #[inline(always)]
    pub(crate) fn row_of<const P: usize>(&self, i: usize) -> (usize, usize) {
        if P == 0 {
            return (i, 0);
        }
        let (minor, major) = (i % MINOR[P], i / MINOR[P]);
        let ms = self.minor_sym[P][minor] as usize;
        let sym = ms & (MAX_SYMS - 1);
        ((ms >> SYM_BITS) * MAJOR[P] + self.major_conj[P][sym * MAJOR[P] + major] as usize, sym)
    }

    /// Row `r`'s alternatives (`NO_ALG` past the last).
    #[inline(always)]
    pub(crate) fn row<const P: usize>(&self, r: usize) -> &[u64] {
        &self.algs[P][r * MAX_ALTS[P]..(r + 1) * MAX_ALTS[P]]
    }

    /// Row `r`'s first alternative, from the table of first alternatives.
    #[inline(always)]
    pub(crate) fn first<const P: usize>(&self, r: usize) -> u64 {
        debug_assert!(r < self.first[P].len());
        // SAFETY: rows come from `row_of`, below the table's row count
        unsafe { *self.first[P].get_unchecked(r) }
    }

    /// Split half `H`'s case of `c` (see [`N_HALVES`]).
    #[inline(always)]
    pub(crate) fn half_case<const H: usize>(&self, c: &Cube) -> usize {
        match H {
            HALF_SLICE => slice_rank(c),
            HALF_TWIST => twist(c),
            HALF_CORNERS => g3_corners(&self.ct, c),
            _ => g3_edges(c),
        }
    }

    /// Split half `H`'s algorithm for case `k`.
    #[inline(always)]
    pub(crate) fn half<const H: usize>(&self, k: usize) -> u64 {
        debug_assert!(k < HALF_SIZES[H]);
        // SAFETY: the half's coordinate stays below its size (debug-asserted)
        unsafe { *self.halves[H].get_unchecked(k) }
    }

    /// Start loading the first `alts` entries of a row.
    #[inline(always)]
    pub(crate) fn prefetch_row<const P: usize>(&self, r: usize, alts: usize) {
        let row = self.row::<P>(r);
        prefetch(row.as_ptr());
        if alts * std::mem::size_of::<u64>() > CACHE_LINE {
            prefetch(row[CACHE_LINE / std::mem::size_of::<u64>()..].as_ptr());
        }
    }

    #[inline(always)]
    fn algs<const P: usize>(&self, i: usize) -> (&[u64], usize) {
        let (r, sym) = self.row_of::<P>(i);
        (self.row::<P>(r), sym)
    }

    /// Phase `p`'s distance, algorithm, minor-symmetry and major-conjugate
    /// tables as embedded (for tests).
    pub fn raw(&self, p: usize) -> (&[u8], &[u64], &[u16], &[u16]) {
        (self.dist[p], self.algs[p], self.minor_sym[p], self.major_conj[p])
    }

    /// Split half `h`'s table as embedded (for tests).
    pub fn raw_half(&self, h: usize) -> &[u64] {
        self.halves[h]
    }

    /// Split half `h`'s algorithm for `c`, as a move sequence (for tests).
    pub fn half_alg(&self, h: usize, c: &Cube) -> Vec<u8> {
        let k = match h {
            HALF_SLICE => self.half_case::<HALF_SLICE>(c),
            HALF_TWIST => self.half_case::<HALF_TWIST>(c),
            HALF_CORNERS => self.half_case::<HALF_CORNERS>(c),
            _ => self.half_case::<HALF_EDGES>(c),
        };
        decode(HALF_PHASE[h], self.halves[h][k], 0)
    }

    /// Distance from the goal of phase `p` (for tests and tools).
    pub fn phase_dist(&self, p: usize, c: &Cube) -> u8 {
        match p {
            0 => self.dist::<0>(self.index::<0>(c)),
            1 => self.dist::<1>(self.index::<1>(c)),
            2 => self.dist::<2>(self.index::<2>(c)),
            _ => self.dist::<LAST>(self.index::<LAST>(c)),
        }
    }

    /// The stored alternatives for `c` in phase `p`, as move sequences.
    pub fn phase_algs(&self, p: usize, c: &Cube) -> Vec<Vec<u8>> {
        let (row, sym) = match p {
            0 => self.algs::<0>(self.index::<0>(c)),
            1 => self.algs::<1>(self.index::<1>(c)),
            2 => self.algs::<2>(self.index::<2>(c)),
            _ => self.algs::<LAST>(self.index::<LAST>(c)),
        };
        row.iter().take_while(|&&a| a != NO_ALG).map(|&a| decode(p, a, sym)).collect()
    }
}

impl Default for Tables {
    fn default() -> Self {
        Tables::new()
    }
}
