//! Pruning tables over small sets of tracked pieces.
//!
//! A table is indexed directly by the tracked lanes of a [`Cube`]. Each lane
//! is a 0..24 "(position, orientation)" value; a tiny per-lane remap squeezes
//! it to only the values that lane can reach under the table's move set
//! (e.g. 18 corner values and 7 edge values under <R,U>), and the digits form
//! a mixed-radix index, optionally followed by the 12-bit edge-orientation
//! mask. Distances are stored as 4-bit nibbles. Both keep the tables small
//! enough to live in cache, which is what the search is bound by. The shape
//! is a const generic so a lookup unrolls to a handful of multiply-adds.

use std::borrow::Cow;

use crate::cube::{Cube, CORNER_TABLE, EDGE_TABLE, EO_MASKS, LANE_VALUES, MOVE_CUBIE, MT, NMOVES, N_EDGES};

/// Distance marker for states not (yet) reached by the BFS; also the largest
/// value a nibble can hold, so real distances must stay below it.
pub const UNSEEN: u8 = 0xF;
const NIBBLE_BITS: usize = 4;
const NO_DIGIT: u8 = u8::MAX;

/// Weight given to lane values the table's move set cannot reach: large
/// enough that any index containing one is out of bounds (and panics), small
/// enough that summing a few never overflows.
const UNREACHABLE: u32 = u32::MAX >> NIBBLE_BITS;

/// Per-lane mapping between lane values and compact digits.
#[derive(Clone, Copy)]
struct Digit {
    lane: usize,
    radix: usize,
    to_digit: [u8; LANE_VALUES],
    to_value: [u8; LANE_VALUES],
    /// to_digit * place value, so an index is a sum of independent loads.
    weighted: [u32; LANE_VALUES],
}

impl Digit {
    /// Values a piece starting at `lane` can reach using `moves`.
    fn new(lane: usize, kind: usize, moves: &[u8]) -> Digit {
        let mut d = Digit {
            lane,
            radix: 0,
            to_digit: [NO_DIGIT; LANE_VALUES],
            to_value: [0; LANE_VALUES],
            weighted: [UNREACHABLE; LANE_VALUES],
        };
        let mut queue = vec![lane as u8];
        d.to_digit[lane] = 0;
        d.radix = 1;
        while let Some(v) = queue.pop() {
            for &m in moves {
                let w = MT.0[m as usize][kind][v as usize];
                if d.to_digit[w as usize] == NO_DIGIT {
                    d.to_digit[w as usize] = d.radix as u8;
                    d.radix += 1;
                    queue.push(w);
                }
            }
        }
        for (v, &dig) in d.to_digit.iter().enumerate() {
            if dig != NO_DIGIT {
                d.to_value[dig as usize] = v as u8;
            }
        }
        d
    }
}

/// Distance table over `NC` corners and `NE` edges, plus all-edge
/// orientation when `EO` is set.
#[derive(Clone)]
pub struct PieceTable<const NC: usize, const NE: usize, const EO: bool> {
    corners: [Digit; NC],
    edges: [Digit; NE],
    data: Cow<'static, [u8]>,
    pub size: usize,
    pub reachable: usize,
    pub max_depth: u8,
}

/// Edge-orientation mask after each move: `eo_move_table()[mask][m]`.
pub fn eo_move_table() -> Vec<[u16; NMOVES]> {
    let mut t = vec![[0u16; NMOVES]; EO_MASKS];
    for (mask, row) in t.iter_mut().enumerate() {
        for (m, out) in row.iter_mut().enumerate() {
            let k = &MOVE_CUBIE[m];
            let mut r = 0u16;
            for p in 0..N_EDGES {
                let b = ((mask >> k.ep[p]) & 1) as u8 ^ k.eo[p];
                r |= (b as u16) << p;
            }
            *out = r;
        }
    }
    t
}

impl<const NC: usize, const NE: usize, const EO: bool> PieceTable<NC, NE, EO> {
    fn layout(corners: [usize; NC], edges: [usize; NE], moves: &[u8], data: Cow<'static, [u8]>) -> Self {
        let mut corners = corners.map(|l| Digit::new(l, CORNER_TABLE, moves));
        let mut edges = edges.map(|l| Digit::new(l, EDGE_TABLE, moves));
        // place values, least significant last; the EO mask sits below all
        let mut place: usize = if EO { EO_MASKS } else { 1 };
        for d in edges.iter_mut().rev().chain(corners.iter_mut().rev()) {
            for v in 0..LANE_VALUES {
                if d.to_digit[v] != NO_DIGIT {
                    d.weighted[v] = (d.to_digit[v] as usize * place) as u32;
                }
            }
            place *= d.radix;
        }
        let data = if data.is_empty() { Cow::Owned(vec![u8::MAX; place.div_ceil(2)]) } else { data };
        assert_eq!(data.len(), place.div_ceil(2), "table data does not match its shape");
        PieceTable { corners, edges, data, size: place, reachable: 0, max_depth: 0 }
    }

    /// Wrap precomputed distance data (e.g. embedded at build time).
    pub fn with_data(corners: [usize; NC], edges: [usize; NE], moves: &[u8], data: &'static [u8]) -> Self {
        Self::layout(corners, edges, moves, Cow::Borrowed(data))
    }

    pub fn into_data(self) -> Vec<u8> {
        self.data.into_owned()
    }

    /// Breadth-first search from solved over `moves`.
    pub fn build(corners: [usize; NC], edges: [usize; NE], moves: &[u8]) -> Self {
        let mut t = Self::layout(corners, edges, moves, Cow::Borrowed(&[]));
        let eo_mv = if EO { eo_move_table() } else { Vec::new() };
        let start = t.index(&Cube::SOLVED);
        t.set(start, 0);
        let mut frontier = vec![start as u32];
        let mut next = Vec::new();
        let mut depth = 0u8;
        let mut reachable = 1;
        while !frontier.is_empty() {
            for &idx in &frontier {
                let mut x = idx as usize;
                let eo_bits = if EO {
                    let b = x % EO_MASKS;
                    x /= EO_MASKS;
                    b
                } else {
                    0
                };
                let mut cv = [0u8; NC];
                let mut ev = [0u8; NE];
                for (v, d) in ev.iter_mut().zip(&t.edges).rev() {
                    *v = d.to_value[x % d.radix];
                    x /= d.radix;
                }
                for (v, d) in cv.iter_mut().zip(&t.corners).rev() {
                    *v = d.to_value[x % d.radix];
                    x /= d.radix;
                }
                for &m in moves {
                    let tab = &MT.0[m as usize];
                    let mut y = 0usize;
                    for (&v, d) in cv.iter().zip(&t.corners) {
                        y = y * d.radix + d.to_digit[tab[CORNER_TABLE][v as usize] as usize] as usize;
                    }
                    for (&v, d) in ev.iter().zip(&t.edges) {
                        y = y * d.radix + d.to_digit[tab[EDGE_TABLE][v as usize] as usize] as usize;
                    }
                    if EO {
                        y = y * EO_MASKS + eo_mv[eo_bits][m as usize] as usize;
                    }
                    if t.get_index(y) == UNSEEN {
                        t.set(y, depth + 1);
                        next.push(y as u32);
                        reachable += 1;
                    }
                }
            }
            if !next.is_empty() {
                depth += 1;
            }
            std::mem::swap(&mut frontier, &mut next);
            next.clear();
        }
        assert!(depth < UNSEEN, "distances must fit in a nibble");
        t.reachable = reachable;
        t.max_depth = depth;
        t
    }

    fn set(&mut self, i: usize, v: u8) {
        let shift = (i & 1) * NIBBLE_BITS;
        let b = &mut self.data.to_mut()[i >> 1];
        *b = *b & !(UNSEEN << shift) | v << shift;
    }

    #[inline(always)]
    fn get_index(&self, i: usize) -> u8 {
        // bounds-checked: a lane value outside the move group maps to
        // NO_DIGIT and must panic rather than read garbage
        (self.data[i >> 1] >> ((i & 1) * NIBBLE_BITS)) & UNSEEN
    }

    #[inline(always)]
    pub fn index(&self, c: &Cube) -> usize {
        let mut y = if EO { c.eo12() as usize } else { 0 };
        for d in &self.corners {
            y += d.weighted[c.c[d.lane] as usize] as usize;
        }
        for d in &self.edges {
            y += d.weighted[c.e[d.lane] as usize] as usize;
        }
        y
    }

    /// Distance of `c`'s tracked pieces from solved. `c` must be reachable
    /// from solved with the table's move set (true for every state the
    /// solver produces with that table's step).
    #[inline(always)]
    pub fn get(&self, c: &Cube) -> u8 {
        self.get_index(self.index(c))
    }

    pub fn same_data(&self, other: &Self) -> bool {
        self.data == other.data
    }

    /// Bytes of distance data.
    pub fn bytes(&self) -> usize {
        self.data.len()
    }
}
