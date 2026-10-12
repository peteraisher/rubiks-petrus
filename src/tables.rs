//! Pruning tables over small sets of tracked pieces.
//!
//! A table is indexed directly by the tracked lanes of a [`Cube`]: each lane
//! is already a 0..24 "(position, orientation)" digit, so the index is just a
//! base-24 number (optionally followed by the 12-bit edge-orientation mask).
//! The sparse encoding wastes a little space but needs no ranking, which keeps
//! the per-node cost of a lookup to a few multiply-adds.

use crate::cube::{Cube, CORNER_TABLE, EDGE_TABLE, EO_MASKS, LANE_VALUES, MOVE_CUBIE, MT, NMOVES, N_EDGES};

/// Upper bound on tracked pieces per table (keeps digit buffers on the stack).
pub const MAX_TRACKED: usize = 6;

/// Distance marker for states not reached by the BFS.
pub const UNSEEN: u8 = u8::MAX;

#[derive(Clone)]
pub struct PieceTable {
    /// (is_corner, lane) per digit, most significant first.
    lanes: Vec<(bool, usize)>,
    eo: bool,
    pub data: Vec<u8>,
    pub reachable: usize,
    pub max_depth: u8,
}

/// Edge-orientation mask after each move: EO_MOVE[m][mask].
pub fn eo_move_table() -> Vec<[u16; NMOVES]> {
    let mut t = vec![[0u16; NMOVES]; EO_MASKS];
    for mask in 0..EO_MASKS {
        for m in 0..NMOVES {
            let k = &MOVE_CUBIE[m];
            let mut r = 0u16;
            for p in 0..N_EDGES {
                let b = ((mask >> k.ep[p]) & 1) as u8 ^ k.eo[p];
                r |= (b as u16) << p;
            }
            t[mask][m] = r;
        }
    }
    t
}

impl PieceTable {
    pub fn build(corners: &[usize], edges: &[usize], eo: bool, moves: &[u8]) -> PieceTable {
        let lanes: Vec<(bool, usize)> =
            corners.iter().map(|&c| (true, c)).chain(edges.iter().map(|&e| (false, e))).collect();
        assert!(lanes.len() <= MAX_TRACKED);
        let mut size = LANE_VALUES.pow(lanes.len() as u32);
        if eo {
            size *= EO_MASKS;
        }
        let mut t = PieceTable { lanes, eo, data: vec![UNSEEN; size], reachable: 0, max_depth: 0 };
        let eo_mv = if eo { eo_move_table() } else { Vec::new() };
        let start = t.index(&Cube::SOLVED);
        t.data[start] = 0;
        let mut frontier = vec![start as u32];
        let mut next = Vec::new();
        let mut depth = 0u8;
        let n = t.lanes.len();
        let mut digits = [0u8; MAX_TRACKED];
        let mut reachable = 1;
        while !frontier.is_empty() {
            for &idx in &frontier {
                let mut x = idx as usize;
                let eo_bits = if eo {
                    let b = x % EO_MASKS;
                    x /= EO_MASKS;
                    b
                } else {
                    0
                };
                for i in (0..n).rev() {
                    digits[i] = (x % LANE_VALUES) as u8;
                    x /= LANE_VALUES;
                }
                for &m in moves {
                    let tab = &MT.0[m as usize];
                    let mut y = 0usize;
                    for i in 0..n {
                        let kind = if t.lanes[i].0 { CORNER_TABLE } else { EDGE_TABLE };
                        y = y * LANE_VALUES + tab[kind][digits[i] as usize] as usize;
                    }
                    if eo {
                        y = y * EO_MASKS + eo_mv[eo_bits][m as usize] as usize;
                    }
                    if t.data[y] == UNSEEN {
                        t.data[y] = depth + 1;
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
        t.reachable = reachable;
        t.max_depth = depth;
        t
    }

    #[inline(always)]
    pub fn index(&self, c: &Cube) -> usize {
        let mut y = 0usize;
        for &(corner, lane) in &self.lanes {
            let d = if corner { c.c[lane] } else { c.e[lane] };
            y = y * LANE_VALUES + d as usize;
        }
        if self.eo {
            y = y * EO_MASKS + c.eo12() as usize;
        }
        y
    }

    #[inline(always)]
    pub fn get(&self, c: &Cube) -> u8 {
        // SAFETY: every lane of a valid cube is < 24 and eo12 < 4096, so the
        // index is within the table built for exactly this shape.
        unsafe { *self.data.get_unchecked(self.index(c)) }
    }
}
