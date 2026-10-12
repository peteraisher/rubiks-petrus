//! Pruning tables over small sets of tracked pieces.
//!
//! A table is indexed directly by the tracked lanes of a [`Cube`]: each lane
//! is already a 0..24 "(position, orientation)" digit, so the index is just a
//! base-24 number (optionally followed by the 12-bit edge-orientation mask).
//! The sparse encoding wastes a little space but needs no ranking, and the
//! shape is a const generic so a lookup unrolls to a handful of multiply-adds.

use crate::cube::{Cube, CORNER_TABLE, EDGE_TABLE, EO_MASKS, LANE_VALUES, MOVE_CUBIE, MT, NMOVES, N_EDGES};

/// Distance marker for states not reached by the BFS.
pub const UNSEEN: u8 = u8::MAX;

/// Distance table over `NC` corners and `NE` edges, plus all-edge
/// orientation when `EO` is set.
#[derive(Clone)]
pub struct PieceTable<const NC: usize, const NE: usize, const EO: bool> {
    corners: [usize; NC],
    edges: [usize; NE],
    pub data: Vec<u8>,
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
    const SIZE: usize = LANE_VALUES.pow((NC + NE) as u32) * if EO { EO_MASKS } else { 1 };

    pub fn build(corners: [usize; NC], edges: [usize; NE], moves: &[u8]) -> Self {
        let mut t = PieceTable { corners, edges, data: vec![UNSEEN; Self::SIZE], reachable: 0, max_depth: 0 };
        let eo_mv = if EO { eo_move_table() } else { Vec::new() };
        let start = t.index(&Cube::SOLVED);
        t.data[start] = 0;
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
                let mut cd = [0u8; NC];
                let mut ed = [0u8; NE];
                for d in ed.iter_mut().rev().chain(cd.iter_mut().rev()) {
                    *d = (x % LANE_VALUES) as u8;
                    x /= LANE_VALUES;
                }
                for &m in moves {
                    let tab = &MT.0[m as usize];
                    let mut y = 0usize;
                    for &d in &cd {
                        y = y * LANE_VALUES + tab[CORNER_TABLE][d as usize] as usize;
                    }
                    for &d in &ed {
                        y = y * LANE_VALUES + tab[EDGE_TABLE][d as usize] as usize;
                    }
                    if EO {
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
        for &lane in &self.corners {
            y = y * LANE_VALUES + c.c[lane] as usize;
        }
        for &lane in &self.edges {
            y = y * LANE_VALUES + c.e[lane] as usize;
        }
        if EO {
            y = y * EO_MASKS + c.eo12() as usize;
        }
        y
    }

    #[inline(always)]
    pub fn get(&self, c: &Cube) -> u8 {
        let i = self.index(c);
        debug_assert!(i < self.data.len());
        // SAFETY: every lane of a valid cube is < LANE_VALUES and eo12 <
        // EO_MASKS, so the index is below SIZE, the length of `data`.
        unsafe { *self.data.get_unchecked(i) }
    }
}
