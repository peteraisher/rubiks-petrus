//! Piece sets, move sets and pruning-table definitions for the canonical
//! Petrus frame. Shared with `build.rs`, which builds the tables at compile
//! time so the solver starts instantly.

use crate::cube::*;
use crate::tables::PieceTable;

/// Piece sets as (corners, edges).
pub type PieceSet<const NC: usize, const NE: usize> = ([usize; NC], [usize; NE]);
pub const BLOCK_222: PieceSet<1, 3> = ([DBL], [DL, DB, BL]);
pub const BLOCK_222_FRONT: PieceSet<1, 3> = ([DLF], [DL, DF, FL]);
/// The 2x2x3 split into two overlapping halves that fit in a table each.
pub const BLOCK_223_D: PieceSet<2, 3> = ([DLF, DBL], [DF, DL, DB]);
pub const BLOCK_223_L: PieceSet<2, 3> = ([DLF, DBL], [DL, FL, BL]);
/// Two more halves pairing the new expansion edges (DF, FL) with each end
/// of the block; they cut the 2x2x3 search by ~2.6x in nodes.
pub const BLOCK_223_FD: PieceSet<2, 3> = ([DLF, DBL], [DF, FL, DB]);
pub const BLOCK_223_FB: PieceSet<2, 3> = ([DLF, DBL], [DF, FL, BL]);
pub const BLOCK_223_CORNERS: PieceSet<2, 0> = ([DLF, DBL], []);
pub const F2L_REST: PieceSet<2, 3> = ([DFR, DRB], [DR, FR, BR]);

pub const ALL_MOVES: [u8; NMOVES] = {
    let mut a = [0u8; NMOVES];
    let mut i = 0;
    while i < NMOVES {
        a[i] = i as u8;
        i += 1;
    }
    a
};
/// All turns of the given faces; `M` must be `N * POWERS`.
const fn face_moves<const N: usize, const M: usize>(faces: [u8; N]) -> [u8; M] {
    assert!(M == N * POWERS);
    let mut a = [0u8; M];
    let mut i = 0;
    while i < M {
        a[i] = mv(faces[i / POWERS], (i % POWERS) as u8);
        i += 1;
    }
    a
}
pub const RUFB_MOVES: [u8; 4 * POWERS] = face_moves([FACE_U, FACE_R, FACE_F, FACE_B]);
pub const RU_MOVES: [u8; 2 * POWERS] = face_moves([FACE_U, FACE_R]);

/// Table shapes: (corners, edges, all-edge orientation).
pub type Table222 = PieceTable<1, 3, false>;
pub type Table223 = PieceTable<2, 3, false>;
pub type TableEo = PieceTable<2, 0, true>;

/// A named pruning table: which pieces, under which moves.
pub struct TableSpec<const NC: usize, const NE: usize> {
    pub name: &'static str,
    pub set: PieceSet<NC, NE>,
    pub moves: &'static [u8],
}

impl<const NC: usize, const NE: usize> TableSpec<NC, NE> {
    pub fn build<const EO: bool>(&self) -> PieceTable<NC, NE, EO> {
        PieceTable::build(self.set.0, self.set.1, self.moves)
    }

    pub fn with_data<const EO: bool>(&self, data: &'static [u8]) -> PieceTable<NC, NE, EO> {
        PieceTable::with_data(self.set.0, self.set.1, self.moves, data)
    }
}

pub const T222: TableSpec<1, 3> = TableSpec { name: "t222", set: BLOCK_222, moves: &ALL_MOVES };
pub const T222_FRONT: TableSpec<1, 3> = TableSpec { name: "t222_front", set: BLOCK_222_FRONT, moves: &ALL_MOVES };
pub const T223_D: TableSpec<2, 3> = TableSpec { name: "t223_d", set: BLOCK_223_D, moves: &ALL_MOVES };
pub const T223_L: TableSpec<2, 3> = TableSpec { name: "t223_l", set: BLOCK_223_L, moves: &ALL_MOVES };
pub const T223_FD: TableSpec<2, 3> = TableSpec { name: "t223_fd", set: BLOCK_223_FD, moves: &ALL_MOVES };
pub const T223_FB: TableSpec<2, 3> = TableSpec { name: "t223_fb", set: BLOCK_223_FB, moves: &ALL_MOVES };
/// Edge orientation plus the two block corners, under <R,U,F,B>.
pub const TEO: TableSpec<2, 0> = TableSpec { name: "teo", set: BLOCK_223_CORNERS, moves: &RUFB_MOVES };
/// Remaining F2L pieces under <R,U>: exact.
pub const TF2L: TableSpec<2, 3> = TableSpec { name: "tf2l", set: F2L_REST, moves: &RU_MOVES };

/// Every table the solver embeds, as (file name, nibble data), built in
/// parallel.
pub fn build_all() -> Vec<(&'static str, Vec<u8>)> {
    std::thread::scope(|s| {
        let jobs = [
            s.spawn(|| (T222.name, T222.build::<false>().into_data())),
            s.spawn(|| (T222_FRONT.name, T222_FRONT.build::<false>().into_data())),
            s.spawn(|| (T223_D.name, T223_D.build::<false>().into_data())),
            s.spawn(|| (T223_L.name, T223_L.build::<false>().into_data())),
            s.spawn(|| (T223_FD.name, T223_FD.build::<false>().into_data())),
            s.spawn(|| (T223_FB.name, T223_FB.build::<false>().into_data())),
            s.spawn(|| (TEO.name, TEO.build::<true>().into_data())),
            s.spawn(|| (TF2L.name, TF2L.build::<false>().into_data())),
        ];
        jobs.into_iter().map(|j| j.join().unwrap()).collect()
    })
}
