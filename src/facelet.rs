//! Facelet level model, built from 3D geometry at compile time.
//!
//! Each sticker is a (cubie position, outward normal) pair of integer
//! vectors with x = L->R, y = D->U, z = B->F. Face turns and whole-cube
//! symmetries are integer rotation/reflection matrices acting on those pairs,
//! so move tables and the 48 symmetries fall out of the geometry instead of
//! being transcribed. Facelet order is Kociemba's: U1..U9 R1..R9 F1..F9
//! D1..D9 L1..L9 B1..B9.

use crate::cube::{
    Cubie, CORNER_TWISTS, EDGE_FLIPS, FACE_D, FACE_U, LANE_VALUES, NMOVES, N_CORNERS, N_EDGES, N_FACES, POWERS,
};

/// Spatial dimensions, and stickers along one side of a face.
const DIM: usize = 3;
const SIDE: usize = 3;
pub const PER_FACE: usize = SIDE * SIDE;
pub const N_FACELETS: usize = N_FACES * PER_FACE;

type V3 = [i32; DIM];
type M3 = [[i32; DIM]; DIM]; // row-major

const NORMAL: [V3; N_FACES] = [[0, 1, 0], [1, 0, 0], [0, 0, 1], [0, -1, 0], [-1, 0, 0], [0, 0, -1]];
// Unit vectors pointing along a face's rows (left->right) and down its columns,
// as seen when looking at that face in the standard net.
const RIGHT: [V3; N_FACES] = [[1, 0, 0], [0, 0, -1], [1, 0, 0], [1, 0, 0], [0, 0, 1], [-1, 0, 0]];
const DOWN: [V3; N_FACES] = [[0, 0, 1], [0, -1, 0], [0, -1, 0], [0, 0, -1], [0, -1, 0], [0, -1, 0]];

pub const CORNER_FACELET: [[usize; CORNER_TWISTS]; N_CORNERS] =
    [[8, 9, 20], [6, 18, 38], [0, 36, 47], [2, 45, 11], [29, 26, 15], [27, 44, 24], [33, 53, 42], [35, 17, 51]];
pub const EDGE_FACELET: [[usize; EDGE_FLIPS]; N_EDGES] = [
    [5, 10],
    [7, 19],
    [3, 37],
    [1, 46],
    [32, 16],
    [28, 25],
    [30, 43],
    [34, 52],
    [23, 12],
    [21, 41],
    [50, 39],
    [48, 14],
];

pub const SOLVED_FACELETS: [u8; N_FACELETS] = {
    let mut f = [0u8; N_FACELETS];
    let mut i = 0;
    while i < N_FACELETS {
        f[i] = (i / PER_FACE) as u8;
        i += 1;
    }
    f
};

const fn sticker(f: usize) -> (V3, V3) {
    // rows and columns run -1..=1 across the face
    let face = f / PER_FACE;
    let row = ((f % PER_FACE) / SIDE) as i32 - 1;
    let col = (f % SIDE) as i32 - 1;
    let mut p = [0; DIM];
    let mut k = 0;
    while k < DIM {
        p[k] = NORMAL[face][k] + col * RIGHT[face][k] + row * DOWN[face][k];
        k += 1;
    }
    (p, NORMAL[face])
}

const fn eq3(a: V3, b: V3) -> bool {
    a[0] == b[0] && a[1] == b[1] && a[2] == b[2]
}

const fn find_sticker(p: V3, n: V3) -> usize {
    let mut f = 0;
    while f < N_FACELETS {
        let (q, m) = sticker(f);
        if eq3(p, q) && eq3(n, m) {
            return f;
        }
        f += 1;
    }
    panic!("no such sticker")
}

const fn mat_vec(m: &M3, v: V3) -> V3 {
    let mut r = [0; DIM];
    let mut i = 0;
    while i < DIM {
        r[i] = dot(m[i], v);
        i += 1;
    }
    r
}

const fn dot(a: V3, b: V3) -> i32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

const fn mat_mul(a: &M3, b: &M3) -> M3 {
    let mut r = [[0; DIM]; DIM];
    let mut i = 0;
    while i < DIM {
        let mut j = 0;
        while j < DIM {
            r[i][j] = dot(a[i], [b[0][j], b[1][j], b[2][j]]);
            j += 1;
        }
        i += 1;
    }
    r
}

const fn det(m: &M3) -> i32 {
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
}

/// Quarter turn clockwise when looking at the face with outward normal `a`:
/// a rotation of -90 degrees about `a`, i.e. v' = a(a.v) - a x v.
const fn quarter(a: V3) -> M3 {
    let mut m = [[0; DIM]; DIM];
    let mut j = 0;
    while j < DIM {
        let mut v = [0; DIM];
        v[j] = 1;
        let d = dot(a, v);
        let cross = [a[1] * v[2] - a[2] * v[1], a[2] * v[0] - a[0] * v[2], a[0] * v[1] - a[1] * v[0]];
        let mut i = 0;
        while i < DIM {
            m[i][j] = a[i] * d - cross[i];
            i += 1;
        }
        j += 1;
    }
    m
}

const fn transform_perm(m: &M3, only_layer: Option<V3>) -> [u8; N_FACELETS] {
    let mut perm = [0u8; N_FACELETS];
    let mut f = 0;
    while f < N_FACELETS {
        let (p, n) = sticker(f);
        let moves = match only_layer {
            None => true,
            Some(a) => dot(p, a) == 1,
        };
        perm[f] = if moves { find_sticker(mat_vec(m, p), mat_vec(m, n)) as u8 } else { f as u8 };
        f += 1;
    }
    perm
}

/// perm[f] = where the sticker currently at f goes under a clockwise quarter turn.
pub const fn face_turn_perm(face: usize) -> [u8; N_FACELETS] {
    transform_perm(&quarter(NORMAL[face]), Some(NORMAL[face]))
}

pub const fn apply_perm(state: &[u8; N_FACELETS], perm: &[u8; N_FACELETS]) -> [u8; N_FACELETS] {
    let mut r = [0u8; N_FACELETS];
    let mut f = 0;
    while f < N_FACELETS {
        r[perm[f] as usize] = state[f];
        f += 1;
    }
    r
}

/// Facelet colours (0..6 = U R F D L B) to cubie form. Assumes a well-formed
/// colouring; use [`parse_facelets`] for untrusted input.
pub const fn facelets_to_cubie(f: &[u8; N_FACELETS]) -> Cubie {
    let mut k = Cubie::SOLVED;
    let mut i = 0;
    while i < N_CORNERS {
        // orientation = which of the position's stickers shows the U/D colour
        let mut ori = 0;
        while ori < CORNER_TWISTS {
            let c = f[CORNER_FACELET[i][ori]];
            if c == FACE_U || c == FACE_D {
                break;
            }
            ori += 1;
        }
        let ori = if ori == CORNER_TWISTS { 0 } else { ori };
        let c1 = f[CORNER_FACELET[i][(ori + 1) % CORNER_TWISTS]];
        let c2 = f[CORNER_FACELET[i][(ori + 2) % CORNER_TWISTS]];
        let mut j = 0;
        while j < N_CORNERS {
            if c1 as usize == CORNER_FACELET[j][1] / PER_FACE && c2 as usize == CORNER_FACELET[j][2] / PER_FACE {
                k.cp[i] = j as u8;
                k.co[i] = ori as u8;
            }
            j += 1;
        }
        i += 1;
    }
    let mut i = 0;
    while i < N_EDGES {
        let a = f[EDGE_FACELET[i][0]] as usize;
        let b = f[EDGE_FACELET[i][1]] as usize;
        let mut j = 0;
        while j < N_EDGES {
            let (x, y) = (EDGE_FACELET[j][0] / PER_FACE, EDGE_FACELET[j][1] / PER_FACE);
            if a == x && b == y {
                k.ep[i] = j as u8;
                k.eo[i] = 0;
            } else if a == y && b == x {
                k.ep[i] = j as u8;
                k.eo[i] = 1;
            }
            j += 1;
        }
        i += 1;
    }
    k
}

pub fn cubie_to_facelets(k: &Cubie) -> [u8; N_FACELETS] {
    let mut f = SOLVED_FACELETS;
    for i in 0..N_CORNERS {
        let j = k.cp[i] as usize;
        let o = k.co[i] as usize;
        for n in 0..CORNER_TWISTS {
            f[CORNER_FACELET[i][(n + o) % CORNER_TWISTS]] = (CORNER_FACELET[j][n] / PER_FACE) as u8;
        }
    }
    for i in 0..N_EDGES {
        let j = k.ep[i] as usize;
        let o = k.eo[i] as usize;
        for n in 0..EDGE_FLIPS {
            f[EDGE_FACELET[i][(n + o) % EDGE_FLIPS]] = (EDGE_FACELET[j][n] / PER_FACE) as u8;
        }
    }
    f
}

/// Parse a 54-character facelet string. Any six distinct letters are
/// accepted; each centre's letter names that face's colour.
pub fn parse_facelets(s: &str) -> Result<Cubie, String> {
    const CENTRE: usize = PER_FACE / 2;
    let b = s.as_bytes();
    if b.len() != N_FACELETS {
        return Err(format!("facelet string must have {N_FACELETS} characters, got {}", b.len()));
    }
    let mut map = [None; 1 << u8::BITS];
    for face in 0..N_FACES {
        let c = b[face * PER_FACE + CENTRE] as usize;
        if map[c].is_some() {
            return Err("two centres share a colour".into());
        }
        map[c] = Some(face as u8);
    }
    let mut f = [0u8; N_FACELETS];
    let mut count = [0; N_FACES];
    for i in 0..N_FACELETS {
        let v = map[b[i] as usize].ok_or_else(|| format!("unknown colour '{}'", b[i] as char))?;
        f[i] = v;
        count[v as usize] += 1;
    }
    if count.iter().any(|&c| c != PER_FACE) {
        return Err(format!("each colour must appear exactly {PER_FACE} times"));
    }
    let k = facelets_to_cubie(&f);
    if cubie_to_facelets(&k) != f {
        return Err("facelets do not describe real pieces".into());
    }
    Ok(k)
}

// ------------------------------------------------------------- symmetries

/// Whole-cube rotations (24) and their mirror images.
pub const NSYM: usize = 48;
pub const N_ROTATIONS: usize = NSYM / 2;

/// All 48 symmetry matrices; index 0 is the identity, 0..24 are rotations.
pub const SYM_MAT: [M3; NSYM] = {
    const N_GENS: usize = 3;
    let gens: [M3; N_GENS] = [quarter([1, 0, 0]), quarter([0, 1, 0]), [[-1, 0, 0], [0, 1, 0], [0, 0, 1]]];
    let mut list = [[[0; DIM]; DIM]; NSYM];
    list[0] = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    let mut n = 1;
    let mut i = 0;
    while i < n {
        let mut g = 0;
        while g < N_GENS {
            let m = mat_mul(&gens[g], &list[i]);
            let mut found = false;
            let mut j = 0;
            while j < n {
                if mat_eq(&m, &list[j]) {
                    found = true;
                }
                j += 1;
            }
            if !found {
                list[n] = m;
                n += 1;
            }
            g += 1;
        }
        i += 1;
    }
    assert!(n == NSYM);
    // stable partition: rotations first
    let mut out = [[[0; DIM]; DIM]; NSYM];
    let (mut rot, mut mir) = (0, N_ROTATIONS);
    let mut i = 0;
    while i < NSYM {
        if det(&list[i]) > 0 {
            out[rot] = list[i];
            rot += 1;
        } else {
            out[mir] = list[i];
            mir += 1;
        }
        i += 1;
    }
    assert!(rot == N_ROTATIONS && mir == NSYM);
    out
};

const fn mat_eq(a: &M3, b: &M3) -> bool {
    eq3(a[0], b[0]) && eq3(a[1], b[1]) && eq3(a[2], b[2])
}

const fn face_of(n: V3) -> usize {
    let mut f = 0;
    while f < N_FACES {
        if eq3(NORMAL[f], n) {
            return f;
        }
        f += 1;
    }
    panic!("not a face normal")
}

pub const SYM_PERM: [[u8; N_FACELETS]; NSYM] = {
    let mut t = [[0u8; N_FACELETS]; NSYM];
    let mut s = 0;
    while s < NSYM {
        t[s] = transform_perm(&SYM_MAT[s], None);
        s += 1;
    }
    t
};

pub const SYM_COLOR: [[u8; N_FACES]; NSYM] = {
    let mut t = [[0u8; N_FACES]; NSYM];
    let mut s = 0;
    while s < NSYM {
        let mut c = 0;
        while c < N_FACES {
            t[s][c] = face_of(mat_vec(&SYM_MAT[s], NORMAL[c])) as u8;
            c += 1;
        }
        s += 1;
    }
    t
};

pub const SYM_IS_MIRROR: [bool; NSYM] = {
    let mut t = [false; NSYM];
    let mut s = 0;
    while s < NSYM {
        t[s] = det(&SYM_MAT[s]) < 0;
        s += 1;
    }
    t
};

/// A move `m` made on the conjugated cube corresponds to `SYM_MOVE[s][m]` on
/// the original: the face is mapped back by S^-1 and a reflection reverses
/// the turning direction.
pub const SYM_MOVE: [[u8; NMOVES]; NSYM] = {
    let mut t = [[0u8; NMOVES]; NSYM];
    let mut s = 0;
    while s < NSYM {
        let mut f = 0;
        while f < N_FACES {
            let mut g = 0;
            while g < N_FACES {
                if SYM_COLOR[s][g] as usize == f {
                    break;
                }
                g += 1;
            }
            let mut p = 0;
            while p < POWERS {
                let q = if SYM_IS_MIRROR[s] { POWERS - 1 - p } else { p };
                t[s][f * POWERS + p] = (g * POWERS + q) as u8;
                p += 1;
            }
            f += 1;
        }
        s += 1;
    }
    t
};

pub fn conj_facelets(f: &[u8; N_FACELETS], s: usize) -> [u8; N_FACELETS] {
    let mut g = [0u8; N_FACELETS];
    for i in 0..N_FACELETS {
        g[SYM_PERM[s][i] as usize] = SYM_COLOR[s][f[i] as usize];
    }
    g
}

/// Conjugation acts on each piece independently: piece `i` at lane value
/// `v` (position, orientation) becomes piece `CONJ_*_PIECE[s][i]` at value
/// `CONJ_*_VALUE[s][i][v]`. Derived by pushing that piece's stickers through
/// the symmetry, exactly as [`conj_facelets`] does for a whole cube.
pub struct ConjTables {
    pub corner_piece: [[u8; N_CORNERS]; NSYM],
    pub corner_value: [[[u8; LANE_VALUES]; N_CORNERS]; NSYM],
    pub edge_piece: [[u8; N_EDGES]; NSYM],
    pub edge_value: [[[u8; LANE_VALUES]; N_EDGES]; NSYM],
}

const fn find_corner_facelet(f: usize) -> (usize, usize) {
    let mut p = 0;
    while p < N_CORNERS {
        let mut k = 0;
        while k < CORNER_TWISTS {
            if CORNER_FACELET[p][k] == f {
                return (p, k);
            }
            k += 1;
        }
        p += 1;
    }
    panic!("not a corner facelet")
}

const fn find_edge_facelet(f: usize) -> (usize, usize) {
    let mut p = 0;
    while p < N_EDGES {
        let mut k = 0;
        while k < EDGE_FLIPS {
            if EDGE_FACELET[p][k] == f {
                return (p, k);
            }
            k += 1;
        }
        p += 1;
    }
    panic!("not an edge facelet")
}

/// (new piece, new lane value) of corner `i` at position `p`, twist `o`.
const fn conj_corner(s: usize, i: usize, p: usize, o: usize) -> (u8, u8) {
    let mut colour_at = [0u8; CORNER_TWISTS];
    let mut new_pos = 0;
    let mut n = 0;
    while n < CORNER_TWISTS {
        let f = SYM_PERM[s][CORNER_FACELET[p][(n + o) % CORNER_TWISTS]] as usize;
        let (q, k) = find_corner_facelet(f);
        new_pos = q;
        colour_at[k] = SYM_COLOR[s][CORNER_FACELET[i][n] / PER_FACE];
        n += 1;
    }
    let mut ori = 0;
    while colour_at[ori] != FACE_U && colour_at[ori] != FACE_D {
        ori += 1;
    }
    let c1 = colour_at[(ori + 1) % CORNER_TWISTS] as usize;
    let c2 = colour_at[(ori + 2) % CORNER_TWISTS] as usize;
    let mut j = 0;
    while !(CORNER_FACELET[j][1] / PER_FACE == c1 && CORNER_FACELET[j][2] / PER_FACE == c2) {
        j += 1;
    }
    (j as u8, (new_pos + N_CORNERS * ori) as u8)
}

const fn conj_edge(s: usize, i: usize, p: usize, o: usize) -> (u8, u8) {
    let mut colour_at = [0usize; EDGE_FLIPS];
    let mut new_pos = 0;
    let mut n = 0;
    while n < EDGE_FLIPS {
        let f = SYM_PERM[s][EDGE_FACELET[p][(n + o) % EDGE_FLIPS]] as usize;
        let (q, k) = find_edge_facelet(f);
        new_pos = q;
        colour_at[k] = SYM_COLOR[s][EDGE_FACELET[i][n] / PER_FACE] as usize;
        n += 1;
    }
    let mut j = 0;
    while j < N_EDGES {
        let (x, y) = (EDGE_FACELET[j][0] / PER_FACE, EDGE_FACELET[j][1] / PER_FACE);
        if colour_at[0] == x && colour_at[1] == y {
            return (j as u8, new_pos as u8);
        }
        if colour_at[0] == y && colour_at[1] == x {
            return (j as u8, (new_pos + N_EDGES) as u8);
        }
        j += 1;
    }
    panic!("no such edge")
}

pub static CONJ: ConjTables = {
    let mut t = ConjTables {
        corner_piece: [[0; N_CORNERS]; NSYM],
        corner_value: [[[0; LANE_VALUES]; N_CORNERS]; NSYM],
        edge_piece: [[0; N_EDGES]; NSYM],
        edge_value: [[[0; LANE_VALUES]; N_EDGES]; NSYM],
    };
    let mut s = 0;
    while s < NSYM {
        let mut i = 0;
        while i < N_CORNERS {
            let mut v = 0;
            while v < LANE_VALUES {
                let (j, w) = conj_corner(s, i, v % N_CORNERS, v / N_CORNERS);
                assert!(v == 0 || j == t.corner_piece[s][i]);
                t.corner_piece[s][i] = j;
                t.corner_value[s][i][v] = w;
                v += 1;
            }
            i += 1;
        }
        let mut i = 0;
        while i < N_EDGES {
            let mut v = 0;
            while v < LANE_VALUES {
                let (j, w) = conj_edge(s, i, v % N_EDGES, v / N_EDGES);
                assert!(v == 0 || j == t.edge_piece[s][i]);
                t.edge_piece[s][i] = j;
                t.edge_value[s][i][v] = w;
                v += 1;
            }
            i += 1;
        }
        s += 1;
    }
    t
};
