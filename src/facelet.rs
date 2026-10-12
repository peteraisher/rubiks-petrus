//! Facelet level model, built from 3D geometry at compile time.
//!
//! Each sticker is a (cubie position, outward normal) pair of integer
//! vectors with x = L->R, y = D->U, z = B->F. Face turns and whole-cube
//! symmetries are integer rotation/reflection matrices acting on those pairs,
//! so move tables and the 48 symmetries fall out of the geometry instead of
//! being transcribed. Facelet order is Kociemba's: U1..U9 R1..R9 F1..F9
//! D1..D9 L1..L9 B1..B9.

use crate::cube::{Cubie, NMOVES};

type V3 = [i32; 3];
type M3 = [[i32; 3]; 3]; // row-major

const NORMAL: [V3; 6] = [[0, 1, 0], [1, 0, 0], [0, 0, 1], [0, -1, 0], [-1, 0, 0], [0, 0, -1]];
// Unit vectors pointing along a face's rows (left->right) and down its columns,
// as seen when looking at that face in the standard net.
const RIGHT: [V3; 6] = [[1, 0, 0], [0, 0, -1], [1, 0, 0], [1, 0, 0], [0, 0, 1], [-1, 0, 0]];
const DOWN: [V3; 6] = [[0, 0, 1], [0, -1, 0], [0, -1, 0], [0, 0, -1], [0, -1, 0], [0, -1, 0]];

pub const CORNER_FACELET: [[usize; 3]; 8] = [
    [8, 9, 20],
    [6, 18, 38],
    [0, 36, 47],
    [2, 45, 11],
    [29, 26, 15],
    [27, 44, 24],
    [33, 53, 42],
    [35, 17, 51],
];
pub const EDGE_FACELET: [[usize; 2]; 12] = [
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

pub const SOLVED_FACELETS: [u8; 54] = {
    let mut f = [0u8; 54];
    let mut i = 0;
    while i < 54 {
        f[i] = (i / 9) as u8;
        i += 1;
    }
    f
};

const fn sticker(f: usize) -> (V3, V3) {
    let face = f / 9;
    let row = ((f % 9) / 3) as i32 - 1;
    let col = (f % 3) as i32 - 1;
    let mut p = [0; 3];
    let mut k = 0;
    while k < 3 {
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
    while f < 54 {
        let (q, m) = sticker(f);
        if eq3(p, q) && eq3(n, m) {
            return f;
        }
        f += 1;
    }
    panic!("no such sticker")
}

const fn mat_vec(m: &M3, v: V3) -> V3 {
    let mut r = [0; 3];
    let mut i = 0;
    while i < 3 {
        r[i] = m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2];
        i += 1;
    }
    r
}

const fn mat_mul(a: &M3, b: &M3) -> M3 {
    let mut r = [[0; 3]; 3];
    let mut i = 0;
    while i < 3 {
        let mut j = 0;
        while j < 3 {
            r[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
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
    let mut m = [[0; 3]; 3];
    let mut j = 0;
    while j < 3 {
        let mut v = [0; 3];
        v[j] = 1;
        let d = a[0] * v[0] + a[1] * v[1] + a[2] * v[2];
        let cross = [a[1] * v[2] - a[2] * v[1], a[2] * v[0] - a[0] * v[2], a[0] * v[1] - a[1] * v[0]];
        let mut i = 0;
        while i < 3 {
            m[i][j] = a[i] * d - cross[i];
            i += 1;
        }
        j += 1;
    }
    m
}

const fn transform_perm(m: &M3, only_layer: Option<V3>) -> [u8; 54] {
    let mut perm = [0u8; 54];
    let mut f = 0;
    while f < 54 {
        let (p, n) = sticker(f);
        let moves = match only_layer {
            None => true,
            Some(a) => p[0] * a[0] + p[1] * a[1] + p[2] * a[2] == 1,
        };
        perm[f] = if moves { find_sticker(mat_vec(m, p), mat_vec(m, n)) as u8 } else { f as u8 };
        f += 1;
    }
    perm
}

/// perm[f] = where the sticker currently at f goes under a clockwise quarter turn.
pub const fn face_turn_perm(face: usize) -> [u8; 54] {
    transform_perm(&quarter(NORMAL[face]), Some(NORMAL[face]))
}

pub const fn apply_perm(state: &[u8; 54], perm: &[u8; 54]) -> [u8; 54] {
    let mut r = [0u8; 54];
    let mut f = 0;
    while f < 54 {
        r[perm[f] as usize] = state[f];
        f += 1;
    }
    r
}

/// Facelet colours (0..6 = U R F D L B) to cubie form. Assumes a well-formed
/// colouring; use [`parse_facelets`] for untrusted input.
pub const fn facelets_to_cubie(f: &[u8; 54]) -> Cubie {
    let mut k = Cubie::SOLVED;
    let mut i = 0;
    while i < 8 {
        let mut ori = 0;
        while ori < 3 {
            let c = f[CORNER_FACELET[i][ori]];
            if c == 0 || c == 3 {
                break;
            }
            ori += 1;
        }
        let ori = if ori == 3 { 0 } else { ori };
        let c1 = f[CORNER_FACELET[i][(ori + 1) % 3]];
        let c2 = f[CORNER_FACELET[i][(ori + 2) % 3]];
        let mut j = 0;
        while j < 8 {
            if c1 as usize == CORNER_FACELET[j][1] / 9 && c2 as usize == CORNER_FACELET[j][2] / 9 {
                k.cp[i] = j as u8;
                k.co[i] = ori as u8;
            }
            j += 1;
        }
        i += 1;
    }
    let mut i = 0;
    while i < 12 {
        let a = f[EDGE_FACELET[i][0]] as usize;
        let b = f[EDGE_FACELET[i][1]] as usize;
        let mut j = 0;
        while j < 12 {
            let (x, y) = (EDGE_FACELET[j][0] / 9, EDGE_FACELET[j][1] / 9);
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

pub fn cubie_to_facelets(k: &Cubie) -> [u8; 54] {
    let mut f = SOLVED_FACELETS;
    for i in 0..8 {
        let j = k.cp[i] as usize;
        let o = k.co[i] as usize;
        for n in 0..3 {
            f[CORNER_FACELET[i][(n + o) % 3]] = (CORNER_FACELET[j][n] / 9) as u8;
        }
    }
    for i in 0..12 {
        let j = k.ep[i] as usize;
        let o = k.eo[i] as usize;
        for n in 0..2 {
            f[EDGE_FACELET[i][(n + o) % 2]] = (EDGE_FACELET[j][n] / 9) as u8;
        }
    }
    f
}

/// Parse a 54-character facelet string. Any six distinct letters are
/// accepted; each centre's letter names that face's colour.
pub fn parse_facelets(s: &str) -> Result<Cubie, String> {
    let b = s.as_bytes();
    if b.len() != 54 {
        return Err(format!("facelet string must have 54 characters, got {}", b.len()));
    }
    let mut map = [255u8; 256];
    for face in 0..6 {
        let c = b[face * 9 + 4] as usize;
        if map[c] != 255 {
            return Err("two centres share a colour".into());
        }
        map[c] = face as u8;
    }
    let mut f = [0u8; 54];
    let mut count = [0; 6];
    for i in 0..54 {
        let v = map[b[i] as usize];
        if v == 255 {
            return Err(format!("unknown colour '{}'", b[i] as char));
        }
        f[i] = v;
        count[v as usize] += 1;
    }
    if count.iter().any(|&c| c != 9) {
        return Err("each colour must appear exactly 9 times".into());
    }
    let k = facelets_to_cubie(&f);
    if cubie_to_facelets(&k) != f {
        return Err("facelets do not describe real pieces".into());
    }
    Ok(k)
}

// ------------------------------------------------------------- symmetries

pub const NSYM: usize = 48;

/// All 48 symmetry matrices; index 0 is the identity, 0..24 are rotations.
pub const SYM_MAT: [M3; NSYM] = {
    let gens = [quarter([1, 0, 0]), quarter([0, 1, 0]), [[-1, 0, 0], [0, 1, 0], [0, 0, 1]]];
    let mut list = [[[0; 3]; 3]; NSYM];
    list[0] = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    let mut n = 1;
    let mut i = 0;
    while i < n {
        let mut g = 0;
        while g < 3 {
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
    let mut out = [[[0; 3]; 3]; NSYM];
    let mut k = 0;
    let mut pass = 0;
    while pass < 2 {
        let mut i = 0;
        while i < NSYM {
            if (det(&list[i]) == 1) == (pass == 0) {
                out[k] = list[i];
                k += 1;
            }
            i += 1;
        }
        pass += 1;
    }
    out
};

const fn mat_eq(a: &M3, b: &M3) -> bool {
    eq3(a[0], b[0]) && eq3(a[1], b[1]) && eq3(a[2], b[2])
}

const fn face_of(n: V3) -> usize {
    let mut f = 0;
    while f < 6 {
        if eq3(NORMAL[f], n) {
            return f;
        }
        f += 1;
    }
    panic!("not a face normal")
}

pub const SYM_PERM: [[u8; 54]; NSYM] = {
    let mut t = [[0u8; 54]; NSYM];
    let mut s = 0;
    while s < NSYM {
        t[s] = transform_perm(&SYM_MAT[s], None);
        s += 1;
    }
    t
};

pub const SYM_COLOR: [[u8; 6]; NSYM] = {
    let mut t = [[0u8; 6]; NSYM];
    let mut s = 0;
    while s < NSYM {
        let mut c = 0;
        while c < 6 {
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
        while f < 6 {
            let mut g = 0;
            while g < 6 {
                if SYM_COLOR[s][g] as usize == f {
                    break;
                }
                g += 1;
            }
            let mut p = 0;
            while p < 3 {
                let q = if SYM_IS_MIRROR[s] { 2 - p } else { p };
                t[s][f * 3 + p] = (g * 3 + q) as u8;
                p += 1;
            }
            f += 1;
        }
        s += 1;
    }
    t
};

pub fn conj_facelets(f: &[u8; 54], s: usize) -> [u8; 54] {
    let mut g = [0u8; 54];
    for i in 0..54 {
        g[SYM_PERM[s][i] as usize] = SYM_COLOR[s][f[i] as usize];
    }
    g
}
