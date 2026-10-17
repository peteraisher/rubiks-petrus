//! Table generation, run by `build.rs`: a breadth-first search over each
//! phase's coset space, then up to `MAX_ALTS` optimal algorithms per case.
//!
//! Alternatives are spread evenly over all optimal algorithms of a case in
//! lexicographic order (so they differ early, not just in their last
//! moves), found without enumerating them: the number of optimal canonical
//! paths from every (case, previous face) is counted once, and the chosen
//! ranks are unranked. They are stored in van der Corput order, so the first
//! k of them are spread too.

use crate::coord::*;
use petrus::cube::*;
use petrus::facelet::SYM_MOVE;

pub const UNSEEN: u8 = u8::MAX;
/// Previous-face states for canonical sequences: each face, or none yet.
const LAST_STATES: usize = N_FACES + 1;

pub type IndexFn = fn(&CornerTables, &Cube) -> usize;

pub struct PhaseDef {
    pub moves: &'static [u8],
    pub n: usize,
    pub idx: IndexFn,
}

pub const PHASES: [PhaseDef; N_PHASES] = [
    PhaseDef { moves: &MOVES1, n: N1, idx: |_, c| idx1(c) },
    PhaseDef { moves: &MOVES2, n: N2, idx: |_, c| idx2(c) },
    PhaseDef { moves: &MOVES3, n: N3, idx: idx3 },
    PhaseDef { moves: &MOVES4, n: N4, idx: idx4 },
];

/// Canonical sequences: no face twice in a row, opposite faces in one order.
pub fn allowed(last: u8, f: u8) -> bool {
    f != last && !(opposite(f) == last && f < last)
}

/// Corner tables (see [`CornerTables`]), from the squares group's 96
/// corner permutations.
///
/// Relabelling the pieces of a corner arrangement x by a member h of that
/// group (x -> x o h) keeps each tetrad's set of positions and permutes the
/// pieces within the tetrads, so x's coset is the placement of tetrad A
/// (70) times the orbit of its pair of tetrad orders (576 / 96 = 6).
pub fn corner_tables() -> (Vec<u8>, Vec<u8>) {
    let mut group = vec![Cube::SOLVED.c];
    let mut i = 0;
    while i < group.len() {
        for &m in &MOVES4 {
            let mut c = Cube::SOLVED;
            c.c = group[i];
            let n = c.apply_move(m as usize).c;
            if !group.contains(&n) {
                group.push(n);
            }
        }
        i += 1;
    }
    assert_eq!(group.len(), G3_CORNER_PERMS);
    let pats = |x: &[u8; LANES]| {
        let mut c = Cube::SOLVED;
        c.c = *x;
        corner_patterns(&c)
    };
    let mut g3 = vec![u8::MAX; PATTERNS * PATTERNS];
    let mut members: Vec<usize> = group.iter().map(|h| pats(h).1).collect();
    members.sort_unstable();
    for (k, &p) in members.iter().enumerate() {
        assert_eq!(g3[p], u8::MAX, "order patterns do not identify the group's members");
        g3[p] = k as u8;
    }
    // every arrangement, its relabellings, and the classes of their orders
    let mut perms = Vec::new();
    let mut p: Vec<u8> = (0..N_CORNERS as u8).collect();
    loop {
        let mut c = [EMPTY_LANE; LANES];
        c[..N_CORNERS].copy_from_slice(&p);
        perms.push(c);
        if !next_permutation(&mut p) {
            break;
        }
    }
    let relabel = |x: &[u8; LANES], h: &[u8; LANES]| {
        let mut y = *x;
        for q in 0..N_CORNERS {
            y[q] = x[h[q] as usize];
        }
        y
    };
    let mut class = vec![u8::MAX; PATTERNS * PATTERNS];
    let mut next = 0u8;
    for x in &perms {
        if class[pats(x).1] != u8::MAX {
            continue;
        }
        for h in &group {
            let (placement, orders) = pats(&relabel(x, h));
            assert_eq!(placement, pats(x).0);
            assert!(class[orders] == u8::MAX || class[orders] == next, "orders do not split into orbits");
            class[orders] = next;
        }
        next += 1;
    }
    assert_eq!(next as usize, TETRAD_CLASSES);
    // the coset coordinate is constant on cosets and tells them apart
    let mut seen = std::collections::HashMap::new();
    for x in &perms {
        let key = |y: &[u8; LANES]| {
            let (placement, orders) = pats(y);
            (placement, class[orders])
        };
        for h in &group {
            assert_eq!(key(&relabel(x, h)), key(x));
        }
        seen.entry(key(x)).or_insert(0usize);
    }
    assert_eq!(seen.len(), CORNER_COSETS);
    (class, g3)
}

fn next_permutation(p: &mut [u8]) -> bool {
    let Some(i) = (1..p.len()).rev().find(|&i| p[i - 1] < p[i]) else { return false };
    let j = (i..p.len()).rev().find(|&j| p[j] > p[i - 1]).unwrap();
    p.swap(i - 1, j);
    p[i..].reverse();
    true
}

/// Distance of every case from the phase goal, and a representative cube.
pub fn bfs(ct: &CornerTables, ph: &PhaseDef) -> (Vec<u8>, Vec<Cube>) {
    let mut dist = vec![UNSEEN; ph.n];
    let mut reps = vec![Cube::SOLVED; ph.n];
    let start = (ph.idx)(ct, &Cube::SOLVED);
    dist[start] = 0;
    let mut frontier = vec![start];
    let mut depth = 0;
    while !frontier.is_empty() {
        let mut next = Vec::new();
        for &s in &frontier {
            for &m in ph.moves {
                let c = reps[s].apply_move(m as usize);
                let i = (ph.idx)(ct, &c);
                if dist[i] == UNSEEN {
                    dist[i] = depth + 1;
                    reps[i] = c;
                    next.push(i);
                }
            }
        }
        depth += 1;
        frontier = next;
    }
    assert!(dist.iter().all(|&d| d != UNSEEN), "coordinate is not onto");
    (dist, reps)
}

/// Distance of every case of a phase to the set of cases where `goal`
/// holds, with the phase's moves (a multi-source breadth-first search; the
/// move sets are closed under inverses).
pub fn bfs_to(ct: &CornerTables, ph: &PhaseDef, reps: &[Cube], goal: impl Fn(usize) -> bool) -> Vec<u8> {
    let mut dist = vec![UNSEEN; ph.n];
    let mut frontier: Vec<usize> = (0..ph.n).filter(|&i| goal(i)).collect();
    for &i in &frontier {
        dist[i] = 0;
    }
    let mut depth = 0;
    while !frontier.is_empty() {
        let mut next = Vec::new();
        for &s in &frontier {
            for &m in ph.moves {
                let i = (ph.idx)(ct, &reps[s].apply_move(m as usize));
                if dist[i] == UNSEEN {
                    dist[i] = depth + 1;
                    next.push(i);
                }
            }
        }
        depth += 1;
        frontier = next;
    }
    dist
}

/// The split phases' tables (see [`N_HALVES`]): each half's first optimal
/// canonical algorithm per case, from the BFS of phases 2 and 4 (`dist`,
/// `reps` by phase).
pub fn split_algorithms(ct: &CornerTables, dist: &[Vec<u8>], reps: &[Vec<Cube>]) -> Vec<Vec<u64>> {
    let (p2, p4) = (HALF_PHASE[HALF_SLICE], HALF_PHASE[HALF_CORNERS]);
    let (ph2, ph4) = (&PHASES[p2], &PHASES[p4]);
    // the goals' cases: each half fixes one part of its phase's goal
    let (g2, g4) = ((ph2.idx)(ct, &Cube::SOLVED), (ph4.idx)(ct, &Cube::SOLVED));
    let (slice, twist0) = (g2 % SLICE_COMBS, g2 - g2 % SLICE_COMBS);
    let (edges0, corners) = (g4 % G3_EDGE_PERMS, g4 - g4 % G3_EDGE_PERMS);
    let slice_placed = bfs_to(ct, ph2, &reps[p2], |i| i % SLICE_COMBS == slice);
    let corners_solved = bfs_to(ct, ph4, &reps[p4], |i| i - i % G3_EDGE_PERMS == corners);
    let encode_path = |ph: &PhaseDef, p: usize, path: Vec<u8>| {
        let slots: Vec<usize> = path.iter().map(|&m| ph.moves.iter().position(|&x| x == m).unwrap()).collect();
        encode(&slots, MOVE_BITS[p])
    };
    // each half's cases, as cases of its phase's space
    let cases = |h: usize, f: &dyn Fn(usize) -> usize| (0..HALF_SIZES[h]).map(f).collect::<Vec<usize>>();
    let halves = [
        (ph2, p2, &slice_placed, cases(HALF_SLICE, &|k| twist0 + k)),
        (ph2, p2, &dist[p2], cases(HALF_TWIST, &|k| k * SLICE_COMBS + slice)),
        (ph4, p4, &corners_solved, cases(HALF_CORNERS, &|k| k * G3_EDGE_PERMS + edges0)),
        (ph4, p4, &dist[p4], cases(HALF_EDGES, &|k| corners + k)),
    ];
    let out: Vec<Vec<u64>> = halves
        .iter()
        .map(|(ph, p, d, cs)| cs.iter().map(|&i| encode_path(ph, *p, first_path(ct, ph, d, &reps[*p][i]))).collect())
        .collect();
    for (h, algs) in out.iter().enumerate() {
        assert_eq!(algs.iter().map(|&a| alg_len(a)).max(), Some(HALF_MAX_LEN[h]), "half {} longest", h + 1);
    }
    out
}

/// Optimal canonical paths from each case, per previous face.
fn count_paths(ct: &CornerTables, ph: &PhaseDef, dist: &[u8], reps: &[Cube]) -> Vec<[u64; LAST_STATES]> {
    let mut count = vec![[0u64; LAST_STATES]; ph.n];
    let mut order: Vec<usize> = (0..ph.n).collect();
    order.sort_by_key(|&i| dist[i]);
    for &s in &order {
        if dist[s] == 0 {
            count[s] = [1; LAST_STATES];
            continue;
        }
        let mut row = [0u64; LAST_STATES];
        for &m in ph.moves {
            let c = (ph.idx)(ct, &reps[s].apply_move(m as usize));
            if dist[c] + 1 != dist[s] {
                continue;
            }
            let f = face(m);
            for (last, r) in row.iter_mut().enumerate() {
                if allowed(last as u8, f) {
                    *r += count[c][f as usize];
                }
            }
        }
        count[s] = row;
    }
    count
}

/// The optimal canonical path of lexicographic rank `r` from case `s`.
fn unrank(
    ct: &CornerTables,
    ph: &PhaseDef,
    dist: &[u8],
    count: &[[u64; LAST_STATES]],
    rep: &Cube,
    mut r: u64,
) -> Vec<u8> {
    let mut c = *rep;
    let mut last = NO_FACE;
    let mut path = Vec::new();
    loop {
        let d = dist[(ph.idx)(ct, &c)];
        if d == 0 {
            return path;
        }
        let mut taken = false;
        for &m in ph.moves {
            if !allowed(last, face(m)) {
                continue;
            }
            let n = c.apply_move(m as usize);
            let i = (ph.idx)(ct, &n);
            if dist[i] + 1 != d {
                continue;
            }
            let k = count[i][face(m) as usize];
            if r < k {
                path.push(m);
                c = n;
                last = face(m);
                taken = true;
                break;
            }
            r -= k;
        }
        assert!(taken, "rank out of range");
    }
}

/// The first optimal canonical path (greedy descent, first move first).
fn first_path(ct: &CornerTables, ph: &PhaseDef, dist: &[u8], rep: &Cube) -> Vec<u8> {
    let mut c = *rep;
    let mut last = NO_FACE;
    let mut path = Vec::new();
    let mut d = dist[(ph.idx)(ct, &c)];
    while d > 0 {
        let m = *ph
            .moves
            .iter()
            .find(|&&m| allowed(last, face(m)) && dist[(ph.idx)(ct, &c.apply_move(m as usize))] + 1 == d)
            .unwrap();
        c = c.apply_move(m as usize);
        path.push(m);
        last = face(m);
        d -= 1;
    }
    path
}

/// 0..k in van der Corput (bit-reversal) order; `k` a power of two.
fn spread_order(k: usize) -> Vec<usize> {
    let bits = k.trailing_zeros();
    (0..k).map(|i| if bits == 0 { 0 } else { i.reverse_bits() >> (usize::BITS - bits) }).collect()
}

/// `k` alternatives for each of `cases` (packed algorithms, `NO_ALG`
/// padding), built on `threads` threads.
#[allow(clippy::too_many_arguments)]
pub fn algorithms(
    ct: &CornerTables,
    ph: &PhaseDef,
    bits: usize,
    dist: &[u8],
    reps: &[Cube],
    k: usize,
    threads: usize,
    cases: &[usize],
) -> Vec<u64> {
    assert!(k.is_power_of_two());
    let count = if k > 1 { count_paths(ct, ph, dist, reps) } else { Vec::new() };
    let slot = |m: u8| ph.moves.iter().position(|&x| x == m).unwrap();
    let order = spread_order(k);
    let mut out = vec![NO_ALG; cases.len() * k];
    let chunk = cases.len().div_ceil(threads);
    std::thread::scope(|sc| {
        for (part, which) in out.chunks_mut(chunk * k).zip(cases.chunks(chunk)) {
            let (count, order) = (&count, &order);
            sc.spawn(move || {
                for (row, &s) in part.chunks_mut(k).zip(which) {
                    let paths: Vec<Vec<u8>> = if k == 1 {
                        vec![first_path(ct, ph, dist, &reps[s])]
                    } else {
                        let total = count[s][NO_FACE as usize];
                        if total <= k as u64 {
                            (0..total).map(|r| unrank(ct, ph, dist, count, &reps[s], r)).collect()
                        } else {
                            order
                                .iter()
                                .map(|&i| unrank(ct, ph, dist, count, &reps[s], i as u64 * total / k as u64))
                                .collect()
                        }
                    };
                    for (dst, p) in row.iter_mut().zip(&paths) {
                        let slots: Vec<usize> = p.iter().map(|&m| slot(m)).collect();
                        *dst = encode(&slots, bits);
                    }
                }
            });
        }
    });
    out
}

/// Symmetry reduction of a phase's minor coordinate (see [`MINOR`]).
pub struct Reduction {
    /// Per minor value: its class << SYM_BITS | the phase symmetry taking it
    /// to the class representative.
    pub minor_sym: Vec<u16>,
    /// Per phase symmetry and major value: the conjugate's major value.
    pub major_conj: Vec<u16>,
    /// Class representatives (minor values), by class.
    pub classes: Vec<usize>,
}

/// Conjugates every case by every phase symmetry, checking that the minor
/// and major parts transform independently.
pub fn reduce(ct: &CornerTables, ph: &PhaseDef, p: usize, reps: &[Cube]) -> Reduction {
    let (minor, major) = (MINOR[p], MAJOR[p]);
    let syms = PHASE_SYMS[p];
    let mut minor_map = vec![[u16::MAX; MAX_SYMS]; minor];
    let mut major_conj = vec![u16::MAX; syms.len() * major];
    for (i, r) in reps.iter().enumerate() {
        for (k, &s) in syms.iter().enumerate() {
            let j = (ph.idx)(ct, &r.conj(s as usize));
            for (slot, v) in
                [(&mut minor_map[i % minor][k], j % minor), (&mut major_conj[k * major + i / minor], j / minor)]
            {
                assert!(*slot == u16::MAX || *slot as usize == v, "coordinate parts do not conjugate independently");
                *slot = v as u16;
            }
        }
    }
    let rep_of = |m: usize| {
        minor_map[m][..syms.len()].iter().enumerate().min_by_key(|&(_, &v)| v).map(|(k, &v)| (v as usize, k)).unwrap()
    };
    let mut classes: Vec<usize> = (0..minor).map(|m| rep_of(m).0).collect();
    classes.sort_unstable();
    classes.dedup();
    let minor_sym = (0..minor)
        .map(|m| {
            let (r, k) = rep_of(m);
            ((classes.binary_search(&r).unwrap() << SYM_BITS) | k) as u16
        })
        .collect();
    Reduction { minor_sym, major_conj, classes }
}

/// Cases whose algorithms a reduced table stores: every major value with
/// each class representative, class by class.
pub fn reduced_cases(p: usize, red: &Reduction) -> Vec<usize> {
    red.classes.iter().flat_map(|&m| (0..MAJOR[p]).map(move |major| major * MINOR[p] + m)).collect()
}

/// Phase `p`'s first algorithm for each of its `n` cases, mapped back
/// through the case's symmetry as the solver does (`Tables::row_of`), and
/// composed into one permutation: lane tables, 64 bytes per case.
pub fn composed(p: usize, n: usize, algs: &[u64], red: Option<&Reduction>) -> Vec<u8> {
    let mut out = Vec::with_capacity(n * std::mem::size_of::<LaneTables>());
    for i in 0..n {
        let (row, sym) = match red {
            None => (i, 0),
            Some(red) => {
                let (minor, major) = (i % MINOR[p], i / MINOR[p]);
                let ms = red.minor_sym[minor] as usize;
                let sym = ms & (MAX_SYMS - 1);
                ((ms >> SYM_BITS) * MAJOR[p] + red.major_conj[sym * MAJOR[p] + major] as usize, sym)
            }
        };
        out.extend(composite(p, algs[row * MAX_ALTS[p]], sym).as_flattened());
    }
    out
}

/// A split half's algorithms (see [`N_HALVES`]) composed as `composed`.
pub fn composed_half(h: usize, algs: &[u64]) -> Vec<u8> {
    algs.iter().flat_map(|&a| composite(HALF_PHASE[h], a, 0).into_iter().flatten()).collect()
}

/// Phase `p`'s algorithm `a`, mapped through phase symmetry `sym`, as one
/// permutation.
fn composite(p: usize, a: u64, sym: usize) -> LaneTables {
    let mut k = Cubie::SOLVED;
    for j in 0..alg_len(a) {
        let slot = alg_slot(a, MOVE_BITS[p], j);
        k = k.mul(&MOVE_CUBIE[SYM_MOVE[PHASE_SYMS[p][sym] as usize][PHASE_MOVES[p][slot] as usize] as usize]);
    }
    lane_tables(&k)
}

// ------------------------------------------------------------- the join table

/// The join table (see [`JOIN_SIZE`]): each entry from merging two moves
/// into a solution ending in two, as petrus' `Moves::push_merged` would.
/// The solution's earlier moves are unknown; an entry whose result depends
/// on them, or after which a third move could still merge (the second
/// merged), is flagged cold.
pub fn join_table() -> Vec<u8> {
    let none = NMOVES as u8;
    let mut out = Vec::with_capacity(JOIN_SIZE * std::mem::size_of::<u32>());
    for i in 0..JOIN_SIZE {
        // the index's digits, last first (see `join_index`)
        let mut x = i;
        let mut digit = || {
            let d = (x % JOIN_VALUES) as u8;
            x /= JOIN_VALUES;
            d
        };
        let (m1, m0, a1, a2) = (digit(), digit(), digit(), digit());
        out.extend(join_entry(a2, a1, m0, m1, none).to_le_bytes());
    }
    out
}

const QUARTERS: u8 = 4;

fn join_entry(a2: u8, a1: u8, m0: u8, m1: u8, none: u8) -> u32 {
    // the known history, last move last; NO_MOVE marks its start, so a
    // history of two may have unknown moves before it
    let hist: Vec<u8> = match (a2 == none, a1 == none) {
        (_, true) => vec![],
        (true, false) => vec![a1],
        (false, false) => vec![a2, a1],
    };
    let unknown_before = hist.len() == 2;
    let mut q = hist.clone();
    let mut cold = false;
    let mut second_joined = false;
    for (j, &m) in [m0, m1].iter().enumerate() {
        if m == none {
            continue;
        }
        let n = q.len();
        let f = face(m);
        // reading past the known history needs the unknown moves
        if n == 0 && unknown_before {
            cold = true;
            break;
        }
        let same = n >= 1 && face(q[n - 1]) == f;
        let behind = n >= 1 && !same && face(q[n - 1]) == opposite(f);
        if behind && n < 2 && unknown_before {
            cold = true;
            break;
        }
        let hidden = behind && n >= 2 && face(q[n - 2]) == f;
        if same || hidden {
            let at = if same { n - 1 } else { n - 2 };
            let turns = (power(q[at]) + power(m) + 2) % QUARTERS;
            if turns != 0 {
                q[at] = mv(f, turns - 1);
            } else {
                q.remove(at);
            }
            second_joined |= j == 1;
        } else {
            q.push(m);
        }
    }
    if cold || second_joined {
        return JOIN_COLD;
    }
    let keep = hist.iter().zip(&q).take_while(|(a, b)| a == b).count();
    let replace = &q[keep..];
    assert!(replace.len() <= JOIN_MAX_REPLACE);
    let taken = (m0 != none) as u32 + (m1 != none) as u32;
    let mut e = 0u32;
    for (k, &m) in replace.iter().enumerate() {
        e |= (m as u32) << (k as u32 * u8::BITS);
    }
    e | ((hist.len() - keep) as u32) << JOIN_DROP | (replace.len() as u32) << JOIN_REPLACE | taken << JOIN_TAKEN
}
