//! Last layer.
//!
//! After step 4 the F2L is solved and every edge is oriented, which leaves
//! 4! x 3^3 x 4! / 2 = 7776 last-layer states. We store a move-optimal
//! (half-turn metric) algorithm for every one of them, produced once by
//! [`generate`] and embedded in the binary.
//!
//! Lars Petrus' steps 5-7 (position corners, twist corners, position edges)
//! and COLL+EPLL fall out of the same table: an algorithm that solves state Y
//! solves the corner permutation (or all corners) of every state X sharing
//! Y's corner permutation (or corners), so the best step algorithm for a
//! projection key is the shortest table entry with that key. All algorithms
//! keep the F2L and edge orientation intact, as Petrus' steps require.

use crate::cube::*;
use crate::tables::PieceTable;

pub const LL_PIECES: usize = 4;
const PERMS: usize = 24; // 4!
const TWISTS: usize = 27; // 3^3; the fourth twist is implied
/// Size of the (sparse: half are parity-impossible) last-layer index.
pub const LL_INDEX_SIZE: usize = PERMS * TWISTS * PERMS;
/// Reachable last-layer states.
pub const LL_STATES: usize = LL_INDEX_SIZE / 2;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LlMode {
    /// Petrus steps 5, 6, 7: corner permutation, corner twist, edge permutation.
    Lars,
    /// Corners in one step, then edges.
    Coll,
    /// The whole last layer in one optimal algorithm.
    Zbll,
}

impl LlMode {
    pub fn parse(s: &str) -> Option<LlMode> {
        match s {
            "lars" => Some(LlMode::Lars),
            "coll" => Some(LlMode::Coll),
            "zbll" => Some(LlMode::Zbll),
            _ => None,
        }
    }
}

fn perm_rank(p: [u8; LL_PIECES]) -> usize {
    let mut r = 0;
    for i in 0..LL_PIECES {
        let smaller_after = p[i + 1..].iter().filter(|&&x| x < p[i]).count();
        r = r * (LL_PIECES - i) + smaller_after;
    }
    r
}

/// Index of a state whose F2L and edge orientation are solved, else None.
pub fn ll_index(c: &Cube) -> Option<usize> {
    let mut cpos = [0u8; LL_PIECES];
    let mut epos = [0u8; LL_PIECES];
    let mut twist = 0;
    for i in (0..LL_PIECES).rev() {
        let v = c.c[i] as usize;
        let (pos, ori) = (v % N_CORNERS, v / N_CORNERS);
        if pos >= LL_PIECES || c.e[i] as usize >= LL_PIECES {
            return None;
        }
        cpos[i] = pos as u8;
        epos[i] = c.e[i];
        if i < LL_PIECES - 1 {
            twist = twist * CORNER_TWISTS + ori;
        }
    }
    Some((perm_rank(cpos) * TWISTS + twist) * PERMS + perm_rank(epos))
}

fn perm_unrank(mut r: usize) -> [u8; LL_PIECES] {
    let mut digits = [0usize; LL_PIECES];
    for i in (0..LL_PIECES).rev() {
        let base = LL_PIECES - i;
        digits[i] = r % base;
        r /= base;
    }
    let mut avail: Vec<u8> = (0..LL_PIECES as u8).collect();
    digits.map(|d| avail.remove(d))
}

/// Inverse of [`ll_index`]: the cube with that last layer, or None for
/// parity-impossible indices.
pub fn ll_state(idx: usize) -> Option<Cube> {
    let epos = perm_unrank(idx % PERMS);
    let mut twist = idx / PERMS % TWISTS;
    let cpos = perm_unrank(idx / (PERMS * TWISTS));
    let mut c = Cube::SOLVED;
    let mut sum = 0;
    for i in 0..LL_PIECES {
        let ori = if i < LL_PIECES - 1 {
            let o = twist % CORNER_TWISTS;
            twist /= CORNER_TWISTS;
            sum += o;
            o
        } else {
            (CORNER_TWISTS - sum % CORNER_TWISTS) % CORNER_TWISTS
        };
        c.c[i] = (cpos[i] as usize + N_CORNERS * ori) as u8;
        c.e[i] = epos[i];
    }
    c.is_valid().then_some(c)
}

/// Projection keys used by the multi-step modes.
fn corner_perm_key(idx: usize) -> usize {
    idx / (TWISTS * PERMS)
}
fn corners_key(idx: usize) -> usize {
    idx / PERMS
}

pub struct LastLayer {
    /// Optimal algorithm solving each last-layer state.
    algs: Vec<Option<Vec<u8>>>,
    /// Shortest table entry per corner-permutation key / per corners key.
    best_cp: Vec<usize>,
    best_corners: Vec<usize>,
}

const MAGIC: &[u8; 4] = b"PLL1";
const NONE: u8 = u8::MAX;
static EMBEDDED: &[u8] = include_bytes!("../data/ll.bin");

impl LastLayer {
    pub fn load_embedded() -> Result<LastLayer, String> {
        LastLayer::from_bytes(EMBEDDED)
    }

    pub fn from_bytes(b: &[u8]) -> Result<LastLayer, String> {
        if b.len() < MAGIC.len() || &b[..MAGIC.len()] != MAGIC {
            return Err("missing last-layer table; run `petrus gen-ll data/ll.bin`".into());
        }
        let mut algs = Vec::with_capacity(LL_INDEX_SIZE);
        let mut i = MAGIC.len();
        while algs.len() < LL_INDEX_SIZE {
            let len = *b.get(i).ok_or("truncated table")?;
            i += 1;
            if len == NONE {
                algs.push(None);
            } else {
                let moves = b.get(i..i + len as usize).ok_or("truncated table")?;
                if moves.iter().any(|&m| m as usize >= NMOVES) {
                    return Err("bad move in table".into());
                }
                algs.push(Some(moves.to_vec()));
                i += len as usize;
            }
        }
        Ok(LastLayer::from_algs(algs))
    }

    pub fn from_algs(algs: Vec<Option<Vec<u8>>>) -> LastLayer {
        let best = |key: fn(usize) -> usize, n: usize| {
            let mut best = vec![usize::MAX; n];
            for (idx, a) in algs.iter().enumerate() {
                if let Some(a) = a {
                    let k = key(idx);
                    if best[k] == usize::MAX || a.len() < algs[best[k]].as_ref().unwrap().len() {
                        best[k] = idx;
                    }
                }
            }
            best
        };
        let best_cp = best(corner_perm_key, PERMS);
        let best_corners = best(corners_key, PERMS * TWISTS);
        LastLayer { algs, best_cp, best_corners }
    }

    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = MAGIC.to_vec();
        for a in &self.algs {
            match a {
                None => out.push(NONE),
                Some(a) => {
                    out.push(a.len() as u8);
                    out.extend_from_slice(a);
                }
            }
        }
        out
    }

    pub fn alg(&self, idx: usize) -> Option<&[u8]> {
        self.algs[idx].as_deref()
    }

    /// Step algorithms finishing a cube whose F2L and EO are solved.
    pub fn solve(&self, c: &Cube, mode: LlMode) -> Vec<(&'static str, &[u8])> {
        let mut out = Vec::new();
        let mut state = *c;
        let mut step = |name: &'static str, pick: &dyn Fn(usize) -> usize, state: &mut Cube| {
            let idx = ll_index(state).expect("last layer reached with F2L/EO unsolved");
            let alg = self.algs[pick(idx)].as_deref().expect("missing last-layer case");
            *state = state.apply_seq(alg);
            out.push((name, alg));
        };
        let cp = |i| self.best_cp[corner_perm_key(i)];
        let corners = |i| self.best_corners[corners_key(i)];
        let full = |i| i;
        match mode {
            LlMode::Lars => {
                step("CP", &cp, &mut state);
                step("CO", &corners, &mut state);
                step("EP", &full, &mut state);
            }
            LlMode::Coll => {
                step("COLL", &corners, &mut state);
                step("EPLL", &full, &mut state);
            }
            LlMode::Zbll => step("ZBLL", &full, &mut state),
        }
        out
    }
}

// ---------------------------------------------------------------- generator

/// Every F2L piece plus EO: the subgroup an LL algorithm must land in.
const F2L_CORNERS: [usize; N_CORNERS - LL_PIECES] = [DFR, DLF, DBL, DRB];
const F2L_EDGES: [usize; N_EDGES - LL_PIECES] = [DR, DF, DL, DB, FR, FL, BL, BR];

fn in_ll_group(c: &Cube) -> bool {
    F2L_CORNERS.iter().all(|&i| c.c[i] as usize == i)
        && F2L_EDGES.iter().all(|&i| c.e[i] as usize == i)
        && c.eo12() == 0
}

struct GenTables {
    blocks: Vec<crate::defs::Table223>,
    eo: PieceTable<0, 0, true>,
}

impl GenTables {
    #[inline(always)]
    fn h(&self, c: &Cube) -> u8 {
        self.blocks.iter().map(|t| t.get(c)).fold(self.eo.get(c), u8::max)
    }
}

/// Enumerate "core" algorithms (first and last move not a U turn) of exactly
/// `depth` moves that preserve F2L and EO, recording the first hit per element.
fn core_dfs(
    g: &GenTables,
    c: &Cube,
    togo: u8,
    last: u8,
    path: &mut Vec<u8>,
    hits: &mut [Option<Vec<u8>>],
) {
    for m in 0..NMOVES as u8 {
        let f = face(m);
        if f == last || opposite(f) == last && f < last || togo == 1 && f == FACE_U {
            continue;
        }
        let n = c.apply_move(m as usize);
        if g.h(&n) >= togo {
            continue;
        }
        path.push(m);
        if togo == 1 {
            if in_ll_group(&n) {
                // n is the element the core produces from solved
                let idx = ll_index(&n).unwrap();
                if hits[idx].is_none() {
                    hits[idx] = Some(path.clone());
                }
            }
        } else {
            core_dfs(g, &n, togo - 1, f, path, hits);
        }
        path.pop();
    }
}

/// Build optimal algorithms for all 7776 last-layer states. Algorithms are
/// AUF + core + AUF; cores are enumerated by increasing length, so an entry
/// of length L is certified optimal once every core of length L - 1 has
/// been seen.
pub fn generate(threads: usize, log: &mut dyn FnMut(String)) -> LastLayer {
    use crate::defs::{ALL_MOVES, BLOCK_223_D, BLOCK_223_L};
    use crate::defs::PieceSet;
    const RIGHT_223_D: PieceSet<2, 3> = ([DFR, DRB], [DR, DF, DB]);
    const RIGHT_223_R: PieceSet<2, 3> = ([DFR, DRB], [DR, FR, BR]);
    let g = GenTables {
        blocks: [BLOCK_223_D, BLOCK_223_L, RIGHT_223_D, RIGHT_223_R]
            .iter()
            .map(|(c, e)| PieceTable::build(*c, *e, &ALL_MOVES))
            .collect(),
        eo: PieceTable::build([], [], &ALL_MOVES),
    };
    let u_turns: [Vec<u8>; LL_PIECES] =
        [vec![], vec![mv(FACE_U, 0)], vec![mv(FACE_U, 1)], vec![mv(FACE_U, 2)]];
    // best[idx] = shortest algorithm known that *solves* state idx
    let mut best: Vec<Option<Vec<u8>>> = vec![None; LL_INDEX_SIZE];
    let mut core_seen = vec![false; LL_INDEX_SIZE];
    let first_moves: Vec<u8> = (0..NMOVES as u8).filter(|&m| face(m) != FACE_U).collect();
    let mut depth: u8 = 0;
    loop {
        let cores: Vec<Vec<u8>> = if depth == 0 {
            vec![Vec::new()]
        } else {
            // parallel over first moves
            let next = std::sync::atomic::AtomicUsize::new(0);
            let results = std::sync::Mutex::new(vec![None; LL_INDEX_SIZE]);
            std::thread::scope(|s| {
                for _ in 0..threads.max(1) {
                    s.spawn(|| {
                        let mut hits = vec![None; LL_INDEX_SIZE];
                        loop {
                            let i = next.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                            let Some(&m) = first_moves.get(i) else { break };
                            let c = Cube::SOLVED.apply_move(m as usize);
                            let mut path = vec![m];
                            if depth == 1 {
                                if in_ll_group(&c) {
                                    hits[ll_index(&c).unwrap()] = Some(path.clone());
                                }
                            } else if g.h(&c) < depth {
                                core_dfs(&g, &c, depth - 1, face(m), &mut path, &mut hits);
                            }
                        }
                        let mut r = results.lock().unwrap();
                        for (i, h) in hits.into_iter().enumerate() {
                            if r[i].is_none() {
                                r[i] = h;
                            }
                        }
                    });
                }
            });
            results.into_inner().unwrap().into_iter().flatten().collect()
        };
        let mut new_cores = 0;
        for core in cores {
            let e = ll_index(&Cube::from_seq(&core)).unwrap();
            if std::mem::replace(&mut core_seen[e], true) {
                continue;
            }
            new_cores += 1;
            for pre in &u_turns {
                for post in &u_turns {
                    let mut alg = pre.clone();
                    alg.extend_from_slice(&core);
                    alg.extend_from_slice(post);
                    // the algorithm solves the inverse of the element it makes
                    let solves = ll_index(&Cube::from_seq(&seq_inverse(&alg))).unwrap();
                    if best[solves].as_ref().is_none_or(|b| alg.len() < b.len()) {
                        best[solves] = Some(alg);
                    }
                }
            }
        }
        let known = best.iter().flatten().count();
        let certified = best.iter().flatten().filter(|a| a.len() <= depth as usize + 1).count();
        log(format!(
            "core depth {depth:2}: {new_cores:5} new cores, {known:5} states reached, {certified:5} certified optimal"
        ));
        if certified == LL_STATES {
            break;
        }
        depth += 1;
    }
    LastLayer::from_algs(best)
}
