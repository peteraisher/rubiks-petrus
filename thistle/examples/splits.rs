//! Five- and six-phase chains: Thistlethwaite with one or two phases split
//! at an intermediate set, measured exactly (BFS distances, each phase
//! optimal) on random cubes. For each split it prints the case count of
//! both halves and their mean optimal lengths; then whole chains.
//!   cargo run --release --example splits -- [N]
//!
//! A split of phase p (start group A, goal group B, coordinate space X =
//! B-cosets of A) at a set S of points of X with B's point in S:
//! - first half: reach S with A's moves (multi-source BFS distance to S);
//! - second half: from S to B's point, either with a subgroup's moves when S
//!   is that subgroup's orbit (closed under them), or with A's moves
//!   restoring whatever the path disturbs (A's own distance, restricted to S).
//!
//! The first half's cases are S's cosets: |X| / |S| when S is a subgroup
//! orbit; for a restore split the first half's goal is S itself, so its case
//! count is the size of the coordinate it solves.
use petrus::cube::*;
use thistle::coord::{self, *};

#[allow(dead_code)]
#[path = "../src/gen.rs"]
mod gen;

const DEFAULT_N: usize = 20_000;
const SEED: u64 = 2024;
const UNSEEN: u8 = u8::MAX;

/// A phase of the measured chain: a distance table over a phase's
/// coordinate space and the moves its descent may use.
struct Step {
    name: String,
    phase: usize,
    dist: Vec<u8>,
    moves: Vec<u8>,
    cases: usize,
}

/// Distance to the set `goal` over phase `p`'s space with `moves`, from the
/// representatives `reps` (multi-source BFS).
fn bfs_to(ct: &CornerTables, p: usize, reps: &[Cube], goal: &[bool], moves: &[u8]) -> Vec<u8> {
    let idx = gen::PHASES[p].idx;
    let mut dist = vec![UNSEEN; reps.len()];
    let mut frontier: Vec<usize> = (0..reps.len()).filter(|&i| goal[i]).collect();
    for &i in &frontier {
        dist[i] = 0;
    }
    // moves are their own inverses up to power, and every move set used is
    // closed under inverses, so distance from the set equals distance to it
    let mut d = 0;
    while !frontier.is_empty() {
        let mut next = Vec::new();
        for &s in &frontier {
            for &m in moves {
                let i = idx(ct, &reps[s].apply_move(m as usize));
                if dist[i] == UNSEEN {
                    dist[i] = d + 1;
                    next.push(i);
                }
            }
        }
        d += 1;
        frontier = next;
    }
    dist
}

/// Points reachable from the goal point with `moves`: the orbit of the
/// subgroup they generate.
fn orbit(ct: &CornerTables, p: usize, reps: &[Cube], moves: &[u8]) -> Vec<bool> {
    let idx = gen::PHASES[p].idx;
    let mut goal = vec![false; reps.len()];
    goal[idx(ct, &Cube::SOLVED)] = true;
    let d = bfs_to(ct, p, reps, &goal, moves);
    d.iter().map(|&x| x != UNSEEN).collect()
}

fn faces(moves: &[u8]) -> String {
    const NAMES: [&str; N_FACES] = ["U", "R", "F", "D", "L", "B"];
    let mut out = Vec::new();
    for f in 0..N_FACES as u8 {
        let p: Vec<u8> = moves.iter().filter(|&&m| face(m) == f).map(|&m| power(m)).collect();
        match p.len() {
            0 => {}
            1 => out.push(format!("{}2", NAMES[f as usize])),
            _ => out.push(NAMES[f as usize].to_string()),
        }
    }
    format!("<{}>", out.join(","))
}

/// Walk one phase optimally: any move that lowers the distance.
fn descend(ct: &CornerTables, s: &Step, c: &mut Cube) -> usize {
    let idx = gen::PHASES[s.phase].idx;
    let mut d = s.dist[idx(ct, c)];
    assert!(d != UNSEEN, "{}: start outside its space", s.name);
    let n = d as usize;
    while d > 0 {
        let m = *s.moves.iter().find(|&&m| s.dist[idx(ct, &c.apply_move(m as usize))] == d - 1).unwrap();
        *c = c.apply_move(m as usize);
        d -= 1;
    }
    n
}

fn main() {
    let n: usize = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(DEFAULT_N);
    let (class, g3) = gen::corner_tables();
    let ct = CornerTables { class: class.leak(), g3: g3.leak() };
    let spaces: Vec<(Vec<u8>, Vec<Cube>)> = gen::PHASES.iter().map(|ph| gen::bfs(&ct, ph)).collect();
    let whole = |p: usize| Step {
        name: format!("phase {}", p + 1),
        phase: p,
        dist: spaces[p].0.clone(),
        moves: gen::PHASES[p].moves.to_vec(),
        cases: gen::PHASES[p].n,
    };
    let sub = |moves: &[u8], drop: &[u8]| moves.iter().copied().filter(|m| !drop.contains(m)).collect::<Vec<u8>>();
    let quarters = |f: u8| [mv(f, 0), mv(f, 2)];

    // ---- candidate splits: (label, phase, first half, second half)
    let mut splits: Vec<(String, Step, Step)> = Vec::new();
    // subgroup splits: drop some generators of the phase's start group
    let sub_cands: Vec<(usize, Vec<u8>)> = vec![
        (0, sub(&MOVES1, &quarters(FACE_B))),
        (1, sub(&MOVES2, &quarters(FACE_L))),
        (2, sub(&MOVES3, &quarters(FACE_D))),
        (3, sub(&MOVES4, &[mv(FACE_B, 1)])),
        (3, sub(&MOVES4, &[mv(FACE_B, 1), mv(FACE_F, 1)])),
        (3, sub(&MOVES4, &[mv(FACE_B, 1), mv(FACE_L, 1)])),
        (3, sub(&MOVES4, &[mv(FACE_B, 1), mv(FACE_L, 1), mv(FACE_D, 1)])),
    ];
    for (p, hm) in sub_cands {
        let (dist, reps) = &spaces[p];
        let o = orbit(&ct, p, reps, &hm);
        let size = o.iter().filter(|&&x| x).count();
        let to = bfs_to(&ct, p, reps, &o, gen::PHASES[p].moves);
        let mut within = bfs_to(
            &ct,
            p,
            reps,
            &{
                let mut g = vec![false; reps.len()];
                g[(gen::PHASES[p].idx)(&ct, &Cube::SOLVED)] = true;
                g
            },
            &hm,
        );
        for (w, &inside) in within.iter_mut().zip(&o) {
            if !inside {
                *w = UNSEEN;
            }
        }
        let label = format!("phase {} via {} (orbit {size})", p + 1, faces(&hm));
        let _ = dist;
        splits.push((
            label.clone(),
            Step {
                name: format!("{label}: to"),
                phase: p,
                dist: to,
                moves: gen::PHASES[p].moves.to_vec(),
                cases: reps.len() / size,
            },
            Step { name: format!("{label}: within"), phase: p, dist: within, moves: hm, cases: size },
        ));
    }
    // restore splits: fix one coordinate, then the rest with the phase's moves
    let restore = |p: usize, label: &str, set: Vec<bool>, first_cases: usize| {
        let (dist, reps) = &spaces[p];
        let to = bfs_to(&ct, p, reps, &set, gen::PHASES[p].moves);
        // the rest may leave the set on its way: the phase's own distance
        let rest = dist.clone();
        let size = set.iter().filter(|&&x| x).count();
        let label = format!("phase {} {label}", p + 1);
        (
            label.clone(),
            Step {
                name: format!("{label}: first"),
                phase: p,
                dist: to,
                moves: gen::PHASES[p].moves.to_vec(),
                cases: first_cases,
            },
            Step {
                name: format!("{label}: rest"),
                phase: p,
                dist: rest,
                moves: gen::PHASES[p].moves.to_vec(),
                cases: size,
            },
        )
    };
    // each restore split fixes one part of its phase's goal coordinate
    // (`major * minor_size + minor`)
    let parts = |p: usize, minor_size: usize| {
        let g = (gen::PHASES[p].idx)(&ct, &Cube::SOLVED);
        let n = gen::PHASES[p].n;
        let minor: Vec<bool> = (0..n).map(|i| i % minor_size == g % minor_size).collect();
        let major: Vec<bool> = (0..n).map(|i| i / minor_size == g / minor_size).collect();
        (minor, major)
    };
    let (slice, twist) = parts(1, SLICE_COMBS);
    splits.push(restore(1, "E-slice, then twist", slice, SLICE_COMBS));
    splits.push(restore(1, "twist, then E-slice", twist, TWISTS));
    let (edges, corners) = parts(3, G3_EDGE_PERMS);
    splits.push(restore(3, "corners, then edges", corners, G3_CORNER_PERMS));
    splits.push(restore(3, "edges, then corners", edges, G3_EDGE_PERMS));
    let (m_slice, coset) = parts(2, M_COMBS);
    splits.push(restore(2, "M-slice, then corners", m_slice, M_COMBS));
    splits.push(restore(2, "corners, then M-slice", coset, CORNER_COSETS));

    // ---- random cubes, each phase walked optimally
    let mut rng = Rng::new(SEED);
    let cubes: Vec<Cube> = (0..n).map(|_| Cube::random(&mut rng)).collect();
    let run = |chain: &[&Step]| -> Vec<f64> {
        let mut sums = vec![0usize; chain.len()];
        for c in &cubes {
            let mut x = *c;
            for (k, s) in chain.iter().enumerate() {
                sums[k] += descend(&ct, s, &mut x);
            }
            assert!(x.is_solved());
        }
        sums.iter().map(|&s| s as f64 / n as f64).collect()
    };
    let wholes: Vec<Step> = (0..N_PHASES).map(whole).collect();
    let base = run(&wholes.iter().collect::<Vec<_>>());
    println!(
        "Thistlethwaite: {:?} = {:.2} (unmerged), cases {:?}",
        base.iter().map(|x| format!("{x:.2}")).collect::<Vec<_>>(),
        base.iter().sum::<f64>(),
        PHASE_SIZES
    );
    let _ = coord::LAST;
    println!(
        "{:58} {:>9} {:>9} {:>6} {:>6} {:>7} {:>7}",
        "split", "cases 1", "cases 2", "len 1", "len 2", "phase", "+moves"
    );
    let mut results = Vec::new();
    for (label, a, b) in &splits {
        let p = a.phase;
        let mut chain: Vec<&Step> = Vec::new();
        for (q, w) in wholes.iter().enumerate() {
            if q == p {
                chain.push(a);
                chain.push(b);
            } else {
                chain.push(w);
            }
        }
        let r = run(&chain);
        let (la, lb) = (r[p], r[p + 1]);
        let extra = la + lb - base[p];
        println!("{label:58} {:>9} {:>9} {la:6.2} {lb:6.2} {:7.2} {extra:+7.2}", a.cases, b.cases, base[p]);
        results.push((label.clone(), p, extra));
    }
    // ---- six phases: the best split of two different phases together
    let mut best: Vec<(usize, f64, usize)> = Vec::new();
    for (k, &(_, p, extra)) in results.iter().enumerate() {
        if splits[k].1.cases == 1 {
            continue; // the subgroup is the whole start group: nothing shrinks
        }
        if best.iter().all(|&(_, _, q)| q != p) || best.iter().any(|&(_, e, q)| q == p && extra < e) {
            best.retain(|&(_, _, q)| q != p);
            best.push((k, extra, p));
        }
    }
    best.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
    for i in 0..best.len() {
        for j in i + 1..best.len() {
            let (ka, kb) = (best[i].0, best[j].0);
            let (pa, pb) = (results[ka].1, results[kb].1);
            let mut chain: Vec<&Step> = Vec::new();
            for (q, w) in wholes.iter().enumerate() {
                if q == pa {
                    chain.push(&splits[ka].1);
                    chain.push(&splits[ka].2);
                } else if q == pb {
                    chain.push(&splits[kb].1);
                    chain.push(&splits[kb].2);
                } else {
                    chain.push(w);
                }
            }
            let r = run(&chain);
            println!(
                "six phases: {} + {}: {:.2} ({:+.2})",
                splits[ka].0,
                splits[kb].0,
                r.iter().sum::<f64>(),
                r.iter().sum::<f64>() - base.iter().sum::<f64>()
            );
        }
    }
}
