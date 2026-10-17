use petrus::cube::*;
use std::sync::OnceLock;
use thistle::coord::{self, *};
use thistle::solver::*;

// the generator, compiled again here to check the embedded tables
#[allow(dead_code)]
#[path = "../src/gen.rs"]
mod gen;

fn tables() -> &'static Tables {
    static T: OnceLock<Tables> = OnceLock::new();
    T.get_or_init(Tables::new)
}

/// Corner tables from a fresh generation.
fn corner_tables() -> &'static (Vec<u8>, Vec<u8>) {
    static CT: OnceLock<(Vec<u8>, Vec<u8>)> = OnceLock::new();
    CT.get_or_init(gen::corner_tables)
}

const PHASE_MOVES: [&[u8]; N_PHASES] = [&MOVES1, &MOVES2, &MOVES3, &MOVES4];
/// God's number of each phase (half-turn metric, the phase's own moves).
const PHASE_DIAMETERS: [u8; N_PHASES] = [7, 10, 13, 15];
const _: () = assert!(PHASE_DIAMETERS[LAST] as usize == PHASE_MAX_LEN[LAST]);
const SAMPLES: usize = 300;
const WALK: usize = 60;

/// A random element of the group phase `p` starts in.
fn random_in(p: usize, rng: &mut Rng) -> Cube {
    if p == 0 {
        return Cube::random(rng);
    }
    let gens = PHASE_MOVES[p];
    (0..WALK).fold(Cube::SOLVED, |c, _| c.apply_move(gens[rng.below(gens.len())] as usize))
}

/// Membership of the group phase `p` ends in, checked from the pieces.
fn in_goal(p: usize, c: &Cube) -> bool {
    let stay = |pieces: [usize; 4]| pieces.iter().all(|&e| pieces.contains(&(c.e[e] as usize)));
    match p {
        0 => c.eo12() == 0,
        1 => in_goal(0, c) && twist(c) == 0 && stay([FR, FL, BL, BR]),
        2 => {
            let (placement, orders) = corner_patterns(c);
            let home = TETRAD_A.iter().map(|&p| 1 << p).sum::<usize>();
            in_goal(1, c) && placement == home && corner_tables().1[orders] != u8::MAX && stay([UF, UB, DF, DB])
        }
        _ => c.is_solved(),
    }
}

#[test]
fn embedded_tables_match_fresh_generation() {
    let t = tables();
    let (class, g3) = corner_tables();
    let ct = CornerTables { class, g3 };
    let threads = std::thread::available_parallelism().map_or(1, |n| n.get());
    let (mut dists, mut all_reps) = (Vec::new(), Vec::new());
    for (p, ph) in gen::PHASES.iter().enumerate() {
        let (dist, reps) = gen::bfs(&ct, ph);
        assert_eq!(*dist.iter().max().unwrap(), PHASE_DIAMETERS[p], "phase {} diameter", p + 1);
        let (e_dist, e_algs, e_minor, e_major) = t.raw(p);
        assert!(e_dist == dist.as_slice(), "phase {} distances", p + 1);
        let cases: Vec<usize> = if p == 0 {
            (0..ph.n).collect()
        } else {
            let red = gen::reduce(&ct, ph, p, &reps);
            assert!(e_minor == red.minor_sym.as_slice() && e_major == red.major_conj.as_slice());
            gen::reduced_cases(p, &red)
        };
        let algs = gen::algorithms(&ct, ph, MOVE_BITS[p], &dist, &reps, MAX_ALTS[p], threads, &cases);
        assert!(e_algs == algs.as_slice(), "phase {} algorithms", p + 1);
        dists.push(dist);
        all_reps.push(reps);
    }
    for (h, algs) in gen::split_algorithms(&ct, &dists, &all_reps).iter().enumerate() {
        assert!(t.raw_half(h) == algs.as_slice(), "split half {} algorithms", h + 1);
    }
}

/// Where split half `h` ends (see `N_HALVES`), checked from the pieces.
fn in_half_goal(h: usize, c: &Cube) -> bool {
    let stay = |pieces: [usize; 4]| pieces.iter().all(|&e| pieces.contains(&(c.e[e] as usize)));
    match h {
        0 => in_goal(0, c) && stay([FR, FL, BL, BR]),
        1 => in_goal(1, c),
        2 => (0..N_CORNERS).all(|k| c.c[k] as usize == k) && in_goal(2, c),
        _ => c.is_solved(),
    }
}

#[test]
fn split_halves_reach_their_goals_and_split_presets_are_their_chains() {
    let t = tables();
    let mut rng = Rng::new(17);
    for _ in 0..SAMPLES {
        let c = Cube::random(&mut rng);
        for (o, split) in [(Options::FAST5, [true, false]), (Options::FAST6, [true, true])] {
            let mut x = c;
            let mut chain = Vec::new();
            for p in 0..N_PHASES {
                let halves = match p {
                    1 if split[0] => vec![0, 1],
                    LAST if split[1] => vec![2, 3],
                    _ => vec![],
                };
                if halves.is_empty() {
                    let a = t.phase_algs(p, &x).swap_remove(0);
                    x = x.apply_seq(&a);
                    seq_append(&mut chain, &a);
                }
                for h in halves {
                    let a = t.half_alg(h, &x);
                    assert!(
                        a.iter().all(|m| PHASE_MOVES[HALF_PHASE[h]].contains(m)),
                        "half {} leaves its group",
                        h + 1
                    );
                    x = x.apply_seq(&a);
                    assert!(in_half_goal(h, &x), "half {} misses its goal", h + 1);
                    seq_append(&mut chain, &a);
                }
            }
            assert!(x.is_solved());
            assert_eq!(solve(t, &c, &o).moves.as_slice(), chain.as_slice());
        }
    }
}

#[test]
fn stored_algorithms_are_optimal_distinct_and_reach_the_next_group() {
    let t = tables();
    let mut rng = Rng::new(5);
    for p in 0..N_PHASES {
        for _ in 0..SAMPLES {
            let c = random_in(p, &mut rng);
            let d = t.phase_dist(p, &c);
            let algs = t.phase_algs(p, &c);
            assert!(!algs.is_empty() && algs.len() <= MAX_ALTS[p]);
            for (k, a) in algs.iter().enumerate() {
                assert_eq!(a.len(), d as usize, "phase {} alternative {k} not optimal", p + 1);
                assert!(a.iter().all(|m| PHASE_MOVES[p].contains(m)), "phase {} leaves its group", p + 1);
                assert!(in_goal(p, &c.apply_seq(a)), "phase {} alternative {k} misses the goal", p + 1);
                assert!(!algs[..k].contains(a), "phase {} alternatives repeat", p + 1);
            }
        }
    }
}

#[test]
fn every_preset_solves_and_fast_is_the_greedy_chain() {
    let t = tables();
    let mut rng = Rng::new(11);
    for _ in 0..SAMPLES {
        let c = Cube::random(&mut rng);
        // fast: the first stored algorithm of each phase, merged
        let mut x = c;
        let mut chain = Vec::new();
        for p in 0..N_PHASES {
            let a = t.phase_algs(p, &x).swap_remove(0);
            x = x.apply_seq(&a);
            seq_append(&mut chain, &a);
        }
        assert!(x.is_solved());
        let fast = solve(t, &c, &Options::FAST);
        assert_eq!(fast.moves.as_slice(), chain.as_slice());
        for name in ["fast", "fast5", "fast6", "short", "shorter", "shortest"] {
            let s = solve(t, &c, &Options::preset(name).unwrap());
            assert!(c.apply_seq(s.moves.as_slice()).is_solved(), "{name} failed");
        }
    }
}

/// The single chains merge turns across phase boundaries with their own
/// fast code: on many states, the same moves as petrus' general merge.
#[test]
fn chains_merge_like_the_reference_on_many_states() {
    const STATES: usize = 100_000;
    let t = tables();
    let mut rng = Rng::new(19);
    for _ in 0..STATES {
        let c = Cube::random(&mut rng);
        for (o, halves) in [(Options::FAST, [false; 2]), (Options::FAST5, [true, false]), (Options::FAST6, [true; 2])] {
            let mut x = c;
            let mut chain = Vec::new();
            for p in 0..N_PHASES {
                let split = match p {
                    1 => halves[0],
                    LAST => halves[1],
                    _ => false,
                };
                let algs = if !split {
                    vec![t.phase_algs(p, &x).swap_remove(0)]
                } else {
                    let h = if p == LAST { HALF_CORNERS } else { HALF_SLICE };
                    let a = t.half_alg(h, &x);
                    let b = t.half_alg(h + 1, &x.apply_seq(&a));
                    vec![a, b]
                };
                for a in algs {
                    x = x.apply_seq(&a);
                    seq_append(&mut chain, &a);
                }
            }
            assert_eq!(solve(t, &c, &o).moves.as_slice(), chain.as_slice());
        }
    }
}

/// Batches solve each cube as `solve` does, merging one by one or in lanes.
#[test]
fn batches_solve_like_single_solves() {
    const STATES: usize = 48_000;
    let t = tables();
    let mut rng = Rng::new(29);
    let cubes: Vec<Cube> = (0..STATES).map(|_| Cube::random(&mut rng)).collect();
    for o in [Options::FAST, Options::FAST5, Options::FAST6] {
        let want: Vec<Seq> = cubes.iter().map(|c| solve(t, c, &o).moves).collect();
        check_batches::<1>(t, &cubes, &o, &want);
        check_batches::<3>(t, &cubes, &o, &want);
        check_batches::<8>(t, &cubes, &o, &want);
        check_batches::<16>(t, &cubes, &o, &want);
    }
    let short: Vec<Seq> = cubes[..SAMPLES].iter().map(|c| solve(t, c, &Options::SHORT).moves).collect();
    check_batches::<4>(t, &cubes[..SAMPLES], &Options::SHORT, &short);
}

fn check_batches<const K: usize>(t: &Tables, cubes: &[Cube], o: &Options, want: &[Seq]) {
    for (c, w) in cubes.as_chunks::<K>().0.iter().zip(want.as_chunks::<K>().0) {
        let one_by_one = solve_batch_merging::<K, false>(t, c, o);
        let lanes = solve_batch_merging::<K, true>(t, c, o);
        for i in 0..K {
            assert_eq!(one_by_one[i].moves.as_slice(), w[i].as_slice(), "{K} one by one, {i}");
            assert_eq!(lanes[i].moves.as_slice(), w[i].as_slice(), "{K} in lanes, {i}");
        }
    }
}

#[test]
fn chain_phases_one_at_a_time_are_fast() {
    let t = tables();
    let mut rng = Rng::new(23);
    for _ in 0..SAMPLES {
        let c = Cube::random(&mut rng);
        let mut x = c;
        let mut chain = Vec::new();
        for p in 0..N_PHASES {
            let (moves, after) = chain_phase(t, p, &x);
            assert_eq!(moves.as_slice(), t.phase_algs(p, &x)[0].as_slice());
            assert!(after == x.apply_seq(moves.as_slice()));
            seq_append(&mut chain, moves.as_slice());
            x = after;
        }
        assert!(x.is_solved());
        assert_eq!(solve(t, &c, &Options::FAST).moves.as_slice(), chain.as_slice());
    }
}

#[test]
fn every_start_and_option_combination_solves() {
    let t = tables();
    let mut rng = Rng::new(13);
    for starts in 1..=MAX_STARTS {
        for (width, alts) in [(1, [1, 1, 1]), (2, [2, 1, 3]), (5, [16, 1, 16]), (MAX_WIDTH, [16, 1, 16])] {
            let o = Options { starts, width, alts, ..Options::FAST };
            for _ in 0..SAMPLES / MAX_STARTS {
                let c = Cube::random(&mut rng);
                let s = solve(t, &c, &o);
                assert!(c.apply_seq(s.moves.as_slice()).is_solved());
            }
        }
    }
}

#[test]
fn starts_are_the_six_axis_orders_each_with_its_inverse() {
    use petrus::facelet::SYM_MOVE;
    const N_AXES: u8 = 3;
    let axis = |s: u8, m: u8| face(SYM_MOVE[s as usize][m as usize]) % N_AXES;
    let mut orders: Vec<(u8, u8)> =
        STARTS.iter().step_by(2).map(|&(s, _)| (axis(s, mv(FACE_U, 0)), axis(s, mv(FACE_F, 0)))).collect();
    orders.sort_unstable();
    orders.dedup();
    assert_eq!(orders.len(), MAX_STARTS / 2);
    assert!(STARTS.chunks(2).all(|p| p[0].0 == p[1].0 && !p[0].1 && p[1].1));
}

#[test]
fn special_states() {
    let t = tables();
    let superflip = Cube::from_cubie(&Cubie { eo: [1; N_EDGES], ..Cubie::SOLVED });
    for name in ["fast", "short", "shorter", "shortest"] {
        let o = Options::preset(name).unwrap();
        assert!(solve(t, &Cube::SOLVED, &o).moves.is_empty());
        for m in 0..NMOVES as u8 {
            let s = solve(t, &Cube::from_seq(&[m]), &o);
            assert_eq!(s.moves.as_slice(), &[inverse_move(m)], "{name}: {}", MOVE_NAMES[m as usize]);
        }
        let s = solve(t, &superflip, &o);
        assert!(superflip.apply_seq(s.moves.as_slice()).is_solved());
    }
    // the split chains are not optimal for single moves, but solve them
    for o in [Options::FAST5, Options::FAST6] {
        assert!(solve(t, &Cube::SOLVED, &o).moves.is_empty());
        for m in 0..NMOVES as u8 {
            let c = Cube::from_seq(&[m]);
            assert!(c.apply_seq(solve(t, &c, &o).moves.as_slice()).is_solved());
        }
        assert!(superflip.apply_seq(solve(t, &superflip, &o).moves.as_slice()).is_solved());
    }
}
