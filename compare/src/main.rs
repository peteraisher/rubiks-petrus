//! Solve the same random states with petrus and with other solvers that are
//! available as Rust crates, on the same machine, single-threaded.
use petrus::cube::{Cube, Rng};
use petrus::solver::{solve, Options, Tables};
use rcuber::cubie::CubieCube;
use rcuber::facelet::FaceCube;
use rcuber::moves::Move;
use std::time::Instant;

const DEFAULT_SAMPLES: usize = 200;
const SEED: u64 = 31337;
/// Target lengths for min2phase (it stops at the first solution this short;
/// its API clamps the target to 25).
const MIN2PHASE_TARGETS: [u8; 3] = [21, 23, 25];
const MICROS: f64 = 1e6;

/// Half-turn-metric length of an rcuber solution: slices count 2, cube
/// rotations 0.
fn htm(moves: &[Move]) -> usize {
    use Move::*;
    moves
        .iter()
        .map(|m| match m {
            M | M2 | M3 | E | E2 | E3 | S | S2 | S3 => 2,
            x | x2 | x3 | y | y2 | y3 | z | z2 | z3 | N => 0,
            _ => 1,
        })
        .sum()
}

fn report(name: &str, lens: &[usize], secs: f64, setup: f64) {
    let n = lens.len() as f64;
    println!(
        "{name:34} {:6.2} moves  {:10.1} us/solve  (setup {:.0} ms)",
        lens.iter().sum::<usize>() as f64 / n,
        secs / n * MICROS,
        setup * 1e3
    );
}

fn main() {
    let n: usize = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(DEFAULT_SAMPLES);
    let mut rng = Rng::new(SEED);
    let cubes: Vec<Cube> = (0..n).map(|_| Cube::random(&mut rng)).collect();
    let facelets: Vec<String> = cubes.iter().map(|c| c.to_facelets()).collect();
    println!("{n} random states, one thread, lengths in HTM\n");

    let t0 = Instant::now();
    let t = Tables::new();
    let setup = t0.elapsed().as_secs_f64();
    // warm up: fault in the embedded tables (other solvers get their first
    // solve untimed too)
    for c in &cubes {
        std::hint::black_box(solve(&t, c, &Options::SHORT));
    }
    for (name, opts) in [
        ("petrus classic (Lars' 7 steps)", Options::CLASSIC),
        ("petrus fast (ZBLL)", Options::FAST),
        ("petrus neutral", Options::NEUTRAL),
        ("petrus short", Options::SHORT),
        ("petrus shortest", Options::SHORTEST),
    ] {
        let t0 = Instant::now();
        let lens: Vec<usize> = cubes
            .iter()
            .map(|c| {
                let s = solve(&t, c, &opts);
                assert!(c.apply_seq(&s.moves).is_solved());
                s.moves.len()
            })
            .collect();
        report(name, &lens, t0.elapsed().as_secs_f64(), setup);
    }

    // min2phase: first call builds its tables
    let t0 = Instant::now();
    let _ = min2phase::solve(&facelets[0], MIN2PHASE_TARGETS[0]);
    let setup = t0.elapsed().as_secs_f64();
    for target in MIN2PHASE_TARGETS {
        let t0 = Instant::now();
        let lens: Vec<usize> =
            facelets.iter().map(|f| min2phase::solve(f, target).split_whitespace().count()).collect();
        report(&format!("min2phase two-phase (<= {target})"), &lens, t0.elapsed().as_secs_f64(), setup);
    }

    let cubies: Vec<CubieCube> = facelets
        .iter()
        .map(|f| CubieCube::try_from(&FaceCube::try_from(f.as_str()).unwrap()).unwrap())
        .collect();
    let run = |name: &str, f: &dyn Fn(CubieCube) -> Vec<Move>| {
        let t0 = Instant::now();
        let _ = f(cubies[0].clone());
        let setup = t0.elapsed().as_secs_f64();
        let t0 = Instant::now();
        let lens: Vec<usize> = cubies.iter().map(|c| htm(&f(c.clone()))).collect();
        report(name, &lens, t0.elapsed().as_secs_f64(), setup);
    };
    run("rcuber CFOP", &|c| rcuber::solver::cfop::CFOPSolver { cube: c }.solve());
    run("rcuber Roux", &|c| rcuber::solver::roux::RouxSolver { cube: c }.solve());
}
