//! First-solve latency: in a fresh process, the time and page faults of
//! setting up the tables, of the first solve and of the next ones, then the
//! steady state. Run it as a new process each time (its tables are paged
//! in from the file cache only as solves touch them).
//!   cargo run --release --example firstsolve -- SOLVER
//! SOLVER: thistle-PRESET or petrus-PRESET. Prints, in µs and faults:
//!   setup  first  next  first_100  setup_faults  first_faults  first_100_faults  steady_ns
use petrus::cube::{Cube, Rng};
use std::time::Instant;

const FIRST_N: usize = 100;
const STEADY_N: usize = 20_000;
const MICROS: f64 = 1e6;
const NANOS: f64 = 1e9;
const SEED: u64 = 2024;

/// `struct rusage` on 64-bit macOS and Linux: two timevals, then 14 longs.
#[repr(C)]
struct Rusage {
    times: [i64; 4],
    longs: [i64; 14],
}
/// Index of `ru_minflt` among the longs (after maxrss, ixrss, idrss, isrss).
const MINFLT: usize = 4;
const MAJFLT: usize = 5;
const RUSAGE_SELF: i32 = 0;

extern "C" {
    fn getrusage(who: i32, usage: *mut Rusage) -> i32;
}

fn faults() -> i64 {
    let mut r = Rusage { times: [0; 4], longs: [0; 14] };
    // SAFETY: getrusage writes one struct rusage, whose layout Rusage matches.
    assert_eq!(unsafe { getrusage(RUSAGE_SELF, &mut r) }, 0);
    r.longs[MINFLT] + r.longs[MAJFLT]
}

fn main() {
    let which = std::env::args().nth(1).unwrap_or("thistle-fast".into());
    let (solver, preset) = which.split_once('-').expect("SOLVER-PRESET");
    let mut rng = Rng::new(SEED);
    let cubes: Vec<Cube> = (0..STEADY_N).map(|_| Cube::random(&mut rng)).collect();
    let f0 = faults();
    let t0 = Instant::now();
    let solve: Box<dyn Fn(&Cube) -> usize> = match solver {
        "thistle" => {
            let t = thistle::solver::Tables::new();
            let o = thistle::solver::Options::preset(preset).expect("preset");
            Box::new(move |c| thistle::solver::solve(&t, c, &o).moves.len())
        }
        "petrus" => {
            let t = petrus::solver::Tables::new();
            let o = petrus::solver::Options::preset(preset).expect("preset");
            Box::new(move |c| petrus::solver::solve(&t, c, &o).moves.len())
        }
        _ => panic!("unknown solver {solver}"),
    };
    let setup = t0.elapsed();
    let f1 = faults();
    let t1 = Instant::now();
    std::hint::black_box(solve(&cubes[0]));
    let first = t1.elapsed();
    let f2 = faults();
    let t2 = Instant::now();
    std::hint::black_box(solve(&cubes[1]));
    let next = t2.elapsed();
    for c in &cubes[2..FIRST_N] {
        std::hint::black_box(solve(c));
    }
    let first_100 = t1.elapsed();
    let f3 = faults();
    // steady state: everything touched once, then timed
    for c in &cubes {
        std::hint::black_box(solve(c));
    }
    let t3 = Instant::now();
    for c in &cubes {
        std::hint::black_box(solve(c));
    }
    let steady = t3.elapsed().as_secs_f64() / STEADY_N as f64 * NANOS;
    let us = |d: std::time::Duration| d.as_secs_f64() * MICROS;
    println!(
        "{:.1} {:.1} {:.1} {:.1} {} {} {} {:.0}",
        us(setup),
        us(first),
        us(next),
        us(first_100),
        f1 - f0,
        f2 - f1,
        f3 - f1,
        steady
    );
}
