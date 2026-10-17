//! One solve in a fresh process: the cold-start case.
//!
//!   cargo run --release --example cold -- SEED [SOLVER-PRESET | none]
//!
//! SOLVER: thistle or petrus; `none` does nothing (the process with this
//! binary's size). Prints microseconds for the table set-up and the solve,
//! then the solution's length. `examples/coldrun.rs` runs it in many fresh
//! processes.
use petrus::cube::{Cube, Rng};
use std::time::Instant;

const MICROS: f64 = 1e6;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let seed: u64 = args.get(1).map_or(0, |s| s.parse().unwrap());
    let which = args.get(2).map_or("thistle-fast", |s| s.as_str());
    let cube = Cube::random(&mut Rng::new(seed));
    let t0 = Instant::now();
    let len = match which.split_once('-') {
        Some(("thistle", p)) => {
            let o = thistle::solver::Options::preset(p).expect("preset");
            thistle::solver::solve(&thistle::solver::Tables::new(), &cube, &o).moves.len()
        }
        Some(("petrus", p)) => {
            let o = petrus::solver::Options::preset(p).expect("preset");
            petrus::solver::solve(&petrus::solver::Tables::new(), &cube, &o).moves.len()
        }
        _ => 0,
    };
    let t = t0.elapsed().as_secs_f64() * MICROS;
    println!("{t:.0} {len}");
}
