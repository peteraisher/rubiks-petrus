//! Per-step node counts and timings for the greedy fixed-orientation solver.
//! `cargo run --release --example stepstats [N]`
use petrus::cube::{Cube, Rng};
use petrus::solver::*;
use std::time::Instant;

const DEFAULT_SAMPLES: usize = 100_000;
const SEED: u64 = 99;
const NANOS: f64 = 1e9;

fn main() {
    let n: usize = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(DEFAULT_SAMPLES);
    let t = Tables::new();
    let mut rng = Rng::new(SEED);
    let cubes: Vec<Cube> = (0..n).map(|_| Cube::random(&mut rng)).collect();
    let mut stats = [(0u64, 0f64, 0usize); 4];
    let names = ["2x2x2", "2x2x3", "EO", "F2L"];
    for c in &cubes {
        let mut s = *c;
        let mut step = |i: usize, s: &mut Cube, f: &dyn Fn(&Cube, &mut u64) -> Vec<Vec<u8>>| {
            let t0 = Instant::now();
            let mut nodes = 0;
            let sol = f(s, &mut nodes);
            stats[i].0 += nodes;
            stats[i].1 += t0.elapsed().as_secs_f64();
            stats[i].2 += sol[0].len();
            *s = s.apply_seq(&sol[0]);
        };
        step(0, &mut s, &|c, n| solve_step::<Block222>(&t, c, 1, 0, n));
        step(1, &mut s, &|c, n| solve_step::<Expand223>(&t, c, 1, 0, n));
        step(2, &mut s, &|c, n| solve_step::<EdgeOrient>(&t, c, 1, 0, n));
        step(3, &mut s, &|c, n| solve_step::<FirstTwoLayers>(&t, c, 1, 0, n));
    }
    for (name, (nodes, time, len)) in names.iter().zip(stats) {
        println!(
            "{name:6} len {:5.2}  nodes {:7.1}  {:7.0} ns  {:5.1} ns/node",
            len as f64 / n as f64,
            nodes as f64 / n as f64,
            time / n as f64 * NANOS,
            time * NANOS / nodes.max(1) as f64
        );
    }
}
