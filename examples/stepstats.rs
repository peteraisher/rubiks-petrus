//! Per-step node counts and timings for the greedy fixed-orientation solver,
//! searched and compiled (one lookup per step, the same solutions).
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
    type StepFn = fn(&Tables, &Cube, usize, u8, &mut u64) -> Vec<Vec<u8>>;
    let modes: [(&str, [StepFn; 4]); 2] = [
        (
            "search",
            [
                search_step::<Block222>,
                search_step::<Expand223>,
                search_step::<EdgeOrient>,
                search_step::<FirstTwoLayers>,
            ],
        ),
        (
            "compiled",
            [solve_step::<Block222>, solve_step::<Expand223>, solve_step::<EdgeOrient>, solve_step::<FirstTwoLayers>],
        ),
    ];
    let names = ["2x2x2", "2x2x3", "EO", "F2L"];
    for (mode, fs) in modes {
        let mut stats = [(0u64, 0f64, 0usize); 4];
        for c in &cubes {
            let mut s = *c;
            for (i, f) in fs.iter().enumerate() {
                let t0 = Instant::now();
                let mut nodes = 0;
                let sol = f(&t, &s, 1, 0, &mut nodes);
                stats[i].0 += nodes;
                stats[i].1 += t0.elapsed().as_secs_f64();
                stats[i].2 += sol[0].len();
                s = s.apply_seq(&sol[0]);
            }
        }
        println!("{mode}:");
        for (name, (nodes, time, len)) in names.iter().zip(stats) {
            let per_node =
                if nodes == 0 { String::new() } else { format!("  {:5.1} ns/node", time * NANOS / nodes as f64) };
            println!(
                "{name:6} len {:5.2}  nodes {:7.1}  {:7.0} ns{per_node}",
                len as f64 / n as f64,
                nodes as f64 / n as f64,
                time / n as f64 * NANOS,
            );
        }
    }
}
