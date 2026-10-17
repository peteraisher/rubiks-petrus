//! Mean ns per solve over N distinct random cubes, one number per preset on
//! one line. Used by prof/ab.sh to compare builds interleaved.
//!   cargo run --release --example abtime -- N PRESET...
use petrus::cube::{Cube, Rng};
use std::time::Instant;
use thistle::solver::{solve, Options, Tables};

const WARM: usize = 2000;
const NANOS: f64 = 1e9;
/// Benchmark states use this seed; the warm-up uses another.
const SEED: u64 = 2024;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let n: usize = args[0].parse().unwrap();
    let t = Tables::new();
    let mut rng = Rng::new(!SEED);
    for _ in 0..WARM {
        let c = Cube::random(&mut rng);
        std::hint::black_box(solve(&t, &c, &Options::SHORT));
        std::hint::black_box(solve(&t, &c, &Options::FAST));
    }
    let mut rng = Rng::new(SEED);
    let cubes: Vec<Cube> = (0..n).map(|_| Cube::random(&mut rng)).collect();
    for name in &args[1..] {
        let o = Options::preset(name).expect("preset");
        let t0 = Instant::now();
        for c in &cubes {
            std::hint::black_box(solve(&t, c, &o));
        }
        print!("{:.0} ", t0.elapsed().as_secs_f64() / n as f64 * NANOS);
    }
    println!();
}
