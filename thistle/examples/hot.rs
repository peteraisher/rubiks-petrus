//! Compute vs memory: the same cube solved repeatedly (tables hot) against
//! distinct random cubes. `cargo run --release --example hot [preset]`
use petrus::cube::{Cube, Rng};
use std::time::Instant;
use thistle::solver::{solve, Options, Tables};

const N: usize = 200_000;
const ROUNDS: usize = 5;
const NANOS: f64 = 1e9;

fn main() {
    let preset = std::env::args().nth(1).unwrap_or("fast".into());
    let o = Options::preset(&preset).expect("preset");
    let t = Tables::new();
    let mut rng = Rng::new(1);
    let cubes: Vec<Cube> = (0..N).map(|_| Cube::random(&mut rng)).collect();
    for c in &cubes {
        std::hint::black_box(solve(&t, c, &o));
    }
    for _ in 0..ROUNDS {
        let t0 = Instant::now();
        for c in &cubes {
            std::hint::black_box(solve(&t, std::hint::black_box(c), &o));
        }
        let distinct = t0.elapsed().as_secs_f64() / N as f64 * NANOS;
        let t0 = Instant::now();
        for _ in 0..N {
            std::hint::black_box(solve(&t, std::hint::black_box(&cubes[0]), &o));
        }
        let same = t0.elapsed().as_secs_f64() / N as f64 * NANOS;
        println!("{preset}: distinct cubes {distinct:6.0} ns, same cube {same:6.0} ns");
    }
}
