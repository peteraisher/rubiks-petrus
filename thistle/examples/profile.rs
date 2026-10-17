//! A solve loop for profilers: `profile [N] [PRESET] [ROUNDS] [same]`; with
//! `same`, one cube solved over and over (compute only, tables hot).
use petrus::cube::{Cube, Rng};
use thistle::solver::{solve, Options, Tables};

const DEFAULT_N: usize = 20_000;
const DEFAULT_ROUNDS: usize = 200;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let n = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(DEFAULT_N);
    let o = Options::preset(args.get(2).map_or("fast", String::as_str)).expect("preset");
    let rounds = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(DEFAULT_ROUNDS);
    let same = args.get(4).is_some_and(|s| s == "same");
    let t = Tables::new();
    let mut rng = Rng::new(1);
    let cubes: Vec<Cube> = (0..n).map(|_| Cube::random(&mut rng)).collect();
    for _ in 0..rounds {
        for c in &cubes {
            let c = if same { &cubes[0] } else { c };
            std::hint::black_box(solve(&t, std::hint::black_box(c), &o));
        }
    }
}
