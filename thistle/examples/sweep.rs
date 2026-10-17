//! Length / time frontier over beam settings. Prints one line per setting:
//! starts width alts mean-length evaluations ns (min over rounds).
//!   cargo run --release --example sweep -- [N] [ROUNDS]
use petrus::cube::{Cube, Rng};
use std::time::Instant;
use thistle::solver::{solve, Options, Tables};

const DEFAULT_N: usize = 5000;
const DEFAULT_ROUNDS: usize = 3;
const NANOS: f64 = 1e9;
const SEED: u64 = 2024;
const STARTS: [usize; 4] = [2, 4, 6, 12];
const WIDTHS: [usize; 4] = [3, 4, 6, 8];
const ALTS: [[usize; 3]; 8] = [[2, 2, 4], [4, 1, 4], [4, 2, 4], [4, 2, 8], [4, 4, 8], [8, 1, 8], [8, 2, 8], [8, 4, 8]];

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let n = args.first().and_then(|s| s.parse().ok()).unwrap_or(DEFAULT_N);
    let rounds = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(DEFAULT_ROUNDS);
    let t = Tables::new();
    let mut rng = Rng::new(SEED);
    let cubes: Vec<Cube> = (0..n).map(|_| Cube::random(&mut rng)).collect();
    for c in &cubes {
        std::hint::black_box(solve(&t, c, &Options::SHORTER));
    }
    for starts in STARTS {
        for width in WIDTHS {
            for alts in ALTS {
                let o = Options { starts, width, alts, ..Options::FAST };
                let (mut len, mut evals) = (0usize, 0u64);
                for c in &cubes {
                    let s = solve(&t, c, &o);
                    len += s.moves.len();
                    evals += s.evals as u64;
                }
                let best = (0..rounds)
                    .map(|_| {
                        let t0 = Instant::now();
                        for c in &cubes {
                            std::hint::black_box(solve(&t, c, &o));
                        }
                        t0.elapsed().as_secs_f64() / n as f64 * NANOS
                    })
                    .fold(f64::MAX, f64::min);
                println!(
                    "{starts:2} {width:2} {:2},{},{:2}  {:6.3} {:5.0} {best:6.0}",
                    alts[0],
                    alts[1],
                    alts[2],
                    len as f64 / n as f64,
                    evals as f64 / n as f64
                );
            }
        }
    }
}
