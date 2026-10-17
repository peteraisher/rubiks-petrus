//! `abtime` for batches: mean ns per solve over N distinct random cubes, one
//! number per preset on one line.
//!   cargo run --release --example abbatch -- N PRESET...
//! `PRESET@K` solves K cubes per call (`solve_batch`, K in 1, 2, 4, 8, 16);
//! `PRESET@Kv` merges their turns in NEON lanes. (A separate example so that
//! `abtime` still builds in revisions without batches.)
use petrus::cube::{Cube, Rng};
use std::time::Instant;
use thistle::solver::{solve, solve_batch_merging, Options, Tables};

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
    // the first timed pass otherwise runs measurably slower
    for c in &cubes {
        std::hint::black_box(solve(&t, c, &Options::FAST));
    }
    for arg in &args[1..] {
        let (name, k) = arg.split_once('@').map_or((arg.as_str(), None), |(p, k)| (p, Some(k)));
        let o = Options::preset(name).expect("preset");
        let t0 = Instant::now();
        match k {
            None => {
                for c in &cubes {
                    std::hint::black_box(solve(&t, c, &o));
                }
            }
            Some("1") => batches::<1, false>(&t, &cubes, &o),
            Some("1v") => batches::<1, true>(&t, &cubes, &o),
            Some("2") => batches::<2, false>(&t, &cubes, &o),
            Some("2v") => batches::<2, true>(&t, &cubes, &o),
            Some("4") => batches::<4, false>(&t, &cubes, &o),
            Some("4v") => batches::<4, true>(&t, &cubes, &o),
            Some("8") => batches::<8, false>(&t, &cubes, &o),
            Some("8v") => batches::<8, true>(&t, &cubes, &o),
            Some("16") => batches::<16, false>(&t, &cubes, &o),
            Some("16v") => batches::<16, true>(&t, &cubes, &o),
            Some(k) => panic!("batch size {k}"),
        }
        print!("{:.0} ", t0.elapsed().as_secs_f64() / n as f64 * NANOS);
    }
    println!();
}

fn batches<const K: usize, const LANE_MERGE: bool>(t: &Tables, cubes: &[Cube], o: &Options) {
    for c in cubes.as_chunks::<K>().0 {
        std::hint::black_box(solve_batch_merging::<K, LANE_MERGE>(t, c, o));
    }
}
