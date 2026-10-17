//! A solve loop for profilers: `profile [N] [PRESET] [ROUNDS] [same]`; with
//! `same`, one cube solved over and over (compute only, tables hot).
//! `PRESET@K` solves K cubes per call (`solve_batch`, K in 1, 2, 4, 8, 16).
use petrus::cube::{Cube, Rng};
use thistle::solver::{solve, solve_batch, Options, Tables};

const DEFAULT_N: usize = 20_000;
const DEFAULT_ROUNDS: usize = 200;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let n = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(DEFAULT_N);
    let arg = args.get(2).map_or("fast", String::as_str);
    let (name, k) = arg.split_once('@').map_or((arg, None), |(p, k)| (p, Some(k)));
    let o = Options::preset(name).expect("preset");
    let rounds = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(DEFAULT_ROUNDS);
    let same = args.get(4).is_some_and(|s| s == "same");
    let t = Tables::new();
    let mut rng = Rng::new(1);
    let mut cubes: Vec<Cube> = (0..n).map(|_| Cube::random(&mut rng)).collect();
    if same {
        let first = cubes[0];
        cubes.fill(first);
    }
    for _ in 0..rounds {
        match k {
            None => {
                for c in &cubes {
                    std::hint::black_box(solve(&t, std::hint::black_box(c), &o));
                }
            }
            Some("1") => batches::<1>(&t, &cubes, &o),
            Some("2") => batches::<2>(&t, &cubes, &o),
            Some("4") => batches::<4>(&t, &cubes, &o),
            Some("8") => batches::<8>(&t, &cubes, &o),
            Some("16") => batches::<16>(&t, &cubes, &o),
            Some(k) => panic!("batch size {k}"),
        }
    }
}

fn batches<const K: usize>(t: &Tables, cubes: &[Cube], o: &Options) {
    for c in cubes.as_chunks::<K>().0 {
        std::hint::black_box(solve_batch::<K>(t, std::hint::black_box(c), o));
    }
}
