//! Control experiment: petrus `fast` searched step by step against petrus
//! as it ships, with its steps compiled into tables of the search's own
//! solutions (one lookup per step), and thistle's presets. Checks that the
//! two petrus variants give identical solutions, then times everything on
//! the same states, interleaved, keeping each one's best round.
//!   cargo run --release --example compiled_petrus -- [N] [ROUNDS]
use petrus::cube::*;
use petrus::ll::LlMode;
use petrus::solver::*;
use std::time::Instant;

const DEFAULT_N: usize = 20_000;
const DEFAULT_ROUNDS: usize = 5;
const NANOS: f64 = 1e9;
const SEED: u64 = 2024;

type StepFn = fn(&Tables, &Cube, usize, u8, &mut u64) -> Vec<Vec<u8>>;

/// petrus `fast` with every step searched: the solver before compiled steps
/// (without its beam bookkeeping).
fn search_only(t: &Tables, c: &Cube) -> Vec<u8> {
    let mut x = *c;
    let mut out = FullSeq::default();
    let mut nodes = 0;
    let steps: [StepFn; 4] =
        [search_step::<Block222>, search_step::<Expand223>, search_step::<EdgeOrient>, search_step::<FirstTwoLayers>];
    for f in steps {
        let s = f(t, &x, 1, 0, &mut nodes).swap_remove(0);
        x = x.apply_seq(&s);
        out.extend_merged(&s);
    }
    t.ll.each_step(&x, LlMode::Zbll, |_, alg| out.extend_merged(alg));
    out.as_slice().to_vec()
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let n = args.first().and_then(|s| s.parse().ok()).unwrap_or(DEFAULT_N);
    let rounds = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(DEFAULT_ROUNDS);
    let t = Tables::new();
    let mut rng = Rng::new(SEED);
    let cubes: Vec<Cube> = (0..n).map(|_| Cube::random(&mut rng)).collect();
    let mut total = 0;
    for c in &cubes {
        let a = solve(&t, c, &Options::FAST);
        assert_eq!(a.moves, search_only(&t, c), "compiled petrus differs from the search");
        total += a.moves.len();
    }
    println!("compiled petrus identical to the search on {n} states");
    let tt = thistle::solver::Tables::new();
    let presets = ["fast", "fast5", "fast6", "short", "shorter", "shortest"];
    let opts: Vec<thistle::solver::Options> =
        presets.iter().map(|p| thistle::solver::Options::preset(p).expect("preset")).collect();
    let lens: Vec<f64> = opts
        .iter()
        .map(|o| cubes.iter().map(|c| thistle::solver::solve(&tt, c, o).moves.len()).sum::<usize>() as f64 / n as f64)
        .collect();
    let time = |f: &dyn Fn(&Cube)| {
        let t0 = Instant::now();
        for c in &cubes {
            f(c);
        }
        t0.elapsed().as_secs_f64() / n as f64 * NANOS
    };
    const PETRUS_ROWS: usize = 3;
    let neutral_len =
        cubes.iter().map(|c| solve(&t, c, &Options::NEUTRAL).moves.len()).sum::<usize>() as f64 / n as f64;
    let mut best = vec![f64::MAX; PETRUS_ROWS + presets.len()];
    for _ in 0..rounds {
        best[0] = best[0].min(time(&|c| {
            std::hint::black_box(search_only(&t, c));
        }));
        best[1] = best[1].min(time(&|c| {
            std::hint::black_box(solve(&t, c, &Options::FAST));
        }));
        best[2] = best[2].min(time(&|c| {
            std::hint::black_box(solve(&t, c, &Options::NEUTRAL));
        }));
        for (k, o) in opts.iter().enumerate() {
            best[PETRUS_ROWS + k] = best[PETRUS_ROWS + k].min(time(&|c| {
                std::hint::black_box(thistle::solver::solve(&tt, c, o));
            }));
        }
    }
    println!("{:28} {:>6} {:>8}", "", "moves", "ns");
    println!("{:28} {:6.2} {:8.0}", "petrus fast, searched", total as f64 / n as f64, best[0]);
    println!("{:28} {:6.2} {:8.0}", "petrus fast (compiled steps)", total as f64 / n as f64, best[1]);
    println!("{:28} {:6.2} {:8.0}", "petrus neutral", neutral_len, best[2]);
    for (k, p) in presets.iter().enumerate() {
        println!("{:28} {:6.2} {:8.0}", format!("thistle {p}"), lens[k], best[PETRUS_ROWS + k]);
    }
}
