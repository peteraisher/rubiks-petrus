use petrus::cube::{parse_seq, seq_to_string, Cube, Rng};
use std::time::Instant;
use thistle::coord::N_PHASES;
use thistle::solver::{solve, Options, Solution, Tables};

const USAGE: &str = "\
thistle - Rubik's cube solver: Thistlethwaite's phases as table lookups

USAGE:
  thistle [OPTIONS] [SCRAMBLE MOVES...]     solve a scramble, e.g. thistle R U F2 D'
  thistle [OPTIONS] -f FACELETS             solve a 54-char facelet string (URFDLB order)
  thistle [OPTIONS] --random N              solve N random states, print each
  thistle [OPTIONS] --bench N               benchmark N random states

OPTIONS:
  -p, --preset NAME   fast (default) | fast5 | fast6 | short | shorter | shortest
  -s, --starts N      starting orientations tried, 1..12 (axis orders x inverse)
  -b, --beam N        partial solutions kept after each phase
  -a, --alts A,B,C    stored alternatives tried in phases 1, 2, 3
  -t, --threads N     worker threads for --bench/--random (default: 1)
      --seed N        random seed
";

/// Same default seed as `petrus --bench`, so both solve the same states.
const DEFAULT_RANDOM_SEED: u64 = 2024;
const PHASE_NAMES: [&str; N_PHASES] = ["EO", "CO+E", "G3", "squares"];

struct Args {
    opts: Options,
    facelets: Option<String>,
    random: Option<usize>,
    bench: Option<usize>,
    threads: usize,
    seed: u64,
    scramble: Vec<String>,
}

fn parse_args() -> Result<Args, String> {
    let mut a = Args {
        opts: Options::FAST,
        facelets: None,
        random: None,
        bench: None,
        threads: 1,
        seed: DEFAULT_RANDOM_SEED,
        scramble: Vec::new(),
    };
    let mut it = std::env::args().skip(1);
    while let Some(arg) = it.next() {
        let mut val = |name: &str| it.next().ok_or(format!("{name} needs a value"));
        let num = |s: &str| s.parse::<usize>().map_err(|e| format!("bad number '{s}': {e}"));
        match arg.as_str() {
            "-h" | "--help" => return Err(String::new()),
            "-p" | "--preset" => {
                let p = val(&arg)?;
                a.opts = Options::preset(&p).ok_or(format!("unknown preset '{p}'"))?;
            }
            "-s" | "--starts" => a.opts.starts = num(&val(&arg)?)?,
            "-b" | "--beam" => a.opts.width = num(&val(&arg)?)?,
            "-a" | "--alts" => {
                let v = val(&arg)?;
                let parts: Vec<&str> = v.split(',').collect();
                if parts.len() != a.opts.alts.len() {
                    return Err(format!("--alts needs {} comma-separated numbers", a.opts.alts.len()));
                }
                for (dst, p) in a.opts.alts.iter_mut().zip(parts) {
                    *dst = num(p)?;
                }
            }
            "-f" | "--facelets" => a.facelets = Some(val(&arg)?),
            "-r" | "--random" => a.random = Some(num(&val(&arg)?)?),
            "--bench" => a.bench = Some(num(&val(&arg)?)?),
            "-t" | "--threads" => a.threads = num(&val(&arg)?)?.max(1),
            "--seed" => a.seed = num(&val(&arg)?)? as u64,
            _ if arg.starts_with('-') => return Err(format!("unknown option '{arg}'")),
            _ => a.scramble.push(arg),
        }
    }
    Ok(a)
}

fn print_solution(s: &Solution) {
    let lens: Vec<String> = PHASE_NAMES.iter().zip(s.phase_lens).map(|(n, l)| format!("{n} {l}")).collect();
    println!("  phases: {}", lens.join(", "));
    println!("  total {} moves (HTM): {}", s.moves.len(), seq_to_string(s.moves.as_slice()));
}

fn run_many(t: &Tables, args: &Args, n: usize, print_each: bool) {
    let mut rng = Rng::new(args.seed);
    let cubes: Vec<Cube> = (0..n).map(|_| Cube::random(&mut rng)).collect();
    let threads = args.threads.min(n.max(1));
    // untimed warm-up: fault in the embedded tables and train the caches
    const WARMUP_SOLVES: usize = 200;
    for c in cubes.iter().take(WARMUP_SOLVES) {
        std::hint::black_box(solve(t, c, &args.opts));
    }
    let t0 = Instant::now();
    let results: Vec<(Solution, f64)> = std::thread::scope(|s| {
        let chunk = n.div_ceil(threads);
        let handles: Vec<_> = cubes
            .chunks(chunk.max(1))
            .map(|cs| {
                s.spawn(move || {
                    cs.iter()
                        .map(|c| {
                            let t1 = Instant::now();
                            let sol = solve(t, c, &args.opts);
                            (sol, t1.elapsed().as_secs_f64())
                        })
                        .collect::<Vec<_>>()
                })
            })
            .collect();
        handles.into_iter().flat_map(|h| h.join().unwrap()).collect()
    });
    let wall = t0.elapsed().as_secs_f64();
    for (c, (s, _)) in cubes.iter().zip(&results) {
        assert!(c.apply_seq(s.moves.as_slice()).is_solved(), "solution failed verification");
        if print_each {
            println!("{}", c.to_facelets());
            print_solution(s);
        }
    }
    let lens: Vec<usize> = results.iter().map(|(s, _)| s.moves.len()).collect();
    let mean = |v: &mut dyn Iterator<Item = f64>| {
        let (sum, k) = v.fold((0.0, 0usize), |(a, k), x| (a + x, k + 1));
        sum / k as f64
    };
    println!("solved {n} random states, all verified");
    println!(
        "  length: mean {:.2}  min {}  max {}",
        mean(&mut lens.iter().map(|&x| x as f64)),
        lens.iter().min().unwrap(),
        lens.iter().max().unwrap()
    );
    let per_phase: Vec<String> = (0..N_PHASES)
        .map(|p| format!("{} {:.2}", PHASE_NAMES[p], mean(&mut results.iter().map(|(s, _)| s.phase_lens[p] as f64))))
        .collect();
    println!("  per phase: {}", per_phase.join(", "));
    const NANOS: f64 = 1e9;
    println!(
        "  time: {:.0} ns/solve per thread, {:.0} solves/s on {threads} threads, {:.0} evaluations/solve",
        mean(&mut results.iter().map(|(_, t)| *t)) * NANOS,
        n as f64 / wall,
        mean(&mut results.iter().map(|(s, _)| s.evals as f64))
    );
}

fn main() {
    let args = match parse_args() {
        Ok(a) => a,
        Err(e) => {
            if !e.is_empty() {
                eprintln!("error: {e}\n");
            }
            eprint!("{USAGE}");
            std::process::exit(if e.is_empty() { 0 } else { 2 });
        }
    };
    let tables = Tables::new();
    if let Some(n) = args.bench {
        run_many(&tables, &args, n, false);
        return;
    }
    if let Some(n) = args.random {
        run_many(&tables, &args, n, true);
        return;
    }
    let cube = if let Some(f) = &args.facelets {
        Cube::from_facelets(f)
    } else {
        parse_seq(&args.scramble.join(" ")).map(|q| Cube::from_seq(&q))
    };
    let cube = cube.unwrap_or_else(|e| {
        eprintln!("error: {e}");
        std::process::exit(2);
    });
    let t1 = Instant::now();
    let s = solve(&tables, &cube, &args.opts);
    let dt = t1.elapsed();
    print_solution(&s);
    assert!(cube.apply_seq(s.moves.as_slice()).is_solved());
    eprintln!("solved in {dt:.2?}, verified");
}
