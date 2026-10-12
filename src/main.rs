use petrus::cube::{parse_seq, seq_to_string, Cube, Rng};
use petrus::ll::{self, LlMode};
use petrus::solver::{solve, Options, Solution, Tables};
use std::time::Instant;

const USAGE: &str = "\
petrus - Rubik's cube solver using the Lars Petrus method

USAGE:
  petrus [OPTIONS] [SCRAMBLE MOVES...]     solve a scramble, e.g. petrus R U F2 D'
  petrus [OPTIONS] -f FACELETS             solve a 54-char facelet string (URFDLB order)
  petrus [OPTIONS] --random N              solve N random states, print each
  petrus [OPTIONS] --bench N               benchmark N random states
  petrus gen-ll FILE                       regenerate the last-layer table

OPTIONS:
  -p, --preset NAME   classic (default) | fast | neutral | short | shortest
  -o, --orient N      orientations to try: 1, 24 (rotations) or 48 (with mirrors)
  -b, --beam N        partial solutions kept after each step (1 = greedy)
  -a, --alts N        alternative solutions per step
  -s, --slack N       alternatives may be N moves above the step optimum
  -j, --joint         solve the 2x2x3 directly (steps 1+2 jointly optimal)
  -l, --ll MODE       lars (steps 5,6,7) | coll (COLL+EPLL) | zbll (one look)
      --no-lookahead  rank beam candidates by moves so far only
  -t, --threads N     worker threads for --bench/--random (default: all cores)
      --seed N        random seed
";

const DEFAULT_RANDOM_SEED: u64 = 2024;

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
        opts: Options::default(),
        facelets: None,
        random: None,
        bench: None,
        threads: std::thread::available_parallelism().map_or(1, |n| n.get()),
        seed: DEFAULT_RANDOM_SEED,
        scramble: Vec::new(),
    };
    let mut it = std::env::args().skip(1);
    while let Some(arg) = it.next() {
        let mut val = |name: &str| it.next().ok_or(format!("{name} needs a value"));
        let num = |s: String| s.parse::<usize>().map_err(|e| format!("bad number '{s}': {e}"));
        match arg.as_str() {
            "-h" | "--help" => return Err(String::new()),
            "-p" | "--preset" => {
                let p = val(&arg)?;
                a.opts = Options::preset(&p).ok_or(format!("unknown preset '{p}'"))?;
            }
            "-o" | "--orient" => a.opts.orientations = num(val(&arg)?)?,
            "-b" | "--beam" => a.opts.beam = num(val(&arg)?)?,
            "-a" | "--alts" => a.opts.alts = num(val(&arg)?)?,
            "-s" | "--slack" => a.opts.slack = num(val(&arg)?)? as u8,
            "-j" | "--joint" => a.opts.joint_blocks = true,
            "--no-lookahead" => a.opts.lookahead = false,
            "-l" | "--ll" => {
                let m = val(&arg)?;
                a.opts.ll = LlMode::parse(&m).ok_or(format!("unknown last-layer mode '{m}'"))?;
            }
            "-f" | "--facelets" => a.facelets = Some(val(&arg)?),
            "-r" | "--random" => a.random = Some(num(val(&arg)?)?),
            "--bench" => a.bench = Some(num(val(&arg)?)?),
            "-t" | "--threads" => a.threads = num(val(&arg)?)?.max(1),
            "--seed" => a.seed = num(val(&arg)?)? as u64,
            _ => a.scramble.push(arg),
        }
    }
    Ok(a)
}

fn print_solution(s: &Solution) {
    let width = s.steps.iter().map(|x| x.name.len()).max().unwrap_or(0);
    for st in &s.steps {
        println!("  {:width$}  ({:2})  {}", st.name, st.moves.len(), seq_to_string(&st.moves));
    }
    println!("  total {} moves (HTM): {}", s.moves.len(), seq_to_string(&s.moves));
}

fn gen_ll(path: &str) {
    let threads = std::thread::available_parallelism().map_or(1, |n| n.get());
    let t0 = Instant::now();
    let table = ll::generate(threads, &mut |line| eprintln!("{line}  [{:.1?}]", t0.elapsed()));
    std::fs::write(path, table.to_bytes()).expect("write table");
    eprintln!("wrote {path}");
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
        assert!(c.apply_seq(&s.moves).is_solved(), "solution failed verification");
        if print_each {
            println!("{}", c.to_facelets());
            print_solution(s);
        }
    }
    let lens: Vec<usize> = results.iter().map(|(s, _)| s.moves.len()).collect();
    let mean = |v: &[f64]| v.iter().sum::<f64>() / v.len() as f64;
    let lens_f: Vec<f64> = lens.iter().map(|&x| x as f64).collect();
    let times: Vec<f64> = results.iter().map(|(_, t)| *t).collect();
    let nodes: u64 = results.iter().map(|(s, _)| s.nodes).sum();
    println!("solved {n} random states, all verified");
    println!(
        "  length: mean {:.2}  min {}  max {}",
        mean(&lens_f),
        lens.iter().min().unwrap(),
        lens.iter().max().unwrap()
    );
    let names: Vec<&str> = results[0].0.steps.iter().map(|s| s.name).collect();
    let per_step: Vec<String> = (0..names.len())
        .map(|i| {
            let v: Vec<f64> = results.iter().map(|(s, _)| s.steps[i].moves.len() as f64).collect();
            format!("{} {:.2}", names[i], mean(&v))
        })
        .collect();
    println!("  per step: {}", per_step.join(", "));
    const MICROS: f64 = 1e6;
    println!(
        "  time: {:.1} us/solve per thread, {:.0} solves/s on {threads} threads, {:.0} nodes/solve",
        mean(&times) * MICROS,
        n as f64 / wall,
        nodes as f64 / n as f64
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
    if args.scramble.first().map(String::as_str) == Some("gen-ll") {
        let path = args.scramble.get(1).map_or("data/ll.bin", String::as_str);
        gen_ll(path);
        return;
    }
    let t0 = Instant::now();
    let tables = Tables::new();
    eprintln!("tables ready in {:.2?}", t0.elapsed());
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
    assert!(cube.apply_seq(&s.moves).is_solved());
    eprintln!("solved in {dt:.2?} ({} nodes), verified", s.nodes);
}
