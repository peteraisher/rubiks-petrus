//! Data for the solve video: one scramble solved by thistle (`fast`:
//! Thistlethwaite's phases as table lookups), petrus (`fast`, its steps
//! compiled into table lookups), min2phase and rcuber CFOP, with each step's
//! moves, its time (median of many warm repeats on this machine, one thread)
//! and, when built with the `count` feature, its search node count.
//!
//!   solvedata pick [N] [SEED]   choose a scramble whose solution lengths
//!                               are closest to each solver's mean
//!   solvedata data FACELETS     print the JSON for that scramble
use petrus::cube::{seq_to_string, Cube, Rng};
use petrus::ll::LlMode;
use petrus::solver::*;
use rcuber::cubie::CubieCube;
use rcuber::facelet::FaceCube;
use rcuber::moves::Move;
use rcuber::solver::cfop::{CFOPSolver, CrossSolver, F2LSolver, OLLSolver, PLLSolver};
use std::hint::black_box;
use std::time::{Duration, Instant};

const DEFAULT_CANDIDATES: usize = 100;
const DEFAULT_SEED: u64 = 2026;
/// min2phase stops at the first solution this short; 25 (its maximum) is
/// its fastest setting.
const MIN2PHASE_TARGET: u8 = 25;
/// Timing: repeat a call until a batch lasts this long, then take the
/// median per-call time over this many batches.
const MIN_BATCH: Duration = Duration::from_millis(20);
const BATCHES: usize = 15;
/// Quicker timing for screening candidate scrambles.
const PICK_BATCH: Duration = Duration::from_millis(2);
const PICK_BATCHES: usize = 3;
const N_SOLVERS: usize = 4;

/// Whole-solve times, as for a scramble the solver has not seen before:
/// each timed solve of the scramble comes after at least FLUSH of solving
/// other random cubes with the same solver. Fewer would not do: the branch
/// predictor still remembers petrus' search for this scramble after 100
/// other solves (about 0.2 ms), and its time only settles after about 1000.
const FLUSH: Duration = Duration::from_millis(2);
const INTERLEAVED_SAMPLES: usize = 401;
/// The timer ticks every 41.7 ns (24 MHz on Apple silicon), so a single
/// solve of ~150 ns reads as 3, 4 or 5 ticks and a median would round to a
/// tick. Each reading starts at a random phase of the tick, so the mean of
/// many is unbiased: the mean of the middle half (robust to interrupts like
/// a median).
const TRIM: usize = 4;
const OTHER_CUBES: usize = 4096;
const OTHER_SEED: u64 = 4242;
/// Before timing, solve other cubes for this long so every table page is
/// mapped and cached as in steady use.
const WARM: Duration = Duration::from_millis(300);

/// Nanoseconds of single calls of `target`, each preceded by at least FLUSH
/// of calls of `other` on different cubes: the mean of the middle half.
fn time_interleaved(mut target: impl FnMut(), mut other: impl FnMut(usize)) -> f64 {
    let mut next = 0;
    let mut run_others = |d: Duration| {
        let t0 = Instant::now();
        loop {
            other(next % OTHER_CUBES);
            next += 1;
            if t0.elapsed() >= d {
                break;
            }
        }
    };
    run_others(WARM);
    let mut v: Vec<f64> = (0..INTERLEAVED_SAMPLES)
        .map(|_| {
            run_others(FLUSH);
            let t0 = Instant::now();
            target();
            t0.elapsed().as_nanos() as f64
        })
        .collect();
    v.sort_by(f64::total_cmp);
    let mid = &v[v.len() / TRIM..v.len() - v.len() / TRIM];
    mid.iter().sum::<f64>() / mid.len() as f64
}

/// Median nanoseconds per call of `f`, repeated warm. Used for the split
/// between steps (too short to time singly); the renderer scales the steps
/// to the interleaved whole-solve time.
fn time_ns(f: impl FnMut()) -> f64 {
    time_ns_with(MIN_BATCH, BATCHES, f)
}

fn time_ns_with(min_batch: Duration, batches: usize, mut f: impl FnMut()) -> f64 {
    f();
    let mut iters = 1u32;
    loop {
        let t0 = Instant::now();
        for _ in 0..iters {
            f();
        }
        if t0.elapsed() >= min_batch {
            break;
        }
        iters *= 2;
    }
    let mut v: Vec<f64> = (0..batches)
        .map(|_| {
            let t0 = Instant::now();
            for _ in 0..iters {
                f();
            }
            t0.elapsed().as_nanos() as f64 / iters as f64
        })
        .collect();
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

#[cfg(feature = "count")]
fn rival_nodes() -> u64 {
    use std::sync::atomic::Ordering::Relaxed;
    rcuber::NODES.load(Relaxed) + min2phase::NODES.load(Relaxed)
}
#[cfg(not(feature = "count"))]
fn rival_nodes() -> u64 {
    0
}

/// Half-turn-metric length of an rcuber solution: slices count 2, cube
/// rotations 0.
fn htm(moves: &[Move]) -> usize {
    use Move::*;
    moves
        .iter()
        .map(|m| match m {
            M | M2 | M3 | E | E2 | E3 | S | S2 | S3 => 2,
            x | x2 | x3 | y | y2 | y3 | z | z2 | z3 | N => 0,
            _ => 1,
        })
        .sum()
}

/// rcuber writes a counter-clockwise turn as `R3`; print it as `R'`.
fn rcuber_string(moves: &[Move]) -> String {
    moves
        .iter()
        .filter(|&&m| m != Move::N)
        .map(|m| {
            let s = format!("{m:?}");
            match s.strip_suffix('3') {
                Some(base) => format!("{base}'"),
                None => s,
            }
        })
        .collect::<Vec<_>>()
        .join(" ")
}

fn cubie(facelets: &str) -> CubieCube {
    CubieCube::try_from(&FaceCube::try_from(facelets).unwrap()).unwrap()
}

struct StepOut {
    name: &'static str,
    /// Cube after this step (petrus only, to check the renderer's model).
    facelets: Option<String>,
    moves: String,
    htm: usize,
    ns: f64,
    nodes: Option<u64>,
}

struct SolverOut {
    id: &'static str,
    name: &'static str,
    method: &'static str,
    total_ns: f64,
    steps: Vec<StepOut>,
}

type RcuberStep = (&'static str, fn(CubieCube) -> (Vec<Move>, CubieCube));

const CFOP_STEPS: [RcuberStep; 4] = [
    ("Cross", |c| {
        let mut s = CrossSolver { cube: c };
        (s.solve(), s.cube)
    }),
    ("F2L", |c| {
        let mut s = F2LSolver { cube: c };
        (s.solve(), s.cube)
    }),
    ("OLL", |c| {
        let mut s = OLLSolver::new(c);
        (s.solve(), s.cube)
    }),
    ("PLL", |c| {
        let mut s = PLLSolver::new(c);
        (s.solve(), s.cube)
    }),
];

fn rcuber_steps(start: CubieCube, steps: &[RcuberStep]) -> Vec<StepOut> {
    let mut c = start;
    let mut out = Vec::new();
    for &(name, f) in steps {
        let n0 = rival_nodes();
        let (moves, next) = f(c);
        let nodes = rival_nodes() - n0;
        let ns = time_ns(|| {
            black_box(f(black_box(c)));
        });
        out.push(StepOut {
            name,
            facelets: None,
            moves: rcuber_string(&moves),
            htm: htm(&moves),
            ns,
            nodes: cfg!(feature = "count").then_some(nodes),
        });
        c = next;
    }
    assert!(c == CubieCube::default(), "rcuber steps did not solve the cube");
    out
}

fn petrus_steps(t: &Tables, cube: &Cube) -> Vec<StepOut> {
    let sol = solve(t, cube, &Options::FAST);
    let mut out = Vec::new();
    let mut c = *cube;
    let mut push = |name, moves: &[u8], ns, nodes, after: &Cube| {
        out.push(StepOut {
            name,
            facelets: Some(after.to_facelets()),
            moves: seq_to_string(moves),
            htm: moves.len(),
            ns,
            nodes: Some(nodes),
        })
    };
    fn one<S: Step>(t: &Tables, c: &Cube, nodes: &mut u64) -> Vec<u8> {
        solve_step::<S>(t, c, 1, 0, nodes).swap_remove(0)
    }
    let searches: [(&'static str, fn(&Tables, &Cube, &mut u64) -> Vec<u8>); 4] = [
        ("2x2x2 block", one::<Block222>),
        ("2x2x3 block", one::<Expand223>),
        ("Edge orientation", one::<EdgeOrient>),
        ("First two layers", one::<FirstTwoLayers>),
    ];
    for (i, (name, f)) in searches.into_iter().enumerate() {
        let mut nodes = 0;
        let moves = f(t, &c, &mut nodes);
        // a compiled step searches nothing: it is one table lookup
        let nodes = nodes.max(1);
        assert_eq!(moves, sol.steps[i].moves, "step {name} differs from solve()");
        let ns = time_ns(|| {
            black_box(f(t, black_box(&c), &mut 0));
        });
        c = c.apply_seq(&moves);
        push(name, &moves, ns, nodes, &c);
    }
    // ZBLL: one lookup in the optimal last-layer table
    let ll: Vec<u8> = t.ll.solve(&c, LlMode::Zbll).into_iter().flat_map(|(_, a)| a.iter().copied()).collect();
    let ns = time_ns(|| {
        black_box(t.ll.solve(black_box(&c), LlMode::Zbll));
    });
    let c = c.apply_seq(&ll);
    assert!(c.is_solved());
    push("Last layer (ZBLL)", &ll, ns, 1, &c);
    out
}

const THISTLE_PHASES: [&str; thistle::coord::N_PHASES] =
    ["Orient edges", "Twist corners, place E-slice", "Into the squares group", "Half turns only"];

/// thistle `fast`, phase by phase: each phase is one table lookup.
fn thistle_steps(t: &thistle::solver::Tables, cube: &Cube) -> Vec<StepOut> {
    let sol = thistle::solver::solve(t, cube, &thistle::solver::Options::FAST);
    let mut c = *cube;
    let mut out = Vec::new();
    for (p, name) in THISTLE_PHASES.into_iter().enumerate() {
        let (moves, after) = thistle::solver::chain_phase(t, p, &c);
        assert_eq!(moves.len(), sol.phase_lens[p] as usize, "phase {name} differs from solve()");
        let ns = time_ns(|| {
            black_box(thistle::solver::chain_phase(t, p, black_box(&c)));
        });
        out.push(StepOut {
            name,
            facelets: Some(after.to_facelets()),
            moves: seq_to_string(moves.as_slice()),
            htm: moves.len(),
            ns,
            nodes: Some(1),
        });
        c = after;
    }
    assert!(c.is_solved());
    out
}

/// Solution length and time per solver, or None if petrus' or thistle's
/// steps cancel moves across a step boundary (the video plays steps as
/// found).
fn measure(t: &Tables, tt: &thistle::solver::Tables, c: &Cube) -> Option<[(f64, f64); N_SOLVERS]> {
    let f = c.to_facelets();
    let k = cubie(&f);
    let p = solve(t, c, &Options::FAST);
    let th = thistle::solver::solve(tt, c, &thistle::solver::Options::FAST);
    if p.steps.iter().map(|s| s.moves.len()).sum::<usize>() != p.moves.len()
        || th.phase_lens.iter().map(|&n| n as usize).sum::<usize>() != th.moves.len()
    {
        return None;
    }
    let quick = |g: &mut dyn FnMut()| time_ns_with(PICK_BATCH, PICK_BATCHES, g);
    Some([
        (th.moves.len() as f64, quick(&mut || {
            black_box(thistle::solver::solve(tt, black_box(c), &thistle::solver::Options::FAST));
        })),
        (p.moves.len() as f64, quick(&mut || {
            black_box(solve(t, black_box(c), &Options::FAST));
        })),
        (
            min2phase::solve(&f, MIN2PHASE_TARGET).split_whitespace().count() as f64,
            quick(&mut || {
                black_box(min2phase::solve(black_box(&f), MIN2PHASE_TARGET));
            }),
        ),
        (htm(&CFOPSolver { cube: k }.solve()) as f64, quick(&mut || {
            black_box(CFOPSolver { cube: black_box(k) }.solve());
        })),
    ])
}

/// Choose the candidate closest to typical for every solver: relative
/// distance of its length from the mean plus log-distance of its time from
/// the median.
fn pick(n: usize, seed: u64) {
    let t = Tables::new();
    let tt = thistle::solver::Tables::new();
    let mut rng = Rng::new(seed);
    let all: Vec<Cube> = (0..n).map(|_| Cube::random(&mut rng)).collect();
    black_box(solve(&t, &all[0], &Options::SHORT));
    let (cubes, ms): (Vec<Cube>, Vec<[(f64, f64); N_SOLVERS]>) =
        all.iter().filter_map(|c| measure(&t, &tt, c).map(|m| (*c, m))).unzip();
    let mean_len: Vec<f64> = (0..N_SOLVERS).map(|i| ms.iter().map(|m| m[i].0).sum::<f64>() / ms.len() as f64).collect();
    let median_ns: Vec<f64> = (0..N_SOLVERS)
        .map(|i| {
            let mut v: Vec<f64> = ms.iter().map(|m| m[i].1).collect();
            v.sort_by(f64::total_cmp);
            v[v.len() / 2]
        })
        .collect();
    let score = |m: &[(f64, f64); N_SOLVERS]| {
        (0..N_SOLVERS).map(|i| (m[i].0 - mean_len[i]).abs() / mean_len[i] + (m[i].1 / median_ns[i]).ln().abs()).sum::<f64>()
    };
    let best = (0..cubes.len()).min_by(|&a, &b| score(&ms[a]).total_cmp(&score(&ms[b]))).unwrap();
    eprintln!("{} candidates without cancellation (thistle, petrus, min2phase, CFOP)", cubes.len());
    eprintln!("mean lengths {mean_len:.1?}, median ns {median_ns:.0?}");
    eprintln!("picked: {:.0?}", ms[best]);
    println!("{}", cubes[best].to_facelets());
}

fn json_solver(s: &SolverOut) -> String {
    let steps: Vec<String> = s
        .steps
        .iter()
        .map(|st| {
            let nodes = st.nodes.map_or("null".to_string(), |n| n.to_string());
            let facelets = st.facelets.as_ref().map_or("null".to_string(), |f| format!("\"{f}\""));
            format!(
                "      {{\"name\": \"{}\", \"moves\": \"{}\", \"htm\": {}, \"ns\": {:.1}, \"nodes\": {nodes}, \"facelets\": {facelets}}}",
                st.name, st.moves, st.htm, st.ns
            )
        })
        .collect();
    format!(
        "    {{\"id\": \"{}\", \"name\": \"{}\", \"method\": \"{}\", \"total_ns\": {:.1}, \"steps\": [\n{}\n    ]}}",
        s.id,
        s.name,
        s.method,
        s.total_ns,
        steps.join(",\n")
    )
}

fn data(facelets: &str) {
    let t = Tables::new();
    let tt = thistle::solver::Tables::new();
    let cube = Cube::from_facelets(facelets).unwrap();
    // fault in the embedded tables before timing anything
    black_box(solve(&t, &cube, &Options::SHORT));
    black_box(thistle::solver::solve(&tt, &cube, &thistle::solver::Options::SHORT));
    let k = cubie(facelets);
    let f = facelets.to_string();
    let mut rng = Rng::new(OTHER_SEED);
    let others: Vec<Cube> = (0..OTHER_CUBES).map(|_| Cube::random(&mut rng)).collect();
    let other_f: Vec<String> = others.iter().map(|c| c.to_facelets()).collect();
    let other_k: Vec<CubieCube> = other_f.iter().map(|s| cubie(s)).collect();

    let thistle_fast = thistle::solver::Options::FAST;
    let thistle = SolverOut {
        id: "thistle",
        name: "thistle (this project)",
        method: "Thistlethwaite",
        total_ns: time_interleaved(
            || {
                black_box(thistle::solver::solve(&tt, black_box(&cube), &thistle_fast));
            },
            |i| {
                black_box(thistle::solver::solve(&tt, black_box(&others[i]), &thistle_fast));
            },
        ),
        steps: thistle_steps(&tt, &cube),
    };

    let petrus = SolverOut {
        id: "petrus",
        name: "petrus (this project)",
        method: "Petrus",
        total_ns: time_interleaved(
            || {
                black_box(solve(&t, black_box(&cube), &Options::FAST));
            },
            |i| {
                black_box(solve(&t, black_box(&others[i]), &Options::FAST));
            },
        ),
        steps: petrus_steps(&t, &cube),
    };

    let n0 = rival_nodes();
    let sol = min2phase::solve(&f, MIN2PHASE_TARGET);
    let nodes = rival_nodes() - n0;
    let m2p_ns = time_interleaved(
        || {
            black_box(min2phase::solve(black_box(&f), MIN2PHASE_TARGET));
        },
        |i| {
            black_box(min2phase::solve(black_box(&other_f[i]), MIN2PHASE_TARGET));
        },
    );
    let min2phase = SolverOut {
        id: "min2phase",
        name: "min2phase",
        method: "Kociemba two-phase",
        total_ns: m2p_ns,
        steps: vec![StepOut {
            name: "Two-phase search",
            facelets: None,
            htm: sol.split_whitespace().count(),
            moves: sol.split_whitespace().collect::<Vec<_>>().join(" "),
            ns: m2p_ns,
            nodes: cfg!(feature = "count").then_some(nodes),
        }],
    };

    let cfop = SolverOut {
        id: "cfop",
        name: "rcuber",
        method: "CFOP",
        total_ns: time_interleaved(
            || {
                black_box(CFOPSolver { cube: black_box(k) }.solve());
            },
            |i| {
                black_box(CFOPSolver { cube: black_box(other_k[i]) }.solve());
            },
        ),
        steps: rcuber_steps(k, &CFOP_STEPS),
    };

    let cpu = std::process::Command::new("sysctl")
        .args(["-n", "machdep.cpu.brand_string"])
        .output()
        .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
        .unwrap_or_default();
    let solvers: Vec<String> = [thistle, petrus, min2phase, cfop].iter().map(json_solver).collect();
    println!(
        "{{\n  \"facelets\": \"{facelets}\",\n  \"cpu\": \"{cpu}\",\n  \"solvers\": [\n{}\n  ]\n}}",
        solvers.join(",\n")
    );
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.first().map(String::as_str) {
        Some("pick") => pick(
            args.get(1).and_then(|s| s.parse().ok()).unwrap_or(DEFAULT_CANDIDATES),
            args.get(2).and_then(|s| s.parse().ok()).unwrap_or(DEFAULT_SEED),
        ),
        Some("data") if args.len() == 2 => data(&args[1]),
        _ => {
            eprintln!("usage: solvedata pick [N] [SEED] | solvedata data FACELETS");
            std::process::exit(2);
        }
    }
}
