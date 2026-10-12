//! Experiment: node counts of the 2x2x3 expansion search with different
//! sets of pruning tables. `cargo run --release --example heuristic_experiment`
use petrus::cube::*;
use petrus::defs::*;
use petrus::solver::{solve_step, Block222};
use petrus::tables::PieceTable;

const SAMPLES: usize = 3000;
const SEED: u64 = 5;

type T5 = PieceTable<2, 3, false>;
type T4 = PieceTable<1, 3, false>;

struct Ctx<'a> {
    t4: Vec<&'a T4>,
    t5: Vec<&'a T5>,
    nodes: u64,
}

fn goal(c: &Cube) -> bool {
    [DLF, DBL].iter().all(|&i| c.c[i] as usize == i) && [DF, DL, DB, FL, BL].iter().all(|&i| c.e[i] as usize == i)
}

fn h(x: &Ctx, c: &Cube) -> u8 {
    x.t4.iter().map(|t| t.get(c)).chain(x.t5.iter().map(|t| t.get(c))).max().unwrap()
}

fn dfs(x: &mut Ctx, c: &Cube, togo: u8, last: u8) -> bool {
    for m in 0..NMOVES as u8 {
        let f = face(m);
        if f == last || opposite(f) == last && f < last {
            continue;
        }
        let n = c.apply_move(m as usize);
        x.nodes += 1;
        if h(x, &n) >= togo {
            continue;
        }
        if togo == 1 {
            if goal(&n) {
                return true;
            }
            continue;
        }
        if dfs(x, &n, togo - 1, f) {
            return true;
        }
    }
    false
}

fn main() {
    let t = petrus::solver::Tables::new();
    let extra = [([DLF, DBL], [DF, FL, DB]), ([DLF, DBL], [DF, FL, BL]), ([DLF, DBL], [DF, FL, DL])];
    let extra: Vec<T5> = extra.iter().map(|(c, e)| PieceTable::build(*c, *e, &ALL_MOVES)).collect();
    let mut rng = Rng::new(SEED);
    let starts: Vec<Cube> = (0..SAMPLES)
        .map(|_| {
            let c = Cube::random(&mut rng);
            let mut n = 0;
            c.apply_seq(&solve_step::<Block222>(&t, &c, 1, 0, &mut n)[0])
        })
        .collect();
    let configs: Vec<(&str, Vec<&T4>, Vec<&T5>)> = vec![
        ("current (2x 222, 2x 223)", vec![&t.t222, &t.t222_front], vec![&t.t223_d, &t.t223_l]),
        ("+ DF FL DB", vec![&t.t222, &t.t222_front], vec![&t.t223_d, &t.t223_l, &extra[0]]),
        ("+ DF FL DB, DF FL BL", vec![&t.t222, &t.t222_front], vec![&t.t223_d, &t.t223_l, &extra[0], &extra[1]]),
        ("+ all three", vec![&t.t222, &t.t222_front], vec![&t.t223_d, &t.t223_l, &extra[0], &extra[1], &extra[2]]),
    ];
    for (name, t4, t5) in configs {
        let mut x = Ctx { t4, t5, nodes: 0 };
        let start = std::time::Instant::now();
        let mut total_len = 0;
        for c in &starts {
            let mut d = h(&x, c).max(1);
            if goal(c) {
                continue;
            }
            while !dfs(&mut x, c, d, NO_FACE) {
                d += 1;
            }
            total_len += d as usize;
        }
        println!(
            "{name:28} len {:.2} nodes/solve {:7.1}  {:6.0} ns/solve",
            total_len as f64 / SAMPLES as f64,
            x.nodes as f64 / SAMPLES as f64,
            start.elapsed().as_nanos() as f64 / SAMPLES as f64
        );
    }
}
