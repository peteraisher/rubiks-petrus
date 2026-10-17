//! Whole-process timing of single solves: runs `BINARY SEED [ARGS...]` (for
//! example `target/release/examples/cold`) in N fresh processes with
//! distinct seeds, timing spawn to exit, and reports the median and mean of
//! the wall time and of the in-process time the child prints first.
//!
//!   cargo run --release --example coldrun -- N BINARY [ARGS...]
use std::process::Command;
use std::time::Instant;

const SEED_BASE: u64 = 1000;
const MILLIS: f64 = 1e3;

fn summary(v: &mut [f64]) -> String {
    v.sort_by(f64::total_cmp);
    let mean = v.iter().sum::<f64>() / v.len() as f64;
    format!("median {:.3} ms, mean {:.3} ms", v[v.len() / 2], mean)
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let n: u64 = args[0].parse().unwrap();
    let (binary, rest) = (&args[1], &args[2..]);
    let (mut wall, mut inproc) = (vec![], vec![]);
    for seed in SEED_BASE..SEED_BASE + n {
        let t0 = Instant::now();
        let out = Command::new(binary).arg(seed.to_string()).args(rest).output().unwrap();
        wall.push(t0.elapsed().as_secs_f64() * MILLIS);
        let text = String::from_utf8(out.stdout).unwrap();
        inproc.push(text.split_whitespace().next().unwrap().parse::<f64>().unwrap() / MILLIS);
    }
    println!("whole process: {};  solve in process: {}", summary(&mut wall), summary(&mut inproc));
}
