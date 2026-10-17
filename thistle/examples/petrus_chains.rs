//! Petrus `fast` as whole greedy chains from every symmetric start: the best
//! of 48 (and of 96 with the inverse). Compare thistle's model numbers in
//! the README (best of 96 Thistlethwaite chains: 24.50).
//!   cargo run --release --example petrus_chains -- [N]
use petrus::cube::*;
use petrus::facelet::SYM_MOVE;
use petrus::solver::*;

const DEFAULT_N: usize = 2000;
const SEED: u64 = 7;
fn main() {
    let t = Tables::new();
    let mut rng = Rng::new(SEED);
    let n: usize = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(DEFAULT_N);
    let (mut one, mut b48, mut b96) = (0, 0, 0);
    for _ in 0..n {
        let c = Cube::random(&mut rng);
        let mut best = [usize::MAX; 2];
        for inv in [false, true] {
            let base = if inv { c.inverse() } else { c };
            for (s, map) in SYM_MOVE.iter().enumerate() {
                let sol = solve(&t, &base.conj(s), &Options::FAST);
                let mut orig: Vec<u8> = sol.moves.iter().map(|&m| map[m as usize]).collect();
                if inv {
                    orig = seq_inverse(&orig);
                }
                assert!(c.apply_seq(&orig).is_solved());
                if s == 0 && !inv {
                    one += orig.len();
                }
                best[inv as usize] = best[inv as usize].min(orig.len());
            }
        }
        b48 += best[0];
        b96 += best[0].min(best[1]);
    }
    let f = |x: usize| x as f64 / n as f64;
    println!("petrus fast: one {:.2}, best of 48 full chains {:.2}, of 96 {:.2}", f(one), f(b48), f(b96));
}
