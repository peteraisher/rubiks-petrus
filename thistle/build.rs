//! Builds every table at compile time (a few seconds on all cores) and
//! writes them to OUT_DIR, where the solver embeds them. The generator is
//! shared with the crate via `#[path]`.

#[allow(dead_code)]
#[path = "src/coord.rs"]
mod coord;
#[allow(dead_code)]
#[path = "src/gen.rs"]
mod gen;

use coord::*;

fn main() {
    for f in ["build.rs", "src/coord.rs", "src/gen.rs"] {
        println!("cargo:rerun-if-changed={f}");
    }
    let out = std::path::PathBuf::from(std::env::var("OUT_DIR").unwrap());
    let threads = std::thread::available_parallelism().map_or(1, |n| n.get());
    let write = |name: &str, bytes: Vec<u8>| std::fs::write(out.join(format!("{name}.bin")), bytes).unwrap();
    write("join", gen::join_table());
    let (class, g3) = gen::corner_tables();
    write("class", class.clone());
    write("g3", g3.clone());
    let ct = CornerTables { class: class.leak(), g3: g3.leak() };
    let words = |v: &[u16]| v.iter().flat_map(|x| x.to_le_bytes()).collect::<Vec<u8>>();
    let (mut dists, mut all_reps) = (Vec::new(), Vec::new());
    for (i, ph) in gen::PHASES.iter().enumerate() {
        let (dist, reps) = gen::bfs(&ct, ph);
        assert_eq!(*dist.iter().max().unwrap() as usize, PHASE_MAX_LEN[i], "phase {} diameter", i + 1);
        // phase 1 is small enough to store whole; the others by symmetry class
        let red = (i > 0).then(|| gen::reduce(&ct, ph, i, &reps));
        let cases: Vec<usize> = match &red {
            None => (0..ph.n).collect(),
            Some(red) => {
                write(&format!("minor{}", i + 1), words(&red.minor_sym));
                write(&format!("major{}", i + 1), words(&red.major_conj));
                gen::reduced_cases(i, red)
            }
        };
        let algs = gen::algorithms(&ct, ph, MOVE_BITS[i], &dist, &reps, MAX_ALTS[i], threads, &cases);
        if COMPOSED[i] {
            write(&format!("comp{}", i + 1), gen::composed(i, ph.n, &algs, red.as_ref()));
        }
        write(&format!("dist{}", i + 1), dist.clone());
        dists.push(dist);
        all_reps.push(reps);
        let bytes = |v: &mut dyn Iterator<Item = &u64>| v.flat_map(|x| x.to_le_bytes()).collect::<Vec<u8>>();
        write(&format!("alg{}", i + 1), bytes(&mut algs.iter()));
        // each row's first alternative alone, for the single chain: a quarter
        // to a sixteenth of the bytes it would otherwise pull into cache
        if MAX_ALTS[i] > 1 {
            write(&format!("first{}", i + 1), bytes(&mut algs.iter().step_by(MAX_ALTS[i])));
        }
    }
    for (h, algs) in gen::split_algorithms(&ct, &dists, &all_reps).iter().enumerate() {
        write(&format!("half{}", h + 1), algs.iter().flat_map(|x| x.to_le_bytes()).collect());
        if HALF_COMPOSED[h] {
            write(&format!("hcomp{}", h + 1), gen::composed_half(h, algs));
        }
    }
}
