//! Builds the pruning tables at compile time (a BFS of ~0.5 s) and writes
//! them to OUT_DIR, where the solver embeds them with `include_bytes!`. The
//! cube model and table code are shared with the crate via `#[path]`.

#[allow(dead_code, unused_imports, clippy::wrong_self_convention)]
#[path = "src/cube.rs"]
mod cube;
#[allow(dead_code, unused_imports, clippy::wrong_self_convention)]
#[path = "src/defs.rs"]
mod defs;
#[allow(dead_code, unused_imports, clippy::wrong_self_convention)]
#[path = "src/facelet.rs"]
mod facelet;
#[allow(dead_code, unused_imports, clippy::wrong_self_convention)]
#[path = "src/tables.rs"]
mod tables;

fn main() {
    for f in ["build.rs", "src/cube.rs", "src/defs.rs", "src/facelet.rs", "src/tables.rs"] {
        println!("cargo:rerun-if-changed={f}");
    }
    let out = std::path::PathBuf::from(std::env::var("OUT_DIR").unwrap());
    for (name, data) in defs::build_all() {
        std::fs::write(out.join(format!("{name}.bin")), data).unwrap();
    }
}
