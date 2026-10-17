//! Builds the pruning tables at compile time (a BFS of ~0.5 s) and writes
//! them to OUT_DIR, where the solver embeds them with `include_bytes!`. The
//! cube model and table code are shared with the crate via `#[path]`.

#[allow(dead_code, unused_imports, clippy::wrong_self_convention)]
#[path = "src/compiled.rs"]
mod compiled;
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
    for f in ["build.rs", "src/compiled.rs", "src/cube.rs", "src/defs.rs", "src/facelet.rs", "src/tables.rs"] {
        println!("cargo:rerun-if-changed={f}");
    }
    let out = std::path::PathBuf::from(std::env::var("OUT_DIR").unwrap());
    let tables = defs::build_all();
    for (name, data) in &tables {
        std::fs::write(out.join(format!("{name}.bin")), data).unwrap();
    }
    // the compiled steps, searched with the tables just built
    let data = |name: &str| -> &'static [u8] { tables.iter().find(|(n, _)| *n == name).unwrap().1.clone().leak() };
    let b = compiled::Bounds {
        t222: &defs::T222.with_data(data(defs::T222.name)),
        t222_front: &defs::T222_FRONT.with_data(data(defs::T222_FRONT.name)),
        t223: [
            &defs::T223_D.with_data(data(defs::T223_D.name)),
            &defs::T223_L.with_data(data(defs::T223_L.name)),
            &defs::T223_FD.with_data(data(defs::T223_FD.name)),
            &defs::T223_FB.with_data(data(defs::T223_FB.name)),
        ],
        teo: &defs::TEO.with_data(data(defs::TEO.name)),
        tf2l: &defs::TF2L.with_data(data(defs::TF2L.name)),
    };
    for (name, algs) in compiled::NAMES.iter().zip(compiled::build(&b)) {
        let bytes: Vec<u8> = algs.iter().flat_map(|a| a.to_le_bytes()).collect();
        std::fs::write(out.join(format!("{name}.bin")), bytes).unwrap();
    }
}
