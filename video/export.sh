#!/bin/sh
# Pick a representative scramble and export solve data for the renderer:
#   render/data/solve.json  moves and timings (unmodified solver crates)
#   render/data/nodes.json  the same with node counts (instrumented copies)
set -eu
cd "$(dirname "$0")"
. "$HOME/.cargo/env" 2>/dev/null || true
cargo build -q --release --manifest-path solvedata/Cargo.toml
[ -d count/vendor ] || sh count/vendor.sh
cargo build -q --release --manifest-path count/Cargo.toml
mkdir -p render/data
f=${1:-$(solvedata/target/release/solvedata pick)}
count/target/release/solvedata-count data "$f" > render/data/nodes.json
solvedata/target/release/solvedata data "$f" > render/data/solve.json
echo "exported $f"
