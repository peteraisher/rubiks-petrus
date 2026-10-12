#!/bin/sh
# Copy min2phase and rcuber from the cargo registry into vendor/ and add a
# global node counter to each:
#   min2phase: one node per phase-1 / phase-2 search call,
#   rcuber:    one node per cube state generated (CubieCube::apply_move),
#              which covers its A*/IDA* searches, the pruning tables Roux
#              builds on every solve, and algorithm/AUF trials.
set -eu
cd "$(dirname "$0")"
reg=$(ls -d "$HOME"/.cargo/registry/src/*/ | head -n 1)
[ -d "$reg/min2phase-0.2.4" ] && [ -d "$reg/rcuber-0.7.20" ] || {
    echo "fetch the crates first: cargo fetch --manifest-path ../solvedata/Cargo.toml" >&2
    exit 1
}
rm -rf vendor
mkdir vendor
cp -R "$reg/min2phase-0.2.4" "$reg/rcuber-0.7.20" vendor/
chmod -R u+w vendor

counter='pub static NODES: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);'
bump='NODES.fetch_add(1, std::sync::atomic::Ordering::Relaxed);'

m=vendor/min2phase-0.2.4/src/lib.rs
printf '\n%s\n' "$counter" >> "$m"
perl -0pi -e "s/(fn phase1\([^{]*\{)/\$1 $bump/; s/(fn phase2\([^{]*\{)/\$1 $bump/" "$m"

printf '\n%s\n' "$counter" >> vendor/rcuber-0.7.20/src/lib.rs
perl -0pi -e "s/(pub fn apply_move\(self, move_name: Move\) -> Self \{)/\$1 crate::$bump/" \
    vendor/rcuber-0.7.20/src/cubie.rs

n=$(grep -c 'NODES.fetch_add' "$m" vendor/rcuber-0.7.20/src/cubie.rs | awk -F: '{s+=$2} END {print s}')
[ "$n" = 3 ] || { echo "expected 3 counter sites, found $n" >&2; exit 1; }
echo "vendored with node counters"
