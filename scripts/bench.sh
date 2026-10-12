#!/bin/sh
# Benchmark each preset single-threaded and on all cores. Background load
# on a desktop skews single runs, so each configuration runs REPEAT times and
# the best time is kept.
set -e
N=${N:-20000}
REPEAT=${REPEAT:-5}
BIN=${BIN:-target/release/petrus}
cargo build --release --quiet
best() { # args: threads, petrus options...
    t=$1; shift
    for _ in $(seq "$REPEAT"); do
        "$BIN" "$@" --bench "$N" -t "$t" 2>/dev/null | grep -E "length|time" | tr '\n' ' '
        echo
    done | sort -t, -k1 -g | awk -F'time: ' '{ split($2, a, " "); print a[1], $0 }' | sort -g | head -1 | cut -d' ' -f2-
}
for preset in classic fast neutral short shortest; do
    printf '%-8s 1 thread : %s\n' "$preset" "$(best 1 -p "$preset")"
done
printf '%-8s all cores: %s\n' fast "$(best "$(sysctl -n hw.ncpu 2>/dev/null || nproc)" -p fast)"
