#!/bin/sh
# Compare two revisions of the crate, interleaved to cancel background-load
# drift:
#   prof/ab.sh REV_A REV_B [ROUNDS] [N] [PRESETS...]
# A revision is anything `git rev-parse` takes, or `.` for the working tree.
# Each is built (in a worktree under target/ab) with this tree's
# examples/abtime.rs, so both run the same harness; prints per-preset minima
# and medians (ns per solve). EXAMPLE=abbatch times batches instead (both
# revisions need `solve_batch`).
set -e
command -v cargo >/dev/null || . "$HOME/.cargo/env"
cd "$(dirname "$0")/.."
CRATE=$(pwd)
EXAMPLE=${EXAMPLE:-abtime}
A=$1; B=$2; ROUNDS=${3:-15}; N=${4:-20000}
shift 4 2>/dev/null || shift $#
PRESETS=${*:-fast short shorter}
OUT=$CRATE/target/ab
mkdir -p "$OUT"
REPO=$(git rev-parse --show-toplevel)
SUB=${CRATE#"$REPO"/}
for v in A B; do
    if [ $v = A ]; then rev=$A; else rev=$B; fi
    if [ "$rev" = . ]; then
        dir=$CRATE
    else
        tree=$OUT/tree_$v
        git -C "$REPO" worktree remove --force "$tree" 2>/dev/null || true
        git -C "$REPO" worktree add -q --detach "$tree" "$rev"
        dir=$tree/$SUB
        cp "$CRATE/examples/$EXAMPLE.rs" "$dir/examples/$EXAMPLE.rs"
    fi
    (cd "$dir" && cargo build -q --release --example "$EXAMPLE")
    cp "$dir/target/release/examples/$EXAMPLE" "$OUT/abtime_$v"
    [ "$rev" = . ] || git -C "$REPO" worktree remove --force "$OUT/tree_$v"
done
: > "$OUT/A.txt"
: > "$OUT/B.txt"
for _ in $(seq "$ROUNDS"); do
    "$OUT/abtime_A" "$N" $PRESETS >> "$OUT/A.txt"
    "$OUT/abtime_B" "$N" $PRESETS >> "$OUT/B.txt"
done
for v in A B; do
    printf '%s' $v
    i=1
    for p in $PRESETS; do
        printf '   %s min %s med %s' "$p" "$(cut -d' ' -f$i "$OUT/$v.txt" | sort -g | head -1)" \
            "$(cut -d' ' -f$i "$OUT/$v.txt" | sort -g | sed -n "$((ROUNDS / 2 + 1))p")"
        i=$((i + 1))
    done
    echo
done
