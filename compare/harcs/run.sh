#!/bin/sh
# Build and run the HARCS-C timing harness: run.sh [N [SEED]] or run.sh -s "SCRAMBLE"
set -eu
cd "$(dirname "$0")"
[ -d vendor ] || ./fetch.sh
clang -O3 -w -pthread bench.c -o harcs-bench
cd vendor && ../harcs-bench Petrus.harcs "${@:-2000}" 2>/dev/null
