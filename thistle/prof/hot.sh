#!/bin/sh
# Source-line profile of examples/profile (Instruments' Time Profiler, then
# prof/lines.py): prof/hot.sh [N] [PRESET] [ROUNDS] [same|distinct] [TOP]
set -e
command -v cargo >/dev/null || . "$HOME/.cargo/env"
cd "$(dirname "$0")/.."
export DEVELOPER_DIR=${DEVELOPER_DIR:-/Applications/Xcode.app/Contents/Developer}
cargo build -q --release --example profile
rm -rf target/prof/tp.trace
mkdir -p target/prof
xcrun xctrace record --template 'Time Profiler' --output target/prof/tp.trace --launch -- \
    target/release/examples/profile "${1:-20000}" "${2:-fast}" "${3:-200}" "${4:-distinct}" >/dev/null 2>&1
xcrun xctrace export --input target/prof/tp.trace \
    --xpath '/trace-toc/run[@number="1"]/data/table[@schema="time-profile"]' > target/prof/tp.xml
python3 prof/lines.py target/prof/tp.xml target/release/examples/profile "${5:-30}"
