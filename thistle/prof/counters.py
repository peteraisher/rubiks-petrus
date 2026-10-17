"""Top-down CPU cycle split of the solver, from Instruments' CPU Counters
(macOS, needs Xcode).

    python3 prof/counters.py [N] [PRESET] [ROUNDS] [same|distinct]

Runs examples/profile (N cubes, PRESET, ROUNDS, same|distinct) under `xctrace` and prints the
share of cycles that were useful work, back-end stalls (waiting on data or
execution units), front-end stalls (instruction delivery) and discarded work
(mispredicted branches).
"""
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CRATE = os.path.dirname(HERE)
OUT = os.path.join(CRATE, "target", "prof")
BASIS_POINTS = 100
XCTRACE_TIMEOUT_S = 180


def split(trace, env):
    xml = subprocess.run(["xcrun", "xctrace", "export", "--input", trace, "--xpath",
                          '/trace-toc/run[@number="1"]/data/table[@schema="CounterMetricAggregatedForProcess"]'],
                         env=env, capture_output=True, text=True, check=True).stdout
    # rows are [useful, back end, front end, discarded] in basis points; use the precise samples
    yes = re.search(r'<boolean id="(\d+)" fmt="Yes">1</boolean>', xml).group(1)
    rows = re.findall(r'<uint64-array[^>]*>([\d ]+)</uint64-array>'
                      r'(?:<boolean id="\d+" fmt="Yes">1</boolean>|<boolean ref="' + yes + r'"/>)', xml)
    vals = [v for v in (list(map(int, r.split())) for r in rows) if len(v) == 4]
    return [sum(v[i] for v in vals) / len(vals) / BASIS_POINTS for i in range(4)], len(vals)


def main():
    n = sys.argv[1] if len(sys.argv) > 1 else "20000"
    target = sys.argv[2] if len(sys.argv) > 2 else "fast"
    rounds = sys.argv[3] if len(sys.argv) > 3 else "20"
    mode = sys.argv[4] if len(sys.argv) > 4 else "distinct"
    env = dict(os.environ)
    env.setdefault("DEVELOPER_DIR", "/Applications/Xcode.app/Contents/Developer")
    subprocess.run(["cargo", "build", "-q", "--release", "--example", "profile"], cwd=CRATE, check=True)
    exe = os.path.join(CRATE, "target", "release", "examples", "profile")
    os.makedirs(OUT, exist_ok=True)
    trace = os.path.join(OUT, f"c-{target}-{mode}.trace")
    subprocess.run(["rm", "-rf", trace])
    try:  # xctrace occasionally hangs after the target exits
        subprocess.run(["xcrun", "xctrace", "record", "--template", "CPU Counters", "--output", trace,
                        "--launch", "--", exe, n, target, rounds, mode], env=env, capture_output=True, check=True,
                       timeout=XCTRACE_TIMEOUT_S)
    except subprocess.TimeoutExpired:
        print("xctrace timed out; try again")
        return
    (useful, back, front, discarded), samples = split(trace, env)
    print(f"target {target}: useful {useful:4.1f}%  back end {back:4.1f}%  front end {front:4.1f}%  "
          f"discarded {discarded:4.1f}%  ({samples} samples)")


if __name__ == "__main__":
    main()
