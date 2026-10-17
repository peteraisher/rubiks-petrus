"""Source-line profile from an xctrace Time Profiler export.

    python3 prof/lines.py TRACE.xml BINARY [TOP]

Maps each sample's leaf address to file:line (with inlining) via atos and
prints the hottest lines.
"""
import collections
import re
import subprocess
import sys

STATIC_BASE = 0x100000000
INSN_ALIGN = 3


def main():
    xml_path, binary = sys.argv[1], sys.argv[2]
    top = int(sys.argv[3]) if len(sys.argv) > 3 else 30
    xml = open(xml_path).read()
    frames = {fid: int(addr, 16) for fid, addr in re.findall(r'<frame id="(\d+)" name="[^"]*" addr="(0x[0-9a-f]+)"', xml)}
    load = {name: int(la, 16) for name, la in re.findall(r'<binary id="\d+" name="([^"]*)"[^>]*load-addr="(0x[0-9a-f]+)"', xml)}
    base = load[binary.split("/")[-1]]
    leaf_of_tag = {}
    for m in re.finditer(r'<tagged-backtrace id="(\d+)"[^>]*><backtrace id="\d+"><frame (?:id|ref)="(\d+)"', xml):
        leaf_of_tag[m.group(1)] = m.group(2)
    counts = collections.Counter()
    for m in re.finditer(r'<tagged-backtrace (?:id|ref)="(\d+)"', xml):
        leaf = leaf_of_tag.get(m.group(1))
        if leaf in frames:
            counts[(frames[leaf] & ~INSN_ALIGN) - base + STATIC_BASE] += 1
    addrs = sorted(counts)
    # with -i, atos prints the inlined frames innermost first, a blank line
    # after each address
    out = subprocess.run(["atos", "-i", "-o", binary, "-l", hex(STATIC_BASE)] + [hex(a) for a in addrs],
                         capture_output=True, text=True).stdout
    blocks = [b.splitlines() for b in out.strip("\n").split("\n\n")]
    per = collections.Counter()
    total = sum(counts.values())
    for a, block in zip(addrs, blocks):
        locs = [m.group(1) for m in (re.search(r'\(([^()]*:\d+)\)\s*$', ln) for ln in block) if m]
        # innermost location in our sources, plus its caller chain for context
        ours = [l for l in locs if l.startswith(("solver.rs", "chain.rs", "beam.rs", "merge.rs", "algs.rs", "tables.rs", "coord.rs", "cube.rs"))]
        key = " < ".join(ours[:3]) if ours else (locs[0] if locs else "?")
        per[(key, "")] += counts[a]
    for (loc, fn), c in per.most_common(top):
        print(f"{c:6d} {100 * c / total:5.1f}%  {loc:28s} {fn}")


main()
