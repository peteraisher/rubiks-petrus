# thistle

A Rubik's cube solver built to beat this repo's Petrus solver on both axes:
Thistlethwaite's four nested subgroups, with every phase compiled into
tables of optimal algorithms, and a small beam over starting orientations
and stored alternatives. Dependency-free Rust (it reuses petrus' cube
model), NEON on aarch64 with a scalar fallback.

```
$ thistle -p shorter R U2 F\' L D2 B R\' U F2 D\' L2 B\' R2 U\' F L\' D B2 R F\'
  phases: EO 4, CO+E 6, G3 6, squares 6
  total 22 moves (HTM): R B D2 F D' L R U' L' U R B2 D2 R' D2 R B2 D2 L2 B2 D2 R2
```

(The scramble of petrus' README: petrus `fast` takes 42 moves, petrus
`short` 32.)

| solver | mean length (HTM) | time / solve |
|---|---|---|
| petrus `fast`, steps searched (petrus until compiled steps) | 38.91 | 1764 ns |
| petrus `fast`, compiled steps (petrus now, identical solutions) | 38.91 | 446 ns |
| petrus `neutral` (best of 48 orientations) | 37.88 | 1876 ns |
| thistle `fast` | 30.90 | 147 ns |
| thistle `fast5` (phase 2 split in two) | 34.95 | 136 ns |
| thistle `fast6` (phases 2 and 4 split) | 38.20 | 156 ns |
| thistle `short` | 25.49 | 1213 ns |
| thistle `shorter` | 24.00 | 2340 ns |
| thistle `shortest` | 22.69 | 10.5 µs |

Apple M1 Pro, one thread, the same 20 000 random states for every row,
interleaved in one process, best of 11 rounds
(`cargo run --release --example compiled_petrus`), under background
load (load average ~10-16 on 10 cores). The searched row is the step searches
alone; petrus `fast` as shipped before compiled steps took 2.2 µs on a
quiet machine. For scale, two-phase (min2phase 0.2.4) reaches 21.3 moves
in ~520 µs and petrus' own beam (`short`) 30.2 moves in ~165 µs.

Against the original question (petrus `fast`, 2.2 µs, 38.9 moves): **8
fewer moves at ~7x the speed**, or **38% fewer moves (24.0) at about the
same speed**. Against petrus as it is now, with its steps compiled the same
way (section 5): thistle `fast` is shorter by 8 moves and ~3x as
fast, and at `neutral`'s time thistle `shorter` gives 24.0 moves instead
of 37.9.

A first solve in a fresh process takes ~15 µs in process (petrus: ~0.4 ms,
nearly all of it unpacking its last-layer table); see
[First-solve latency](#first-solve-latency).

## How the approach was found

### 1. The cost model

A dependent load costs ~1.6 ns from L1 (≤128 KB), 6-7 ns from L2
(≤4 MB), 13 ns at 8 MB and 50-130 ns beyond 16 MB (DRAM); independent loads
overlap ~8x. petrus `fast` spent its 2.2 µs searching ~160 nodes
(2x2x2 369 ns, 2x2x3 722, EO 412, F2L 337, LL ~30).

### 2. Every step's case space is small

Once the earlier steps are done, a step's starting state is tiny: Petrus'
2x2x3 expansion depends only on DLF, DF and FL (21 x 18 x 16 = 6048
cases), EO only on the flips of seven edges (64), F2L on 56 700 <R,U>
states, the 2x2x2 on 253 440. The search inside a step may be expensive,
but its answer can be tabulated per case: a solve becomes one lookup per
step. That makes the method's choice a question of how few phases, how
small their case spaces, and how few moves each phase costs on average.

### 3. Comparing methods by exact per-phase optimal lengths

A prototype computed each candidate phase's distance table by BFS over its
coset space and solved random cubes phase by phase:

| method, each phase optimal | phases (cases) | mean length |
|---|---|---|
| Petrus + ZBLL (= petrus `fast`) | 2x2x2 (253 440), 2x2x3 (6048), EO (64), F2L (56 700), LL (7776) | 38.9 |
| Thistlethwaite | EO (2048), corner twist + E-slice (1 082 565), corner coset + M-slice (29 400), squares group (663 552) | 30.9 |

Petrus spends its moves late: 12.7 moves on 7776 LL cases, 5 on 64 EO
cases, because late phases must restore everything before them.
Thistlethwaite's early phases carry most of the information cheaply.

### 4. Choosing among solutions is cheap when a phase is a lookup

With each phase a lookup, a whole solution costs ~4 lookups, so many can be
tried. Best of N complete greedy solutions:

| starts | Thistlethwaite | Petrus |
|---|---|---|
| 1 | 30.87 | 38.76 |
| 48 symmetries | 25.27 | 31.69 |
| 96 (and the inverse) | 24.50 | 30.63 |

(`examples/petrus_chains.rs` for Petrus.) Ties between equally short phase
solutions matter as much as orientation: the 8 symmetries that keep the
axes give different solutions only through tie-breaking, and they take
27.7 to 25.3. A beam that stores several *spread-out* optimal algorithms per
case and ranks partial solutions by moves so far plus the next phase's
exact distance does better per evaluation: 24.0 moves from 144 evaluated
algorithms, against 24.5 from 384 for 96 greedy chains. Alternatives one
move longer than optimal did not help (23.85 -> 23.9-24.0).

### 5. The control: compile Petrus too

Is thistle fast because of the method or because of the compilation? The
same compilation applied to Petrus (first as `examples/compiled_petrus.rs`,
now in petrus itself: `src/compiled.rs`) stores every `fast` step's search
result per case (2x2x2 by state, the expansion and EO from the cases above,
F2L by <R,U> state) and returns **identical solutions**. It took petrus
`fast` from 2.2 µs to ~0.45 µs: compilation alone is worth ~5x for the
solver as shipped (~8x for a bare chain of lookups, 285 ns). thistle `fast`
is still faster (309 ns, four phases against five) with 8 fewer moves, and
the method shows in the beam: at equal work Thistlethwaite's chains are ~6
moves shorter than Petrus'.

## Design

### Phases and coordinates

In Kociemba's orientation conventions (F/B quarter turns flip edges):

| phase | moves | coordinate | cases | diameter | mean |
|---|---|---|---|---|---|
| 1: G0 -> G1 | all 18 | edge orientation | 2048 | 7 | 4.61 |
| 2: G1 -> G2 | <U,D,R,L,F2,B2> | corner twist x E-slice placement | 1 082 565 | 10 | 7.80 |
| 3: G2 -> G3 | <U,D,R2,L2,F2,B2> | corner coset x M-slice placement | 29 400 | 13 | 8.81 |
| 4: G3 -> 1 | half turns | the element of the squares group | 663 552 | 15 | 10.12 |

Each coordinate is read off petrus' lane vectors (piece -> position and
orientation) with a few NEON instructions:

- twist: one `TBL` per byte of a per-value weight, `ADDLV`;
- placements: a `TBL` from value to position bit, masked to the pieces,
  `ADDV`;
- permutations need no Lehmer code. In G3 every corner stays in its tetrad
  and every edge in its slice, so a permutation of a group of four is fixed
  by which of its 6 pairs are inverted: two `TBL` shuffles, one compare,
  `AND` with bit weights, `ADDV` give two 6-bit order patterns per vector.
  The squares group's 96 corner permutations are identified by the two
  tetrads' patterns, and a G2 corner coset (of 420) is the placement of one
  tetrad (70) times the orbit of the pattern pair under the group (6). Both
  are 4 KB tables, checked exhaustively by the generator.

### Tables

Built by `build.rs` from BFS over each phase (~8 s on 10 cores) and
embedded, 10.5 MB in all:

- distance per case (bytes), for ranking;
- per case, up to 16 / 4 / 16 / 1 optimal algorithms for phases 1-4, packed
  in a `u64` (move slots of the phase's move set). They are spread evenly
  over *all* optimal canonical algorithms of the case in lexicographic
  order, so they differ early, not just in their last moves: the number of
  optimal paths from every (case, previous face) is counted by dynamic
  programming over the distance table, and evenly spaced ranks are
  unranked. They are stored in van der Corput order, so any first k are
  spread too;
- phases 2-4 store algorithms only for symmetry classes. The 8 symmetries
  that keep each axis preserve the whole chain, and the 16 that keep the
  U/D axis preserve G2 and G3, so phase 2 is reduced by 8 and phases 3-4
  by 16. A case's index splits into a minor part (E-slice placement,
  M-slice placement, G3 edge arrangement) reduced to its class with a
  (class, symmetry) table, and a major part (twist, corner coset, G3
  corner permutation) conjugated with a per-symmetry table. The
  algorithm's moves are mapped back through the symmetry. 18 MB -> 7.1 MB
  of algorithms (phase 4's 2.0 MB -> 0.5 MB);
- each row's first alternative again, alone, for `fast` (1.5 MB): reading
  only those, the chain's working set is ~2 MB instead of ~8 MB;
- the split phases' four small tables (77 KB, below).

### Search

`fast` is a straight chain: four (coordinate, lookup, apply) steps,
written so that no branch depends on the cube. A mispredicted branch costs
more than its own refetch here: it also discards the next solve, which
the core has already started (with no unpredictable branch, consecutive
solves overlap in the out-of-order window). So:

- an algorithm's moves are unpacked with vector shuffles (phase 1's
  moves as bytes, the others' as nibbles mapped through the symmetry with
  one `TBL`);
- it is applied as a fixed number of move pairs per phase (its longest
  algorithm's), padded with an identity "move" that the pair tables
  include: the loop's trip count no longer depends on the cube;
- the last phase is not applied (nothing reads the solved cube);
- turns are merged across phase boundaries with a 6.9 KB table indexed by
  (second-last move, last move, new move) and the last 8 moves held in a
  register, three moves per boundary, then the rest of the algorithm is
  one 16-byte store; the solution is copied out at a fixed size. Forcing
  the merge's five outcomes into selects removed the last mispredictions
  but was slower: each push then waited for the previous one's table
  load, where the predicted branch (mostly "append") runs ahead.

Mispredicted work went from 29% of cycles to 9%, and `fast` from ~190 to
~145 ns. With turns not merged at all (wrong lengths, measured only to
bound this), it was ~115 ns.

The other presets run a beam:

- starts: the 6 symmetries that order the axes differently, each on the
  cube and on its inverse (12);
- each phase extends every kept partial by its stored alternatives, ranked
  by merged length plus the next phase's exact distance; the best `width`
  are kept, and only the best partial gets phase 4;
- children are generated without data-dependent branches (a missing
  alternative is evaluated as the first one and ranked last) and selected
  afterwards: the cutoff key by vector counts, survivors compacted with
  conditional increments. Mispredicted work fell from 27% to 12% of cycles;
- a kept partial's next row of alternatives is computed and prefetched
  (`prfm`) at selection, so it is in L1 when the next phase expands it;
- algorithms are applied two moves per `TBL` pair (a 20 KB table of all
  move pairs), and an odd last move or the identity always as one more
  lookup, so a length's parity costs no branch;
- the ranking merges only adjacent turns of one face; the partials tied for
  the best key are finished with exact merging and the shortest kept.

Presets (`-s` starts, `-b` width, `-a` alternatives for phases 1-3):

| preset | starts | width | alternatives | evaluations | mean |
|---|---|---|---|---|---|
| `fast` | 1 | 1 | 1, 1, 1 | 4 | 30.90 |
| `fast5` | 1 | 1 | 1, 1, 1, phase 2 split | 5 | 34.95 |
| `fast6` | 1 | 1 | 1, 1, 1, phases 2 and 4 split | 6 | 38.20 |
| `short` | 4 | 4 | 4, 2, 8 | 56 | 25.50 |
| `shorter` | 12 | 8 | 4, 4, 8 | 144 | 24.01 |
| `shortest` | 12 | 32 | 16, 4, 16 | 832 | 22.69 |

`examples/sweep.rs` measures the length/time frontier over these settings.

### Smaller tables: more phases

Before the first-alternative tables and the 16-symmetry reduction,
`fast` took ~105 ns with its tables in cache and ~215 ns on distinct cubes
(`examples/hot.rs`), so memory was at most ~50% of it (most of the
remaining gap turned out to be mispredicted branches, see above).
Smaller tables need more phases. `examples/splits.rs` measures every
split of a phase at an intermediate set exactly (BFS, each half optimal,
20 000 random cubes; mean phase lengths before merging):

| split | cases (whole phase) | extra moves |
|---|---|---|
| phases 1-3 via a subgroup generated by fewer moves (e.g. <U,D,R,L2,F2,B2>) | none smaller: each generates the whole start group | |
| phase 4 via <U2,R2,D2,L2> (order 192) | 3456 + 192 (663 552) | +1.82 |
| phase 4 via <U2,R2,F2> | 256 + 2592 | +3.65 |
| phase 4: corners, then edges restoring them | 96 + 6912 | +3.76 |
| phase 4: edges, then corners | 6912 + 96 | +6.88 |
| phase 2: E-slice, then twist restoring it | 495 + 2187 (1 082 565) | +4.43 |
| phase 2: twist, then E-slice | 2187 + 495 | +4.86 |
| phase 3: M-slice, then corners | 70 + 420 (29 400) | +3.67 |

Thistlethwaite's subgroups have no move-generated groups between them
except inside the squares group, so phases 2 and 3 can only split by
fixing one coordinate and then restoring it. Fewer phases is out: merging
any two adjacent phases gives 10^9-10^10 cases.

`fast5` and `fast6` compile the restore splits of phases 2 and 4 (their
coordinates exist already; the phase-4 subgroup split would need a new
coset coordinate). They replace phase 2's 1.4 MB and phase 4's 0.5 MB
with 77 KB: every table `fast6` reads fits in L2. Steady state, it does
not pay much: interleaved, `fast` 147, `fast5` 136, `fast6` 156 ns. The
time saved on misses goes on the extra phase and the 3.5-7 extra moves.
The splits pay mainly while the tables are paging in (below).

## First-solve latency

The third axis, besides length and throughput: one solve in a fresh
process. The tables are embedded and used in place, so there is no
set-up; what is left is page faults, ~1 µs per 16 KB page from the file
cache (`examples/firstsolve.rs`, medians of 9 fresh processes):

| solver | first solve | its faults | first 100 solves | their faults |
|---|---|---|---|---|
| thistle `fast` | 14 µs | 15 | 121 µs | 98 |
| thistle `fast5` | 13 µs | 14 | 62 µs | 46 |
| thistle `fast6` | 13 µs | 14 | 48 µs | 20 |
| thistle `short` | 55 µs | 54 | 0.6 ms | 423 |
| petrus `fast` | 0.3 ms set-up + 7-10 µs | 42 + 6 | 0.17-0.26 ms | 135 |

A first `fast` solve is about a dozen faults (code, the small tables and
one page per lookup), whatever the chain. Whole process, spawn to exit,
300 fresh processes (`examples/coldrun.rs`, binary in the file cache): hello world 1.80 ms, this binary doing nothing 1.95 ms,
thistle `fast` 1.93, `fast6` 1.96, `shorter` 2.21, petrus `fast` 2.38.
So thistle's first solve is within ~0.15 ms of the floor, most of it the
binary's size (33 MB here with both crates' tables).

The first launch after installing (a binary the OS has not yet checked)
is dominated by the OS: ~150 ms for a 0.5 MB hello world, plus ~7.5 ms per
MB of binary (measured with padded hello worlds); thistle's 11.7 MB
binary takes ~250 ms, and that run's faults cost ~20 µs each (`fast`'s
first solve 0.28 ms, `fast6`'s 0.21 ms).

### Code

- `src/coord.rs`: the phases, their coordinates (NEON with a scalar
  fallback), symmetries and the algorithm encoding; shared with `build.rs`;
- `src/gen.rs`: table generation (BFS, symmetry reduction, spread
  alternatives, the split halves), run by `build.rs`;
- `src/tables.rs`: the embedded tables and lookups by case;
- `src/algs.rs`: unpacking stored algorithms and applying them;
- `src/merge.rs`: merging turns across phase boundaries (the beam's
  estimate and exact merge, the chain's table-driven one);
- `src/chain.rs`: the single chain (`fast`, `fast5`, `fast6`);
- `src/beam.rs`: the beam (`short` and longer);
- `src/solver.rs`: options, presets, `Solution` and `solve`.

## Correctness

`cargo test --release`:

- every embedded table (distances, symmetry tables, algorithms) is
  byte-identical to a fresh generation, and each phase's diameter is the
  known 7 / 10 / 13 / 15;
- for random states of each phase's group, every stored alternative is
  optimal, distinct, uses only the phase's moves and lands in the next
  group (checked from the pieces, not the coordinates);
- every preset and a grid of option combinations solve random cubes;
  `fast` equals the greedy chain of first alternatives; solved, single-move
  and superflip states;
- the split halves reach their intermediate goals (from the pieces), and
  `fast5` / `fast6` are the chains of them;
- on 100 000 random states, `fast`, `fast5` and `fast6` merge turns
  exactly as petrus' general merge does;
- the generator asserts its own premises: coordinates are onto, minor and
  major parts conjugate independently, order patterns identify the squares
  group's corner permutations and split the cosets exactly 420 ways;
- the suite also passes on the scalar fallback
  (`--target x86_64-apple-darwin`, under Rosetta);
- `tests/lint_magic_numbers.rs` as in petrus.

## Performance log

Interleaved A/B of two git revisions with `prof/ab.sh REV_A REV_B` (`.` is
the working tree; minimum over rounds, ns per solve);
profiles with `prof/hot.sh` (`prof/lines.py`: source lines through
inlining) and `prof/counters.py` (top-down cycle split).

| change | effect |
|---|---|
| first version: full tables (18 MB of algorithms), sorted insertion | fast 1295 ns, short (12, 8, 8/1/8) 4.8 µs |
| no clearing of the beam's levels; merge checks only at phase boundaries | fast 1073 -> ~400 ns |
| symmetry-reduced algorithm tables (18 -> 4.5 MB) | fast distinct cubes 420 -> 250 ns |
| NEON coordinates, order patterns instead of Lehmer codes (80 KB of rank tables gone) | short 4.1 -> 3.4 µs |
| move-pair tables, skip children that cannot enter the beam | small |
| branch-free child generation and selection | discarded cycles 27% -> 13%; short -6% |
| prefetch kept partials' rows; branch-free odd tail | short -11%, shorter -20% |
| straight chain for `fast` | 415-580 -> ~300 ns |
| 4 stored phase-2 alternatives | 24.58 -> 24.07 moves at fewer evaluations |
| exact merge for the final tie | 24.07 -> 24.01 |
| first-alternative tables for `fast` | fast 271 -> 226 ns |
| phases 3-4 reduced by 16 symmetries, edges as phase 4's minor part (alg4 2.0 -> 0.5 MB) | fast 226 -> ~195 ns |
| vector unpacking, fixed-count application, no last apply | fast ~195 -> ~190 ns, mispredicted work 29% -> 16% |
| boundary merging by table, window in a register, fixed-size copy-out | fast ~190 -> ~145 ns, mispredicted 9% |

Tried and dropped:

- alternatives one move longer than optimal (model: no gain);
- sorting each kept level by key (tie order): no change in length;
- exact boundary merging in the ranking: no change in length;
- fixed-count move application padded with the identity: within noise in
  the beam (in the single chain it removes a mispredicted loop exit per
  phase, kept);
- merging the chain's turns with forced selects (no branch at all): ~20%
  slower than the predicted table-driven merge (see Search); merging once
  at the end instead of per phase: no gain;
- compact child records with the survivors' cubes recomputed: slower.

## Usage

```
cargo build --release
target/release/thistle [-p PRESET] R U F2 D' ...      # solve a scramble
target/release/thistle -f UUUUUUUUURRR...             # 54-char facelets
target/release/thistle -p shorter --bench 20000       # same states as petrus --bench
cargo run --release --example compiled_petrus         # the comparison table above
cargo run --release --example sweep                   # length / time frontier
cargo run --release --example splits                  # five- and six-phase chains
cargo run --release --example firstsolve -- thistle-fast   # first solves in this process
cargo run --release --example coldrun -- 300 target/release/examples/cold thistle-fast
```

As a library: `thistle::solver::{Tables, solve, Options}` with petrus'
`Cube`.

## Limitations

- Half-turn metric only.
- The first solves after start-up page in the tables they touch (see
  [First-solve latency](#first-solve-latency)); the throughput numbers
  above are steady state.
- The two-phase regime (~21 moves) belongs to two-phase solvers: this
  beam's longest preset is 22.7 moves at ~10 µs.
