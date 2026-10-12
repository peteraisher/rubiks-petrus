# Experiments log

Measured on an Apple M1 Pro (10 cores) under desktop background load;
"us" is per solve per thread. Lengths are half-turn metric.

| Change | Result | Kept? |
|---|---|---|
| `Vec`-based partials -> fixed `Moves` buffers, const-generic tables | fast 5.8 -> 2.8 us | yes |
| per-lane digit remap + nibble packing | 1 thread 2.8 -> 3.3 us, 10 threads 1.16M -> 2.08M solves/s | yes (memory, multicore) |
| pre-weighted digit tables (index = sum of independent loads) | 1 thread 3.6 -> 3.3 us | yes |
| build-time embedded tables | startup ~600 ms -> 0 | yes |
| best-bound-first beam expansion | neutral 17.8 -> 4.4 us, short 155 -> 82 us | yes |
| cap step depth by what could enter the beam | joint-48: 65.8K -> 6.7K nodes | yes |
| short-circuit table tests (small tables first) | EO 599 -> 389 ns, 2x2x3 1238 -> ~1040 ns | yes |
| software prefetch of all children's table entries | 2x2x3 1040 -> 1950 ns (eager child generation wastes work) | no |
| two extra 2x2x3 half tables (DF FL DB, DF FL BL) | 2x2x3 nodes 93 -> 36, step 1040 -> 645 ns | yes |
| lookahead ranking (moves so far + next-step bound) | short 32.7 -> 30.8 moves, at ~3x time | yes, switchable |
| consistent next-step bound when ordering step-1 candidates | no measurable change | no |
| lookahead pruning *inside* the step-1 search | short 30.1 -> 29.5 moves but 4x slower; a wider beam is the better trade | no |
