// Single-threaded timing harness for HARCS-C (vendor/main.c, unmodified).
// Builds the method's tables once, then solves each cube step by step the
// way HARCS' own `analyze` does (try candidate sequences shortest first,
// one hash-table lookup each, stop at the first hit), timing each step.
//
//   harcs-bench METHOD.harcs N [SEED]        N random states, mean per step
//   harcs-bench METHOD.harcs -s "R U F' ..." one scramble, median of REPEAT
//
// "nodes" = candidate sequences tried (each is a few moves applied plus one
// hash lookup).
#include "vendor/main.c"

#define MAX_STEPS 8
#define REPEAT 101
/// -s mode: repeat each timed solve until a sample lasts this long.
#define MIN_SAMPLE_NS 1e6

// CLOCK_MONOTONIC only has microsecond resolution on macOS.
static double now_ns(void) {
	return (double)clock_gettime_nsec_np(CLOCK_UPTIME_RAW);
}

struct step_ctx {
	struct STEP *step;
	uint64_t *cand;
	int n;
};

// HARCS' candidate list for a step: every canonical sequence up to the
// search depth, shortest first (as in analyze()).
static void candidates(struct step_ctx *s) {
	s->cand = malloc(estimateSequenceCount(s->step->movegroup, s->step->searchDepth) * sizeof(uint64_t));
	s->cand[0] = 0;
	int c = 1, start = 0, end = 1;
	for (char i = 0; i < s->step->searchDepth; i++) {
		for (int k = start; k < end; k++) addLayers(s->step->movegroup, s->cand[k], i, s->cand, &c);
		start = end;
		end = c;
	}
	s->n = c;
}

// Whether the step's pieces are solved: the empty sequence hits the table.
static int solved(struct step_ctx *s, const struct CUBE *cube) {
	struct CUBE c = *cube;
	applyMaskScrambled(&c, s->step->EPCOmask, s->step->CPEOCNmask);
	uint64_t *out = htRetrieve(s->step->table, hash(s->step->Nbits, &c), &c.EPCO, &c.CPEOCN);
	return out && *out == 0;
}

// Solve one step in place: returns candidates tried, or -1 if none hit.
static int solve(struct step_ctx *s, struct CUBE *cube, uint64_t *sol1, uint64_t *sol2) {
	struct CUBE masked = *cube, c1;
	applyMaskScrambled(&masked, s->step->EPCOmask, s->step->CPEOCNmask);
	for (int j = 0; j < s->n; j++) {
		c1 = masked;
		applySequence(s->cand[j], &c1);
		uint64_t *out = htRetrieve(s->step->table, hash(s->step->Nbits, &c1), &c1.EPCO, &c1.CPEOCN);
		if (out) {
			*sol1 = s->cand[j];
			*sol2 = reverseSequence(*out);
			applySequence(*sol1, cube);
			applySequence(*sol2, cube);
			return j + 1;
		}
	}
	return -1;
}

static void apply_string(struct CUBE *cube, const char *seq) {
	char tmp[1024];
	strncpy(tmp, seq, sizeof tmp - 1);
	tmp[sizeof tmp - 1] = 0;
	for (char *tok = strtok(tmp, " "); tok; tok = strtok(NULL, " ")) {
		int found = 0;
		for (int i = 0; i < 54; i++)
			if (strcmp(tok, moveList[i]) == 0) {
				applyMove(cube, i + 1);
				found = 1;
			}
		if (!found) {
			fprintf(stderr, "bad move %s\n", tok);
			exit(2);
		}
	}
}

static int cmp_double(const void *a, const void *b) {
	double x = *(const double *)a, y = *(const double *)b;
	return (x > y) - (x < y);
}

int main(int argc, char **argv) {
	if (argc < 3) {
		fprintf(stderr, "usage: %s METHOD.harcs N [SEED] | %s METHOD.harcs -s SCRAMBLE\n", argv[0], argv[0]);
		return 2;
	}
	nThreads = 1;
	struct METHOD method = {0};
	// table building prints progress; keep it off stdout
	FILE *saved = stdout;
	stdout = stderr;
	double t0 = now_ns();
	if (!parseInputFile(argv[1], &method)) {
		fprintf(stderr, "cannot read %s\n", argv[1]);
		return 2;
	}
	stdout = saved;
	fprintf(stderr, "\ntables built in %.0f ms\n", (now_ns() - t0) / 1e6);

	struct step_ctx steps[MAX_STEPS];
	int nsteps = 0;
	for (struct STEP *s = method.first; s; s = s->next) {
		steps[nsteps].step = s;
		candidates(&steps[nsteps++]);
	}

	if (strcmp(argv[2], "-s") == 0) {
		struct CUBE start;
		revertCube(&start);
		apply_string(&start, argv[3]);
		struct CUBE cube = start;
		double total = 0;
		printf("{\"steps\": [");
		for (int k = 0; k < nsteps; k++) {
			struct CUBE before = cube;
			uint64_t s1, s2;
			int nodes = solve(&steps[k], &cube, &s1, &s2);
			double v[REPEAT];
			int iters = 1;
			for (;;) {
				double a = now_ns();
				for (int it = 0; it < iters; it++) {
					struct CUBE c = before;
					solve(&steps[k], &c, &s1, &s2);
				}
				if (now_ns() - a >= MIN_SAMPLE_NS) break;
				iters *= 2;
			}
			for (int r = 0; r < REPEAT; r++) {
				double a = now_ns();
				for (int it = 0; it < iters; it++) {
					struct CUBE c = before;
					solve(&steps[k], &c, &s1, &s2);
				}
				v[r] = (now_ns() - a) / iters;
			}
			qsort(v, REPEAT, sizeof v[0], cmp_double);
			total += v[REPEAT / 2];
			printf("%s\n  {\"name\": \"%s\", \"moves\": \"%s%s\", \"ns\": %.0f, \"nodes\": %d}", k ? "," : "", steps[k].step->name,
			       readableSequence(s1), readableSequence(s2), v[REPEAT / 2], nodes);
		}
		printf("\n], \"total_ns\": %.0f}\n", total);
		return 0;
	}

	int n = atoi(argv[2]);
	srand(argc > 3 ? atoi(argv[3]) : 1);
	double ns[MAX_STEPS] = {0}, nodes[MAX_STEPS] = {0}, moves[MAX_STEPS] = {0};
	int fails[MAX_STEPS] = {0}, wrong = 0;
	for (int i = 0; i < n; i++) {
		struct CUBE cube;
		revertCube(&cube);
		randomizeCube(&cube);
		for (int k = 0; k < nsteps; k++) {
			uint64_t s1, s2;
			double a = now_ns();
			int got = solve(&steps[k], &cube, &s1, &s2);
			ns[k] += now_ns() - a;
			if (got < 0) {
				fails[k]++;
				break;
			}
			// this and every earlier step must now be solved
			for (int e = 0; e <= k; e++) wrong += !solved(&steps[e], &cube);
			nodes[k] += got;
			moves[k] += countMoves(&s1) + countMoves(&s2);
		}
	}
	double total = 0, mtotal = 0;
	printf("%d random states, one thread\n", n);
	for (int k = 0; k < nsteps; k++) {
		int ok = n;
		for (int e = 0; e <= k; e++) ok -= fails[e];
		printf("%-6s %6.2f moves %10.0f nodes %12.0f ns  (%d not found)\n", steps[k].step->name, moves[k] / ok, nodes[k] / ok,
		       ns[k] / n, fails[k]);
		total += ns[k] / n;
		mtotal += moves[k] / n;
	}
	printf("total  %6.2f moves %23.0f ns  (%d failed checks)\n", mtotal, total, wrong);
	return 0;
}
