use petrus::cube::*;
use petrus::facelet::{NSYM, SYM_MOVE};
use petrus::ll::{ll_index, ll_state, LlMode, LL_INDEX_SIZE, LL_STATES};
use petrus::solver::*;
use std::sync::OnceLock;

fn tables() -> &'static Tables {
    static T: OnceLock<Tables> = OnceLock::new();
    T.get_or_init(Tables::new)
}

/// Brute-force optimum (no heuristic) for a goal, up to `max` moves.
fn brute_force(c: &Cube, moves: &[u8], goal: fn(&Cube) -> bool, max: u8) -> Option<u8> {
    fn dfs(c: &Cube, moves: &[u8], goal: fn(&Cube) -> bool, togo: u8, last: u8) -> bool {
        if togo == 0 {
            return goal(c);
        }
        moves.iter().any(|&m| {
            let f = face(m);
            f != last && dfs(&c.apply_move(m as usize), moves, goal, togo - 1, f)
        })
    }
    (0..=max).find(|&d| dfs(c, moves, goal, d, NO_FACE))
}

#[test]
fn embedded_tables_match_fresh_bfs_and_combinatorics() {
    let t = tables();
    let t222 = T222.build::<false>();
    let t222_front = T222_FRONT.build::<false>();
    let t223_d = T223_D.build::<false>();
    let t223_l = T223_L.build::<false>();
    let t223_fd = T223_FD.build::<false>();
    let t223_fb = T223_FB.build::<false>();
    let teo = TEO.build::<true>();
    let tf2l = TF2L.build::<false>();
    // 1 corner (8*3) and 3 edges (12*2 * 11*2 * 10*2)
    assert_eq!(t222.reachable, 24 * 24 * 22 * 20);
    assert_eq!(t222_front.reachable, 24 * 24 * 22 * 20);
    // 2 corners (24*21) and 3 edges
    assert_eq!(t223_d.reachable, 24 * 21 * 24 * 22 * 20);
    assert_eq!(t223_l.reachable, 24 * 21 * 24 * 22 * 20);
    // <R,U,F,B> never touches the DL slot, so only 2^10 orientation masks
    // are reachable (the other 11 free bits minus DL's), times 2 corners
    assert_eq!(teo.reachable, 1024 * 24 * 21);
    // <R,U>: 2 corners among 6 slots (any twist) x 3 oriented edges among 7 slots
    assert_eq!(tf2l.reachable, 6 * 3 * 5 * 3 * 7 * 6 * 5);
    // and the per-lane remap packs <R,U> into 18 corner and 7 edge values
    assert_eq!(tf2l.size, 18 * 18 * 7 * 7 * 7);
    // God's number for a 2x2x2 block is 8 (half-turn metric)
    assert_eq!(t222.max_depth, 8);
    assert!(t.t222.same_data(&t222));
    assert!(t.t222_front.same_data(&t222_front));
    assert!(t.t223_d.same_data(&t223_d));
    assert!(t.t223_l.same_data(&t223_l));
    assert!(t.t223_fd.same_data(&t223_fd));
    assert!(t.t223_fb.same_data(&t223_fb));
    assert!(t.teo.same_data(&teo));
    assert!(t.tf2l.same_data(&tf2l));
}

/// States a few random moves from a step's goal; the step optimum must
/// match exhaustive search.
fn check_step_optimal<S: Step>(start: fn(&mut Rng) -> Cube, moves: &[u8], max_scramble: usize) {
    let t = tables();
    let mut rng = Rng::new(7);
    for _ in 0..60 {
        let mut c = start(&mut rng);
        let k = 1 + rng.below(max_scramble);
        for _ in 0..k {
            c = c.apply_move(moves[rng.below(moves.len())] as usize);
        }
        let mut nodes = 0;
        let sols = solve_step::<S>(t, &c, 1, 0, &mut nodes);
        assert!(S::goal(&c.apply_seq(&sols[0])));
        let bf = brute_force(&c, moves, S::goal, k as u8).unwrap();
        assert_eq!(sols[0].len() as u8, bf, "{} not optimal", S::NAME);
    }
}

fn solved_223(rng: &mut Rng) -> Cube {
    // random state with the 2x2x3 solved: scramble with <R,U> only
    (0..30).fold(Cube::SOLVED, |c, _| c.apply_move(RU_MOVES[rng.below(RU_MOVES.len())] as usize))
}

#[test]
fn steps_are_optimal_vs_brute_force() {
    check_step_optimal::<Block222>(|_| Cube::SOLVED, &ALL_MOVES, 5);
    check_step_optimal::<Expand223>(|_| Cube::SOLVED, &ALL_MOVES, 4);
    check_step_optimal::<EdgeOrient>(solved_223, &RUFB_MOVES, 5);
    check_step_optimal::<FirstTwoLayers>(|_| Cube::SOLVED, &RU_MOVES, 7);
}

#[test]
fn step_alternatives_are_valid_and_sorted() {
    let t = tables();
    let mut rng = Rng::new(8);
    for _ in 0..20 {
        let c = Cube::random(&mut rng);
        let mut nodes = 0;
        let sols = solve_step::<Block222>(t, &c, 10, 1, &mut nodes);
        assert!(sols.windows(2).all(|w| w[0].len() <= w[1].len()));
        assert!(sols.iter().all(|s| Block222::goal(&c.apply_seq(s))));
        assert!(sols.last().unwrap().len() <= sols[0].len() + 1);
    }
}

#[test]
fn last_layer_table_is_complete_and_correct() {
    let t = tables();
    let mut valid = 0;
    for idx in 0..LL_INDEX_SIZE {
        match (ll_state(idx), t.ll.alg(idx)) {
            (None, None) => {}
            (Some(c), Some(alg)) => {
                valid += 1;
                assert_eq!(ll_index(&c), Some(idx));
                assert!(c.apply_seq(alg).is_solved(), "LL case {idx}");
            }
            (s, a) => panic!("index {idx}: state {s:?} alg {a:?}"),
        }
    }
    assert_eq!(valid, LL_STATES);
}

#[test]
fn last_layer_modes_follow_their_steps() {
    let t = tables();
    let mut rng = Rng::new(9);
    for _ in 0..300 {
        let idx = rng.below(LL_INDEX_SIZE);
        let Some(c) = ll_state(idx) else { continue };
        for mode in [LlMode::Lars, LlMode::Coll, LlMode::Zbll] {
            let mut s = c;
            for (name, alg) in t.ll.solve(&c, mode) {
                s = s.apply_seq(alg);
                let k = s.to_cubie();
                let corners_placed = k.cp == Cubie::SOLVED.cp;
                let corners_done = corners_placed && k.co == Cubie::SOLVED.co;
                match name {
                    "CP" => assert!(corners_placed),
                    "CO" | "COLL" => assert!(corners_done),
                    _ => assert!(s.is_solved()),
                }
                assert!(ll_index(&s).is_some(), "F2L or EO broken by {name}");
            }
            assert!(s.is_solved());
        }
    }
}

/// Undo SYM_MOVE: the canonical-frame move behind an original-frame move.
fn to_canonical(sym: usize, m: u8) -> u8 {
    (0..NMOVES as u8).find(|&x| SYM_MOVE[sym][x as usize] == m).unwrap()
}

fn check_solution(c: &Cube, s: &Solution) {
    assert!(c.apply_seq(&s.moves).is_solved(), "solution does not solve");
    let total: usize = s.steps.iter().map(|x| x.moves.len()).sum();
    assert!(s.moves.len() <= total);
    let mut state = *c;
    for st in &s.steps {
        state = state.apply_seq(&st.moves);
        let canon = state.conj(s.sym);
        let faces: Vec<u8> = st.moves.iter().map(|&m| face(to_canonical(s.sym, m))).collect();
        match st.name {
            "2x2x2" => assert!(Block222::goal(&canon)),
            "2x2x3" => assert!(Expand223::goal(&canon)),
            "EO" => {
                assert!(EdgeOrient::goal(&canon));
                assert!(faces.iter().all(|f| [FACE_U, FACE_R, FACE_F, FACE_B].contains(f)));
            }
            "F2L" => {
                assert!(FirstTwoLayers::goal(&canon));
                assert!(faces.iter().all(|f| [FACE_U, FACE_R].contains(f)), "F2L must use <R,U>");
            }
            _ => assert!(ll_index(&canon).is_some()),
        }
    }
    assert!(state.is_solved());
}

#[test]
fn random_solves_all_presets() {
    let t = tables();
    let mut rng = Rng::new(10);
    let presets = [
        (Options::CLASSIC, 300),
        (Options::FAST, 300),
        (Options { ll: LlMode::Coll, ..Options::FAST }, 100),
        (Options { joint_blocks: true, ..Options::FAST }, 50),
        (Options::NEUTRAL, 100),
        (Options::SHORT, 20),
        (Options { ll: LlMode::Lars, ..Options::SHORT }, 10),
        (Options { joint_blocks: true, beam: 4, ..Options::SHORT }, 10),
        (Options::SHORTEST, 5),
    ];
    for (opts, n) in presets {
        let mut total = 0;
        for _ in 0..n {
            let c = Cube::random(&mut rng);
            let s = solve(t, &c, &opts);
            check_solution(&c, &s);
            total += s.moves.len();
        }
        eprintln!("{opts:?}: mean {:.2}", total as f64 / n as f64);
    }
}

#[test]
fn more_search_never_hurts() {
    // For each cube, wider options must not give a longer solution.
    let t = tables();
    let mut rng = Rng::new(11);
    for _ in 0..20 {
        let c = Cube::random(&mut rng);
        let fast = solve(t, &c, &Options::FAST).moves.len();
        let neutral = solve(t, &c, &Options { orientations: NSYM, beam: NSYM, ..Options::FAST });
        assert!(neutral.moves.len() <= fast);
    }
}

#[test]
fn special_inputs() {
    let t = tables();
    let s = solve(t, &Cube::SOLVED, &Options::CLASSIC);
    assert!(s.moves.is_empty());
    let sf = parse_seq("U R2 F B R B2 R U2 L B2 R U' D' R2 F R' L B2 U2 F2").unwrap();
    let c = Cube::from_seq(&sf);
    check_solution(&c, &solve(t, &c, &Options::SHORT));
    let c = Cube::from_facelets(&c.to_facelets()).unwrap();
    check_solution(&c, &solve(t, &c, &Options::CLASSIC));
}
