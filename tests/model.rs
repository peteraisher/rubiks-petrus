#![allow(clippy::needless_range_loop)]
use petrus::cube::*;
use petrus::facelet::*;

fn random_seq(rng: &mut Rng, n: usize) -> Vec<u8> {
    (0..n).map(|_| (rng.next_u64() % 18) as u8).collect()
}

#[test]
fn geometry_matches_kociemba_tables() {
    // Published cubie tables from Kociemba's two-phase reference implementation.
    let u = MOVE_CUBIE[0];
    assert_eq!(u.cp, [3, 0, 1, 2, 4, 5, 6, 7]);
    assert_eq!(u.co, [0; 8]);
    assert_eq!(u.ep, [3, 0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11]);
    let r = MOVE_CUBIE[3];
    assert_eq!(r.cp, [4, 1, 2, 0, 7, 5, 6, 3]);
    assert_eq!(r.co, [2, 0, 0, 1, 1, 0, 0, 2]);
    assert_eq!(r.ep, [8, 1, 2, 3, 11, 5, 6, 7, 4, 9, 10, 0]);
    assert_eq!(r.eo, [0; 12]);
    let f = MOVE_CUBIE[6];
    assert_eq!(f.cp, [1, 5, 2, 3, 0, 4, 6, 7]);
    assert_eq!(f.co, [1, 2, 0, 0, 2, 1, 0, 0]);
    assert_eq!(f.ep, [0, 9, 2, 3, 4, 8, 6, 7, 1, 5, 10, 11]);
    assert_eq!(f.eo, [0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0]);
}

#[test]
fn face_turns_have_order_four() {
    for m in 0..18 {
        let mut c = Cube::SOLVED;
        let p = m % 3 + 1;
        let order = if p == 2 { 2 } else { 4 };
        for i in 1..=order {
            c = c.apply_move(m);
            assert_eq!(c.is_solved(), i == order, "move {m} power {i}");
        }
    }
}

#[test]
fn simd_matches_reference_model() {
    let mut rng = Rng::new(1);
    for _ in 0..2000 {
        let q = random_seq(&mut rng, 40);
        let mut k = Cubie::SOLVED;
        for &m in &q {
            k = k.mul(&MOVE_CUBIE[m as usize]);
        }
        let c = Cube::from_seq(&q);
        assert!(c.is_valid());
        assert_eq!(c.to_cubie(), k);
        assert_eq!(Cube::from_cubie(&k), c);
        // facelet model agrees too
        let mut f = SOLVED_FACELETS;
        for &m in &q {
            for _ in 0..(m % 3 + 1) {
                f = apply_perm(&f, &face_turn_perm((m / 3) as usize));
            }
        }
        assert_eq!(cubie_to_facelets(&k), f);
        assert_eq!(facelets_to_cubie(&f), k);
    }
}

#[test]
fn known_orders_and_superflip() {
    let sexy = parse_seq("R U R' U'").unwrap();
    let mut c = Cube::SOLVED;
    for i in 1..=6 {
        c = c.apply_seq(&sexy);
        assert_eq!(c.is_solved(), i == 6);
    }
    let sf = parse_seq("U R2 F B R B2 R U2 L B2 R U' D' R2 F R' L B2 U2 F2").unwrap();
    let k = Cube::from_seq(&sf).to_cubie();
    assert_eq!(k.ep, Cubie::SOLVED.ep);
    assert_eq!(k.eo, [1; 12]);
    assert_eq!(k.cp, Cubie::SOLVED.cp);
    assert_eq!(k.co, [0; 8]);
}

#[test]
fn compose_inverse_random() {
    let mut rng = Rng::new(2);
    for _ in 0..500 {
        let a = Cube::random(&mut rng);
        let b = Cube::random(&mut rng);
        assert!(a.compose(&a.inverse()).is_solved());
        let q = random_seq(&mut rng, 20);
        assert_eq!(a.apply_seq(&q), a.compose(&Cube::from_seq(&q)));
        assert_eq!(a.compose(&b).to_cubie(), a.to_cubie().mul(&b.to_cubie()));
        assert!(Cube::from_seq(&q).apply_seq(&seq_inverse(&q)).is_solved());
    }
}

#[test]
fn eo12_matches_scalar() {
    let mut rng = Rng::new(3);
    for _ in 0..1000 {
        let c = Cube::random(&mut rng);
        let k = c.to_cubie();
        let mut m = 0;
        for p in 0..12 {
            m |= (k.eo[p] as u32) << p;
        }
        assert_eq!(c.eo12(), m);
    }
}

#[test]
fn facelet_strings_round_trip() {
    let mut rng = Rng::new(4);
    assert_eq!(Cube::SOLVED.to_facelets(), "UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB");
    for _ in 0..200 {
        let c = Cube::random(&mut rng);
        assert_eq!(Cube::from_facelets(&c.to_facelets()).unwrap(), c);
    }
    assert!(Cube::from_facelets("UUU").is_err());
    let mut bad = Cube::SOLVED.to_facelets().into_bytes();
    bad.swap(8, 9); // twist a corner sticker pair: invalid
    assert!(Cube::from_facelets(std::str::from_utf8(&bad).unwrap()).is_err());
}

#[test]
fn symmetries_form_a_group_and_map_moves() {
    assert_eq!(SYM_IS_MIRROR.iter().filter(|&&m| m).count(), 24);
    assert!(!SYM_IS_MIRROR[0]);
    let mut rng = Rng::new(5);
    for s in 0..NSYM {
        // conj is a homomorphism and moves map as advertised
        for _ in 0..20 {
            let x = Cube::random(&mut rng);
            let q = random_seq(&mut rng, 15);
            let lhs = x.conj(s).apply_seq(&q);
            let mapped: Vec<u8> = q.iter().map(|&m| SYM_MOVE[s][m as usize]).collect();
            let rhs = x.apply_seq(&mapped).conj(s);
            assert_eq!(lhs, rhs, "sym {s}");
            assert_eq!(x.conj(s), x.conj_reference(s), "fast conj, sym {s}");
        }
        assert!(Cube::SOLVED.conj(s).is_solved());
    }
    let x = Cube::random(&mut rng);
    assert_eq!(x.conj(0), x);
}

#[test]
fn seq_push_cancels() {
    let mut q = parse_seq("R U").unwrap();
    seq_push(&mut q, 0); // U -> U2
    assert_eq!(seq_to_string(&q), "R U2");
    seq_append(&mut q, &parse_seq("D U2").unwrap()); // U2 D U2 -> D
    assert_eq!(seq_to_string(&q), "R D");
    seq_append(&mut q, &parse_seq("D' R'").unwrap());
    assert!(q.is_empty());
    assert_eq!(parse_seq("R2' U3").unwrap(), parse_seq("R2 U'").unwrap());
    assert!(parse_seq("X").is_err());
}
