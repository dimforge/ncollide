#[test]
use std::rand;
#[test]
use nalgebra::traits::dim::Dim;
#[test]
use nalgebra::traits::scalar_op::ScalarSub;
#[test]
use nalgebra::traits::vector::AlgebraicVec;
#[test]
use nalgebra::vec::*;
#[test]
use narrow::algorithm::johnson_simplex::{JohnsonSimplex, RecursionTemplate};
#[test]
use narrow::algorithm::simplex::Simplex;
#[test]
use narrow::algorithm::gjk;
#[test]
use narrow::ball_ball;
#[test]
use narrow::algorithm::brute_force_simplex::BruteForceSimplex;
#[test]
use geom::ball::Ball;
#[test]
use geom::minkowski_sum::AnnotatedPoint;
#[test]
use geom::minkowski_sum;

macro_rules! test_johnson_simplex_impl(
  ($t: ty, $n: ty) => ( {
        let recursion = RecursionTemplate::new::<$t>();

        for  d in range(0, Dim::dim::<$t>() + 1) {
            for i in range(1u, 200 / (d + 1)) {
                // note that this fails with lower precision
                let mut v1: $t = rand::random();
                v1.scalar_sub_inplace(&(0.5 as $n));
                v1 = v1 * (i as $n);

                let mut splx1 = JohnsonSimplex::new(recursion, v1.clone());
                let mut splx2 = BruteForceSimplex::new(v1.clone());

                do d.times {
                    let mut v: $t = rand::random();
                    v.scalar_sub_inplace(&(0.5 as $n));
                    v = v * (i as $n);

                    splx1.add_point(v.clone());
                    splx2.add_point(v);
                }

                let proj2 = splx2.project_origin();
                let proj1 = splx1.project_origin();

                assert!(proj1.approx_eq(&proj2));
            }
        }
    }
  )
)

macro_rules! test_gjk_ball_ball_impl(
  ($t: ty, $n: ty) => ( {
        let recursion   = RecursionTemplate::new::<AnnotatedPoint<$t>>();

        do 200.times {
            let r1 = 10.0 as $n * rand::random();
            let r2 = 10.0 as $n * rand::random();

            let mut c1: $t = rand::random();
            c1.scalar_sub_inplace(&(0.5 as $n));
            c1 = c1 * (100.0 as $n);

            let mut c2: $t = rand::random();
            c2.scalar_sub_inplace(&(0.5 as $n));
            c2 = c2 * (100.0 as $n);

            let b1 = Ball::new(r1);
            let b2 = Ball::new(r2);

            let (p1, p2) = ball_ball::closest_points(&c1, &b1, &c2, &b2);

            // FIXME: a bit verbose…
            let cso_point   = minkowski_sum::cso_support_point(&c1, &b1, &c2, &b2, rand::random());
            let mut simplex: JohnsonSimplex<$n, AnnotatedPoint<$t>> = JohnsonSimplex::new(recursion, cso_point);

            let pts_johnson = gjk::closest_points(&c1, &b1, &c2, &b2, &mut simplex);

            match pts_johnson {
                Some((jp1, jp2)) => assert!(jp1.approx_eq(&p1) && jp2.approx_eq(&p2),
                "found: " + jp1.to_str() + " " + jp2.to_str()
                + " but expected: " + p1.to_str() + p2.to_str()),
                None => assert!((p1 - p2).norm() <= r1 + r2)
            }
        }
    }
  )
)

#[test]
fn test_gjk_ball_ball_1d() {
    test_gjk_ball_ball_impl!(Vec1<f64>, f64)
}

#[test]
fn test_gjk_ball_ball_2d() {
    test_gjk_ball_ball_impl!(Vec2<f64>, f64)
}

#[test]
fn test_gjk_ball_ball_3d() {
    test_gjk_ball_ball_impl!(Vec3<f64>, f64)
}

#[test]
fn test_gjk_ball_ball_4d() {
    test_gjk_ball_ball_impl!(Vec4<f64>, f64);
}

#[test]
fn test_gjk_ball_ball_5d() {
    test_gjk_ball_ball_impl!(Vec5<f64>, f64);
}

#[test]
fn test_gjk_ball_ball_6d() {
    test_gjk_ball_ball_impl!(Vec6<f64>, f64);
}

#[test]
fn test_johnson_simplex_1d() {
    test_johnson_simplex_impl!(Vec1<f64>, f64);
}

#[test]
fn test_johnson_simplex_2d() {
    test_johnson_simplex_impl!(Vec2<f64>, f64);
}

#[test]
fn test_johnson_simplex_3d() {
    test_johnson_simplex_impl!(Vec3<f64>, f64);
}

#[test]
fn test_johnson_simplex_4d() {
    test_johnson_simplex_impl!(Vec4<f64>, f64);
}

#[test]
fn test_johnson_simplex_5d() {
    test_johnson_simplex_impl!(Vec5<f64>, f64);
}

#[test]
fn test_johnson_simplex_6d() {
    test_johnson_simplex_impl!(Vec6<f64>, f64);
}
