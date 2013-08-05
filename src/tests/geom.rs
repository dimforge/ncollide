#[test]
use std::num::{Zero};
#[test]
use std::cmp::ApproxEq;
#[test]
use nalgebra::vec::Vec3;
#[test]
use nalgebra::traits::norm::Norm;
#[test]
use geom::ball::Ball;
#[test]
use geom::implicit::Implicit;
#[test]
use geom::minkowski_sum::MinkowskiSum;
#[test]
use geom::reflection::Reflection;
#[test]
use geom::convex_polytope::ConvexPolytope;

#[test]
fn test_ball_support_function()
{
    let dir  = &Vec3::new(1f64, 1f64, 1f64);
    let ball = Ball::new(Zero::zero(), 42f64);
    let diag = 42f64 / dir.norm();

    assert!(ball.support_point(dir).approx_eq(&Vec3::new(diag, diag, diag)));
}

#[test]
fn test_convex_polytope_support_function()
{
    let dir    = &Vec3::new(1f64, 1f64, 1f64);
    let bestpt = Vec3::new(2f64, 2f64, 0f64);
    let pts    = @[bestpt, Vec3::new(-2f64, -2f64, -0f64)];
    let poly   = ConvexPolytope::new(pts);

    assert!(poly.support_point(dir).approx_eq(&bestpt));
}

#[test]
fn test_minkowski_sum_support_function()
{
    let dir  = Vec3::new(1f64, 1f64, 1f64);
    let ball = Ball::new(Zero::zero::<Vec3<f64>>(), 42f64);
    let diag = 2.0f64 * 42f64 / dir.norm();

    let msum = MinkowskiSum::new(&ball, &ball);

    assert!(msum.support_point(&dir).approx_eq(&Vec3::new(diag, diag, diag)));
}

#[test]
fn test_reflection_support_function()
{
    let dir    = &Vec3::new(1f64, 1f64, 1f64);
    let pts    = @[Vec3::new(2f64, 2f64, 2f64), Vec3::new(-20f64, -20f64, -20f64)];
    let poly   = ConvexPolytope::new::<Vec3<f64>, f64>(pts);

    assert!(Reflection::new(&poly).support_point(dir)
            .approx_eq(&Vec3::new(20f64, 20f64, 20f64)));
}
