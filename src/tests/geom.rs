use std::cmp::ApproxEq;
use na::Vec3;
use na;
use geom::{Ball, MinkowskiSum};
use implicit::Implicit;
use math::{Scalar, Point, Vect};

#[test]
fn test_ball_support_function() {
    let dir  = &Vec3::new(1f64, 1f64, 1f64);
    let ball = Ball::new(42f64);
    let diag = 42f64 / na::norm(dir);

    assert!(na::approx_eq(&ball.support_point(&Vec3::new(0.0f64, 0.0, 0.0), dir), &Vec3::new(diag, diag, diag)));
}

#[test]
fn test_minkowski_sum_support_function() {
    let dir  = Vec3::new(1f64, 1f64, 1f64);
    let ball = Ball::new(42f64);
    let diag = 2.0f64 * 42f64 / na::norm(&dir);
    let _0v  = Vec3::new(0.0f64, 0.0, 0.0);

    let msum = MinkowskiSum::new(&_0v, &ball, &_0v, &ball);

    assert!(na::approx_eq(&msum.support_point(&na::identity(), &dir), &Vec3::new(diag, diag, diag)));
}
