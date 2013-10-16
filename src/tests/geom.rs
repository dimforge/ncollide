use std::cmp::ApproxEq;
use nalgebra::na::Vec3;
use nalgebra::na;
use geom::{Ball, Implicit, MinkowskiSum};

#[test]
fn test_ball_support_function() {
    let dir  = &Vec3::new(1f64, 1f64, 1f64);
    let ball = Ball::new(42f64);
    let diag = 42f64 / na::norm(dir);

    assert!(ball.support_point(&Vec3::new(0.0f64, 0.0, 0.0), dir)
                .approx_eq(&Vec3::new(diag, diag, diag)));
}

#[test]
fn test_minkowski_sum_support_function() {
    let dir  = Vec3::new(1f64, 1f64, 1f64);
    let ball = Ball::new(42f64);
    let diag = 2.0f64 * 42f64 / na::norm(&dir);
    let _0v  = Vec3::new(0.0f64, 0.0, 0.0);

    let msum = MinkowskiSum::new(&_0v, &ball, &_0v, &ball);

    assert!(msum.support_point(&na::identity(), &dir).approx_eq(&Vec3::new(diag, diag, diag)));
}
