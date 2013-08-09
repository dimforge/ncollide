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
use geom::minkowski_sum::NonTransformableMinkowskiSum;

#[test]
fn test_ball_support_function() {
    let dir  = &Vec3::new(1f64, 1f64, 1f64);
    let ball = Ball::new(42f64);
    let diag = 42f64 / dir.norm();

    assert!(ball.support_point(&Vec3::new(0.0f64, 0.0, 0.0), dir)
                .approx_eq(&Vec3::new(diag, diag, diag)));
}

#[test]
fn test_minkowski_sum_support_function() {
    let dir  = Vec3::new(1f64, 1f64, 1f64);
    let ball = Ball::new(42f64);
    let diag = 2.0f64 * 42f64 / dir.norm();
    let _0v  = Vec3::new(0.0f64, 0.0, 0.0);

    let msum = NonTransformableMinkowskiSum::new(&_0v, &ball, &_0v, &ball);

    assert!(msum.support_point(&_0v, &dir).approx_eq(&Vec3::new(diag, diag, diag)));
}
