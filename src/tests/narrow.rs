#[test]
use std::num::Zero;
#[test]
use nalgebra::vec::{Vec3, ScalarAdd};
#[test]
use narrow::ball_ball;
#[test]
use geom::ball::Ball;

#[test]
fn ball_ball_detection() {
    let cb1: Vec3<f64> = Zero::zero();
    let cb2            = cb1.scalar_add(&1.0);

    let c1 = ball_ball::collide(&cb1, &Ball::new(1.5f64), &cb2, &Ball::new(1.0f64), &Zero::zero());
    let c2 = ball_ball::collide(&cb1, &Ball::new(0.4f64), &cb2, &Ball::new(0.4f64), &Zero::zero());

    assert!(c1.is_some());
    assert!(c2.is_none());
}
