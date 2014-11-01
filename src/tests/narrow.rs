use na::Vec3;
use na;
use narrow_phase::collide;
use shape::Ball;
use math::{Scalar, Point, Vect};

#[test]
fn ball_ball_detection() {
    let cb1: Vec3<f64> = na::zero();
    let cb2            = cb1 + 1.0f64;

    let c1 = collide::ball_ball(&cb1, &Ball::new(1.5f64), &cb2, &Ball::new(1.0f64), &na::zero());
    let c2 = collide::ball_ball(&cb1, &Ball::new(0.4f64), &cb2, &Ball::new(0.4f64), &na::zero());

    assert!(c1.is_some());
    assert!(c2.is_none());
}
