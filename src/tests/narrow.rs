#[test]
use std::num::Zero;
#[test]
use nalgebra::vec::Vec3;
#[test]
use narrow::collide;
#[test]
use geom::Ball;

#[test]
fn ball_ball_detection() {
    let cb1: Vec3<f64> = Zero::zero();
    let cb2            = cb1 + 1.0f64;

    let c1 = collide::ball_ball(&cb1, &Ball::new(1.5f64), &cb2, &Ball::new(1.0f64), &Zero::zero());
    let c2 = collide::ball_ball(&cb1, &Ball::new(0.4f64), &cb2, &Ball::new(0.4f64), &Zero::zero());

    assert!(c1.is_some());
    assert!(c2.is_none());
}
