#[test]
use std::num::Zero;
#[test]
use nalgebra::vec::Vec3;
#[test]
use nalgebra::traits::scalar_op::ScalarAdd;
#[test]
use narrow::ball_ball::*;
#[test]
use geom::ball::Ball;

#[test]
fn ball_ball_detection()
{
  let cb1: Vec3<f64> = Zero::zero();
  let cb2            = cb1.scalar_add(&1.0);

  let c1 = collide_ball_ball(&Ball::new(cb1, 1.5f64), &Ball::new(cb2, 1.0f64));
  let c2 = collide_ball_ball(&Ball::new(cb1, 0.4f64), &Ball::new(cb2, 0.4f64));

  assert!(c1.is_some());
  assert!(c2.is_none());
}
