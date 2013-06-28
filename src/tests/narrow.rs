#[test]
use std::num::Zero;
#[test]
use nalgebra::vec::Vec3;
#[test]
use nalgebra::traits::scalar_op::ScalarAdd;
#[test]
use contact::geometric_contact::GeometricContact;
#[test]
use narrow::ball_ball::*;
#[test]
use geom::ball::Ball;

#[test]
fn ball_ball_detection()
{
  type V = Vec3<f64>;
  type N = f64;
  type C = Option<GeometricContact<V, N>>;

  let cb1 = Zero::zero::<V>();
  let cb2 = cb1.scalar_add(&1.0);

  let c1: C = collide_ball_ball(&Ball::new(cb1, 1.5f64),
                                &Ball::new(cb2, 1.0f64));
  let c2: C = collide_ball_ball(&Ball::new(cb1, 0.4f64),
                                &Ball::new(cb2, 0.4f64));

  assert!(c1.is_some());
  assert!(c2.is_none());
}
