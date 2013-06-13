use std::vec;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::vector_space::VectorSpace;
use geom::ball::Ball;
use narrow::collision_detector::CollisionDetector;
use contact::contact;
use contact::contact::Contact;

/**
 * Collision detector between two balls.
 *
 *   - `C`: type of the collision.
 *   - `N`: type of a ball radius.
 *   - `V`: type of a ball center.
 */
pub struct BallBallCollisionDetector<C, N, V>
{
  priv contact: Option<@mut C>
}

impl<N: Real + Copy,
     C: Contact<V, N>,
     V: VectorSpace<N> + Norm<N> + Copy> 
    CollisionDetector<C, Ball<N, V>, Ball<N, V>> for
    BallBallCollisionDetector<C, N, V>
{
  #[inline(always)]
  fn new(_: &Ball<N, V>, _: &Ball<N, V>) -> BallBallCollisionDetector<C, N, V>
  { BallBallCollisionDetector{ contact: None } }

  fn update(&mut self, a: &Ball<N, V>, b: &Ball<N, V>)
  {
    match self.contact
    {
      None    => self.contact = collide_ball_ball(a, b).map(|&c| @mut c),
      Some(c) => if !(update_collide_ball_ball(a, b, c))
                 { self.contact = None }
    }
  }

  #[inline(always)]
  fn num_coll(&self) -> uint
  {
    match self.contact
    {
      None    => 0,
      Some(_) => 1
    }
  }

  #[inline(always)]
  fn colls(&mut self, out_colls: &mut ~[@mut C])
  {
    match self.contact
    {
      Some(c) => vec::push(out_colls, c),
      None    => ()
    }
  }
}

/**
 * Computes the collision between two balls. Returns whether they are
 * colliding.
 *
 *   - `b1`: first ball to test.
 *   - `b2`: second ball to test.
 *   - `out`: collision on which the result will be written.
 */
pub fn update_collide_ball_ball<V: VectorSpace<N> + Norm<N> + Copy,
                                N: Real + Copy,
                                C: Contact<V, N>>
   (b1: &Ball<N, V>, b2: &Ball<N, V>, out: &mut C) -> bool
{
  let r1         = b1.radius();
  let r2         = b2.radius();
  let delta_pos  = b2.center() - b1.center();
  let sqdist     = delta_pos.sqnorm();
  let sum_radius = r1 + r2;

  if (sqdist < sum_radius * sum_radius)
  {
    let normal = delta_pos.normalized();

    contact::set(out,
                 &(b1.center() + normal.scalar_mul(&r1)),
                 &(b2.center() - normal.scalar_mul(&r2)),
                 &normal,
                 &(sum_radius - sqdist.sqrt()));

    true
  }
  else
  { false }
}

/**
 * Same as `update_collide_ball_ball` but returns the collision or `None`.
 *
 *   - `b1`: first ball to test.
 *   - `b2`: second ball to test.
 */
pub fn collide_ball_ball<V: VectorSpace<N> + Norm<N> + Copy,
                         N: Real + Copy,
                         C: Contact<V, N>>
   (b1: &Ball<N, V>, b2: &Ball<N, V>) -> Option<C>
{
  let mut res = contact::zero::<V, N, C>();

  if (update_collide_ball_ball(b1, b2, &mut res))
  { Some(res) }
  else
  { None }
}
