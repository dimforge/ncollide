use nalgebra::traits::norm::Norm;
use nalgebra::traits::vector_space::VectorSpace;
use nalgebra::traits::scalar_op::ScalarMul;
use geom::ball::Ball;
use narrow::collision_detector::CollisionDetector;
use contact::Contact;

/**
 * Collision detector between two balls.
 *
 * # Parameters:
 *   * `N` - type of a ball radius.
 *   * `V` - type of a ball center.
 */
pub struct BallBall<N, V>
{
  priv contact: Option<Contact<N, V>>
}

impl<N, V> BallBall<N, V>
{
  /// Creates a new persistant collision detector between two balls.
  #[inline]
  pub fn new() -> BallBall<N, V>
  { BallBall { contact: None } }
}

impl<N: Real + Clone,
     V: VectorSpace<N> + Norm<N> + Clone> 
    CollisionDetector<N, V, Ball<N, V>, Ball<N, V>> for
    BallBall<N, V>
{
  fn update(&mut self, a: &Ball<N, V>, b: &Ball<N, V>)
  {
    self.contact = collide_ball_ball(a, b);
  }

  #[inline]
  fn num_coll(&self) -> uint
  {
    match self.contact
    {
      None    => 0,
      Some(_) => 1
    }
  }

  #[inline]
  fn colls(&mut self, out_colls: &mut ~[Contact<N, V>])
  {
    match self.contact
    {
      Some(ref c) => out_colls.push(c.clone()),
      None        => ()
    }
  }
}

/// Computes the contact point between two balls. The balls must penetrate to have contact points.
pub fn collide_ball_ball<V: VectorSpace<N> + Norm<N> + Clone, N: Real + Clone>
   (b1: &Ball<N, V>, b2: &Ball<N, V>) -> Option<Contact<N, V>>
{
  let r1         = b1.radius();
  let r2         = b2.radius();
  let delta_pos  = b2.center() - b1.center();
  let sqdist     = delta_pos.sqnorm();
  let sum_radius = r1 + r2;

  if sqdist < sum_radius * sum_radius
  {
    let normal = delta_pos.normalized();

    Some(Contact::new(b1.center() + normal.scalar_mul(&r1),
                      b2.center() - normal.scalar_mul(&r2),
                      normal,
                      (sum_radius - sqdist.sqrt())))
  }
  else
  { None }
}

/// Computes the cloest points between two balls. If they are intersecting, the points
/// corresponding to the penetration depth are returned.
pub fn closest_points<N: Clone,
                      V: ScalarMul<N> + Sub<V, V> + Add<V, V> + Norm<N> + Clone>
       (b1: &Ball<N, V>, b2: &Ball<N, V>) -> (V, V)
{
  let r1     = b1.radius();
  let r2     = b2.radius();
  let normal = (b2.center() - b1.center()).normalized();

  (b1.center() + normal.scalar_mul(&r1), b2.center() - normal.scalar_mul(&r2))
}
