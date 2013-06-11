use nalgebra::traits::norm::Norm;
use nalgebra::traits::workarounds::scalar_op::ScalarMul;
use geom::implicit::Implicit;

/**
 * Implicit description of a ball geometry.
 * 
 *  - `N`: numeric type used for the ball radius.
 *  - `V`: type of the ball center. Typically a vector.
 */
#[deriving(Eq, ToStr)]
pub struct Ball<N, V>
{
  priv center: V,
  priv radius: N
}

impl<N: Copy, V: Copy> Ball<N, V>
{
  /**
   * Creates a new ball from its radius and center.
   */
  pub fn new(&center: &V, &radius: &N) -> Ball<N, V>
  { Ball { center: center, radius: radius } }

  /**
   * The ball radius.
   */
  pub fn radius(&self) -> N
  { self.radius }

  /**
   * The ball center.
   */
  pub fn center(&self) -> V
  { self.center }
}

impl<N, V: Norm<N> + ScalarMul<N>> Implicit<V> for Ball<N, V>
{
  fn support_point(&self, dir: &V) -> V
  { dir.normalized().scalar_mul(&self.radius) }
}
