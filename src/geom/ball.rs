use nalgebra::traits::norm::Norm;
use nalgebra::traits::scalar_op::{ScalarMul, ScalarAdd, ScalarSub};
use nalgebra::traits::rlmul::RMul;
use geom::implicit::Implicit;
use geom::transformable::Transformable;
use bounding_volume::aabb::AABB;
use bounding_volume::has_bounding_volume::HasBoundingVolume;

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
  #[inline(always)]
  pub fn new(&center: &V, &radius: &N) -> Ball<N, V>
  { Ball { center: center, radius: radius } }

  /**
   * The ball radius.
   */
  #[inline(always)]
  pub fn radius(&self) -> N
  { copy self.radius }

  /**
   * The ball center.
   */
  #[inline(always)]
  pub fn center(&self) -> V
  { copy self.center }
}

impl<N, V: Norm<N> + ScalarMul<N> + Add<V, V>> Implicit<V> for Ball<N, V>
{
  #[inline(always)]
  fn support_point(&self, dir: &V) -> V
  { self.center + dir.normalized().scalar_mul(&self.radius) }
}

impl<N: Copy, V: Copy, M: RMul<V>> Transformable<M, Ball<N, V>> for Ball<N, V>
{
  #[inline(always)]
  fn transformed(&self, transform: &M) -> Ball<N, V>
  { Ball::new(&transform.rmul(&self.center), &self.radius) }

  #[inline(always)]
  fn transform_to(&self, transform: &M, out: &mut Ball<N, V>)
  { out.center = transform.rmul(&self.center); }
}

// FIXME: these is something bad here…
// Since we cannot implement HasBoundingVolume twice, we wont be able to
// implement any other bounding volume… That’s bad.
impl<N, V: ScalarAdd<N> + ScalarSub<N> + Ord + Copy>
    HasBoundingVolume<AABB<V>> for Ball<N, V>
{
  fn bounding_volume(&self) -> AABB<V>
  {
    AABB::new(&self.center.scalar_sub(&self.radius),
              &self.center.scalar_add(&self.radius))
  }
}
