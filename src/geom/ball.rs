use std::num::One;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::scalar_op::{ScalarMul, ScalarAdd, ScalarSub};
use nalgebra::traits::rlmul::RMul;
use nalgebra::traits::transformation::{Transformation, Transformable};
use nalgebra::traits::translation::Translation;
use geom::implicit::Implicit;
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

impl<N, V> Ball<N, V>
{
  /**
   * Creates a new ball from its radius and center.
   */
  #[inline]
  pub fn new(center: V, radius: N) -> Ball<N, V>
  { Ball { center: center, radius: radius } }
}

impl<N: Copy, V: Copy> Ball<N, V>
{
  /**
   * The ball radius.
   */
  #[inline]
  pub fn radius(&self) -> N
  { copy self.radius }

  /**
   * The ball center.
   */
  #[inline]
  pub fn center(&self) -> V
  { copy self.center }
}

impl<N, V: Norm<N> + ScalarMul<N> + Add<V, V>> Implicit<V> for Ball<N, V>
{
  #[inline]
  fn support_point(&self, dir: &V) -> V
  { self.center + dir.normalized().scalar_mul(&self.radius) }
}

impl<V: Copy + Add<V, V>, N, M: One + Translation<V> + RMul<V>> Transformation<M> for Ball<N, V>
{
  #[inline]
  fn transformation(&self) -> M
  {
    let mut res = One::one::<M>();

    res.translate(&self.center);

    res
  }

  #[inline]
  fn transform_by(&mut self, m: &M)
  { self.center = m.rmul(&self.center) }
}

impl<N: Copy, V: Copy, M: RMul<V>> Transformable<M, Ball<N, V>> for Ball<N, V>
{
  #[inline]
  fn transformed(&self, transform: &M) -> Ball<N, V>
  { Ball::new(transform.rmul(&self.center), copy self.radius) }
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
