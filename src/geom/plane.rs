use std::num::Bounded;
use nalgebra::traits::rlmul::RMul;
use nalgebra::traits::delta_transform::DeltaTransformVector;
use nalgebra::traits::transformation::{Transformation, Transformable};
use bounding_volume::aabb::AABB;
use bounding_volume::has_bounding_volume::HasBoundingVolume;

/**
 * Implicit description of a plane.
 *
 *   - `V`: type of the plane center and normal.
 */
#[deriving(Eq, ToStr)]
pub struct Plane<V>
{
  priv center: V,
  priv normal: V
}

impl<V> Plane<V>
{
  /// Builds a new plane from its center and its normal.
  #[inline(always)]
  pub fn new(center: V, normal: V) -> Plane<V>
  { Plane { center: center, normal: normal } }
}


impl<V: Copy> Plane<V>
{
  /// The plane normal.
  #[inline(always)]
  pub fn normal(&self) -> V
  { copy self.normal }

  /// The plane center.
  #[inline(always)]
  pub fn center(&self) -> V
  { copy self.center }
}

impl<V, M: RMul<V> + DeltaTransformVector<V>> Transformation<M> for Plane<V>
{
  #[inline(always)]
  fn transformation(&self) -> M
  { fail!("Not yet implemented") } // deduce a transformation from the normal

  #[inline(always)]
  fn transform_by(&mut self, transform: &M)
  {
    self.center = transform.rmul(&self.center);
    self.normal = transform.delta_transform_vector(&self.normal);
  }
}

impl<V, M: RMul<V> + DeltaTransformVector<V>> Transformable<M, Plane<V>> for Plane<V>
{
  #[inline(always)]
  fn transformed(&self, transform: &M) -> Plane<V>
  { Plane::new(transform.rmul(&self.center),
               transform.delta_transform_vector(&self.normal)) }
}

// FIXME: these is something bad here…
// Since we cannot implement HasBoundingVolume twice, we wont be able to
// implement any other bounding volume… That’s bad.
impl<V: Bounded + Neg<V> + Ord + Copy>
    HasBoundingVolume<AABB<V>> for Plane<V>
{
  fn bounding_volume(&self) -> AABB<V>
  { AABB::new(&-Bounded::max_value::<V>(), &Bounded::max_value()) }
}
