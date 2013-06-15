use nalgebra::traits::scalar_op::{ScalarAdd, ScalarSub};
use nalgebra::traits::rlmul::RMul;
use nalgebra::traits::delta_transform::DeltaTransformVector;
use bounding_volume::aabb::AABB;
use bounding_volume::has_bounding_volume::HasBoundingVolume;
use geom::transformable::Transformable;
use geom::ball;
use geom::plane;

/**
 * Enumeration grouping all common shapes. Used to simplify collision detection
 * dispatch.
 */
#[deriving(Eq, ToStr)]
pub enum DefaultGeom<N, V> {
  Plane(plane::Plane<V>),
  Ball(ball::Ball<N, V>)
}

impl<N, V> DefaultGeom<N, V>
{
  /**
   * Convenience method to extract a ball from the enumation. Fails if the
   * pattern `Ball` is not matched.
   */
  #[inline(always)]
  pub fn ball<'r>(&'r self) -> &'r ball::Ball<N, V>
  {
    match *self {
        Ball(ref b) => b,
        _ => fail!("Unexpected geometry: this is not a ball.")
    }
  }

  /**
   * Mutable version of `ball`.
   */
  #[inline(always)]
  pub fn ball_mut<'r>(&'r mut self) -> &'r mut ball::Ball<N, V>
  {
    match *self {
        Ball(ref mut b) => b,
        _ => fail!("Unexpected geometry: this is not a ball.")
    }
  }

  /**
   * Convenience method to extract a plane from the enumation. Fails if the
   * pattern `Plane` is not matched.
   */
  #[inline(always)]
  pub fn plane<'r>(&'r self) -> &'r plane::Plane<V>
  {
    match *self {
        Plane(ref p) => p,
        _ => fail!("Unexpected geometry: this is not a plane.")
    }
  }

  /**
   * Mutable version of `plane`.
   */
  #[inline(always)]
  pub fn plane_mut<'r>(&'r mut self) -> &'r mut plane::Plane<V>
  {
    match *self {
        Plane(ref mut p) => p,
        _ => fail!("Unexpected geometry: this is not a plane.")
    }
  }
}

impl<N: Copy, V: Copy, M: RMul<V> + DeltaTransformVector<V>>
Transformable<M, DefaultGeom<N, V>> for DefaultGeom<N, V>
{
  #[inline(always)]
  fn transformed(&self, transform: &M) -> DefaultGeom<N, V>
  { 
    match *self
    {
      Plane(ref p) => Plane(p.transformed(transform)),
      Ball(ref b)  => Ball(b.transformed(transform))
    }
  }

  #[inline(always)]
  fn transform_to(&self, transform: &M, out: &mut DefaultGeom<N, V>)
  { 
    match *self
    {
      Plane(ref p) => p.transform_to(transform, out.plane_mut()),
      Ball(ref b)  => b.transform_to(transform, out.ball_mut())
    }
  }
}

// FIXME: these is something bad here…
// Since we cannot implement HasBoundingVolume twice, we wont be able to
// implement any other bounding volume… That’s bad.
impl<N, V: Bounded + Neg<V> + ScalarAdd<N> + ScalarSub<N> + Ord + Copy>
    HasBoundingVolume<AABB<V>> for DefaultGeom<N, V>
{
  fn bounding_volume(&self) -> AABB<V>
  {
    match *self
    {
      Plane(ref p) => p.bounding_volume(),
      Ball(ref b)  => b.bounding_volume()
    }
  }
}
