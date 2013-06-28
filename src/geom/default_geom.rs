use std::num::One;
use nalgebra::traits::scalar_op::{ScalarAdd, ScalarSub};
use nalgebra::traits::transformation::Transformation;
use nalgebra::traits::translation::Translation;
use nalgebra::traits::rotation::Rotate;
use nalgebra::traits::transformation::{Transform, Transformable};
use bounding_volume::aabb::AABB;
use bounding_volume::has_bounding_volume::HasBoundingVolume;
use geom::ball;
use geom::plane;

/**
 * Enumeration grouping all common shapes. Used to simplify collision detection
 * dispatch.
 */
#[deriving(Eq, ToStr)]
pub enum DefaultGeom<N, V, M, I> {
  Plane(plane::Plane<V>),
  Ball(ball::Ball<N, V>),
  Implicit(I)
}

impl<N,
     V,
     I,
     M: One + Transform<V> + Rotate<V>>
    DefaultGeom<N, V, M, I>
{
  pub fn new_plane<G: Transformable<M, plane::Plane<V>>>(geom: &G) -> DefaultGeom<N, V, M, I>
  { Plane(geom.transformed(&One::one())) }
}

impl<N,
     V: Copy + Add<V, V> + Neg<V>,
     I,
     M: One + Translation<V> + Transform<V>>
    DefaultGeom<N, V, M, I>
{
  pub fn new_ball<G: Transformable<M, ball::Ball<N, V>>>(geom: &G) -> DefaultGeom<N, V, M, I>
  { Ball(geom.transformed(&One::one())) }
}

impl<N, V, I, M> DefaultGeom<N, V, M, I>
{
  /**
   * Convenience method to extract a ball from the enumation. Fails if the
   * pattern `Ball` is not matched.
   */
  #[inline]
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
  #[inline]
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
  #[inline]
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
  #[inline]
  pub fn plane_mut<'r>(&'r mut self) -> &'r mut plane::Plane<V>
  {
    match *self {
        Plane(ref mut p) => p,
        _ => fail!("Unexpected geometry: this is not a plane.")
    }
  }

  #[inline]
  pub fn implicit<'r>(&'r self) -> &'r I
  {
    match *self {
        Implicit(ref i) => i,
        _ => fail!("Unexpected geometry: this is not an implicit.")
    }
  }

  /**
   * Mutable version of `implicit`.
   */
  #[inline]
  pub fn implicit_mut<'r>(&'r mut self) -> &'r mut I
  {
    match *self {
        Implicit(ref mut i) => i,
        _ => fail!("Unexpected geometry: this is not an implicit.")
    }
  }
}

// FIXME: these is something bad here…
// Since we cannot implement HasBoundingVolume twice, we wont be able to
// implement any other bounding volume… That’s bad.
impl<N,
     V: Bounded + Neg<V> + ScalarAdd<N> + ScalarSub<N> + Ord + Copy,
     M,
     I: HasBoundingVolume<AABB<V>>>
    HasBoundingVolume<AABB<V>> for DefaultGeom<N, V, M, I>
{
  fn bounding_volume(&self) -> AABB<V>
  {
    match *self
    {
      Plane(ref p)    => p.bounding_volume(),
      Ball(ref b)     => b.bounding_volume(),
      Implicit(ref i) => i.bounding_volume()
    }
  }
}

impl<N,
     V: Copy + Add<V, V> + Neg<V>,
     M: One + Translation<V> + Transform<V> + Rotate<V>,
     I: Transformation<M>>
Transformation<M> for DefaultGeom<N, V, M, I>
{
  fn transformation(&self) -> M
  {
    match *self
    {
      Plane(ref p)    => p.transformation(),
      Ball(ref b)     => b.transformation(),
      Implicit(ref i) => i.transformation(),
    }
  }

  fn inv_transformation(&self) -> M
  {
    match *self
    {
      Plane(ref p)    => p.inv_transformation(),
      Ball(ref b)     => b.inv_transformation(),
      Implicit(ref i) => i.inv_transformation(),
    }
  }


  fn transform_by(&mut self, m: &M)
  {
    match *self
    {
      Plane(ref mut p)    => p.transform_by(m),
      Ball(ref mut b)     => b.transform_by(m),
      Implicit(ref mut i) => i.transform_by(m),
    }
  }
}
