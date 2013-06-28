use nalgebra::traits::inv::Inv;
use nalgebra::traits::rotation::Rotate;
use nalgebra::traits::transformation::{Transformation, Transformable};
use geom::implicit::Implicit;

/**
 * Implicit representation of a geometry transformed by an affine
 * transformation.
 *
 *   - `G`: type of the shape being transformed.
 *   - `M`: type of the transformation.
 */
pub struct TransformedRef<'self, G, M, N>
{
  priv t: M,
  priv g: &'self G
}

pub struct Transformed<G, M, N>
{
  priv t: M,
  priv g: G
}

// FIXME: is there a way to not have two different structs for the pointer and
// the non-pointer version?

// implementations for TransformedRef
impl<'self, G, M, N> TransformedRef<'self, G, M, N>
{
  /// Creates a transformed geometry from a transform.
  #[inline]
  pub fn new(transform: M, geometry: &'self G) -> TransformedRef<'self, G, M, N>
  { TransformedRef { t: transform, g: geometry } }
}

impl<'self,
     G: Implicit<V>,
     M: Rotate<V>,
     V: Copy,
     N>
Implicit<V> for TransformedRef<'self, G, M, N>
{
  #[inline]
  fn support_point(&self, dir: &V) -> V
  { self.t.rotate(&self.g.support_point(&self.t.inv_rotate(dir))) }
}

impl<'self, G, M: Mul<M, M> + Inv + Copy, N>
Transformation<M> for TransformedRef<'self, G, M, N>
{
  #[inline]
  fn transformation(&self) -> M
  { copy self.t }

  #[inline]
  fn inv_transformation(&self) -> M
  { self.t.inverse() }


  #[inline]
  fn transform_by(&mut self, transform: &M)
  { self.t = transform * self.t; }
}

// this might do some very interesting structural optimizations
impl<'self,
     G: Transformable<M, Res>,
     M: Mul<M, M>,
     Res: Transformation<M>,
     N>
Transformable<M, Res> for TransformedRef<'self, G, M, N>
{
  #[inline]
  fn transformed(&self, transform: &M) -> Res
  { self.g.transformed(&(transform * self.t)) }
}

// implementations for Transformed
impl<G, M, N> Transformed<G, M, N>
{
  /// Creates a transformed geometry from a transform.
  #[inline]
  pub fn new(transform: M, geometry: G) -> Transformed<G, M, N>
  { Transformed { t: transform, g: geometry } }
}

impl<G: Implicit<V>,
     M: Rotate<V>,
     V,
     N>
Implicit<V> for Transformed<G, M, N>
{
  #[inline]
  fn support_point(&self, dir: &V) -> V
  { self.t.rotate(&self.g.support_point(&self.t.inv_rotate(dir))) }
}

impl<G, M: Mul<M, M> + Inv + Copy, N>
Transformation<M> for Transformed<G, M, N>
{
  #[inline]
  fn transformation(&self) -> M
  { copy self.t }

  #[inline]
  fn inv_transformation(&self) -> M
  { self.t.inverse() }

  #[inline]
  fn transform_by(&mut self, transform: &M)
  { self.t = transform * self.t; }
}

// this might do some very interesting structural optimizations
impl<G: Transformable<M, Res>,
     M: Mul<M, M>,
     Res: Transformation<M>,
     N>
Transformable<M, Res> for Transformed<G, M, N>
{
  #[inline]
  fn transformed(&self, transform: &M) -> Res
  { self.g.transformed(&(transform * self.t)) }
}

/*
// FIXME: these is something bad here…
// Since we cannot implement HasBoundingVolume twice, we wont be able to
// implement any other bounding volume… That’s bad.
impl<'self,
     G: Implicit<V>,
     M,
     V: Basis + Dot<N> + Copy>
HasBoundingVolume<AABB<V>> for Transformed<G, M, N>
{
  fn bounding_volume(&self) -> AABB<V>
  {
    let vres = Zero::zero::<V>();
    let res  = Basis::orthonormal_basis::<V>().iter();

    for res.transform |basis|
    { basis.dot(self.support_point(basis)); }

    // FIXME: cant do that due to limitations:
    // FromAnyIterator::from_iterator(&mut res)
    // instead, we do:

    for res.zip(vres.mut_iter()).advance |(in, out)|
    { *out = in }

    vres
  }
}
*/
