use nalgebra::traits::rlmul::{RMul, LMul};
use nalgebra::traits::delta_transform::DeltaTransform;
use nalgebra::traits::transformation::{Transformation, Transformable};
use geom::implicit::Implicit;

/**
 * Implicit representation of a geometry transformed by an affine
 * transformation.
 *
 *   - `G`: type of the shape being transformed.
 *   - `M`: type of the transformation.
 */
pub struct TransformedRef<'self, G, M>
{
  priv t: M,
  priv g: &'self G
}

pub struct Transformed<G, M>
{
  priv t: M,
  priv g: G
}

// FIXME: is there a way to not have two different structs for the pointer and
// the non-pointer version?

// implementations for TransformedRef
impl<'self, G, M> TransformedRef<'self, G, M>
{
  /// Creates a transformed geometry from a transform.
  #[inline]
  pub fn new(transform: M, geometry: &'self G) -> TransformedRef<'self, G, M>
  { TransformedRef { t: transform, g: geometry } }
}

impl<'self,
     G:  Implicit<V>,
     M:  DeltaTransform<DT>,
     DT: RMul<V> + LMul<V>,
     V:  Copy>
Implicit<V> for TransformedRef<'self, G, M>
{
  #[inline]
  fn support_point(&self, dir: &V) -> V
  {
    let dt = self.t.delta_transform();

    // FIXME: will dt get inlined, preventing implicit copying the matrix?
    dt.lmul(&self.g.support_point(&dt.rmul(dir)))
  }
}

impl<'self, G, M: Mul<M, M> + Copy> Transformation<M> for TransformedRef<'self, G, M>
{
  #[inline]
  fn transformation(&self) -> M
  { copy self.t }

  #[inline]
  fn transform_by(&mut self, transform: &M)
  { self.t = transform * self.t; }
}

// this might do some very interesting structural optimizations
impl<'self,
     G: Transformable<M, Res>,
     M: Mul<M, M>,
     Res: Transformation<M>>
Transformable<M, Res> for TransformedRef<'self, G, M>
{
  #[inline]
  fn transformed(&self, transform: &M) -> Res
  { self.g.transformed(&(transform * self.t)) }
}

// implementations for Transformed
impl<G, M> Transformed<G, M>
{
  /// Creates a transformed geometry from a transform.
  #[inline]
  pub fn new(transform: M, geometry: G) -> Transformed<G, M>
  { Transformed { t: transform, g: geometry } }
}

impl<G:  Implicit<V>,
     M:  DeltaTransform<DT>,
     DT: RMul<V> + LMul<V>,
     V>
Implicit<V> for Transformed<G, M>
{
  #[inline]
  fn support_point(&self, dir: &V) -> V
  {
    let dt = self.t.delta_transform();

    // FIXME: will dt get inlined, preventing implicit copying the matrix?
    dt.lmul(&self.g.support_point(&dt.rmul(dir)))
  }
}

impl<G, M: Mul<M, M> + Copy> Transformation<M> for Transformed<G, M>
{
  #[inline]
  fn transformation(&self) -> M
  { copy self.t }

  #[inline]
  fn transform_by(&mut self, transform: &M)
  { self.t = transform * self.t; }
}

// this might do some very interesting structural optimizations
impl<G: Transformable<M, Res>,
     M: Mul<M, M>,
     Res: Transformation<M>>
Transformable<M, Res> for Transformed<G, M>
{
  #[inline]
  fn transformed(&self, transform: &M) -> Res
  { self.g.transformed(&(transform * self.t)) }
}
