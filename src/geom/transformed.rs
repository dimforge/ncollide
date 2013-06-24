use nalgebra::traits::rlmul::{RMul, LMul};
use nalgebra::traits::delta_transform::DeltaTransform;
use geom::implicit::Implicit;
use geom::transformable::Transformable;

/**
 * Implicit representation of a geometry transformed by an affine
 * transformation.
 *
 *   - `G`: type of the shape being transformed.
 *   - `M`: type of the transformation.
 */
pub struct Transformed<'self, G, M>
{
  priv t: M,
  priv g: &'self G
}

impl<'self, G, M: Copy> Transformed<'self, G, M>
{
  /// Creates a transformed geometry from a transform.
  #[inline(always)]
  pub fn new(transform: M, geometry: &'self G) -> Transformed<'self, G, M>
  { Transformed { t: transform, g: geometry } }
}

impl<'self,
     G:  Implicit<V>,
     M:  DeltaTransform<DT>,
     DT: RMul<V> + LMul<V>,
     V:  Copy>
Implicit<V> for Transformed<'self, G, M>
{
  #[inline(always)]
  fn support_point(&self, dir: &V) -> V
  {
    let dt = self.t.delta_transform();

    // FIXME: will dt get inlined, preventing implicit copying the matrix?
    dt.lmul(&self.g.support_point(&dt.rmul(dir)))
  }
}

impl<'self, G, M: Mul<M, M> + Copy>
Transformable<M, Transformed<'self, G, M>> for Transformed<'self, G, M>
{
  #[inline(always)]
  fn transformed(&self, transform: &M) -> Transformed<'self, G, M>
  { Transformed::new(transform * self.t, self.g) }

  #[inline(always)]
  fn transform_to(&self, transform: &M, out: &mut Transformed<'self, G, M>)
  { out.t = transform * out.t; }
}
