use nalgebra::traits::workarounds::rlmul::{RMul, LMul};
use nalgebra::traits::delta_transform::DeltaTransform;
use geom::implicit::Implicit;

/**
 * Implicit representation of a geometry transformed by an affine
 * transformation.
 *
 *   - `G`: type of the shape being transformed.
 *   - `M`: type of the transformation.
 */
pub struct Transformed<G, M>
{
  priv t: M,
  priv g: @G
}

impl<G, M: Copy> Transformed<G, M>
{
  /// Creates a transformed geometry from a transform.
  pub fn new(transform: &M, geometry: @G) -> Transformed<G, M>
  { Transformed { t: *transform, g: geometry } }
}

impl<G: Implicit<V>, M: DeltaTransform<DT>, DT: RMul<V> + LMul<V>, V: Copy>
Implicit<V> for Transformed<G, M>
{
  fn support_point(&self, dir: &V) -> V
  {
    let dt = self.t.delta_transform();

    // FIXME: will dt get inlined, preventing implicit copying the matrix?
    dt.lmul(&self.g.support_point(&dt.rmul(dir)))
  }
}
