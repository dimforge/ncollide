use nalgebra::traits::workarounds::rlmul::{RMul, LMul};
use nalgebra::traits::delta_transform::DeltaTransform;
use geom::implicit::Implicit;

pub struct Transformed<G, T>
{
  t: T,
  g: @G
}

pub fn transformed<G, T: Copy + DeltaTransform<DT>, DT>
       (t: &T, g: @G) -> Transformed<G, T>
{ Transformed { t: *t, g: g } }

impl<G: Implicit<V>, T: DeltaTransform<DT>, DT: RMul<V> + LMul<V>, V: Copy>
Implicit<V> for Transformed<G, T>
{
  fn support_point(&self, dir: &V) -> V
  {
    let dt = self.t.delta_transform();

    // FIXME: will dt get inlined, preventing implicit copying the matrix?
    dt.lmul(&self.g.support_point(&dt.rmul(dir)))
  }
}
