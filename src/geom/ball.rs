use nalgebra::traits::norm::Norm;
use nalgebra::traits::workarounds::scalar_op::ScalarMul;
use geom::implicit::Implicit;

#[deriving(Eq)]
pub struct Ball<N, V>
{
  center: V,
  radius: N
}

impl<N: Copy, V: Copy> Ball<N, V>
{
  pub fn new(&center: &V, &radius: &N) -> Ball<N, V>
  { Ball { center: center, radius: radius } }
}

impl<N, V: Norm<N> + ScalarMul<N>> Implicit<V> for Ball<N, V>
{
  fn support_point(&self, dir: &V) -> V
  { dir.normalized().scalar_mul(&self.radius) }
}
