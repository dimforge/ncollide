use nalgebra::traits::norm::Norm;
use nalgebra::traits::workarounds::scalar_op::ScalarMul;
use geom::implicit::Implicit;

#[deriving(Eq)]
pub struct Ball<T>
{ radius: T }

pub fn ball<T: Copy>(radius: T) -> Ball<T>
{ Ball { radius: radius } }

impl<T, V: Norm<T> + ScalarMul<T>> Implicit<V> for Ball<T>
{
  fn support_point(&self, dir: &V) -> V
  { dir.normalized().scalar_mul(&self.radius) }
}
