use geom::implicit::Implicit;

#[deriving(Eq)]
pub struct MinkowskiSum<G1, G2>
{
  g1: @G1,
  g2: @G2
}

pub fn minkowski_sum<G1, G2>(g1: @G1, g2: @G2) -> MinkowskiSum<G1, G2>
{ MinkowskiSum { g1: g1, g2: g2 } }

impl<V: Add<V, V>, G1: Implicit<V>, G2: Implicit<V>>
Implicit<V> for MinkowskiSum<G1, G2>
{
  fn support_point(&self, dir: &V) -> V
  { self.g1.support_point(dir) + self.g2.support_point(dir) }
}
