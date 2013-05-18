use geom::implicit::Implicit;

#[deriving(Eq)]
pub struct Reflection<G>
{ g: @G }

pub fn reflection<G>(g: @G) -> Reflection<G>
{ Reflection { g: g } }

impl<V: Neg<V>, G: Implicit<V>> Implicit<V> for Reflection<G>
{
  fn support_point(&self, dir: &V) -> V
  { -self.g.support_point(&-dir) }
}
