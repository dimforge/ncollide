use std::uint;
use std::num::{Zero, Signed};
use nalgebra::traits::dim::Dim;
use nalgebra::traits::indexable::Indexable;
use nalgebra::traits::transformation::Transformable;
use nalgebra::traits::iterable::Iterable;
use nalgebra::traits::inv::Inv;
use geom::implicit::Implicit;
use geom::transformed::Transformed;

#[deriving(Eq, ToStr, Clone)]
pub struct Box<N, V>
{ priv half_extents: V }

impl<V: Iterable<N>, N: Signed> Box<N, V>
{
  #[inline]
  pub fn new(half_extents: V) -> Box<N, V>
  {
    assert!(half_extents.iter().all(|e| e.is_positive()));

    Box { half_extents: half_extents }
  }
}

impl<V: Clone, N> Box<N, V>
{
  #[inline]
  pub fn half_extents(&self) -> V
  { self.half_extents.clone() }
}

impl<V: Dim + Indexable<uint, N> + Zero, N: Signed> Implicit<V> for Box<N, V>
{
  #[inline]
  fn support_point(&self, dir: &V) -> V
  {
    let mut vres = Zero::zero::<V>();

    for uint::iterate(0u, Dim::dim::<V>()) |i|
    {
      if dir.at(i).is_negative()
      { vres.set(i, -self.half_extents.at(i)); }
      else
      { vres.set(i, self.half_extents.at(i)); }
    }

    vres
  }
}

impl<V: Clone, N: Clone, M: Clone + Mul<M, M> + Inv>
Transformable<M, Transformed<Box<N, V>, M, N>> for Box<N, V>
{
  #[inline]
  fn transformed(&self, transform: &M) -> Transformed<Box<N, V>, M, N>
  { Transformed::new(transform.clone(), self.clone()) }
}
