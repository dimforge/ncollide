use std::cmp::{min, max};
use std::num::Zero;
use nalgebra::traits::scalar_op::{ScalarAdd, ScalarSub};
//FIXME: use nalgebra::traits::basis::Basis;
use utils::default::Default;
use bounding_volume::bounding_volume::{BoundingVolume, LooseBoundingVolume};

#[deriving(ToStr, Eq, Clone)]
pub struct AABB<V>
{
  priv mins: V,
  priv maxs: V
}

impl<V: Ord> AABB<V>
{
  pub fn new(mins: V, maxs: V) -> AABB<V>
  {
    assert!(mins <= maxs);

    AABB {
      mins: mins,
      maxs: maxs
    }
  }
}

// FIXME: impl<V: Basis> AABB<V>
// FIXME: {
// FIXME:   pub fn new_from_implicit<G: Implicit<V>>(geom: &G) -> AABB<V>
// FIXME:   {
// FIXME:     let basis = Basis::canonical_basis();
// FIXME: 
// FIXME:     for basis.iter().advance |b|
// FIXME:     {
// FIXME:     }
// FIXME:   }
// FIXME: }

impl<V: Ord + Clone> BoundingVolume for AABB<V>
{
  #[inline]
  fn intersects(&self, other: &AABB<V>) -> bool
  { !(self.mins > other.maxs || other.mins > self.maxs) }

  #[inline]
  fn contains(&self, other: &AABB<V>) -> bool
  { self.mins <= other.mins && self.maxs >= other.maxs }

  #[inline]
  fn merge(&mut self, other: &AABB<V>)
  {
    self.mins = min(self.mins.clone(), other.mins.clone());
    self.maxs = max(self.maxs.clone(), other.maxs.clone());
  }

  #[inline]
  fn merged(&self, other: &AABB<V>) -> AABB<V>
  {
    AABB {
      mins: min(self.mins.clone(), other.mins.clone()),
      maxs: max(self.maxs.clone(), other.maxs.clone())
    }
  }
}

impl<V: Ord + ScalarAdd<N> + ScalarSub<N>, N> LooseBoundingVolume<N> for AABB<V>
{
  #[inline]
  fn loosen(&mut self, amount: N)
  {
    self.mins.scalar_sub_inplace(&amount);
    self.maxs.scalar_add_inplace(&amount);
  }

  #[inline]
  fn loosened(&self, amount: N) -> AABB<V>
  {
    AABB {
      mins: self.mins.scalar_sub(&amount),
      maxs: self.maxs.scalar_add(&amount)
    }
  }
}

impl<V: Zero> Default for AABB<V>
{
  fn default() -> AABB<V>
  {
    AABB {
      mins: Zero::zero(),
      maxs: Zero::zero()
    }
  }
}
