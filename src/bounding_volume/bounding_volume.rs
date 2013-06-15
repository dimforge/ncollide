pub trait BoundingVolume
{
  fn intersects(&self, &Self) -> bool;
  fn contains(&self, &Self)   -> bool;
  fn merge(&mut self, &Self);
  fn merged(&self, &Self) -> Self;
}

pub trait LooseBoundingVolume<N> : BoundingVolume
{
  fn loosen(&mut self, N);
  fn loosened(&self, N) -> Self;
}
