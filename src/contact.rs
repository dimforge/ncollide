use std::num::One;
use nalgebra::traits::workarounds::scalar_op::ScalarDiv;

pub struct Contact<V, T>
{
  world_contact1: V,
  world_contact2: V,
  center:         V,
  normal:         V,
  depth:          T
}

impl<V: Add<V, V> + ScalarDiv<T> + Copy, T: One + Add<T, T> + Copy> Contact<V, T>
{
  pub fn set(&mut self, wc1: &V, wc2: &V, n: &V, d: &T)
  {
    let _2 = &(One::one::<T>() + One::one()); // FIXME: best way to do that?

    self.world_contact1 = *wc1;
    self.world_contact2 = *wc2;
    self.center         = (*wc1 + *wc2).scalar_div(_2);
    self.normal         = *n;
    self.depth          = *d;
  }
}
