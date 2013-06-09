use std::num::{One, Zero};
use nalgebra::traits::workarounds::scalar_op::ScalarDiv;

pub trait Contact<V, T>
{
  fn new(center: &V, normal: &V, depth: &T, world1: &V, world2: &V) -> Self;

  fn flip(&mut self);

  fn set_center(&mut self, &V);
  fn center(&self)         -> V;

  fn set_normal(&mut self, &V);
  fn normal(&self)         -> V;

  fn set_depth(&mut self, &T);
  fn depth(&self)          -> T;

  fn set_world1(&mut self, &V);
  fn world1(&self)         -> V;

  fn set_world2(&mut self, &V);
  fn world2(&self)         -> V;
}

pub fn zero<V: Zero + Copy, T: Zero + Copy, C: Contact<V, T>>() -> C
{
  Contact::new(&Zero::zero::<V>(),
               &Zero::zero::<V>(),
               &Zero::zero::<T>(),
               &Zero::zero::<V>(),
               &Zero::zero::<V>())
}

pub fn set<V: Add<V, V> + ScalarDiv<T> + Copy,
           T: One + Add<T, T> + Copy,
           C: Contact<V, T>>
       (c: &mut C, wc1: &V, wc2: &V, n: &V, d: &T)
{
  let _2 = &(One::one::<T>() + One::one()); // FIXME: a better way to do that?

  c.set_world1(wc1);
  c.set_world2(wc2);
  c.set_center(&(*wc1 + *wc2).scalar_div(_2));
  c.set_normal(n);
  c.set_depth (d);
}
