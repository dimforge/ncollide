use std::util;
use contact::contact::Contact;

pub struct GeometricContact<V, T>
{
  priv world1: V,
  priv world2: V,
  priv center: V,
  priv normal: V,
  priv depth:  T
}

impl<V: Copy + Neg<V>, T: Copy> Contact<V, T> for GeometricContact<V, T>
{
  fn new(center: &V, normal: &V, depth: &T, world1: &V, world2: &V)
     -> GeometricContact<V, T>
  {
    GeometricContact {
      world1: *world1,
      world2: *world2,
      center: *center,
      normal: *normal,
      depth:  *depth
    }
  }

  fn flip(&mut self)
  {
    self.normal = -self.normal;
    util::swap(&mut self.world1, &mut self.world2);
  }

  fn set_center(&mut self, &center: &V)
  { self.center = center; }

  fn center(&self) -> V
  { self.center }

  fn set_normal(&mut self, &normal: &V)
  { self.normal = normal; }

  fn normal(&self) -> V
  { self.normal }

  fn set_depth(&mut self, &depth: &T)
  { self.depth = depth; }

  fn depth(&self) -> T
  { self.depth }

  fn set_world1(&mut self, &world1: &V)
  { self.world1 = world1; }

  fn world1(&self) -> V
  { self.world1 }

  fn set_world2(&mut self, &world2: &V)
  { self.world2 = world2; }

  fn world2(&self) -> V
  { self.world2 }
}
