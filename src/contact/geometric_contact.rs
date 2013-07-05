use std::util;
use std::num::Zero;
use contact::contact::Contact;
use contact::contact::UpdatableContact;

/**
 * Geometric description of a contact.
 *
 *   - `V`: type of all the contact points, its center and its normal.
 *   - `N`: type of the penetration depth.
 */
#[deriving(ToStr, Eq, Clone, DeepClone)]
pub struct GeometricContact<V, N>
{
  priv local1: V,
  priv local2: V,
  priv world1: V,
  priv world2: V,
  priv center: V,
  priv normal: V,
  priv depth:  N
}

impl<V: Neg<V> + Zero + Clone, N: Clone> Contact<V, N> for GeometricContact<V, N>
{
  #[inline]
  fn new(center: V, normal: V, depth: N, world1: V, world2: V)
     -> GeometricContact<V, N>
  {
    GeometricContact {
      local1: Zero::zero(),
      local2: Zero::zero(),
      world1: world1,
      world2: world2,
      center: center,
      normal: normal,
      depth:  depth
    }
  }

  #[inline]
  fn flip(&mut self)
  {
    self.normal = -self.normal;
    util::swap(&mut self.world1, &mut self.world2);
  }

  #[inline]
  fn set_center(&mut self, center: V)
  { self.center = center; }

  #[inline]
  fn center(&self) -> V
  { self.center.clone() }

  #[inline]
  fn set_normal(&mut self, normal: V)
  { self.normal = normal; }

  #[inline]
  fn normal(&self) -> V
  { self.normal.clone() }

  #[inline]
  fn set_depth(&mut self, depth: N)
  { self.depth = depth; }

  #[inline]
  fn depth(&self) -> N
  { self.depth.clone() }

  #[inline]
  fn set_world1(&mut self, world1: V)
  { self.world1 = world1; }

  #[inline]
  fn world1(&self) -> V
  { self.world1.clone() }

  #[inline]
  fn set_world2(&mut self, world2: V)
  { self.world2 = world2; }

  #[inline]
  fn world2(&self) -> V
  { self.world2.clone() }
}

impl<V: Neg<V> + Clone, N: Clone> UpdatableContact<V, N> for GeometricContact<V, N>
{
  #[inline]
  fn set_local1(&mut self, local1: V)
  { self.local1 = local1; }

  #[inline]
  fn local1(&self) -> V
  { self.local1.clone() }

  #[inline]
  fn set_local2(&mut self, local2: V)
  { self.local2 = local2; }

  #[inline]
  fn local2(&self) -> V
  { self.local2.clone() }
}
