use std::util;

/**
 * Geometric description of a contact.
 *
 * # Parameters:
 *   * `N` - type of the penetration depth.
 *   * `V` - type of all the contact points, its center and its normal.
 */
#[deriving(ToStr, Eq, Clone, DeepClone)]
pub struct Contact<N, V>
{
  /// Position of the contact on the first object. The position is expressed in world space.
  world1: V,

  /// Position of the contact on the second object. The position is expressed in world space.
  world2: V,

  /// Contact normal
  normal: V,

  /// Penetration depth
  depth:  N
}

impl<N, V> Contact<N, V>
{
  /// Creates a new contact.
  #[inline]
  pub fn new(world1: V, world2: V, normal: V, depth: N) -> Contact<N, V>
  {
    Contact {
      world1: world1,
      world2: world2,
      normal: normal,
      depth:  depth
    }
  }
}

impl<N, V: Neg<V>> Contact<N, V>
{
  /// Reverts the contact normal and swaps `world1` and `world2`.
  pub fn flip(&mut self)
  {
    util::swap(&mut self.world1, &mut self.world2);
    self.normal = -self.normal;
  }
}
