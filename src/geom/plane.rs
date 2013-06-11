/**
 * Implicit description of a plane.
 *
 *   - `V`: type of the plane center and normal.
 */
#[deriving(Eq, ToStr)]
pub struct Plane<V>
{
  priv center: V,
  priv normal: V
}

impl<V: Copy> Plane<V>
{
  /// Builds a new plane from its center and its normal.
  pub fn new(&center: &V, &normal: &V) -> Plane<V>
  { Plane { center: center, normal: normal } }

  /// The plane normal.
  pub fn normal(&self) -> V
  { self.normal }

  /// The plane center.
  pub fn center(&self) -> V
  { self.center }
}
