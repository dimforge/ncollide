use nalgebra::traits::workarounds::rlmul::RMul;
use nalgebra::traits::delta_transform::DeltaTransformVector;
use geom::transformable::Transformable;

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

impl<V: Copy, M: RMul<V> + DeltaTransformVector<V>>
Transformable<M, Plane<V>> for Plane<V>
{
  fn transform(&self, transform: &M) -> Plane<V>
  { Plane::new(&transform.rmul(&self.center),
               &transform.delta_transform_vector(&self.normal)) }

  fn transform_to(&self, transform: &M, out: &mut Plane<V>)
  {
    out.center = transform.rmul(&self.center);
    out.normal = transform.delta_transform_vector(&self.normal);
  }
}
