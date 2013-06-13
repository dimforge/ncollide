/// Traits of objects having a geometry.
pub trait HasGeom<G>
{
  /// Geometry of the object.
  fn geom<'r>(&'r self) -> &'r G;

  /// Mutable geometry of the object.
  fn geom_mut<'r>(&'r mut self) -> &'r mut G;
}
