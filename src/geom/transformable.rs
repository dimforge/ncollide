/**
 * Trait of geometries which can be transformed. The transformed geometry might
 * have a different type as the original one.
 *
 *   - `M`: type of transformation.
 *   - `TS`: type of the geometry obtained after transformation.
 */
pub trait Transformable<M, TS>
{
  /// The transformed version of a geometry.
  fn transform(&self, &M) -> TS;
  /// Same as `transform` but writing the result on a mutable buffer.
  fn transform_to(&self, &M, &mut TS);
}
