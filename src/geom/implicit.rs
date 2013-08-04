/// Traits of convex geometries representable by a support mapping function.
///
/// # Parameters:
///   * V - type of the support mapping direction argument and of the returnd point.
pub trait Implicit<V>
{
  /**
   * Evaluates the support function of the object. A support function is a
   * function associating a vector to the geometry point which maximizes their
   * dot product.
   * 
   * # Arguments:
   *  * `dir` - the input of the support function. It is not required for it to
   *            be normalized.
   */
  fn support_point(&self, dir: &V) -> V;
}
