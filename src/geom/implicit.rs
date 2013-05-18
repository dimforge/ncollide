pub trait Implicit<V>
{
  /// Evaluates the support function of the object. A support function is a
  /// function associating a vector to the geometry point which maximizes their
  /// dot product.
  /// * `dir` The input of the support function. It is not required for it to
  /// be normalized.
  fn support_point(&self, dir: &V) -> V;
}
