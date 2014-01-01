//! Abstract definition of a simplex usable by the GJK algorithm.

use math::N;

/// Trait of a simplex usable by the GJK algorithm.
///
/// # Parameters:
///   * `V` - type of a point of the simplex.
pub trait Simplex<_V> {
    /// Replace the point of the simplex by a single one. The simplex is reduced to be
    /// 0-dimensional.
    fn reset(&mut self, _V);

    /// Translates each point of this simplex.
    fn translate_by(&mut self, &_V);

    /// Adds a point to the simplex.
    fn add_point(&mut self, _V);

    /// Project the origin on the simplex and remove any sub-simplex which does not contain the
    /// projection.
    fn project_origin_and_reduce(&mut self) -> _V;

    /// Projection the origin on the simplex. The simplex itself in unchanged, although it is mutable
    /// for optimization purpose.
    fn project_origin(&mut self) -> _V;

    /// Checks whether a given point is already part of the simplex points.
    fn contains_point(&self, &_V) -> bool;

    /// Dimension of the simplex. A simplex with `n` must be a `n - 1`-dimensional simplex.
    fn dimension(&self) -> uint;

    /// The maximum among the simplex point squared lengths.
    fn max_sq_len(&self) -> N;
}
