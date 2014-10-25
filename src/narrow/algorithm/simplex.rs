//! Abstract definition of a simplex usable by the GJK algorithm.


/// Trait of a simplex usable by the GJK algorithm.
pub trait Simplex<N, P> {
    /// Replace the point of the simplex by a single one. The simplex is reduced to be
    /// 0-dimensional.
    fn reset(&mut self, P);

    /// Adds a point to the simplex.
    fn add_point(&mut self, P);

    /// Project the origin on the simplex and remove any sub-simplex which does not contain the
    /// projection.
    fn project_origin_and_reduce(&mut self) -> P;

    /// Projection the origin on the simplex. The simplex itself is unchanged, although it is
    /// mutable for optimization purpose.
    fn project_origin(&mut self) -> P;

    /// Checks whether a given point is already part of the simplex points.
    fn contains_point(&self, &P) -> bool;

    /// Dimension of the simplex. A simplex with `n` must be a `n - 1`-dimensional simplex.
    fn dimension(&self) -> uint;

    /// The maximum among the simplex point squared lengths.
    fn max_sq_len(&self) -> N;

    /// Modifies the points contained by this simplex.
    fn modify_pnts(&mut self, f: |&mut P|);
}
