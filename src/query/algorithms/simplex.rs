//! Abstract definition of a simplex usable by the GJK algorithm.

use std::any::Any;
use na::Real;
use math::Point;
use query::algorithms::CSOPoint;

/// Trait of a simplex usable by the GJK algorithm.
pub trait Simplex<N: Real>: Any + Send + Sync {
    /// Replace the point of the simplex by a single one. The simplex is reduced to be
    /// 0-dimensional.
    fn reset(&mut self, CSOPoint<N>);

    /// Adds a point to the simplex.
    fn add_point(&mut self, CSOPoint<N>) -> bool;
    
    fn proj_coord(&self, i: usize) -> N;

    /// Gets the i-th point of this simplex.
    fn point(&self, i: usize) -> &CSOPoint<N>;

    /// Gets the i-th point of this simplex.
    fn prev_point(&self, i: usize) -> &CSOPoint<N>;

    fn prev_proj_coord(&self, i: usize) -> N;

    /// Project the origin on the simplex and remove any sub-simplex which does not contain the
    /// projection.
    fn project_origin_and_reduce(&mut self) -> Point<N>;

    /// Projection the origin on the simplex. The simplex itself is unchanged, although it is
    /// mutable for optimization purpose.
    fn project_origin(&mut self) -> Point<N>;

    /// Checks whether a given point is already part of the simplex.
    fn contains_point(&self, &Point<N>) -> bool;

    /// Dimension of the simplex. A simplex with `n` must be a `n - 1`-dimensional simplex.
    fn dimension(&self) -> usize;

    fn prev_dimension(&self) -> usize;

    /// The maximum among the simplex point squared lengths.
    fn max_sq_len(&self) -> N;

    /// Modifies the points contained by this simplex.
    fn modify_pnts(&mut self, f: &Fn(&mut CSOPoint<N>));
}
