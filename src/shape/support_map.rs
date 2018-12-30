//! Traits for support mapping based shapes.

use crate::math::{Isometry, Point, Vector};
use na::{Real, Unit};

/// Traits of convex shapes representable by a support mapping function.
///
/// # Parameters:
///   * V - type of the support mapping direction argument and of the returned point.
pub trait SupportMap<N: Real> {
    /**
     * Evaluates the support function of the object.
     *
     * A support function is a function associating a vector to the shape point which maximizes
     * their dot product.
     */
    fn support_point(&self, transform: &Isometry<N>, dir: &Vector<N>) -> Point<N>;

    /// Same as `self.support_point` except that `dir` is normalized.
    fn support_point_toward(&self, transform: &Isometry<N>, dir: &Unit<Vector<N>>) -> Point<N> {
        self.support_point(transform, dir.as_ref())
    }
}
