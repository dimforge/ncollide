//! Traits for support mapping based shapes.

use na::{Real, Unit};
use math::{Vector, Point, Isometry};
use utils::IsometryOps;

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
    fn support_point(&self, transform: &Isometry<N>, dir: &Vector<N>) -> Point<N> {
        let local_dir = transform.inverse_transform_vector(dir);
        transform * self.local_support_point(&local_dir)
    }

    /// Same as `self.support_point` except that `dir` is normalized.
    fn support_point_toward(&self, transform: &Isometry<N>, dir: &Unit<Vector<N>>) -> Point<N> {
        let local_dir = transform.inverse_transform_unit_vector(dir);
        transform * self.local_support_point_toward(&local_dir)
    }

    /**
     * Evaluates the support function of the object, in its local-space.
     *
     * A support function is a function associating a vector to the shape point which maximizes
     * their dot product.
     */
    fn local_support_point(&self, dir: &Vector<N>) -> Point<N>;

    /// Same as `self.local_support_point` except that `dir` is normalized.
    fn local_support_point_toward(&self, dir: &Unit<Vector<N>>) -> Point<N> {
        self.local_support_point(dir.as_ref())
    }
}
