//! Traits for support mapping based shapes.

use na::Unit;
use math::Point;
use shape::{ConvexPolyface, FeatureId};

/// Traits of convex shapes representable by a support mapping function.
///
/// # Parameters:
///   * V - type of the support mapping direction argument and of the returned point.
pub trait SupportMap<P: Point, M> {
    /**
     * Evaluates the support function of the object.
     *
     * A support function is a function associating a vector to the shape point which maximizes
     * their dot product.
     */
    fn support_point(&self, transform: &M, dir: &P::Vector) -> P;

    /// Same as `self.support_point` except that `dir` is normalized.
    fn support_point_toward(&self, transform: &M, dir: &Unit<P::Vector>) -> P {
        self.support_point(transform, dir.as_ref())
    }

    fn support_face_toward(
        &self,
        transform: &M,
        dir: &Unit<P::Vector>,
        out: &mut ConvexPolyface<P>,
    ) {
        out.clear();
        out.push(
            self.support_point_toward(transform, dir),
            FeatureId::Unknown,
        );
    }

    fn support_feature_toward(
        &self,
        transform: &M,
        dir: &Unit<P::Vector>,
        _angle: P::Real,
        out: &mut ConvexPolyface<P>,
    ) {
        out.clear();
        out.push(
            self.support_point_toward(transform, dir),
            FeatureId::Unknown,
        );
    }

    fn is_direction_in_normal_cone(
        &self,
        transform: &M,
        feature: FeatureId,
        point: &P,
        dir: &Unit<P::Vector>,
    ) -> bool {
        false
    }
}
