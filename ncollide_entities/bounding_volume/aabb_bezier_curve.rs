use bounding_volume::{AABB, HasBoundingVolume};
use bounding_volume::aabb_utils;
use shape::BezierCurve;
use math::Matrix;
use math::{Scalar, Point, Vect};

impl HasBoundingVolume for BezierCurve {
    #[inline]
    fn bounding_volume(&self, m: &Matrix) -> AABB {
        aabb_utils::point_cloud_aabb(m, self.control_points())
    }
}
