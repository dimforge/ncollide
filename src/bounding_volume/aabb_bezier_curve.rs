use bounding_volume::{AABB, HasAABB};
use bounding_volume::aabb_utils;
use geom::BezierCurve;
use math::Matrix;
use math::{Scalar, Point, Vect};

impl HasAABB for BezierCurve {
    #[inline]
    fn aabb(&self, m: &Matrix) -> AABB {
        aabb_utils::point_cloud_aabb(m, self.control_points())
    }
}
