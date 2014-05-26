use bounding_volume::{AABB, HasAABB};
use bounding_volume::aabb_utils;
use geom::BezierSurface;
use math::Matrix;

impl HasAABB for BezierSurface {
    #[inline]
    fn aabb(&self, m: &Matrix) -> AABB {
        aabb_utils::point_cloud_aabb(m, self.control_points())
    }
}
