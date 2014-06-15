use bounding_volume::{AABB, HasAABB, LooseBoundingVolume};
use bounding_volume::aabb_utils;
use geom::Convex;
use math::Matrix;

impl HasAABB for Convex {
    #[inline]
    fn aabb(&self, m: &Matrix) -> AABB {
        let mut res = aabb_utils::point_cloud_aabb(m, self.pts().as_slice());

        res.loosen(self.margin());

        res
    }
}
