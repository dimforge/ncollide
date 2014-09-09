use bounding_volume::{AABB, HasAABB, LooseBoundingVolume};
use bounding_volume::aabb_utils;
use geom::Convex;
use math::Matrix;

impl HasAABB for Convex {
    #[inline]
    fn aabb(&self, m: &Matrix) -> AABB {
        let (min, max) = aabb_utils::point_cloud_aabb(m, self.pts());
        let mut res    = AABB::new(min, max);

        res.loosen(self.margin());

        res
    }
}
