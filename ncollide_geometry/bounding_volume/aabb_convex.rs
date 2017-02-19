use bounding_volume::{AABB, HasBoundingVolume};
use bounding_volume::aabb_utils;
use shape::ConvexHull;
use math::{Point, Isometry};

impl<P: Point, M: Isometry<P>> HasBoundingVolume<M, AABB<P>> for ConvexHull<P> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> AABB<P> {
        let (min, max) = aabb_utils::point_cloud_aabb(m, self.points());

        AABB::new(min, max)
    }
}
