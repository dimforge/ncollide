use bounding_volume::{HasBoundingVolume, AABB};
use bounding_volume::aabb_utils;
use shape::ConvexPolygon;
use math::{Isometry, Point};

impl<P: Point, M: Isometry<P>> HasBoundingVolume<M, AABB<P>> for ConvexPolygon<P> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> AABB<P> {
        let (min, max) = aabb_utils::point_cloud_aabb(m, self.points());

        AABB::new(min, max)
    }
}
