use na::Transform;
use bounding_volume::{AABB, HasBoundingVolume};
use bounding_volume::aabb_utils;
use shape::ConvexHull;
use math::Point;

impl<P, M> HasBoundingVolume<M, AABB<P>> for ConvexHull<P>
    where P: Point,
          M: Transform<P> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> AABB<P> {
        let (min, max) = aabb_utils::point_cloud_aabb(m, self.points());

        AABB::new(min, max)
    }
}
