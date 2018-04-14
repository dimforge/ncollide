use na::Real;
use bounding_volume::{HasBoundingVolume, AABB};
use bounding_volume::aabb_utils;
use shape::ConvexPolygon;
use math::Isometry;

impl<N: Real> HasBoundingVolume<N, AABB<N>> for ConvexPolygon<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        let (min, max) = aabb_utils::point_cloud_aabb(m, self.points());

        AABB::new(min, max)
    }
}
