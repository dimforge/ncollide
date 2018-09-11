use bounding_volume::{AABB, HasBoundingVolume};
use bounding_volume::aabb_utils;
use math::Isometry;
use na::Real;
use shape::ConvexPolygon;

impl<N: Real> HasBoundingVolume<N, AABB<N>> for ConvexPolygon<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        aabb_utils::point_cloud_aabb(m, self.points())
    }
}
