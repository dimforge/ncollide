use crate::bounding_volume::aabb_utils;
use crate::bounding_volume::{HasBoundingVolume, AABB};
use crate::math::Isometry;
use crate::shape::ConvexPolygon;
use na::RealField;

impl<N: RealField> HasBoundingVolume<N, AABB<N>> for ConvexPolygon<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        aabb_utils::point_cloud_aabb(m, self.points())
    }

    #[inline]
    fn local_bounding_volume(&self) -> AABB<N> {
        aabb_utils::local_point_cloud_aabb(self.points())
    }
}
