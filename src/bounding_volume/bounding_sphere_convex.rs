use crate::bounding_volume;
use crate::bounding_volume::{BoundingSphere, HasBoundingVolume};
use crate::math::Isometry;
use na::RealField;
use crate::shape::ConvexHull;

impl<N: RealField> HasBoundingVolume<N, BoundingSphere<N>> for ConvexHull<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        let (center, radius) = bounding_volume::point_cloud_bounding_sphere(self.points());

        BoundingSphere::new(m * center, radius)
    }
}
