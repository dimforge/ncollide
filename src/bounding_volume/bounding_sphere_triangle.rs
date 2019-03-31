use crate::bounding_volume;
use crate::bounding_volume::{BoundingSphere, HasBoundingVolume};
use crate::math::Isometry;
use na::RealField;
use crate::shape::Triangle;

impl<N: RealField> HasBoundingVolume<N, BoundingSphere<N>> for Triangle<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        let pts = [*self.a(), *self.b(), *self.c()];
        let (center, radius) = bounding_volume::point_cloud_bounding_sphere(&pts[..]);

        BoundingSphere::new(m * center, radius)
    }
}
