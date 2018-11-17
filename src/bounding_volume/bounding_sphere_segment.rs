use bounding_volume;
use bounding_volume::{BoundingSphere, HasBoundingVolume};
use math::Isometry;
use na::Real;
use shape::Segment;

impl<N: Real> HasBoundingVolume<N, BoundingSphere<N>> for Segment<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        let pts = [*self.a(), *self.b()];
        let (center, radius) = bounding_volume::point_cloud_bounding_sphere(&pts[..]);

        BoundingSphere::new(m * center, radius)
    }
}
