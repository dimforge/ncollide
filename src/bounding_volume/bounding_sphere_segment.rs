use na::Real;
use bounding_volume::{BoundingSphere, HasBoundingVolume};
use bounding_volume;
use shape::Segment;
use math::{Isometry};

impl<N: Real> HasBoundingVolume<N, BoundingSphere<N>> for Segment<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        let pts = [*self.a(), *self.b()];
        let (center, radius) = bounding_volume::point_cloud_bounding_sphere(&pts[..]);

        BoundingSphere::new(m * center, radius)
    }
}
