use na::Real;

use bounding_volume::{BoundingSphere, HasBoundingVolume};
use bounding_volume;
use shape::Polyline;
use math::Isometry;

impl<N: Real> HasBoundingVolume<N, BoundingSphere<N>> for Polyline<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        let (center, radius) = bounding_volume::point_cloud_bounding_sphere(&self.vertices()[..]);

        BoundingSphere::new(m * center, radius)
    }
}
