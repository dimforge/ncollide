use crate::bounding_volume::{BoundingSphere, HasBoundingVolume};
use crate::math::{Isometry, Point};
use na::Real;
use crate::shape::Capsule;

impl<N: Real> HasBoundingVolume<N, BoundingSphere<N>> for Capsule<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        let center = Point::from_coordinates(m.translation.vector);
        let radius = self.radius() + self.half_height();

        BoundingSphere::new(center, radius)
    }
}
