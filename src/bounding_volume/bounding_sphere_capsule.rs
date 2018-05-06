use na::Real;
use bounding_volume::{BoundingSphere, HasBoundingVolume};
use shape::Capsule;
use math::{Isometry, Point};

impl<N: Real> HasBoundingVolume<N, BoundingSphere<N>> for Capsule<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        let center = Point::from_coordinates(m.translation.vector);
        let radius = self.radius() + self.half_height();

        BoundingSphere::new(center, radius)
    }
}
