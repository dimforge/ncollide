use alga::general::Real;

use bounding_volume::{BoundingSphere, HasBoundingVolume};
use shape::Cone;
use math::{Isometry, Point};

impl<N: Real> HasBoundingVolume<N, BoundingSphere<N>> for Cone<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        let center = Point::from_coordinates(m.translation.vector);
        let radius =
            (self.radius() * self.radius() + self.half_height() * self.half_height()).sqrt();

        BoundingSphere::new(center, radius)
    }
}
