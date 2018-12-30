use alga::general::Real;

use crate::bounding_volume::{BoundingSphere, HasBoundingVolume};
use crate::math::{Isometry, Point};
use crate::shape::Cylinder;

impl<N: Real> HasBoundingVolume<N, BoundingSphere<N>> for Cylinder<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        let center = Point::from_coordinates(m.translation.vector);
        let radius =
            (self.radius() * self.radius() + self.half_height() * self.half_height()).sqrt();

        BoundingSphere::new(center, radius)
    }
}
