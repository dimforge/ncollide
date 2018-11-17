use bounding_volume::{BoundingSphere, HasBoundingVolume};
use math::{Isometry, Point};
use na::Real;
use shape::Ball;

impl<N: Real> HasBoundingVolume<N, BoundingSphere<N>> for Ball<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        let center = Point::from_coordinates(m.translation.vector);
        let radius = self.radius();

        BoundingSphere::new(center, radius)
    }
}
