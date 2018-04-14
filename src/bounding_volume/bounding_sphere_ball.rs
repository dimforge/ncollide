use na::Real;
use bounding_volume::{BoundingSphere, HasBoundingVolume};
use shape::Ball;
use math::{Isometry, Point};

impl<N: Real> HasBoundingVolume<N, BoundingSphere<N>> for Ball<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        let center = Point::from_coordinates(m.translation.vector);
        let radius = self.radius();

        BoundingSphere::new(center, radius)
    }
}
