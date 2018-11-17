use bounding_volume::{BoundingSphere, HasBoundingVolume};
use math::{Isometry, Point};
use na::Real;
use shape::Plane;

impl<N: Real> HasBoundingVolume<N, BoundingSphere<N>> for Plane<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        let center = Point::from_coordinates(m.translation.vector);
        let radius = N::max_value();

        BoundingSphere::new(center, radius)
    }
}
