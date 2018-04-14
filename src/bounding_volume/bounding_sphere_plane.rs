use na::Real;
use bounding_volume::{BoundingSphere, HasBoundingVolume};
use shape::Plane;
use math::{Isometry, Point};

impl<N: Real> HasBoundingVolume<N, BoundingSphere<N>> for Plane<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        let center = Point::from_coordinates(m.translation.vector);
        let radius = N::max_value();

        BoundingSphere::new(center, radius)
    }
}
