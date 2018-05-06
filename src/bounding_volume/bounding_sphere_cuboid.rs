use na::{self, Real};
use bounding_volume::{BoundingSphere, HasBoundingVolume};
use shape::Cuboid;
use math::{Isometry, Point};

impl<N: Real> HasBoundingVolume<N, BoundingSphere<N>> for Cuboid<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        let center = Point::from_coordinates(m.translation.vector);
        let radius = na::norm(self.half_extents());

        BoundingSphere::new(center, radius)
    }
}
