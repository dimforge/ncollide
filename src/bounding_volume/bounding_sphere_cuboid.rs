use crate::bounding_volume::{BoundingSphere, HasBoundingVolume};
use crate::math::{Isometry, Point};
use na::{self, RealField};
use crate::shape::Cuboid;

impl<N: RealField> HasBoundingVolume<N, BoundingSphere<N>> for Cuboid<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        let center = Point::from(m.translation.vector);
        let radius = self.half_extents().norm();

        BoundingSphere::new(center, radius)
    }
}
