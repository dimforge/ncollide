use crate::bounding_volume::{BoundingSphere, HasBoundingVolume};
use crate::math::{Isometry, Point};
use na::RealField;
use crate::shape::Capsule;

impl<N: RealField> HasBoundingVolume<N, BoundingSphere<N>> for Capsule<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        let center = Point::from(m.translation.vector);
        let radius = self.radius() + self.half_height();

        BoundingSphere::new(center, radius)
    }
}
