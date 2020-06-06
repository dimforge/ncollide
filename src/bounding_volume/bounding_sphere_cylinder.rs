use simba::scalar::RealField;

use crate::bounding_volume::{BoundingSphere, HasBoundingVolume};
use crate::math::{Isometry, Point};
use crate::shape::Cylinder;

impl<N: RealField> HasBoundingVolume<N, BoundingSphere<N>> for Cylinder<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        let bv: BoundingSphere<N> = self.local_bounding_volume();
        bv.transform_by(m)
    }

    #[inline]
    fn local_bounding_volume(&self) -> BoundingSphere<N> {
        let radius = (self.radius * self.radius + self.half_height * self.half_height).sqrt();

        BoundingSphere::new(Point::origin(), radius)
    }
}
