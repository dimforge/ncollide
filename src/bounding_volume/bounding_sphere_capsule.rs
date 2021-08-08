use crate::bounding_volume::{BoundingSphere, HasBoundingVolume};
use crate::math::{Isometry, Point};
use crate::shape::Capsule;
use na::RealField;

impl<N: RealField + Copy> HasBoundingVolume<N, BoundingSphere<N>> for Capsule<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        let bv: BoundingSphere<N> = self.local_bounding_volume();
        bv.transform_by(m)
    }

    #[inline]
    fn local_bounding_volume(&self) -> BoundingSphere<N> {
        let radius = self.radius + self.half_height;

        BoundingSphere::new(Point::origin(), radius)
    }
}
