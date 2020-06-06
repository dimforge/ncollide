use crate::bounding_volume::{BoundingSphere, HasBoundingVolume};
use crate::math::{Isometry, Point};
use crate::shape::Ball;
use na::RealField;

impl<N: RealField> HasBoundingVolume<N, BoundingSphere<N>> for Ball<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        let bv: BoundingSphere<N> = self.local_bounding_volume();
        bv.transform_by(m)
    }

    #[inline]
    fn local_bounding_volume(&self) -> BoundingSphere<N> {
        BoundingSphere::new(Point::origin(), self.radius)
    }
}
