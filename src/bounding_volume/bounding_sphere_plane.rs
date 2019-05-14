use crate::bounding_volume::{BoundingSphere, HasBoundingVolume};
use crate::math::{Isometry, Point};
use na::RealField;
use crate::shape::Plane;

impl<N: RealField> HasBoundingVolume<N, BoundingSphere<N>> for Plane<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        let bv: BoundingSphere<N> = self.local_bounding_volume();
        bv.transform_by(m)
    }

    #[inline]
    fn local_bounding_volume(&self) -> BoundingSphere<N> {
        let radius = N::max_value();

        BoundingSphere::new(Point::origin(), radius)
    }
}
