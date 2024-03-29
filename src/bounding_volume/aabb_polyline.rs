use crate::bounding_volume::{HasBoundingVolume, AABB};
use crate::math::Isometry;
use crate::shape::Polyline;
use na::{self, RealField};

impl<N: RealField + Copy> HasBoundingVolume<N, AABB<N>> for Polyline<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        self.aabb().transform_by(m)
    }

    #[inline]
    fn local_bounding_volume(&self) -> AABB<N> {
        self.aabb().clone()
    }
}
