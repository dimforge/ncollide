use crate::bounding_volume::{HasBoundingVolume, AABB};
use crate::math::Isometry;
use na::{self, RealField};
use crate::shape::Compound;

impl<N: RealField> HasBoundingVolume<N, AABB<N>> for Compound<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        self.aabb().transform_by(m)
    }

    #[inline]
    fn local_bounding_volume(&self) -> AABB<N> {
        self.aabb().clone()
    }
}
