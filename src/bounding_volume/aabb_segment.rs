use crate::bounding_volume;
use crate::bounding_volume::{HasBoundingVolume, AABB};
use crate::math::Matrix;
use crate::math::{Point, Scalar, Vector};
use crate::shape::Segment;

impl<N: RealField + Copy> HasBoundingVolume<N, AABB<N>> for Segment<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        // SPEED: optimize this
        bounding_volume::support_map_aabb(m, self)
    }

    #[inline]
    fn local_bounding_volume(&self) -> AABB<N> {
        bounding_volume::local_support_map_aabb(self)
    }
}
