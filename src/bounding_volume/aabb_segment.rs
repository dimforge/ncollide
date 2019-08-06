use crate::bounding_volume::{HasBoundingVolume, AABB};
use crate::bounding_volume;
use crate::shape::Segment;
use crate::math::Matrix;
use crate::math::{Point, Scalar, Vector};

impl<N: RealField> HasBoundingVolume<N, AABB<N>> for Segment<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        // SPEED: optimize this
        bounding_volume::support_map_aabb(m, self)
    }

    #[inline]
    fn local_bounding_volume(&self) -> AABB<N> {
        // SPEED: add `local_support_map_aabb` function to support map
        bounding_volume::support_map_aabb(&Isometry::identity(), self)
    }
}
