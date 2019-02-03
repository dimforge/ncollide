use crate::bounding_volume::{HasBoundingVolume, AABB};
use crate::bounding_volume;
use crate::shape::Segment;
use crate::math::Matrix;
use crate::math::{Point, Scalar, Vector};

impl<N: Real> HasBoundingVolume for Segment<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        // FIXME: optimize that
        bounding_volume::implicit_shape_aabb(m, self)
    }
}
