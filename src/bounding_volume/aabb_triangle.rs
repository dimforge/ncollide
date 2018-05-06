use bounding_volume::{HasBoundingVolume, AABB};
use bounding_volume;
use shape::Triangle;
use math::Matrix;
use math::{Point, Scalar, Vector};

impl<N: Real> HasBoundingVolume<N, AABB<N>> for Triangle<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        // FIXME: optimize that
        bounding_volume::implicit_shape_aabb(m, self)
    }
}
