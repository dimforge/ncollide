use bounding_volume::{HasBoundingVolume, AABB};
use bounding_volume;
use shape::Triangle;
use math::Matrix;
use math::{Scalar, Point, Vector};

impl HasBoundingVolume for Triangle {
    #[inline]
    fn bounding_volume(&self, m: &Matrix) -> AABB {
        // FIXME: optimize that
        bounding_volume::implicit_shape_aabb(m, self)
    }
}
