use bounding_volume::{HasBoundingVolume, AABB};
use bounding_volume;
use shape::Segment;
use math::Matrix;
use math::{Scalar, Point, Vect};

impl HasBoundingVolume for Segment {
    #[inline]
    fn bounding_volume(&self, m: &Matrix) -> AABB {
        // FIXME: optimize that
        bounding_volume::implicit_shape_aabb(m, self)
    }
}
