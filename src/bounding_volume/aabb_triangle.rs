use bounding_volume::{HasAABB, AABB};
use bounding_volume;
use shape::Triangle;
use math::Matrix;
use math::{Scalar, Point, Vect};

impl HasAABB for Triangle {
    #[inline]
    fn aabb(&self, m: &Matrix) -> AABB {
        // FIXME: optimize that
        bounding_volume::implicit_shape_aabb(m, self)
    }
}
