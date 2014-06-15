use bounding_volume::{HasAABB, AABB};
use bounding_volume;
use geom::Triangle;
use math::Matrix;

impl HasAABB for Triangle {
    #[inline]
    fn aabb(&self, m: &Matrix) -> AABB {
        // FIXME: optimize that
        bounding_volume::implicit_shape_aabb(m, self)
    }
}
