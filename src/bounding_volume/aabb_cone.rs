use bounding_volume::{HasAABB, AABB};
use bounding_volume;
use geom::Cone;
use math::Matrix;

impl HasAABB for Cone {
    #[inline]
    fn aabb(&self, m: &Matrix) -> AABB {
        bounding_volume::implicit_shape_aabb(m, self)
    }
}
