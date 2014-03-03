use bounding_volume::{AABB, HasAABB};
use bounding_volume;
use geom::Cylinder;
use math::Matrix;

impl HasAABB for Cylinder {
    #[inline]
    fn aabb(&self, m: &Matrix) -> AABB {
        bounding_volume::implicit_shape_aabb(m, self)
    }
}
