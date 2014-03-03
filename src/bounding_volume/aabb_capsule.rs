use bounding_volume::{HasAABB, AABB};
use bounding_volume;
use geom::Capsule;
use math::Matrix;

impl HasAABB for Capsule {
    fn aabb(&self, m: &Matrix) -> AABB {
        bounding_volume::implicit_shape_aabb(m, self)
    }
}
