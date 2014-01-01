use bounding_volume::{HasAABB, AABB};
use bounding_volume;
use geom::Capsule;
use math::M;

impl HasAABB for Capsule {
    fn aabb(&self, m: &M) -> AABB {
        bounding_volume::implicit_shape_aabb(m, self)
    }
}
