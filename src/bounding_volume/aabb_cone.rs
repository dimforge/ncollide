use bounding_volume::{HasAABB, AABB};
use bounding_volume;
use geom::Cone;
use math::M;

impl HasAABB for Cone {
    fn aabb(&self, m: &M) -> AABB {
        bounding_volume::implicit_shape_aabb(m, self)
    }
}
