use bounding_volume::{HasAABB, AABB};
use bounding_volume;
use geom::Triangle;
use math::M;

impl HasAABB for Triangle {
    fn aabb(&self, m: &M) -> AABB {
        // FIXME: optimize that
        bounding_volume::implicit_shape_aabb(m, self)
    }
}
