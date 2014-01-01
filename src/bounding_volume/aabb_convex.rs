use bounding_volume::{AABB, HasAABB, implicit_shape_aabb};
use geom::Convex;
use math::M;

impl HasAABB for Convex {
    fn aabb(&self, m: &M) -> AABB {
        // XXX: optimize that
        implicit_shape_aabb(m, self)
    }
}
