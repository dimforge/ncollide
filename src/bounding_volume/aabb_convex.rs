use bounding_volume::{AABB, HasAABB, implicit_shape_aabb};
use geom::Convex;
use math::Matrix;

impl HasAABB for Convex {
    fn aabb(&self, m: &Matrix) -> AABB {
        // XXX: optimize that
        implicit_shape_aabb(m, self)
    }
}
