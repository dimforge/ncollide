use bounding_volume::{AABB, HasAABB};
use bounding_volume;
use geom::Cylinder;
use math::M;

impl HasAABB for Cylinder {
    #[inline]
    fn aabb(&self, m: &M) -> AABB {
        bounding_volume::implicit_shape_aabb(m, self)
    }
}
