use bounding_volume::{HasAABB, AABB};
use bounding_volume;
use geom::Segment;
use math::M;

impl HasAABB for Segment {
    fn aabb(&self, m: &M) -> AABB {
        // FIXME: optimize that
        bounding_volume::implicit_shape_aabb(m, self)
    }
}
