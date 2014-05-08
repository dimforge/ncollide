use nalgebra::na::{AbsoluteRotate, Translation};
use bounding_volume::{HasAABB, AABB};
use geom::Cuboid;
use math::Matrix;

impl HasAABB for Cuboid {
    #[inline]
    fn aabb(&self, m: &Matrix) -> AABB {
        let center          = m.translation();
        let ws_half_extents = m.absolute_rotate(&(self.half_extents() + self.margin()));

        AABB::new(center - ws_half_extents, center + ws_half_extents)
    }
}
