use nalgebra::na::{Translation, AbsoluteRotate, Transform};
use nalgebra::na;
use bounding_volume::{AABB, HasAABB};
use geom::Compound;
use math::{Scalar, Matrix};

impl HasAABB for Compound {
    #[inline]
    fn aabb(&self, m: &Matrix) -> AABB {
        let bv              = self.bvt().root_bounding_volume().unwrap();
        let ls_center       = bv.translation();
        let center          = m.transform(&ls_center);
        let half_extents    = (bv.maxs() - *bv.mins()) / na::cast::<f32, Scalar>(2.0);
        let ws_half_extents = m.absolute_rotate(&half_extents);

        AABB::new(center - ws_half_extents, center + ws_half_extents)
    }
}
