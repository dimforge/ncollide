use na;
use bounding_volume::{HasBoundingVolume, AABB};
use shape::Compound;
use math::{Isometry, Point};

impl<P: Point, M: Isometry<P>, M2: Isometry<P>> HasBoundingVolume<M2, AABB<P>> for Compound<P, M> {
    #[inline]
    fn bounding_volume(&self, m: &M2) -> AABB<P> {
        let bv = self.bvt().root_bounding_volume().unwrap();
        let ls_center = bv.center();
        let center = m.transform_point(&ls_center);
        let half_extents = (*bv.maxs() - *bv.mins()) / na::convert::<f64, P::Real>(2.0);
        let ws_half_extents = m.absolute_rotate_vector(&half_extents);

        AABB::new(center + (-ws_half_extents), center + ws_half_extents)
    }
}
