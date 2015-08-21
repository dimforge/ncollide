use na::{Translation, AbsoluteRotate, Transform, Translate};
use na;
use bounding_volume::{AABB, HasBoundingVolume};
use shape::Compound;
use math::{Scalar, Point, Vect};

impl<P, M, M2> HasBoundingVolume<M2, AABB<P>> for Compound<P, M>
    where P: Point,
          P::Vect: Translate<P>,
          M2: Transform<P> + AbsoluteRotate<P::Vect> {
    #[inline]
    fn bounding_volume(&self, m: &M2) -> AABB<P> {
        let bv              = self.bvt().root_bounding_volume().unwrap();
        let ls_center       = na::orig::<P>() + bv.translation();
        let center          = m.transform(&ls_center);
        let half_extents    = (*bv.maxs() - *bv.mins()) / na::cast::<f64, <P::Vect as Vect>::Scalar>(2.0);
        let ws_half_extents = m.absolute_rotate(&half_extents);

        AABB::new(center + (-ws_half_extents), center + ws_half_extents)
    }
}
