use na::{Translation, AbsoluteRotate, Transform, Translate};
use na;
use bounding_volume::{AABB, HasAABB};
use geom::Compound;
use math::{Scalar, Point, Vect};

impl<N, P, V, M, M2, I> HasAABB<P, M2> for Compound<N, P, V, M, I>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P>,
          M2: Transform<P> + AbsoluteRotate<V> {
    #[inline]
    fn aabb(&self, m: &M2) -> AABB<P> {
        let bv              = self.bvt().root_bounding_volume().unwrap();
        let ls_center       = na::orig::<P>() + bv.translation();
        let center          = m.transform(&ls_center);
        let half_extents    = (bv.maxs() - *bv.mins()) / na::cast::<f64, N>(2.0);
        let ws_half_extents = m.absolute_rotate(&half_extents);

        AABB::new(center + (-ws_half_extents), center + ws_half_extents)
    }
}
