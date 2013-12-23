use nalgebra::na::{Cast, VecExt, Translation, AbsoluteRotate, Transform};
use bounding_volume::{AABB, HasAABB};
use geom::Compound;

impl<N: Primitive + Orderable + Cast<f32>,
     V: VecExt<N> + Clone,
     M: Translation<V> + AbsoluteRotate<V> + Transform<V>,
     II>
HasAABB<N, V, M> for Compound<N, V, M, II> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<N, V> {
        let bv              = self.bvt().root_bounding_volume().unwrap();
        let ls_center       = bv.translation();
        let center          = m.transform(&ls_center);
        let half_extents    = (bv.maxs() - *bv.mins()) / Cast::from(2.0);
        let ws_half_extents = m.absolute_rotate(&half_extents);

        AABB::new(center - ws_half_extents, center + ws_half_extents)
    }
}
