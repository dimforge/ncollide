use na::{Translate, Translation, Transform, AbsoluteRotate};
use na;
use bounding_volume::{AABB, HasAABB};
use geom::{Mesh, MeshElement};
use math::{Scalar, Point, Vect};


impl<N, P, V, M, E> HasAABB<P, M> for Mesh<N, P, V, E>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P>,
          M: AbsoluteRotate<V> + Transform<P> + Translation<V>,
          E: MeshElement<P> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<P> {
        let bv              = self.bvt().root_bounding_volume().unwrap();
        let ls_center       = na::orig::<P>() + bv.translation();
        let center          = m.transform(&ls_center);
        let half_extents    = (*bv.maxs() - *bv.mins()) * na::cast::<f64, N>(0.5);
        let ws_half_extents = m.absolute_rotate(&half_extents);

        AABB::new(center + (-ws_half_extents), center + ws_half_extents)
    }
}
