use na::{Translate, Translation, Transform, AbsoluteRotate};
use na;
use bounding_volume::{AABB, HasAABB};
use shape::{BaseMesh, BaseMeshElement, TriMesh, Polyline};
use math::{Scalar, Point, Vect};


impl<N, P, V, M, I, E> HasAABB<P, M> for BaseMesh<N, P, V, I, E>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P>,
          M: AbsoluteRotate<V> + Transform<P> + Translation<V>,
          E: BaseMeshElement<I, P> {
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

impl<N, P, V, M> HasAABB<P, M> for TriMesh<N, P, V>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P>,
          M: AbsoluteRotate<V> + Transform<P> + Translation<V> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<P> {
        self.base_mesh().aabb(m)
    }
}

impl<N, P, V, M> HasAABB<P, M> for Polyline<N, P, V>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P>,
          M: AbsoluteRotate<V> + Transform<P> + Translation<V> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<P> {
        self.base_mesh().aabb(m)
    }
}
