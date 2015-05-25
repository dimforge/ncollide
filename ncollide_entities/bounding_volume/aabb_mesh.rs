use na::{Translate, Translation, Transform, AbsoluteRotate};
use na;
use bounding_volume::{AABB, HasAABB};
use shape::{BaseMesh, BaseMeshElement, TriMesh, Polyline};
use math::{Scalar, Point, Vect};


impl<P, M, I, E> HasAABB<P, M> for BaseMesh<P, I, E>
    where P: Point,
          P::Vect: Translate<P>,
          M: AbsoluteRotate<P::Vect> + Transform<P> + Translation<P::Vect>,
          E: BaseMeshElement<I, P> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<P> {
        let bv              = self.bvt().root_bounding_volume().unwrap();
        let ls_center       = na::orig::<P>() + bv.translation();
        let center          = m.transform(&ls_center);
        let half_extents    = (*bv.maxs() - *bv.mins()) * na::cast::<f64, <P::Vect as Vect>::Scalar>(0.5);
        let ws_half_extents = m.absolute_rotate(&half_extents);

        AABB::new(center + (-ws_half_extents), center + ws_half_extents)
    }
}

impl<P, M> HasAABB<P, M> for TriMesh<P>
    where P: Point,
          P::Vect: Translate<P>,
          M: AbsoluteRotate<P::Vect> + Transform<P> + Translation<P::Vect> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<P> {
        self.base_mesh().aabb(m)
    }
}

impl<P, M> HasAABB<P, M> for Polyline<P>
    where P: Point,
          P::Vect: Translate<P>,
          M: AbsoluteRotate<P::Vect> + Transform<P> + Translation<P::Vect> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<P> {
        self.base_mesh().aabb(m)
    }
}
