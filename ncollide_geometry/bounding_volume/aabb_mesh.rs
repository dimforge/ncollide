use na::{Translate, Translation, Transform, AbsoluteRotate};
use na;
use bounding_volume::{self, AABB, HasBoundingVolume};
use shape::{BaseMesh, BaseMeshElement, TriMesh, Polyline};
use math::{Point, Vector};


impl<P, M, I, E> HasBoundingVolume<M, AABB<P>> for BaseMesh<P, I, E>
    where P: Point,
          P::Vect: Translate<P>,
          M: AbsoluteRotate<P::Vect> + Transform<P>,
          E: BaseMeshElement<I, P> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> AABB<P> {
        let bv              = self.bvt().root_bounding_volume().unwrap();
        let ls_center       = na::origin::<P>() + bv.translation();
        let center          = m.transform(&ls_center);
        let half_extents    = (*bv.maxs() - *bv.mins()) * na::cast::<f64, <P::Vect as Vector>::Scalar>(0.5);
        let ws_half_extents = m.absolute_rotate(&half_extents);

        AABB::new(center + (-ws_half_extents), center + ws_half_extents)
    }
}

impl<P, M> HasBoundingVolume<M, AABB<P>> for TriMesh<P>
    where P: Point,
          P::Vect: Translate<P>,
          M: AbsoluteRotate<P::Vect> + Transform<P> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> AABB<P> {
        bounding_volume::aabb(self.base_mesh(), m)
    }
}

impl<P, M> HasBoundingVolume<M, AABB<P>> for Polyline<P>
    where P: Point,
          P::Vect: Translate<P>,
          M: AbsoluteRotate<P::Vect> + Transform<P> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> AABB<P> {
        bounding_volume::aabb(self.base_mesh(), m)
    }
}
