use na;
use bounding_volume::{self, AABB, HasBoundingVolume};
use shape::{BaseMesh, BaseMeshElement, TriMesh, Polyline};
use math::{Point, Isometry};


impl<P, M, I, E> HasBoundingVolume<M, AABB<P>> for BaseMesh<P, I, E>
    where P: Point,
          M: Isometry<P>,
          E: BaseMeshElement<I, P> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> AABB<P> {
        let bv              = self.bvt().root_bounding_volume().unwrap();
        let ls_center       = bv.center();
        let center          = m.transform_point(&ls_center);
        let half_extents    = (*bv.maxs() - *bv.mins()) * na::convert::<f64, P::Real>(0.5);
        let ws_half_extents = m.absolute_rotate_vector(&half_extents);

        AABB::new(center + (-ws_half_extents), center + ws_half_extents)
    }
}

impl<P: Point, M: Isometry<P>> HasBoundingVolume<M, AABB<P>> for TriMesh<P> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> AABB<P> {
        bounding_volume::aabb(self.base_mesh(), m)
    }
}

impl<P: Point, M: Isometry<P>> HasBoundingVolume<M, AABB<P>> for Polyline<P> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> AABB<P> {
        bounding_volume::aabb(self.base_mesh(), m)
    }
}
