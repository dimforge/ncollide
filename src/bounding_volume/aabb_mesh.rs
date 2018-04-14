use na;
use bounding_volume::{self, HasBoundingVolume, AABB};
use shape::{BaseMesh, BaseMeshElement, Polyline, TriMesh};
use math::{Isometry, Point};

impl<P, M, I, E> HasBoundingVolume<N, AABB<N>> for BaseMesh<P, I, E>
where
    N: Real,
    M: Isometry<P>,
    E: BaseMeshElement<I, P>,
{
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        let bv = self.bvt().root_bounding_volume().unwrap();
        let ls_center = bv.center();
        let center = m.transform_point(&ls_center);
        let half_extents = (*bv.maxs() - *bv.mins()) * na::convert::<f64, N>(0.5);
        let ws_half_extents = m.absolute_transform_vector(&half_extents);

        AABB::new(center + (-ws_half_extents), center + ws_half_extents)
    }
}

impl<N: Real> HasBoundingVolume<N, AABB<N>> for TriMesh<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        bounding_volume::aabb(self.base_mesh(), m)
    }
}

impl<N: Real> HasBoundingVolume<N, AABB<N>> for Polyline<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        bounding_volume::aabb(self.base_mesh(), m)
    }
}
