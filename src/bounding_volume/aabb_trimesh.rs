use bounding_volume::{HasBoundingVolume, AABB};
use math::Isometry;
use na::{self, Real};
use shape::TriMesh;
use utils::IsometryOps;

impl<N: Real> HasBoundingVolume<N, AABB<N>> for TriMesh<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        let bv = self.aabb();
        let ls_center = bv.center();
        let center = m * ls_center;
        let half_extents = (*bv.maxs() - *bv.mins()) * na::convert::<f64, N>(0.5);
        let ws_half_extents = m.absolute_transform_vector(&half_extents);

        AABB::new(center + (-ws_half_extents), center + ws_half_extents)
    }
}
