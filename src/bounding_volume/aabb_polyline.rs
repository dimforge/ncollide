use na::{self, Real};

use utils::IsometryOps;
use bounding_volume::{HasBoundingVolume, AABB};
use shape::{CompositeShape, Polyline};
use math::Isometry;

impl<N: Real> HasBoundingVolume<N, AABB<N>> for Polyline<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        let bv = self.bvt().root_bounding_volume().unwrap();
        let ls_center = bv.center();
        let center = m * ls_center;
        let half_extents = (*bv.maxs() - *bv.mins()) * na::convert::<f64, N>(0.5);
        let ws_half_extents = m.absolute_transform_vector(&half_extents);

        AABB::new(center + (-ws_half_extents), center + ws_half_extents)
    }
}
