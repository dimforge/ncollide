use bounding_volume::{HasBoundingVolume, AABB};
use math::Isometry;
use na::{self, Real};
use shape::HeightField;
use utils::IsometryOps;

impl<N: Real> HasBoundingVolume<N, AABB<N>> for HeightField<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        let bv = self.aabb();
        bv.transform_by(m)
    }
}
