use crate::bounding_volume::{HasBoundingVolume, AABB};
use crate::math::Isometry;
use na::{self, RealField};
use crate::shape::HeightField;

impl<N: RealField> HasBoundingVolume<N, AABB<N>> for HeightField<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        let bv = self.aabb();
        bv.transform_by(m)
    }
}
