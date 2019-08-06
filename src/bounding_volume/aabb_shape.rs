use crate::bounding_volume::{HasBoundingVolume, AABB};
use crate::math::Isometry;
use na::RealField;
use crate::shape::Shape;

impl<N: RealField> HasBoundingVolume<N, AABB<N>> for dyn Shape<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        self.aabb(m)
    }

    #[inline]
    fn local_bounding_volume(&self) -> AABB<N> {
        self.local_aabb()
    }
}
