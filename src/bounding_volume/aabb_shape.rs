use crate::bounding_volume::{HasBoundingVolume, AABB};
use crate::math::Isometry;
use crate::shape::Shape;
use na::RealField;

impl<N: RealField + Copy> HasBoundingVolume<N, AABB<N>> for dyn Shape<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        self.aabb(m)
    }

    #[inline]
    fn local_bounding_volume(&self) -> AABB<N> {
        self.local_aabb()
    }
}
