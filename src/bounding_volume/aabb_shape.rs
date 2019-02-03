use crate::bounding_volume::{HasBoundingVolume, AABB};
use crate::math::Isometry;
use na::Real;
use crate::shape::Shape;

impl<N: Real> HasBoundingVolume<N, AABB<N>> for Shape<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        self.aabb(m)
    }
}
