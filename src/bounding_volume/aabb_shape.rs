use bounding_volume::{HasBoundingVolume, AABB};
use math::Isometry;
use na::Real;
use shape::Shape;

impl<N: Real> HasBoundingVolume<N, AABB<N>> for Shape<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        self.aabb(m)
    }
}
