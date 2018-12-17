use bounding_volume::{BoundingSphere, BoundingVolume, HasBoundingVolume};
use math::Isometry;
use na::Real;
use shape::Compound;

impl<N: Real> HasBoundingVolume<N, BoundingSphere<N>> for Compound<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        self.aabb().bounding_sphere().transform_by(m)
    }
}
