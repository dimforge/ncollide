use bounding_volume;
use bounding_volume::{BoundingSphere, HasBoundingVolume};
use math::Isometry;
use na::Real;
use shape::HeightField;

impl<N: Real> HasBoundingVolume<N, BoundingSphere<N>> for HeightField<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        self.aabb().bounding_sphere().transform_by(m)
    }
}
