use crate::bounding_volume::{BoundingSphere, HasBoundingVolume};
use crate::math::Isometry;
use crate::shape::Compound;
use na::RealField;

impl<N: RealField> HasBoundingVolume<N, BoundingSphere<N>> for Compound<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        self.aabb().bounding_sphere().transform_by(m)
    }

    #[inline]
    fn local_bounding_volume(&self) -> BoundingSphere<N> {
        self.aabb().bounding_sphere()
    }
}
