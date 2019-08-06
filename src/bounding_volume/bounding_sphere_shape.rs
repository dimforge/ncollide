use crate::bounding_volume::{BoundingSphere, HasBoundingVolume};
use crate::math::Isometry;
use na::RealField;
use crate::shape::Shape;

impl<N: RealField> HasBoundingVolume<N, BoundingSphere<N>> for dyn Shape<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        self.bounding_sphere(m)
    }

    #[inline]
    fn local_bounding_volume(&self) -> BoundingSphere<N> {
        self.local_bounding_sphere()
    }
}
