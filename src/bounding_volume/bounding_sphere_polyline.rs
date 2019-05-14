use na::RealField;

use crate::bounding_volume::{BoundingSphere, HasBoundingVolume};
use crate::math::Isometry;
use crate::shape::Polyline;

impl<N: RealField> HasBoundingVolume<N, BoundingSphere<N>> for Polyline<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        self.aabb().bounding_sphere().transform_by(m)
    }

    #[inline]
    fn local_bounding_volume(&self) -> BoundingSphere<N> {
        self.aabb().bounding_sphere()
    }
}
