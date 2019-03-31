use crate::bounding_volume::{BoundingSphere, HasBoundingVolume};
use crate::math::Isometry;
use na::RealField;
use crate::shape::TriMesh;

impl<N: RealField> HasBoundingVolume<N, BoundingSphere<N>> for TriMesh<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        self.aabb().bounding_sphere().transform_by(m)
    }
}
