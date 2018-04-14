use bounding_volume::{BoundingSphere, HasBoundingVolume};
use bounding_volume;
use shape::{BaseMesh, BaseMeshElement, Polyline, TriMesh};
use math::{Isometry, Point};

impl<P, M, I, E> HasBoundingVolume<N, BoundingSphere<N>> for BaseMesh<P, I, E>
where
    N: Real,
    M: Isometry<P>,
    E: BaseMeshElement<I, P>,
{
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        let (center, radius) = bounding_volume::point_cloud_bounding_sphere(&self.vertices()[..]);

        BoundingSphere::new(m.transform_point(&center), radius)
    }
}

impl<N: Real> HasBoundingVolume<N, BoundingSphere<N>> for TriMesh<P> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        self.base_mesh().bounding_volume(m)
    }
}

impl<N: Real> HasBoundingVolume<N, BoundingSphere<N>> for Polyline<P> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> BoundingSphere<N> {
        self.base_mesh().bounding_volume(m)
    }
}
