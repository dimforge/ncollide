use na::Transform;
use bounding_volume::{BoundingSphere, HasBoundingVolume};
use bounding_volume;
use shape::{BaseMesh, BaseMeshElement, TriMesh, Polyline};
use math::Point;


impl<P, M, I, E> HasBoundingVolume<M, BoundingSphere<P>> for BaseMesh<P, I, E>
    where P: Point,
          M: Transform<P>,
          E: BaseMeshElement<I, P> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> BoundingSphere<P> {
        let (center, radius) = bounding_volume::point_cloud_bounding_sphere(&self.vertices()[..]);

        BoundingSphere::new(m.transform(&center), radius)
    }
}

impl<P, M> HasBoundingVolume<M, BoundingSphere<P>> for TriMesh<P>
    where P: Point,
          M: Transform<P> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> BoundingSphere<P> {
        self.base_mesh().bounding_volume(m)
    }
}

impl<P, M> HasBoundingVolume<M, BoundingSphere<P>> for Polyline<P>
    where P: Point,
          M: Transform<P> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> BoundingSphere<P> {
        self.base_mesh().bounding_volume(m)
    }
}
