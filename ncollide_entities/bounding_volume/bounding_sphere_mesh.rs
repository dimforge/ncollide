use na::Transform;
use bounding_volume::{BoundingSphere, HasBoundingSphere};
use bounding_volume;
use shape::{BaseMesh, BaseMeshElement, TriMesh, Polyline};
use math::{Scalar, Point, Vect};


impl<N, P, V, M, I, E> HasBoundingSphere<N, P, M> for BaseMesh<N, P, V, I, E>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P>,
          E: BaseMeshElement<I, P> {
    #[inline]
    fn bounding_sphere(&self, m: &M) -> BoundingSphere<N, P> {
        let (center, radius) = bounding_volume::point_cloud_bounding_sphere(self.vertices().as_slice());

        BoundingSphere::new(m.transform(&center), radius)
    }
}

impl<N, P, V, M> HasBoundingSphere<N, P, M> for TriMesh<N, P, V>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> {
    #[inline]
    fn bounding_sphere(&self, m: &M) -> BoundingSphere<N, P> {
        self.base_mesh().bounding_sphere(m)
    }
}

impl<N, P, V, M> HasBoundingSphere<N, P, M> for Polyline<N, P, V>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> {
    #[inline]
    fn bounding_sphere(&self, m: &M) -> BoundingSphere<N, P> {
        self.base_mesh().bounding_sphere(m)
    }
}
