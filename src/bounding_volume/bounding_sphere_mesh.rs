use na::Transform;
use bounding_volume::{BoundingSphere, HasBoundingSphere};
use bounding_volume;
use shape::{Mesh, MeshElement};
use math::{Scalar, Point, Vect};


impl<N, P, V, M, E> HasBoundingSphere<N, P, M> for Mesh<N, P, V, E>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P>,
          E: MeshElement<P> {
    #[inline]
    fn bounding_sphere(&self, m: &M) -> BoundingSphere<N, P> {
        let (center, radius) = bounding_volume::point_cloud_bounding_sphere(self.vertices().as_slice());

        BoundingSphere::new(m.transform(&center), radius)
    }
}
