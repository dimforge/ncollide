use na::Transform;
use bounding_volume::{BoundingSphere, HasBoundingSphere};
use bounding_volume;
use shape::Segment;
use math::{Scalar, Point, Vect};


impl<N, P, V, M> HasBoundingSphere<N, P, M> for Segment<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> {
    #[inline]
    fn bounding_sphere(&self, m: &M) -> BoundingSphere<N, P> {
        let pts = [ self.a().clone(), self.b().clone() ];
        let (center, radius) = bounding_volume::point_cloud_bounding_sphere(pts.as_slice());

        BoundingSphere::new(m.transform(&center), radius)
    }
}
