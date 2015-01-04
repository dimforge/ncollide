use na::Transform;
use bounding_volume::{BoundingSphere, HasBoundingSphere};
use bounding_volume;
use shape::Convex;
use math::{Scalar, Point, Vect};


impl<N, P, V, M> HasBoundingSphere<N, P, M> for Convex<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> {
    #[inline]
    fn bounding_sphere(&self, m: &M) -> BoundingSphere<N, P> {
        let (center, radius) = bounding_volume::point_cloud_bounding_sphere(self.points());

        BoundingSphere::new(m.transform(&center), radius)
    }
}
