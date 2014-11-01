use na::Transform;
use bounding_volume::{AABB, HasAABB};
use bounding_volume::aabb_utils;
use shape::BezierSurface;
use math::{Scalar, Point, Vect};

impl<N, P, V, M> HasAABB<P, M> for BezierSurface<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<P> {
        let (min, max) = aabb_utils::point_cloud_aabb(m, self.control_points());

        AABB::new(min, max)
    }
}
