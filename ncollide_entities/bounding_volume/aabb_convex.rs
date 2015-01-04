use na::Transform;
use bounding_volume::{AABB, HasAABB};
use bounding_volume::aabb_utils;
use shape::Convex;
use math::{Scalar, Point};

impl<N, P, V, M> HasAABB<P, M> for Convex<P>
    where N: Scalar,
          P: Point<N, V>,
          M: Transform<P> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<P> {
        let (min, max) = aabb_utils::point_cloud_aabb(m, self.points());

        AABB::new(min, max)
    }
}
