use na::Transform;
use bounding_volume::{BoundingSphere, HasBoundingVolume};
use bounding_volume;
use shape::Convex;
use math::Point;


impl<P, M> HasBoundingVolume<M, BoundingSphere<P>> for Convex<P>
    where P: Point,
          M: Transform<P> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> BoundingSphere<P> {
        let (center, radius) = bounding_volume::point_cloud_bounding_sphere(self.points());

        BoundingSphere::new(m.transform(&center), radius)
    }
}
