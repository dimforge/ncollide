use bounding_volume::{BoundingSphere, HasBoundingVolume};
use bounding_volume;
use shape::ConvexHull;
use math::{Isometry, Point};

impl<P: Point, M: Isometry<P>> HasBoundingVolume<M, BoundingSphere<P>> for ConvexHull<P> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> BoundingSphere<P> {
        let (center, radius) = bounding_volume::point_cloud_bounding_sphere(self.points());

        BoundingSphere::new(m.transform_point(&center), radius)
    }
}
