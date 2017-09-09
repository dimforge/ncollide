use bounding_volume::{BoundingSphere, HasBoundingVolume};
use bounding_volume;
use shape::Triangle;
use math::{Point, Isometry};


impl<P: Point, M: Isometry<P>> HasBoundingVolume<M, BoundingSphere<P>> for Triangle<P> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> BoundingSphere<P> {
        let pts = [ *self.a(), *self.b(), *self.c() ];
        let (center, radius) = bounding_volume::point_cloud_bounding_sphere(&pts[..]);

        BoundingSphere::new(m.transform_point(&center), radius)
    }
}
