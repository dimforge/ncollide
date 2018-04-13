use bounding_volume::{BoundingSphere, HasBoundingVolume};
use math::{Isometry, Point};
use shape::Shape;

impl<P: Point, M: Isometry<P>> HasBoundingVolume<M, BoundingSphere<P>> for Shape<P, M> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> BoundingSphere<P> {
        self.bounding_sphere(m)
    }
}
