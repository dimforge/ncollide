use bounding_volume::{HasBoundingVolume, AABB};
use math::{Point, Isometry};
use shape::Shape;

impl<P: Point, M: Isometry<P>> HasBoundingVolume<M, AABB<P>> for Shape<P, M> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> AABB<P> {
        self.aabb(m)
    }
}
