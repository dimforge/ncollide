use na::Translate;
use bounding_volume::{HasBoundingVolume, AABB};
use math::{Point, Isometry};
use shape::Shape;

impl<P, M> HasBoundingVolume<M, AABB<P>> for Shape<P, M>
    where P: Point,
          P::Vect: Translate<P>,
          M: Isometry<P> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> AABB<P> {
        self.aabb(m)
    }
}
