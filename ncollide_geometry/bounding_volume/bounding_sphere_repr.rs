use na::Translate;
use bounding_volume::{HasBoundingVolume, BoundingSphere};
use math::{Point, Isometry};
use shape::Shape;


impl<P, M> HasBoundingVolume<M, BoundingSphere<P>> for Shape<P, M>
    where P: Point,
          P::Vect: Translate<P>,
          M: Isometry<P> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> BoundingSphere<P> {
        self.bounding_sphere(m)
    }
}
