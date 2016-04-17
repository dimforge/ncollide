use na::Translate;
use na;
use bounding_volume::{HasBoundingVolume, BoundingSphere};
use shape::Ball;
use math::{Point, Vector};


impl<P, M> HasBoundingVolume<M, BoundingSphere<P>> for Ball<<P::Vect as Vector>::Scalar>
    where P: Point,
          M: Translate<P> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> BoundingSphere<P> {
        let center = m.translate(&na::origin());
        let radius = self.radius();

        BoundingSphere::new(center, radius)
    }
}
