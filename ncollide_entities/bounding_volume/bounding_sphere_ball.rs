use na::Translate;
use na;
use bounding_volume::{HasBoundingVolume, BoundingSphere};
use shape::Ball;
use math::{Scalar, Point, Vect};


impl<P, M> HasBoundingVolume<M, BoundingSphere<P>> for Ball<<P::Vect as Vect>::Scalar>
    where P: Point,
          M: Translate<P> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> BoundingSphere<P> {
        let center = m.translate(&na::orig());
        let radius = self.radius();

        BoundingSphere::new(center, radius)
    }
}
