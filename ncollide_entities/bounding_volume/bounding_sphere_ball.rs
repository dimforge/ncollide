use na::Translate;
use na;
use bounding_volume::{HasBoundingSphere, BoundingSphere};
use shape::Ball;
use math::{Scalar, Point, Vect};


impl<P, M> HasBoundingSphere<P, M> for Ball<<P::Vect as Vect>::Scalar>
    where P: Point,
          M: Translate<P> {
    #[inline]
    fn bounding_sphere(&self, m: &M) -> BoundingSphere<P> {
        let center = m.translate(&na::orig());
        let radius = self.radius();

        BoundingSphere::new(center, radius)
    }
}
