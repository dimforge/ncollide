use num::Float;
use na::{Translate};
use na;
use bounding_volume::{HasBoundingSphere, BoundingSphere};
use shape::Cone;
use math::{Scalar, Point, Vect};


impl<P, M> HasBoundingSphere<P, M> for Cone<<P::Vect as Vect>::Scalar>
    where P: Point,
          M: Translate<P> {
    #[inline]
    fn bounding_sphere(&self, m: &M) -> BoundingSphere<P> {
        let center = m.translate(&na::orig());
        let radius = (self.radius() * self.radius() + self.half_height() * self.half_height()).sqrt();

        BoundingSphere::new(center, radius)
    }
}
