use num::Float;
use na::{Translate};
use na;
use bounding_volume::{HasBoundingVolume, BoundingSphere};
use shape::Cylinder;
use math::{Point, Vect};


impl<P, M> HasBoundingVolume<M, BoundingSphere<P>> for Cylinder<<P::Vect as Vect>::Scalar>
    where P: Point,
          M: Translate<P> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> BoundingSphere<P> {
        let center = m.translate(&na::orig());
        let radius = (self.radius() * self.radius() + self.half_height() * self.half_height()).sqrt();

        BoundingSphere::new(center, radius)
    }
}
