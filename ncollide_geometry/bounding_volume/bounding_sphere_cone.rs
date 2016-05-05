use num::Float;
use na::{Translate};
use na;
use bounding_volume::{HasBoundingVolume, BoundingSphere};
use shape::Cone;
use math::{Point, Vector};


impl<P, M> HasBoundingVolume<M, BoundingSphere<P>> for Cone<<P::Vect as Vector>::Scalar>
    where P: Point,
          M: Translate<P> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> BoundingSphere<P> {
        let center = m.translate(&na::origin());
        let radius = (self.radius() * self.radius() + self.half_height() * self.half_height()).sqrt();

        BoundingSphere::new(center, radius)
    }
}
