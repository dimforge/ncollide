use na::Translate;
use na;
use bounding_volume::{HasBoundingVolume, BoundingSphere};
use shape::Capsule;
use math::{Point, Vector};


impl<P, M> HasBoundingVolume<M, BoundingSphere<P>> for Capsule<<P::Vect as Vector>::Scalar>
    where P: Point,
          M: Translate<P> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> BoundingSphere<P> {
        let center = m.translate(&na::origin());
        let radius = self.radius() + self.half_height();

        BoundingSphere::new(center, radius)
    }
}
