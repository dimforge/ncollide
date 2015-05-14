use na::Translate;
use na;
use bounding_volume::{HasBoundingSphere, BoundingSphere};
use shape::Cuboid;
use math::{Point, Vect};



impl<P, M> HasBoundingSphere<P, M> for Cuboid<P::Vect>
    where P: Point,
          M: Translate<P> {
    #[inline]
    fn bounding_sphere(&self, m: &M) -> BoundingSphere<P> {
        let center = m.translate(&na::orig::<P>());
        let radius = na::norm(self.half_extents());

        BoundingSphere::new(center, radius)
    }
}
