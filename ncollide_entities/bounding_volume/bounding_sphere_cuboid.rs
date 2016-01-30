use na::Translate;
use na;
use bounding_volume::{HasBoundingVolume, BoundingSphere};
use shape::Cuboid;
use math::Point;



impl<P, M> HasBoundingVolume<M, BoundingSphere<P>> for Cuboid<P::Vect>
    where P: Point,
          M: Translate<P> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> BoundingSphere<P> {
        let center = m.translate(&na::orig::<P>());
        let radius = na::norm(self.half_extents());

        BoundingSphere::new(center, radius)
    }
}
