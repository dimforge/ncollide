use na::Bounded;
use na;
use bounding_volume::{HasBoundingVolume, BoundingSphere};
use shape::Plane;
use math::Point;


impl<P, M> HasBoundingVolume<M, BoundingSphere<P>> for Plane<P::Vect>
    where P: Point {
    #[inline]
    fn bounding_volume(&self, _: &M) -> BoundingSphere<P> {
        let center = na::origin();
        let radius = Bounded::max_value(); // FIXME: is this a good idea?

        BoundingSphere::new(center, radius)
    }
}
