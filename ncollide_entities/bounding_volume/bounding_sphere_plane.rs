use na::Bounded;
use na;
use bounding_volume::{HasBoundingSphere, BoundingSphere};
use shape::Plane;
use math::{Point, Vect};


impl<P, M> HasBoundingSphere<P, M> for Plane<P::Vect>
    where P: Point {
    #[inline]
    fn bounding_sphere(&self, _: &M) -> BoundingSphere<P> {
        let center = na::orig();
        let radius = Bounded::max_value(); // FIXME: is this a good idea?

        BoundingSphere::new(center, radius)
    }
}
