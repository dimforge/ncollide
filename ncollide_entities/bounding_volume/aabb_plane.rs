use na::Bounded;
use na;
use bounding_volume::{HasBoundingVolume, AABB};
use shape::Plane;
use math::{Scalar, Point, Vect};


impl<P, M> HasBoundingVolume<M, AABB<P>> for Plane<P::Vect>
    where P: Point {
    #[inline]
    fn bounding_volume(&self, _: &M) -> AABB<P> {
        // we divide by 2.0  so that we can still make some operations with it (like loosening)
        // without breaking the box.
        let max: P = Bounded::max_value();
        let half: <P::Vect as Vect>::Scalar = na::cast(0.5f64);
        let half_max = max * half;

        AABB::new(-half_max, half_max)
    }
}
