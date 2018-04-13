use na;
use bounding_volume::{HasBoundingVolume, AABB};
use shape::Plane;
use math::Point;

impl<P, M> HasBoundingVolume<M, AABB<P>> for Plane<P::Vector>
where
    P: Point,
{
    #[inline]
    fn bounding_volume(&self, _: &M) -> AABB<P> {
        // We divide by 2.0  so that we can still make some operations with it (like loosening)
        // without breaking the box.
        let max = P::max_value() * na::convert(0.5f64);

        AABB::new(-max, max)
    }
}
