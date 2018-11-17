use bounding_volume::{HasBoundingVolume, AABB};
use math::{Isometry, Point};
use na::{self, Real};
use num::Bounded;
use shape::Plane;

impl<N: Real> HasBoundingVolume<N, AABB<N>> for Plane<N> {
    #[inline]
    fn bounding_volume(&self, _: &Isometry<N>) -> AABB<N> {
        // We divide by 2.0  so that we can still make some operations with it (like loosening)
        // without breaking the box.
        let max = Point::max_value() * na::convert(0.5f64);

        AABB::new(-max, max)
    }
}
