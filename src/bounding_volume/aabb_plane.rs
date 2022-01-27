use crate::bounding_volume::{HasBoundingVolume, AABB};
use crate::math::{Isometry, Point, Vector};
use crate::shape::Plane;
use na::{self, RealField};

impl<N: RealField + Copy> HasBoundingVolume<N, AABB<N>> for Plane<N> {
    #[inline]
    fn bounding_volume(&self, _: &Isometry<N>) -> AABB<N> {
        self.local_bounding_volume()
    }

    #[inline]
    fn local_bounding_volume(&self) -> AABB<N> {
        // We divide by 2.0  so that we can still make some operations with it (like loosening)
        // without breaking the box.
        let max = Point::from(Vector::repeat(
            N::max_value().unwrap() * na::convert(0.5f64),
        ));

        AABB::new(-max, max)
    }
}
