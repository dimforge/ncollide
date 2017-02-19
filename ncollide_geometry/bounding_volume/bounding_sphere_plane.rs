use num::Bounded;

use bounding_volume::{HasBoundingVolume, BoundingSphere};
use shape::Plane;
use math::{Point, Isometry};


impl<P: Point, M: Isometry<P>> HasBoundingVolume<M, BoundingSphere<P>> for Plane<P::Vector> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> BoundingSphere<P> {
        let center = m.translate_point(&P::origin());
        let radius = P::Real::max_value();

        BoundingSphere::new(center, radius)
    }
}
