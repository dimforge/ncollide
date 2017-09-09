use alga::general::Real;

use bounding_volume::{HasBoundingVolume, BoundingSphere};
use shape::Cylinder;
use math::{Point, Isometry};


impl<P: Point, M: Isometry<P>> HasBoundingVolume<M, BoundingSphere<P>> for Cylinder<P::Real> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> BoundingSphere<P> {
        let center = m.translate_point(&P::origin());
        let radius = (self.radius() * self.radius() + self.half_height() * self.half_height()).sqrt();

        BoundingSphere::new(center, radius)
    }
}
