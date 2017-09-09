use bounding_volume::{HasBoundingVolume, BoundingSphere};
use shape::Capsule;
use math::{Point, Isometry};


impl<P: Point, M: Isometry<P>> HasBoundingVolume<M, BoundingSphere<P>> for Capsule<P::Real> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> BoundingSphere<P> {
        let center = m.translate_point(&P::origin());
        let radius = self.radius() + self.half_height();

        BoundingSphere::new(center, radius)
    }
}
