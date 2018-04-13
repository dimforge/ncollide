use bounding_volume::{BoundingSphere, HasBoundingVolume};
use shape::Ball;
use math::{Isometry, Point};

impl<P: Point, M: Isometry<P>> HasBoundingVolume<M, BoundingSphere<P>> for Ball<P::Real> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> BoundingSphere<P> {
        let center = m.translate_point(&P::origin());
        let radius = self.radius();

        BoundingSphere::new(center, radius)
    }
}
