use na;
use bounding_volume::{BoundingSphere, HasBoundingVolume};
use shape::Cuboid;
use math::{Isometry, Point};

impl<P: Point, M: Isometry<P>> HasBoundingVolume<M, BoundingSphere<P>> for Cuboid<P::Vector> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> BoundingSphere<P> {
        let center = m.translate_point(&P::origin());
        let radius = na::norm(self.half_extents());

        BoundingSphere::new(center, radius)
    }
}
