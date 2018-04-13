use alga::linear::Translation;
use bounding_volume::{HasBoundingVolume, AABB};
use shape::Cuboid;
use math::{Isometry, Point};

impl<P: Point, M: Isometry<P>> HasBoundingVolume<M, AABB<P>> for Cuboid<P::Vector> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> AABB<P> {
        let center = P::from_coordinates(m.translation().to_vector());
        let ws_half_extents = m.absolute_rotate_vector(self.half_extents());

        AABB::new(center + -ws_half_extents, center + ws_half_extents)
    }
}
