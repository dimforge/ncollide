use na::Real;
use utils::IsometryOps;
use alga::linear::Translation;
use bounding_volume::{HasBoundingVolume, AABB};
use shape::Cuboid;
use math::{Isometry, Point};

impl<N: Real> HasBoundingVolume<N, AABB<N>> for Cuboid<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        let center = Point::from_coordinates(m.translation.to_vector());
        let ws_half_extents = m.absolute_transform_vector(self.half_extents());

        AABB::new(center + -ws_half_extents, center + ws_half_extents)
    }
}
