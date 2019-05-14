use alga::linear::Translation;
use crate::bounding_volume::{HasBoundingVolume, AABB};
use crate::math::{Isometry, Point};
use na::RealField;
use crate::shape::Cuboid;
use crate::utils::IsometryOps;

impl<N: RealField> HasBoundingVolume<N, AABB<N>> for Cuboid<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        let center = Point::from(m.translation.to_vector());
        let ws_half_extents = m.absolute_transform_vector(self.half_extents());

        AABB::from_half_extents(center, ws_half_extents)
    }

    #[inline]
    fn local_bounding_volume(&self) -> AABB<N> {
        let half_extents = Point::from(*self.half_extents());

        AABB::new(-half_extents, half_extents)
    }
}
