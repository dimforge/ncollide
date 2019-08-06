use crate::bounding_volume;
use crate::bounding_volume::{HasBoundingVolume, AABB};
use crate::math::Isometry;
use crate::shape::{Capsule, Segment};
#[cfg(feature = "dim3")]
use crate::shape::{Cone, Cylinder};
use na::RealField;

#[cfg(feature = "dim3")]
impl<N: RealField> HasBoundingVolume<N, AABB<N>> for Cone<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        bounding_volume::support_map_aabb(m, self)
    }

    #[inline]
    fn local_bounding_volume(&self) -> AABB<N> {
        // SPEED: add `local_support_map_aabb` function to support map
        bounding_volume::support_map_aabb(&Isometry::identity(), self)
    }
}

#[cfg(feature = "dim3")]
impl<N: RealField> HasBoundingVolume<N, AABB<N>> for Cylinder<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        bounding_volume::support_map_aabb(m, self)
    }

    #[inline]
    fn local_bounding_volume(&self) -> AABB<N> {
        // SPEED: add `local_support_map_aabb` function to support map
        bounding_volume::support_map_aabb(&Isometry::identity(), self)
    }
}

impl<N: RealField> HasBoundingVolume<N, AABB<N>> for Capsule<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        bounding_volume::support_map_aabb(m, self)
    }

    #[inline]
    fn local_bounding_volume(&self) -> AABB<N> {
        // SPEED: add `local_support_map_aabb` function to support map
        bounding_volume::support_map_aabb(&Isometry::identity(), self)
    }
}

impl<N: RealField> HasBoundingVolume<N, AABB<N>> for Segment<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        // SPEED: optimize this
        bounding_volume::support_map_aabb(m, self)
    }

    #[inline]
    fn local_bounding_volume(&self) -> AABB<N> {
        // SPEED: add `local_support_map_aabb` function to support map
        bounding_volume::support_map_aabb(&Isometry::identity(), self)
    }
}
