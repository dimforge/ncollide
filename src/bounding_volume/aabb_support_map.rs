use crate::bounding_volume;
use crate::bounding_volume::{HasBoundingVolume, AABB};
use crate::math::Isometry;
use na::Real;
use crate::shape::{Segment, Capsule};
#[cfg(feature = "dim3")]
use crate::shape::{Cone, Cylinder, Triangle};

#[cfg(feature = "dim3")]
impl<N: Real> HasBoundingVolume<N, AABB<N>> for Cone<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        bounding_volume::support_map_aabb(m, self)
    }
}

#[cfg(feature = "dim3")]
impl<N: Real> HasBoundingVolume<N, AABB<N>> for Cylinder<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        bounding_volume::support_map_aabb(m, self)
    }
}

impl<N: Real> HasBoundingVolume<N, AABB<N>> for Capsule<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        bounding_volume::support_map_aabb(m, self)
    }
}

#[cfg(feature = "dim3")]
impl<N: Real> HasBoundingVolume<N, AABB<N>> for Triangle<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        // FIXME: optimize that
        bounding_volume::support_map_aabb(m, self)
    }
}

impl<N: Real> HasBoundingVolume<N, AABB<N>> for Segment<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        // FIXME: optimize that
        bounding_volume::support_map_aabb(m, self)
    }
}
