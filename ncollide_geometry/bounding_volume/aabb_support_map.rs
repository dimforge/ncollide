use bounding_volume::{HasBoundingVolume, AABB};
use bounding_volume;
use shape::{Cone, Cylinder, Capsule};
use shape::{Triangle, Segment};
use math::{Point, Isometry};

impl<P: Point, M: Isometry<P>> HasBoundingVolume<M, AABB<P>> for Cone<P::Real> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> AABB<P> {
        bounding_volume::support_map_aabb(m, self)
    }
}

impl<P: Point, M: Isometry<P>> HasBoundingVolume<M, AABB<P>> for Cylinder<P::Real> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> AABB<P> {
        bounding_volume::support_map_aabb(m, self)
    }
}

impl<P: Point, M: Isometry<P>> HasBoundingVolume<M, AABB<P>> for Capsule<P::Real> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> AABB<P> {
        bounding_volume::support_map_aabb(m, self)
    }
}

impl<P: Point, M: Isometry<P>> HasBoundingVolume<M, AABB<P>> for Triangle<P> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> AABB<P> {
        // FIXME: optimize that
        bounding_volume::support_map_aabb(m, self)
    }
}

impl<P: Point, M: Isometry<P>> HasBoundingVolume<M, AABB<P>> for Segment<P> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> AABB<P> {
        // FIXME: optimize that
        bounding_volume::support_map_aabb(m, self)
    }
}
