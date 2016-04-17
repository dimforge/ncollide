use na::{Rotate, Transform};
use bounding_volume::{HasBoundingVolume, AABB};
use bounding_volume;
use shape::{Cone, Cylinder, Capsule};
use shape::{Triangle, Segment};
use math::{Point, Vector};

impl<P, M> HasBoundingVolume<M, AABB<P>> for Cone<<P::Vect as Vector>::Scalar>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> AABB<P> {
        bounding_volume::implicit_shape_aabb(m, self)
    }
}

impl<P, M> HasBoundingVolume<M, AABB<P>> for Cylinder<<P::Vect as Vector>::Scalar>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> AABB<P> {
        bounding_volume::implicit_shape_aabb(m, self)
    }
}

impl<P, M> HasBoundingVolume<M, AABB<P>> for Capsule<<P::Vect as Vector>::Scalar>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> AABB<P> {
        bounding_volume::implicit_shape_aabb(m, self)
    }
}

impl<P, M> HasBoundingVolume<M, AABB<P>> for Triangle<P>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> AABB<P> {
        // FIXME: optimize that
        bounding_volume::implicit_shape_aabb(m, self)
    }
}

impl<P, M> HasBoundingVolume<M, AABB<P>> for Segment<P>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> AABB<P> {
        // FIXME: optimize that
        bounding_volume::implicit_shape_aabb(m, self)
    }
}
