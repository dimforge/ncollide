use na::{Rotate, Transform};
use bounding_volume::{HasAABB, AABB};
use bounding_volume;
use shape::{Cone, Cylinder, Capsule};
use shape::{Triangle, Segment};
use math::{Scalar, Point, Vect};

impl<P, M> HasAABB<P, M> for Cone<<P::Vect as Vect>::Scalar>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<P> {
        bounding_volume::implicit_shape_aabb(m, self)
    }
}

impl<P, M> HasAABB<P, M> for Cylinder<<P::Vect as Vect>::Scalar>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<P> {
        bounding_volume::implicit_shape_aabb(m, self)
    }
}

impl<P, M> HasAABB<P, M> for Capsule<<P::Vect as Vect>::Scalar>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<P> {
        bounding_volume::implicit_shape_aabb(m, self)
    }
}

impl<P, M> HasAABB<P, M> for Triangle<P>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<P> {
        // FIXME: optimize that
        bounding_volume::implicit_shape_aabb(m, self)
    }
}

impl<P, M> HasAABB<P, M> for Segment<P>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<P> {
        // FIXME: optimize that
        bounding_volume::implicit_shape_aabb(m, self)
    }
}
