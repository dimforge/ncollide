use na::{Rotate, Transform};
use bounding_volume::{HasAABB, AABB};
use bounding_volume;
use shape::{Cone, Cylinder, Capsule};
use shape::{Triangle, Segment};
use math::{Scalar, Point, Vect};

impl<N, P, V, M> HasAABB<P, M> for Cone<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> + Rotate<V> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<P> {
        bounding_volume::implicit_shape_aabb(m, self)
    }
}

impl<N, P, V, M> HasAABB<P, M> for Cylinder<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> + Rotate<V> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<P> {
        bounding_volume::implicit_shape_aabb(m, self)
    }
}

impl<N, P, V, M> HasAABB<P, M> for Capsule<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> + Rotate<V> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<P> {
        bounding_volume::implicit_shape_aabb(m, self)
    }
}

impl<N, P, V, M> HasAABB<P, M> for Triangle<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> + Rotate<V> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<P> {
        // FIXME: optimize that
        bounding_volume::implicit_shape_aabb(m, self)
    }
}

impl<N, P, V, M> HasAABB<P, M> for Segment<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> + Rotate<V> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<P> {
        // FIXME: optimize that
        bounding_volume::implicit_shape_aabb(m, self)
    }
}
