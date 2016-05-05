use na::Translate;
use na;
use bounding_volume::{HasBoundingVolume, AABB};
use shape::Ball;
use math::{Point, Vector};

/// Computes the Axis-Aligned Bounding Box of a ball.
#[inline]
pub fn ball_aabb<P>(center: &P, radius: <P::Vect as Vector>::Scalar) -> AABB<P>
    where P: Point {
    AABB::new(*center + na::repeat(-radius), *center + na::repeat(radius))
}

impl<P, M> HasBoundingVolume<M, AABB<P>> for Ball<<P::Vect as Vector>::Scalar>
    where P: Point,
          M: Translate<P> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> AABB<P> {
        ball_aabb(&m.translate(&na::origin()), self.radius())
    }
}
