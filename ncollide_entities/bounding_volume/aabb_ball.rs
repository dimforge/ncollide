use na::Translate;
use na;
use bounding_volume::{HasAABB, AABB};
use shape::Ball;
use math::{Scalar, Point, Vect};

/// Computes the Axis-Aligned Bounding Box of a ball.
#[inline]
pub fn ball_aabb<P>(center: &P, radius: <P::Vect as Vect>::Scalar) -> AABB<P>
    where P: Point {
    AABB::new(*center + na::repeat(-radius), *center + na::repeat(radius))
}

impl<P, M> HasAABB<P, M> for Ball<<P::Vect as Vect>::Scalar>
    where P: Point,
          M: Translate<P> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<P> {
        ball_aabb(&m.translate(&na::orig()), self.radius())
    }
}
