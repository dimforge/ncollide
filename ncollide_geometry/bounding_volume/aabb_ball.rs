use alga::linear::Translation;
use utils;
use bounding_volume::{HasBoundingVolume, AABB};
use shape::Ball;
use math::{Point, Isometry};

/// Computes the Axis-Aligned Bounding Box of a ball.
#[inline]
pub fn ball_aabb<P: Point>(center: &P, radius: P::Real) -> AABB<P> {
    AABB::new(*center + utils::repeat(-radius), *center + utils::repeat(radius))
}

impl<P: Point, M: Isometry<P>> HasBoundingVolume<M, AABB<P>> for Ball<P::Real> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> AABB<P> {
        ball_aabb(&P::from_coordinates(m.translation().to_vector()), self.radius())
    }
}
