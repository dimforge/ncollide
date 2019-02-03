use alga::linear::Translation;
use crate::bounding_volume::{HasBoundingVolume, AABB};
use crate::math::{Isometry, Point, Vector};
use na::Real;
use crate::shape::Ball;

/// Computes the Axis-Aligned Bounding Box of a ball.
#[inline]
pub fn ball_aabb<N: Real>(center: &Point<N>, radius: N) -> AABB<N> {
    AABB::new(
        *center + Vector::repeat(-radius),
        *center + Vector::repeat(radius),
    )
}

impl<N: Real> HasBoundingVolume<N, AABB<N>> for Ball<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        ball_aabb(
            &Point::from(m.translation.to_vector()),
            self.radius(),
        )
    }
}
