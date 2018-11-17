use alga::linear::Translation;
use bounding_volume::{HasBoundingVolume, AABB};
use math::{Isometry, Point, Vector};
use na::Real;
use shape::Ball;

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
            &Point::from_coordinates(m.translation.to_vector()),
            self.radius(),
        )
    }
}
