use alga::linear::Translation;
use crate::bounding_volume::{HasBoundingVolume, AABB};
use crate::math::{Isometry, Point, Vector};
use na::RealField;
use crate::shape::Ball;

/// Computes the Axis-Aligned Bounding Box of a ball transformed by `center`.
#[inline]
pub fn ball_aabb<N: RealField>(center: &Point<N>, radius: N) -> AABB<N> {
    AABB::new(
        *center + Vector::repeat(-radius),
        *center + Vector::repeat(radius),
    )
}

/// Computes the Axis-Aligned Bounding Box of a ball.
#[inline]
pub fn local_ball_aabb<N: RealField>(radius: N) -> AABB<N> {
    let half_extents = Point::from(Vector::repeat(radius));

    AABB::new(-half_extents, half_extents)
}

impl<N: RealField> HasBoundingVolume<N, AABB<N>> for Ball<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        ball_aabb(
            &Point::from(m.translation.to_vector()),
            self.radius(),
        )
    }

    #[inline]
    fn local_bounding_volume(&self) -> AABB<N> {
        local_ball_aabb(self.radius())
    }
}
