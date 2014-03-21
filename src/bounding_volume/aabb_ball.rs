use nalgebra::na::Translation;
use bounding_volume::{HasAABB, AABB};
use geom::Ball;
use math::{Scalar, Vect, Matrix};

/// Computes the Axis-Aligned Bounding Box of a ball.
pub fn ball_aabb(center: &Vect, radius: &Scalar) -> AABB {
    AABB::new(center - *radius, center + *radius)
}

impl HasAABB for Ball {
    #[inline]
    fn aabb(&self, m: &Matrix) -> AABB {
        ball_aabb(&m.translation(), &self.radius())
    }
}
