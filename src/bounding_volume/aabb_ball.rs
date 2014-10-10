use na::Translation;
use bounding_volume::{HasAABB, AABB};
use geom::Ball;
use math::{Scalar, Point, Matrix};

/// Computes the Axis-Aligned Bounding Box of a ball.
#[inline]
pub fn ball_aabb(center: &Point, radius: &Scalar) -> AABB {
    AABB::new(center - *radius, center + *radius)
}

impl HasAABB for Ball {
    #[inline]
    fn aabb(&self, m: &Matrix) -> AABB {
        ball_aabb(m.translation().as_pnt(), &self.radius())
    }
}
