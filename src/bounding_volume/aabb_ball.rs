use nalgebra::na::Translation;
use bounding_volume::{HasAABB, AABB};
use geom::Ball;
use math::{N, V, M};

pub fn ball_aabb(center: &V, radius: &N) -> AABB {
    AABB::new(center - *radius, center + *radius)
}

impl HasAABB for Ball {
    #[inline]
    fn aabb(&self, m: &M) -> AABB {
        ball_aabb(&m.translation(), &self.radius())
    }
}
