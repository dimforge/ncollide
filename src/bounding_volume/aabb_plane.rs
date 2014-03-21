use std::num::Bounded;
use bounding_volume::{HasAABB, AABB};
use geom::Plane;
use math::{Matrix, Vect};

impl HasAABB for Plane {
    fn aabb(&self, _: &Matrix) -> AABB {
        let _m: Vect = Bounded::max_value();

        AABB::new(-_m, Bounded::max_value())
    }
}
