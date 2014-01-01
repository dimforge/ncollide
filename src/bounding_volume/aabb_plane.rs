use bounding_volume::{HasAABB, AABB};
use geom::Plane;
use math::{M, V};

impl HasAABB for Plane {
    fn aabb(&self, _: &M) -> AABB {
        let _m: V = Bounded::max_value();

        AABB::new(-_m, Bounded::max_value())
    }
}
