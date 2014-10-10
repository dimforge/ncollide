use std::num::Bounded;
use na;
use bounding_volume::{HasAABB, AABB};
use geom::Plane;
use math::{Matrix, Point, Scalar};

impl HasAABB for Plane {
    #[inline]
    fn aabb(&self, _: &Matrix) -> AABB {
        // we divide by 2.0  so that we can still make some operations with it (like loosening)
        // without breaking the box.
        let max:      Point  = Bounded::max_value();
        let half:     Scalar = na::cast(0.5f64);
        let half_max: Point  = max * half;

        AABB::new(-half_max, half_max)
    }
}
