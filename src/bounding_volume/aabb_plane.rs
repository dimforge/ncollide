use std::num::Bounded;
use na;
use bounding_volume::{HasAABB, AABB};
use geom::Plane;
use math::{Matrix, Vect, Scalar};

impl HasAABB for Plane {
    #[inline]
    fn aabb(&self, _: &Matrix) -> AABB {
        // we divide by 2.0  so that we can still make some operations with it (like loosening)
        // without breaking the box.
        let max:  Vect     = Bounded::max_value();
        let half: Scalar   = na::cast(0.5f64);
        let half_max: Vect = max * half;

        AABB::new(-half_max, half_max)
    }
}
