use std::num::Bounded;
use nalgebra::na::Translation;
use math::Matrix;
use bounding_volume::{HasBoundingSphere, BoundingSphere};
use geom::Plane;


impl HasBoundingSphere for Plane {
    #[inline]
    fn bounding_sphere(&self, m: &Matrix) -> BoundingSphere {
        let center = m.translation();
        let radius = Bounded::max_value(); // FIXME: is this a good idea?

        BoundingSphere::new(center, radius)
    }
}
