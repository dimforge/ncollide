use nalgebra::na::Translation;
use math::Matrix;
use bounding_volume::{HasBoundingSphere, BoundingSphere};
use geom::Capsule;


impl HasBoundingSphere for Capsule {
    #[inline]
    fn bounding_sphere(&self, m: &Matrix) -> BoundingSphere {
        let center = m.translation();
        let radius = self.radius() + self.half_height();

        BoundingSphere::new(center, radius)
    }
}
