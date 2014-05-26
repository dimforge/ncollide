use nalgebra::na::Translation;
use math::Matrix;
use bounding_volume::{HasBoundingSphere, BoundingSphere};
use geom::Ball;


impl HasBoundingSphere for Ball {
    #[inline]
    fn bounding_sphere(&self, m: &Matrix) -> BoundingSphere {
        let center = m.translation();
        let radius = self.radius();

        BoundingSphere::new(center, radius)
    }
}
