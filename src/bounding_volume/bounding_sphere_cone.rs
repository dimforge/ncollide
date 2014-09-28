use na::Translation;
use math::Matrix;
use bounding_volume::{HasBoundingSphere, BoundingSphere};
use geom::Cylinder;


impl HasBoundingSphere for Cylinder {
    #[inline]
    fn bounding_sphere(&self, m: &Matrix) -> BoundingSphere {
        let center = m.translation();
        let radius = (self.radius() * self.radius() + self.half_height() * self.half_height()).sqrt();

        BoundingSphere::new(center, radius)
    }
}
