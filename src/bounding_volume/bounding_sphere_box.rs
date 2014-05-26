use nalgebra::na::Translation;
use nalgebra::na;
use math::Matrix;
use bounding_volume::{HasBoundingSphere, BoundingSphere};
use geom::Cuboid;


impl HasBoundingSphere for Cuboid {
    #[inline]
    fn bounding_sphere(&self, m: &Matrix) -> BoundingSphere {
        let center = m.translation();
        let radius = na::norm(&self.half_extents());

        BoundingSphere::new(center, radius)
    }
}
