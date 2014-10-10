use na::Translation;
use na;
use math::Matrix;
use bounding_volume::{HasBoundingSphere, BoundingSphere};
use geom::Cuboid;


impl HasBoundingSphere for Cuboid {
    #[inline]
    fn bounding_sphere(&self, m: &Matrix) -> BoundingSphere {
        let center = m.translation().to_pnt();
        let radius = na::norm(&self.half_extents());

        BoundingSphere::new(center, radius)
    }
}
